#pragma once
// Fractal Impedance Control: the per-channel building blocks.
//
// FIC<T> holds one set of three blocks per channel (a joint, or a Cartesian axis):
//
//   Controller            the FIC law itself: a stiffness that saturates in bands (tanh blends
//                         between breakpoints) while the error grows, and a convergence branch
//                         that brings the effort down in proportion as the error shrinks.
//   Planner               a second-order trajectory generator that moves a set-point toward
//                         its goal with bounded speed and acceleration.
//   AdmittanceController  integrates an effort into a bounded position offset (how far a push
//                         moves the set-point).
//
// fic_control.h wires these into the joint, Cartesian and H-PMP controllers. fic_joint_core.h
// is the same maths without the heap, exceptions or Eigen, for the MCU and the headset;
// test/test_joint_core_equivalence.cpp keeps the two bit-identical. Change one, change both.

#include <cmath>
#include <algorithm>
#include <iostream>
#include <limits>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>

template <typename T>
class FIC {
public:
    // A single symmetric channel with two saturation points and no admittance.
    FIC()
      : N_s_(2)
      , N_negSat_(N_s_/2)
      , asymmetric_(false)
      , n_dof_(1)
      , enable_admittance_(false)
    {
        init_controllers();
    }

    // N_s:               saturation points per channel
    // asymmetric:        whether the negative side has its own breakpoints (then N_negSat of
    //                    the N_s points are on the negative side)
    // n_dof:             how many channels to build
    // enable_admittance: whether to build an admittance block per channel
    FIC(int N_s,
        bool asymmetric        = false,
        int N_negSat           = 0,
        size_t n_dof           = 1,
        bool enable_admittance = false)
      : N_s_(N_s)
      , N_negSat_(asymmetric ? N_negSat : (N_s/2))
      , asymmetric_(asymmetric)
      , n_dof_(n_dof)
      , enable_admittance_(enable_admittance)
    {
        init_controllers();
    }

    // The FIC effort for this channel's error.
    T control(size_t idx_dof, T error) {
        return controllers_[idx_dof].control(idx_dof, error);
    }

    // The planned set-point for this channel, one step of length dt toward posDesired.
    T plan(size_t idx_dof, T posDesired, T dt) {
        return planners_[idx_dof].plan(posDesired, dt);
    }

    // How many channels this instance has.
    size_t channels() const { return n_dof_; }

    // The admittance offset for this channel after one step of length dt.
    T admittance(size_t idx_dof, T dt) {
        if (!enable_admittance_) {
            throw std::logic_error("Admittance is disabled in this FIC instance");
        }
        return admittance_controllers_[idx_dof].computeDeltaPose(dt);
    }

private:
    int    N_s_;
    int    N_negSat_;
    bool   asymmetric_;
    size_t n_dof_;
    bool   enable_admittance_;

    void init_controllers() {
        controllers_.clear();
        planners_.clear();
        admittance_controllers_.clear();

        controllers_.reserve(n_dof_);
        planners_.reserve(n_dof_);
        if (enable_admittance_)
            admittance_controllers_.reserve(n_dof_);

        for (size_t i = 0; i < n_dof_; ++i) {
            if (asymmetric_)
                controllers_.emplace_back(N_s_, N_negSat_);
            else
                controllers_.emplace_back(N_s_);
            planners_.emplace_back();
            if (enable_admittance_)
                admittance_controllers_.emplace_back();
        }
    }

public:
    //=========================================================================================
    // Controller: the FIC law for one channel
    //=========================================================================================
    template <typename TC = T>
    class Controller {
        // Saturation points: N_S in all, N_NegSat of them on the negative side. N_PSat is the
        // positive count, kept for reference (the law reads N_S and N_NegSat).
        T N_S = 2;
        T N_NegSat = 1;
        T N_PSat = N_S - N_NegSat;

        // Per saturation point: where its band starts, as a fraction of the distance from the
        // previous breakpoint (0.4 to 0.9, default 0.9); the force it saturates at; and the
        // breakpoint (error) where it reaches that force.
        Eigen::VectorXd S_Start;
        Eigen::VectorXd E_s;
        Eigen::VectorXd x_b;

        // Stiffness of the linear range: K0(0) for negative errors, K0(1) for positive.
        Eigen::VectorXd K0;

        // State of the divergence/convergence logic.
        bool isInitialised = false;
        TC error_old{};
        TC effort_old{};
        TC error_max{};    // the error where the last divergence ended
        TC effort_max{};   // the effort there

        // Set by setNumSaturations; the law reads N_S, so this is informational only.
        int n_ = 0;

        // Restart on a stall (setRestartOnStall; on unless the robot turns it off). In
        // convergence the effort is zero at half the peak error, so a joint damped enough to
        // creep up to that point stops there with no torque and stays: an overdamped joint
        // stepped by 2.5 deg settles 1.25 deg short, and a run of steps leaves it a whole step
        // behind (stepped moves in the Kinova simulation). With this on, a stall starts a new convergence from
        // where the arm is, as a divergence ending there would: the effort becomes the curve's
        // value at that error and the arm heads for half of it, then half again. A stall is
        // the error not changing (by less than kStallStep a tick, the Planner's own test) for
        // kStallTicks ticks in a row (50 ms at 1 kHz) while the law pushes with less than
        // kStallEffort of its peak effort. Waiting keeps the restart out of the slow moments
        // of an ordinary move (its turning points); the effort test leaves alone an arm held
        // still against the law (a hand, a wall), which feels the law as before. ficControl turns
        // it off where an admittance moves the goal (fic_control.h restartAllowedInMode).
        bool restartOnStall_ = true;
        static constexpr double kStallStep = 1e-5;
        static constexpr int kStallTicks = 50;
        static constexpr double kStallEffort = 0.5;
        int stallTicks_ = 0;           // convergence ticks in a row with the error unchanged

        // The divergence curve: linear near zero, then a tanh blend into each saturation band
        // in turn, flat beyond the outermost breakpoint.
        TC getEffort(TC error) {
            if (error < x_b.coeff(0)) {                                   // beyond the most negative breakpoint
                return E_s.coeff(0);
            }
            else if (error > x_b.coeff(static_cast<int>(N_S) - 1)) {     // beyond the most positive one
                return E_s.coeff(static_cast<int>(N_S) - 1);
            }
            else if (error < 0 && error > S_Start.coeff(N_NegSat - 1) * x_b.coeff(N_NegSat - 1)) {
                return K0(0) * error;                                      // negative linear range
            }
            else if (error > 0 && error < S_Start.coeff(N_NegSat) * x_b.coeff(N_NegSat)) {
                return K0(1) * error;                                      // positive linear range
            }
            // Saturation bands. Band k blends from the force before it to E_s(k) with a tanh
            // that starts at the band's start point and ends at its breakpoint x_b(k); from
            // there to the next band's start the effort holds E_s(k). So band k covers
            // [start(k), start(next band)], and the outermost band runs to x_b at the ends.
            //   positive k >= N_NegSat: start = S(k)*x_b(k) for the first band,
            //       else S(k)*(x_b(k) - x_b(k-1)) + x_b(k-1)
            //   negative k <= N_NegSat-1: start = S(k)*x_b(k) for the first band,
            //       else S(k)*(x_b(k) - x_b(k+1)) + x_b(k+1)
            else {
                const int nS = static_cast<int>(N_S);
                auto posStart = [&](int k) -> TC {
                    return (k == N_NegSat) ? S_Start.coeff(k) * x_b.coeff(k)
                        : S_Start.coeff(k) * (x_b.coeff(k) - x_b.coeff(k - 1)) + x_b.coeff(k - 1);
                };
                auto negStart = [&](int k) -> TC {
                    return (k == N_NegSat - 1) ? S_Start.coeff(k) * x_b.coeff(k)
                        : S_Start.coeff(k) * (x_b.coeff(k) - x_b.coeff(k + 1)) + x_b.coeff(k + 1);
                };
                if (error > 0) {
                    for (int k = N_NegSat; k < nS; ++k) {
                        TC lo = posStart(k);
                        TC hi = (k + 1 < nS) ? posStart(k + 1) : x_b.coeff(nS - 1);
                        if (error >= lo && error <= hi) {
                            if (k == N_NegSat)   // first band: from the linear range's end force
                                return getEffortValue(S_Start.coeff(k), E_s.coeff(k),
                                    K0(1) * S_Start.coeff(k) * x_b.coeff(k),
                                    x_b.coeff(k), S_Start.coeff(k) * x_b.coeff(k), error);
                            return getEffortValue(S_Start(k), E_s(k), E_s(k - 1),
                                                  x_b(k), x_b(k - 1), error);
                        }
                    }
                } else {
                    for (int k = N_NegSat - 1; k >= 0; --k) {
                        TC hi = negStart(k);
                        TC lo = (k - 1 >= 0) ? negStart(k - 1) : x_b.coeff(0);
                        if (error <= hi && error >= lo) {
                            if (k == N_NegSat - 1)   // first band: from the linear range's end force
                                return getEffortValue(S_Start.coeff(k), E_s.coeff(k),
                                    K0(0) * S_Start.coeff(k) * x_b.coeff(k),
                                    x_b.coeff(k), S_Start.coeff(k) * x_b.coeff(k), error);
                            return getEffortValue(S_Start(k), E_s(k), E_s(k + 1),
                                                  x_b(k), x_b(k + 1), error);
                        }
                    }
                }
            }
            return T(0);   // no band matched (only between ill-ordered breakpoints)
        }

        // The tanh blend of one band: F_0 at the band's start, F_sat at its breakpoint xb.
        T getEffortValue(T S, T F_sat, T F_0, T xb, T xb0, T x) {
            T deltaF = F_sat - F_0;
            T tanhArg = ((2 * M_PI * (x - (S * (xb - xb0) + xb0))) / ((1 - S) * (xb - xb0))) - M_PI;
            return deltaF / 2.0 * (1.0 + std::tanh(tanhArg)) + F_0;
        }

        int Sign(T value) { return (value > 0) - (value < 0); }

        // Every saturation point: band start 0.9, no force, breakpoint at zero; linear stiffness 0.
        void resetProfile() {
            S_Start = Eigen::VectorXd::Constant(N_S, 0.9);
            E_s = Eigen::VectorXd::Zero(N_S);
            x_b = Eigen::VectorXd::Zero(N_S);
        }

    public:
        Controller() {
            resetProfile();
            K0 = Eigen::VectorXd::Zero(2);
        }

        // Symmetric: half the points on each side.
        Controller(T _N_s)
            : N_S(_N_s) {
            N_NegSat = N_S/2;
            N_PSat = N_S - N_NegSat;
            resetProfile();
            K0 = Eigen::VectorXd::Zero(2);
        }

        // Asymmetric: _N_negSat of the points on the negative side.
        Controller(T _N_s, T _N_negSat)
            : N_S(_N_s), N_NegSat(_N_negSat) {
            N_PSat = N_S - N_NegSat;
            resetProfile();
            K0 = Eigen::VectorXd::Zero(2);
        }

        // One step of the law. While the error grows (divergence) the effort follows the
        // saturating curve; while it shrinks (convergence) the effort falls linearly from the
        // last peak to zero at half the peak error, so the channel lets go of stored energy
        // instead of springing back. A new peak (the error growing again, or crossing zero)
        // restarts the convergence from the curve at that error, and so does a stall
        // (restartOnStall_).
        T control([[maybe_unused]] size_t idx_dof, T error) {
            if (!isInitialised) {
                isInitialised = true;
                error_old = 0;
                error_max = error;
                effort_max = 0;
                effort_old = 0;
            }

            T effort = 0;
            if (std::fabs(error) > std::fabs(error_old)) {
                // Divergence: on the curve, and remember where this phase got to.
                effort = getEffort(error);
                error_max = error;
                effort_max = effort;
                stallTicks_ = 0;
            } else {
                // Convergence. Reset the peak with the error (to the curve's value at the new
                // error): keeping the old peak effort would give K_conv = |effort_max| /
                // |error / 2|, the full previous peak at near-zero error, then a step down to
                // K0*e on the next divergence tick. A stall restarts it the same way (above).
                const bool still = std::fabs(error - error_old) < T(kStallStep)
                                && std::fabs(effort_old) < T(kStallEffort) * std::fabs(effort_max);
                stallTicks_ = still ? stallTicks_ + 1 : 0;
                const bool stalled = restartOnStall_ && stallTicks_ >= kStallTicks;
                if (stalled) stallTicks_ = 0;
                if (std::fabs(error_max) < std::fabs(error) || Sign(error_max) != Sign(error) || stalled) {
                    error_max = error;
                    effort_max = getEffort(error);
                }
                T error_mid = error_max / 2.0;
                T K_conv = std::fabs(effort_max) / std::max(std::fabs(error_mid), T(1e-5));
                effort = K_conv * (error - error_mid);
            }

            error_old = error;
            effort_old = effort;
            return effort;
        }

        //------------------------------- parameters -------------------------------------------

        Eigen::VectorXd get_x_b() const { return x_b; }

        // Breakpoints. Symmetric: the positive ones (N_NegSat of them), mirrored to the negative
        // side. Asymmetric: all N_S of them, most negative first.
        void set_x_b(const Eigen::VectorXd& x_b_in) {
            if (x_b_in.size() == 0)
                throw std::logic_error("Error max vector cannot be empty");
            x_b.resize(N_S);
            if (N_NegSat * 2 == N_S) {
                if (static_cast<Eigen::Index>(N_NegSat) != x_b_in.size())
                    throw std::logic_error("Symmetric: x_b_in size must be N_NegSat");
                for (int i = 0; i < N_NegSat; ++i)
                    x_b(i) = -x_b_in.coeff(N_NegSat - 1 - i);
                for (int i = N_NegSat; i < N_S; ++i)
                    x_b(i) =  x_b_in.coeff(i - N_NegSat);
            } else {
                if (static_cast<Eigen::Index>(N_S) != x_b_in.size())
                    throw std::logic_error("Asymmetric: x_b_in size must be N_S");
                x_b = x_b_in;
            }
        }

        Eigen::VectorXd getE_max() const { return E_s; }

        // Saturation forces, laid out as the breakpoints.
        void setE_max(const Eigen::VectorXd& E_s_in) {
            if (E_s_in.size() == 0)
                throw std::logic_error("Effort max vector cannot be empty");
            E_s.resize(N_S);
            if (N_NegSat * 2 == N_S) {
                if (E_s_in.size() != N_NegSat)
                    throw std::logic_error("Symmetric: E_s_in size must be N_NegSat");
                for (int i = 0; i < N_NegSat; ++i)
                    E_s(i) = -E_s_in.coeff(N_NegSat - 1 - i);
                for (int i = N_NegSat; i < N_S; ++i)
                    E_s(i) =  E_s_in.coeff(i - N_NegSat);
            } else {
                if (E_s_in.size() != N_S)
                    throw std::logic_error("Asymmetric: E_s_in size must be N_S");
                E_s = E_s_in;
            }
        }

        Eigen::VectorXd getS_Start() const { return S_Start; }

        // Band starts, each clamped to [0.4, 0.9]. One value: every band. Several: band by band;
        // bands with no value of their own keep 0.9. (Before 18 Oct 2026 fewer values than
        // bands shortened the vector and the law read past its end; fic_joint_core.h has always
        // kept 0.9 there.)
        void setS_Start(const Eigen::VectorXd& S_Start_in) {
            if (S_Start_in.size() == 0)
                throw std::logic_error("Saturation ratios vector cannot be empty");
            if (S_Start_in.size() == 1) {
                S_Start = Eigen::VectorXd::Constant(N_S, std::clamp(S_Start_in.coeff(0), T(0.4), T(0.9)));
            } else {
                Eigen::VectorXd s = S_Start_in;
                if (s.size() < static_cast<Eigen::Index>(N_S)) {
                    const Eigen::Index given = s.size();
                    s.conservativeResize(static_cast<Eigen::Index>(N_S));
                    s.tail(s.size() - given).setConstant(0.9);
                }
                S_Start = s;
                for (int i = 0; i < S_Start.size(); ++i)
                    S_Start(i) = std::clamp(S_Start(i), T(0.4), T(0.9));
            }
        }

        Eigen::VectorXd getStiffness() const { return K0; }

        // Linear-range stiffness [negative side, positive side]; both must be positive.
        void setStiffness(const Eigen::VectorXd& K0_in) {
            if (K0_in.minCoeff() <= 0.0)
                throw std::logic_error("Invalid stiffness: must be positive");
            K0 = K0_in;
        }

        int getNumSaturations() const { return static_cast<int>(N_S); }

        void setNumSaturations(int n) {
            if (n < 0)
                throw std::logic_error("Number of saturations must be non-negative");
            n_ = n;
        }

        // Off gives the law exactly as it was before FIC 5.11.0 (to reproduce older runs).
        bool getRestartOnStall() const { return restartOnStall_; }
        void setRestartOnStall(bool on) { restartOnStall_ = on; }

        // A new layout; clears the profile (set the breakpoints, forces and band starts again).
        void reconfigure(int new_N_S, bool asymmetric, int new_N_NegSat) {
            N_S = new_N_S;
            N_NegSat = asymmetric ? new_N_NegSat : (new_N_S / 2);
            N_PSat = N_S - N_NegSat;
            resetProfile();
        }
    };

    //=========================================================================================
    // Planner: moves a set-point toward its goal with bounded speed and acceleration
    //=========================================================================================
    // Each time the goal moves by more than 1 mm (or 1 mrad) the planner re-derives its natural
    // frequency from the desired speed and the distance left (capped at maxFrequency_, or 50 Hz
    // below a 0.1 ms step), and from that its stiffness, damping and speed and acceleration
    // caps. The acceleration then follows the same divergence/convergence rule as the FIC law.
    //
    // Per joint (config.xml): vel_desired and damping_ratio shape the move, and acc_max caps
    // its acceleration (0: no cap). Per robot (setShape, robots_config.yaml fic_law): the
    // frequency cap and the speed margin. The per-joint stiffness and frequency are kept so
    // older files load, but do nothing: the stiffness is derived at each goal change (and the
    // acceleration cap is zero before the first), and the frequency is not read.
    template <typename TP = T>
    class Planner {
        T stiffness_;
        T velDesired_;
        T accMax_;
        T dampingRatio_;
        T frequency_;       // stored, not read (see above)
        // Cap on the natural frequency (Hz) for steps of 0.1 ms or more, and how much faster
        // than the planned speed the set-point may move (the speed cap is speedMargin_ times
        // it; 1.5708 = pi/2). Before FIC 5.11.0 both were fixed at these values.
        T maxFrequency_ = 8.75;
        T speedMargin_ = 1.5708;
        // A hard cap on the set-point's speed (setSpeedLimit; 0: none). The joint planners get the
        // robot's max joint speed, so they never plan a move the velocity safety would then resist.
        T speedLimit_ = 0;
        // Position limits for a joint that does not wrap. Set by every caller (setPosLimits);
        // until then +-pi, as fic_joint_core.h.
        T posLower_ = -M_PI;
        T posUpper_ = M_PI;
        // 0: no wrap; 1: wrap to [-pi, pi]; 2: wrap to [0, 2 pi].
        T wrapped_;

        bool isInitialized_;
        T lastPosError_;
        T lastDivergencePosError_;   // largest error since the last convergence phase
        T lastPosDesired_;           // the goal when the limits were last derived
        T posPlanner_;
        T velPlanner_;
        T accPlanner_;
        T stateVelMax_;              // speed cap for the current move
        T stateAccMax_;              // acceleration cap for the current move
        // Derived at each goal change; zero until the first one (as fic_joint_core.h).
        T fd_ = 0;
        T omega_n_ = 0;
        T gainDamping_ = 0;

        int Sign(T v) { return (v >= 0) - (v < 0); }

        // min(max(x, lo), hi): the same as std::clamp unless lo > hi.
        T clamp(T x, T minX, T maxX) {
            return std::min(std::max(x, minX), maxX);
        }

        void initializeState([[maybe_unused]] T posDesired, T error) {
            isInitialized_ = true;
            lastPosError_ = 0;
            lastDivergencePosError_ = error;
            lastPosDesired_ = posPlanner_;
            stateVelMax_ = 0;
            stateAccMax_ = 0;
        }

        // distanceToGoal: posDesired - posPlanner_, wrapped for a continuous joint (plan()), so a
        // goal just across +-pi is a short move, not a 2*pi one.
        void updateVelocityAndAccelerationLimits(T distanceToGoal, T dt) {
            T dist = std::fabs(distanceToGoal);
            dist = std::max(T(0.0001), dist);

            if (dt <= 1e-4)
                fd_ = std::min(std::fabs(velDesired_ / dist), T(50.0));
            else
                fd_ = std::min(std::fabs(velDesired_ / dist), maxFrequency_);

            omega_n_ = fd_ * T(2.0) * T(M_PI);
            gainDamping_ = dampingRatio_ * T(2.0) * omega_n_;
            stiffness_ = omega_n_ * omega_n_;

            T vel_n = std::min(velDesired_, omega_n_ * dist);
            stateVelMax_ = speedMargin_ * vel_n;

            T a_max = stateVelMax_ * vel_n / dist;
            T An = omega_n_ * omega_n_ * dist;

            // acc_max on top. The shipped 5000 is above anything the two terms reach at a
            // 1 ms step (about 1000 rad/s^2 at 3 rad/s), so those profiles move as before.
            stateAccMax_ = std::min(std::min(a_max, An), accMax_);
        }

        T profileLinear(T error, T effortMax) {
            T effort = stiffness_ * error;
            return clamp(effort, -effortMax, effortMax);
        }

        // The acceleration toward the goal, with the FIC law's divergence/convergence rule.
        T calculateAcceleration(T error) {
            if (std::fabs(error) > std::fabs(lastPosError_)) {
                accPlanner_ = profileLinear(error, stateAccMax_);
                lastDivergencePosError_ = error;
            } else {
                if (std::fabs(lastDivergencePosError_) < std::fabs(error) ||
                    Sign(lastDivergencePosError_) != Sign(error) || std::fabs(error-lastPosError_) < 1e-5) {
                    lastDivergencePosError_ = error;
                }
                T tmpAcc = profileLinear(lastDivergencePosError_, stateAccMax_);
                T tmpPosMid = lastDivergencePosError_ / T(2.0);
                T tmpGainOut = std::fabs(tmpAcc) / std::max(std::fabs(tmpPosMid), T(1e-5));
                accPlanner_ = tmpGainOut * (error - tmpPosMid);
            }
            return accPlanner_;
        }

    public:
        Planner() {
            stiffness_ = 15790.0;
            velDesired_ = 0.4;
            accMax_ = 5000.0;
            dampingRatio_ = 0.0005;
            frequency_ = 20.0;
            wrapped_ = 0;
            isInitialized_ = false;
            lastPosError_ = 0;
            lastDivergencePosError_ = 0;
            lastPosDesired_ = 0;
            posPlanner_ = 0;
            velPlanner_ = 0;
            accPlanner_ = 0;
            stateVelMax_ = 0;
            stateAccMax_ = 0;
        }

        // One step: the new set-point. Steps longer than 5 ms jump straight to the goal.
        T plan(T posDesired, T dt) {
            T error = posDesired - posPlanner_;
            if (wrapped_ != 0 && fabs(error) > M_PI) {
                error = -Sign(error)*(2*M_PI - std::fabs(error));
            }

            if (dt <= 0.005) {
                if (!isInitialized_) {
                    initializeState(posDesired, error);
                }

                // How far the goal moved since the limits were last derived, wrapped for a
                // continuous joint like the error above: unwrapped, a goal crossing +-pi looked
                // like a 2*pi move (a 0.5 Hz planner that coasted away from a goal just across
                // the seam).
                T moved = posDesired - lastPosDesired_;
                if (wrapped_ != 0 && std::fabs(moved) > M_PI) {
                    moved = -Sign(moved) * (2 * M_PI - std::fabs(moved));
                }
                if (std::fabs(moved) > T(1e-3)) {
                    updateVelocityAndAccelerationLimits(error, dt);
                    lastPosDesired_ = posDesired;   // small streamed steps add up to the next check
                }

                accPlanner_ = calculateAcceleration(error) - gainDamping_ * velPlanner_;
                const T velMax = speedLimit_ > 0 ? std::min(stateVelMax_, speedLimit_) : stateVelMax_;
                velPlanner_ = clamp(velPlanner_ + dt * accPlanner_, -velMax, velMax);
                posPlanner_ += dt * velPlanner_;
            } else {
                posPlanner_ = posDesired;
            }

            if (wrapped_ == 0) {
                posPlanner_ = std::clamp(posPlanner_, posLower_, posUpper_);
            }
            if (posPlanner_ >= 2.0 * M_PI && wrapped_ == 2)
                posPlanner_ -= 2.0 * M_PI;
            else if (posPlanner_ < 0 && wrapped_ == 2)
                posPlanner_ += 2.0 * M_PI;
            else if (posPlanner_ > M_PI && wrapped_ == 1)
                posPlanner_ -= 2.0 * M_PI;
            else if (posPlanner_ <= -M_PI && wrapped_ == 1)
                posPlanner_ += 2.0 * M_PI;

            lastPosError_ = error;
            return posPlanner_;
        }

        //------------------------------- parameters -------------------------------------------

        T getStiffness() const { return stiffness_; }
        void setStiffness(T s) { stiffness_ = std::max(s, T(0.01)); }

        T getVelDesired() const { return velDesired_; }
        void setVelDesired(T v) { velDesired_ = std::max(v, T(0.01)); }

        // 0 (or less): no cap.
        T getAccMax() const { return accMax_; }
        void setAccMax(T a) { accMax_ = a > 0 ? std::max(a, T(0.01)) : std::numeric_limits<T>::infinity(); }

        T getDampingRatio() const { return dampingRatio_; }
        void setDampingRatio(T d) { dampingRatio_ = std::max(d, T(0.0001)); }

        T getFrequency() const { return frequency_; }
        void setFrequency(T f) { frequency_ = std::max(f, T(0.0001)); }

        // The robot's frequency cap (Hz) and speed margin; a value that is not a positive finite
        // number keeps the one before.
        T getMaxFrequency() const { return maxFrequency_; }
        T getSpeedMargin() const { return speedMargin_; }
        void setShape(T max_frequency_hz, T speed_margin) {
            if (std::isfinite(max_frequency_hz) && max_frequency_hz > 0) maxFrequency_ = max_frequency_hz;
            if (std::isfinite(speed_margin) && speed_margin > 0) speedMargin_ = speed_margin;
        }

        // The set-point's speed cap; anything but a positive finite number removes it.
        T getSpeedLimit() const { return speedLimit_; }
        void setSpeedLimit(T v) { speedLimit_ = (std::isfinite(v) && v > 0) ? v : T(0); }

        T getWrapped() const { return wrapped_; }
        void setWrapped(T w) { wrapped_ = w; }

        T getPosLower() const { return posLower_; }
        T getPosUpper() const { return posUpper_; }

        // Limits in either order; +-infinity for none.
        void setPosLimits(T lower, T upper) {
            if (lower > upper) {
                std::swap(lower, upper);
            }
            posLower_ = lower;
            posUpper_ = upper;
        }

        // Start again from pos, at rest.
        void resetState(T pos) {
            posPlanner_ = pos;
            velPlanner_ = 0;
            accPlanner_ = 0;
            isInitialized_ = false;
        }
    };

    //=========================================================================================
    // AdmittanceController: an effort integrated into a bounded offset
    //=========================================================================================
    // A unit mass (scaled by invM) with damping Kv, driven by desired - current effort. The
    // acceleration, velocity and position are each clamped to their own bounds.
    template <typename TA = T>
    class AdmittanceController {
        T desired_effort_;
        T current_effort_;
        Eigen::Vector2d minMax_dFlow_;   // acceleration bounds [min, max]
        Eigen::Vector2d minMax_Flow_;    // velocity bounds
        Eigen::Vector2d minMax_Pose_;    // offset bounds
        T invM_;                         // inverse mass
        T Kv_;                           // damping
        T pose_;                         // the offset
        T velocity_;

    public:
        AdmittanceController() {
            desired_effort_ = 0;
            current_effort_ = 0;
            Kv_ = 10.0;
            minMax_dFlow_ = Eigen::Vector2d(-10.0, 10.0);
            minMax_Flow_ = Eigen::Vector2d(-5.0, 5.0);
            minMax_Pose_ = Eigen::Vector2d(-0.05, 0.05);
            invM_ = 1.0;
            pose_ = 0;
            velocity_ = 0;
        }

        void setDesiredEffort(T desired_effort) { desired_effort_ = desired_effort; }
        void setCurrentEffort(T current_effort) { current_effort_ = current_effort; }

        // Back to no offset, at rest.
        void resetState() { pose_ = 0; velocity_ = 0; }

        void setMinMax_dFlow(T min, T max) { minMax_dFlow_ << min, max; }
        void setMinMax_Flow(T min, T max)  { minMax_Flow_ << min, max; }
        void setMinMax_Pose(T min, T max)  { minMax_Pose_ << min, max; }

        void setInvM_Kv(T invM, T Kv) {
            if (invM < 0) {
                throw std::logic_error("Inverse mass must be positive");
            }
            invM_ = invM;
            if (Kv < 0) {
                throw std::logic_error("Damping coefficient must be positive");
            }
            Kv_ = Kv;
        }

        // One step: the new offset.
        T computeDeltaPose(T dt) {
            T effective_effort = desired_effort_ - current_effort_;
            T acceleration = invM_ * (effective_effort - Kv_*velocity_);
            acceleration = std::clamp(acceleration, minMax_dFlow_[0], minMax_dFlow_[1]);

            velocity_ += acceleration * dt;
            velocity_ = std::clamp(velocity_, minMax_Flow_[0], minMax_Flow_[1]);

            pose_ += velocity_ * dt;
            pose_ = std::clamp(pose_, minMax_Pose_[0], minMax_Pose_[1]);

            // Anti-windup: at a bound, the velocity still pushing into it is dropped, so the
            // offset rests at the bound instead of winding up (no snap-back on release). Velocity
            // away from the bound is kept, so motion resumes at once. It only removes energy, so
            // passivity holds, and it changes nothing away from the bounds.
            if (pose_ >= minMax_Pose_[1] && velocity_ > T(0)) velocity_ = T(0);
            else if (pose_ <= minMax_Pose_[0] && velocity_ < T(0)) velocity_ = T(0);

            return pose_;
        }
    };

    using Ctr_t = Controller<T>;
    using Pln_t = Planner<T>;
    using Adm_t = AdmittanceController<T>;

    std::vector<Ctr_t> controllers_;
    std::vector<Pln_t> planners_;
    std::vector<Adm_t> admittance_controllers_;

    Controller<T>& getController(size_t idx) {
        return controllers_.at(idx);
    }
    Planner<T>& getPlanner(size_t idx) {
        return planners_.at(idx);
    }
    AdmittanceController<T>& getAdmittanceController(size_t idx) {
        if (!enable_admittance_) {
            throw std::logic_error("AdmittanceController is disabled in this FIC instance");
        }
        return admittance_controllers_.at(idx);
    }
};
