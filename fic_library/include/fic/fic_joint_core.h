#ifndef FIC_JOINT_CORE_H
#define FIC_JOINT_CORE_H

// Portable FIC joint-space core: the per-joint impedance / planner / admittance
// path of FIC.cpp + fic_control.h, with no heap, no exceptions, no Eigen and no
// iostream, so the same code runs in fic_control (Linux) and on the MCU.
//
// This is a faithful port — including its quirks — so the MCU inner loop and the
// Linux reference produce identical torques for identical inputs. Keep it in
// lock-step with FIC.cpp / fic_control.h; test/fic_joint_core_equivalence.cpp
// replays both and fails on any difference.
//
// Mapping to the original:
//   SatController   <- FIC<T>::Controller
//   Planner         <- FIC<T>::Planner
//   Admittance      <- FIC<T>::AdmittanceController
//   JointCore       <- ficControl<T> joint path: initJointController/Planner/
//                      Admittance, handleJointImpedanceControl(WithPlanner),
//                      handleJointAdmitanceControl

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace fic {
namespace core {

inline constexpr int kMaxSat = 16;          // saturation breakpoints per joint
inline constexpr int kMaxCoreJoints = 16;   // matches shared_params.h kMaxJoints

// Same semantics as std::clamp (returns lo if v < lo, hi if hi < v, else v).
template <typename T>
inline T clampv(T v, T lo, T hi) { return (v < lo) ? lo : (hi < v) ? hi : v; }

template <typename T>
inline T wrapToPi(T x) { return std::atan2(std::sin(x), std::cos(x)); }

//=============================================================================
// Plain-data joint parameters (one joint, one mode). Filled on Linux from the
// ParameterLoader structs (joint_params_pod.h) and shipped to the MCU as-is.
//=============================================================================
struct JointParamsPOD {
    // ScalarParams
    double stiffness = 0.0;
    // ListParams
    int32_t n_saturations = 4;
    int32_t n_neg_sat = 2;
    int32_t asymmetric = 0;
    int32_t n_error_max = 0;
    int32_t n_force_max = 0;
    int32_t n_saturation_ratios = 0;
    double error_max[kMaxSat] = {};
    double force_max[kMaxSat] = {};
    double saturation_ratios[kMaxSat] = {};
    // PlannerParams
    double planner_stiffness = 0.0;
    double vel_desired = 0.0;
    double acc_max = 0.0;
    double damping_ratio = 0.0;
    double frequency = 0.0;
    // AdmittanceParams
    double inv_m = 1.0;
    double adm_damping = 0.0;
    double dflow[2] = {0.0, 0.0};
    double flow[2] = {0.0, 0.0};
};

//=============================================================================
// FIC saturation controller (FIC<T>::Controller)
//=============================================================================
template <typename T>
class SatController {
public:
    // FIC(4.0, true, 2.0, ...) in fic_control builds every channel as 4 / 2.
    SatController() { reconfigure(4, true, 2); }

    void reconfigure(int new_N_S, bool asymmetric, int new_N_NegSat) {
        N_S = new_N_S;
        N_NegSat = asymmetric ? new_N_NegSat : (new_N_S / 2);
        for (int i = 0; i < kMaxSat; ++i) { S_Start[i] = T(0.9); E_s[i] = T(0); x_b[i] = T(0); }
    }

    bool set_x_b(const double* in, int n) {
        if (n <= 0 || N_S > kMaxSat) return false;
        if (N_NegSat * 2 == N_S) {
            if (N_NegSat != n) return false;
            for (int i = 0; i < N_NegSat; ++i) x_b[i] = -in[N_NegSat - 1 - i];
            for (int i = N_NegSat; i < N_S; ++i) x_b[i] = in[i - N_NegSat];
        } else {
            if (N_S != n) return false;
            for (int i = 0; i < N_S; ++i) x_b[i] = in[i];
        }
        return true;
    }

    bool setE_max(const double* in, int n) {
        if (n <= 0 || N_S > kMaxSat) return false;
        if (N_NegSat * 2 == N_S) {
            if (N_NegSat != n) return false;
            for (int i = 0; i < N_NegSat; ++i) E_s[i] = -in[N_NegSat - 1 - i];
            for (int i = N_NegSat; i < N_S; ++i) E_s[i] = in[i - N_NegSat];
        } else {
            if (N_S != n) return false;
            for (int i = 0; i < N_S; ++i) E_s[i] = in[i];
        }
        return true;
    }

    bool setS_Start(const double* in, int n) {
        if (n <= 0 || n > kMaxSat) return false;
        if (n == 1) {
            const T v = clampv(T(in[0]), T(0.4), T(0.9));
            for (int i = 0; i < N_S; ++i) S_Start[i] = v;
        } else {
            // Band by band; bands with no value keep their 0.9 default (FIC.cpp does the
            // same since 18 Oct 2026; before, it read past the end of a short list).
            for (int i = 0; i < n; ++i) S_Start[i] = clampv(T(in[i]), T(0.4), T(0.9));
        }
        return true;
    }

    bool setStiffness(T k) {
        if (k <= T(0)) return false;
        K0[0] = k; K0[1] = k;
        return true;
    }

    const T* get_x_b() const { return x_b; }
    int numSaturations() const { return N_S; }

    // Restart on a stall, as FIC.cpp Controller::setRestartOnStall (off: the law before 5.11.0).
    void setRestartOnStall(bool on) { restartOnStall_ = on; }

    T control(T error) {
        if (!isInitialised) {
            isInitialised = true;
            error_old = 0;
            error_max = error;
            effort_max = 0;
            effort_old = 0;
        }
        T effort = 0;
        if (std::fabs(error) > std::fabs(error_old)) {        // divergence
            effort = getEffort(error);
            error_max = error;
            effort_max = effort;
            stallTicks_ = 0;
        } else {                                              // convergence
            // Reset effort_max with error_max (to the divergence-curve value at the
            // new error), as FIC.cpp: the old peak effort would give the full previous
            // peak at near-zero error, then a step down on the next divergence tick.
            // A stall (the error unchanged for 50 ticks while the law pushes with under half
            // its peak effort) restarts it too, as FIC.cpp.
            const bool still = std::fabs(error - error_old) < T(1e-5)
                            && std::fabs(effort_old) < T(0.5) * std::fabs(effort_max);
            stallTicks_ = still ? stallTicks_ + 1 : 0;
            const bool stalled = restartOnStall_ && stallTicks_ >= 50;
            if (stalled) stallTicks_ = 0;
            if (std::fabs(error_max) < std::fabs(error) || Sign(error_max) != Sign(error) || stalled) {
                error_max = error;
                effort_max = getEffort(error);
            }
            T error_mid = error_max / T(2);
            T K_conv = std::fabs(effort_max) / std::fmax(std::fabs(error_mid), T(1e-5));
            effort = K_conv * (error - error_mid);
        }
        error_old = error;
        effort_old = effort;
        return effort;
    }

private:
    int N_S = 4;
    int N_NegSat = 2;
    T S_Start[kMaxSat];
    T E_s[kMaxSat];
    T x_b[kMaxSat];
    T K0[2] = {T(0), T(0)};

    bool restartOnStall_ = true;
    int stallTicks_ = 0;
    bool isInitialised = false;
    T error_old{};
    T effort_old{};
    T error_max{};
    T effort_max{};

    static int Sign(T v) { return (v > 0) - (v < 0); }

    T posStart(int k) const {
        return (k == N_NegSat) ? S_Start[k] * x_b[k] : S_Start[k] * (x_b[k] - x_b[k - 1]) + x_b[k - 1];
    }
    T negStart(int k) const {
        return (k == N_NegSat - 1) ? S_Start[k] * x_b[k] : S_Start[k] * (x_b[k] - x_b[k + 1]) + x_b[k + 1];
    }

    T getEffortValue(T S, T F_sat, T F_0, T xb, T xb0, T x) const {
        T deltaF = F_sat - F_0;
        // Literals typed T: in float builds (MCU) a bare 2.0 or M_PI would drag
        // the whole expression into software double. Identical for T = double.
        T tanhArg = ((T(2) * T(M_PI) * (x - (S * (xb - xb0) + xb0))) / ((T(1) - S) * (xb - xb0))) - T(M_PI);
        return deltaF / T(2) * (T(1) + std::tanh(tanhArg)) + F_0;
    }

    T getEffort(T error) const {
        const int nn = N_NegSat;
        if (error < x_b[0]) {
            return E_s[0];
        } else if (error > x_b[N_S - 1]) {
            return E_s[N_S - 1];
        } else if (error < 0 && error > S_Start[nn - 1] * x_b[nn - 1]) {
            return K0[0] * error;
        } else if (error > 0 && error < S_Start[nn] * x_b[nn]) {
            return K0[1] * error;
        }
        // Saturation bands, as FIC.cpp: band k blends from the force before it to
        // E_s[k] with a tanh that starts at the band's start point and ends at x_b[k],
        // then holds E_s[k] up to the next band's start; the outermost band runs to
        // the outer breakpoint.
        //   positive k >= nn:     start = S[k]*x_b[k] for the first band,
        //                         else S[k]*(x_b[k] - x_b[k-1]) + x_b[k-1]
        //   negative k <= nn - 1: start = S[k]*x_b[k] for the first band,
        //                         else S[k]*(x_b[k] - x_b[k+1]) + x_b[k+1]
        if (error > 0) {
            for (int k = nn; k < N_S; ++k) {
                const T lo = posStart(k);
                const T hi = (k + 1 < N_S) ? posStart(k + 1) : x_b[N_S - 1];
                if (error >= lo && error <= hi) {
                    if (k == nn)   // first band: from the linear range's end force
                        return getEffortValue(S_Start[k], E_s[k], K0[1] * S_Start[k] * x_b[k],
                                              x_b[k], S_Start[k] * x_b[k], error);
                    return getEffortValue(S_Start[k], E_s[k], E_s[k - 1], x_b[k], x_b[k - 1], error);
                }
            }
        } else {
            for (int k = nn - 1; k >= 0; --k) {
                const T hi = negStart(k);
                const T lo = (k - 1 >= 0) ? negStart(k - 1) : x_b[0];
                if (error <= hi && error >= lo) {
                    if (k == nn - 1)   // first band: from the linear range's end force
                        return getEffortValue(S_Start[k], E_s[k], K0[0] * S_Start[k] * x_b[k],
                                              x_b[k], S_Start[k] * x_b[k], error);
                    return getEffortValue(S_Start[k], E_s[k], E_s[k + 1], x_b[k], x_b[k + 1], error);
                }
            }
        }
        return T(0);
    }
};

//=============================================================================
// FIC planner (FIC<T>::Planner)
//=============================================================================
template <typename T>
class Planner {
public:
    T plan(T posDesired, T dt) {
        T error = posDesired - posPlanner_;
        if (wrapped_ != 0 && std::fabs(error) > T(M_PI)) {
            error = T(-Sign(error)) * (T(2) * T(M_PI) - std::fabs(error));
        }
        if (dt <= T(0.005)) {
            if (!isInitialized_) initializeState(error);
            // How far the goal moved since the last recomputation, wrapped like the
            // error (FIC.cpp): unwrapped, a goal crossing +-pi looked like a 2*pi move.
            T moved = posDesired - lastPosDesired_;
            if (wrapped_ != 0 && std::fabs(moved) > T(M_PI)) {
                moved = T(-Sign(moved)) * (T(2) * T(M_PI) - std::fabs(moved));
            }
            if (std::fabs(moved) > T(1e-3)) {
                updateVelocityAndAccelerationLimits(error, dt);
                lastPosDesired_ = posDesired;   // small streamed steps accumulate to the next check
            }
            accPlanner_ = calculateAcceleration(error) - gainDamping_ * velPlanner_;
            const T velMax = speedLimit_ > T(0) ? std::fmin(stateVelMax_, speedLimit_) : stateVelMax_;
            velPlanner_ = clampMinMax(velPlanner_ + dt * accPlanner_, -velMax, velMax);
            posPlanner_ += dt * velPlanner_;
        } else {
            posPlanner_ = posDesired;
        }
        if (wrapped_ == 0) posPlanner_ = clampv(posPlanner_, posLower_, posUpper_);
        const T two_pi = T(2.0 * M_PI);
        if (posPlanner_ >= two_pi && wrapped_ == 2)          posPlanner_ -= two_pi;
        else if (posPlanner_ < 0 && wrapped_ == 2)           posPlanner_ += two_pi;
        else if (posPlanner_ > T(M_PI) && wrapped_ == 1)     posPlanner_ -= two_pi;
        else if (posPlanner_ <= T(-M_PI) && wrapped_ == 1)   posPlanner_ += two_pi;
        lastPosError_ = error;
        return posPlanner_;
    }

    void setStiffness(T s)    { stiffness_ = std::fmax(s, T(0.01)); }
    void setVelDesired(T v)   { velDesired_ = std::fmax(v, T(0.01)); }
    void setAccMax(T a)       { accMax_ = a > T(0) ? std::fmax(a, T(0.01)) : T(INFINITY); }   // 0: no cap
    void setDampingRatio(T d) { dampingRatio_ = std::fmax(d, T(0.0001)); }
    void setFrequency(T f)    { frequency_ = std::fmax(f, T(0.0001)); }
    void setWrapped(T w)      { wrapped_ = w; }
    // The robot's frequency cap (Hz) and speed margin, as FIC.cpp Planner::setShape.
    void setShape(T max_frequency_hz, T speed_margin) {
        if (std::isfinite(max_frequency_hz) && max_frequency_hz > T(0)) maxFrequency_ = max_frequency_hz;
        if (std::isfinite(speed_margin) && speed_margin > T(0)) speedMargin_ = speed_margin;
    }
    void setSpeedLimit(T v) { speedLimit_ = (std::isfinite(v) && v > T(0)) ? v : T(0); }
    void setPosLimits(T lower, T upper) {
        if (lower > upper) { T t = lower; lower = upper; upper = t; }
        posLower_ = lower; posUpper_ = upper;
    }
    void resetState(T pos) {
        posPlanner_ = pos; velPlanner_ = 0; accPlanner_ = 0; isInitialized_ = false;
    }

private:
    T stiffness_ = 15790.0, velDesired_ = 0.4, accMax_ = 5000.0;
    T dampingRatio_ = 0.0005, frequency_ = 20.0;   // frequency_: stored, not read (FIC.cpp)
    T maxFrequency_ = 8.75, speedMargin_ = 1.5708;
    T speedLimit_ = 0;   // set-point speed cap, 0: none (FIC.cpp Planner::setSpeedLimit)
    T posLower_ = -M_PI, posUpper_ = M_PI;
    T wrapped_ = 0;
    bool isInitialized_ = false;
    T lastPosError_ = 0, lastDivergencePosError_ = 0, lastPosDesired_ = 0;
    T posPlanner_ = 0, velPlanner_ = 0, accPlanner_ = 0;
    T stateVelMax_ = 0, stateAccMax_ = 0;
    // Uninitialised in FIC.cpp until the first target change; zero here.
    T fd_ = 0, omega_n_ = 0, gainDamping_ = 0;

    static int Sign(T v) { return (v >= 0) - (v < 0); }
    // FIC.cpp Planner::clamp is min(max(x, lo), hi) — differs from std::clamp
    // only when lo > hi; keep it exact.
    static T clampMinMax(T x, T lo, T hi) { return std::fmin(std::fmax(x, lo), hi); }

    void initializeState(T error) {
        isInitialized_ = true;
        lastPosError_ = 0;
        lastDivergencePosError_ = error;
        lastPosDesired_ = posPlanner_;
        stateVelMax_ = 0;
        stateAccMax_ = 0;
    }

    // distanceToGoal: posDesired - posPlanner_, wrapped for a continuous joint (plan()).
    void updateVelocityAndAccelerationLimits(T distanceToGoal, T dt) {
        T dist = std::fabs(distanceToGoal);
        dist = std::fmax(T(0.0001), dist);
        if (dt <= T(1e-4)) fd_ = std::fmin(std::fabs(velDesired_ / dist), T(50.0));
        else            fd_ = std::fmin(std::fabs(velDesired_ / dist), maxFrequency_);
        omega_n_ = fd_ * T(2.0) * T(M_PI);
        gainDamping_ = dampingRatio_ * T(2.0) * omega_n_;
        stiffness_ = omega_n_ * omega_n_;
        T vel_n = std::fmin(velDesired_, omega_n_ * dist);
        stateVelMax_ = speedMargin_ * vel_n;
        T a_max = stateVelMax_ * vel_n / dist;
        T An = omega_n_ * omega_n_ * dist;
        stateAccMax_ = std::fmin(std::fmin(a_max, An), accMax_);   // acc_max on top (FIC.cpp)
    }

    T profileLinear(T error, T effortMax) const {
        return clampMinMax(stiffness_ * error, -effortMax, effortMax);
    }

    T calculateAcceleration(T error) {
        if (std::fabs(error) > std::fabs(lastPosError_)) {
            accPlanner_ = profileLinear(error, stateAccMax_);
            lastDivergencePosError_ = error;
        } else {
            if (std::fabs(lastDivergencePosError_) < std::fabs(error) ||
                Sign(lastDivergencePosError_) != Sign(error) ||
                std::fabs(error - lastPosError_) < T(1e-5)) {
                lastDivergencePosError_ = error;
            }
            T tmpAcc = profileLinear(lastDivergencePosError_, stateAccMax_);
            T tmpPosMid = lastDivergencePosError_ / T(2.0);
            T tmpGainOut = std::fabs(tmpAcc) / std::fmax(std::fabs(tmpPosMid), T(1e-5));
            accPlanner_ = tmpGainOut * (error - tmpPosMid);
        }
        return accPlanner_;
    }
};

//=============================================================================
// FIC admittance (FIC<T>::AdmittanceController)
//=============================================================================
template <typename T>
class Admittance {
public:
    void setDesiredEffort(T e) { desired_effort_ = e; }
    void setCurrentEffort(T e) { current_effort_ = e; }
    void resetState() { pose_ = 0; velocity_ = 0; }
    void setMinMax_dFlow(T lo, T hi) { dflow_[0] = lo; dflow_[1] = hi; }
    void setMinMax_Flow(T lo, T hi)  { flow_[0] = lo;  flow_[1] = hi; }
    void setMinMax_Pose(T lo, T hi)  { pose_lim_[0] = lo; pose_lim_[1] = hi; }
    bool setInvM_Kv(T invM, T Kv) {
        if (invM < 0) return false;
        invM_ = invM;
        if (Kv < 0) return false;
        Kv_ = Kv;
        return true;
    }

    T computeDeltaPose(T dt) {
        T effective_effort = desired_effort_ - current_effort_;
        T acceleration = invM_ * (effective_effort - Kv_ * velocity_);
        acceleration = clampv(acceleration, dflow_[0], dflow_[1]);
        velocity_ += acceleration * dt;
        velocity_ = clampv(velocity_, flow_[0], flow_[1]);
        pose_ += velocity_ * dt;
        pose_ = clampv(pose_, pose_lim_[0], pose_lim_[1]);
        if (pose_ >= pose_lim_[1] && velocity_ > T(0)) velocity_ = T(0);
        else if (pose_ <= pose_lim_[0] && velocity_ < T(0)) velocity_ = T(0);
        return pose_;
    }

private:
    T desired_effort_ = 0, current_effort_ = 0;
    T dflow_[2] = {T(-10.0), T(10.0)};
    T flow_[2] = {T(-5.0), T(5.0)};
    T pose_lim_[2] = {T(-0.05), T(0.05)};
    T invM_ = 1.0, Kv_ = 10.0;
    T pose_ = 0, velocity_ = 0;
};

//=============================================================================
// Second-order Butterworth low-pass — butterworth_filter.hpp without the
// console print, so it can run on the MCU. Same coefficients, same DF-II-T.
//=============================================================================
template <typename T = double>
class Butterworth2 {
public:
    Butterworth2() = default;
    // Coefficients are computed in double (once), stored as T.
    Butterworth2(double cutoff_hz, double sample_hz, double initial = 0.0) {
        double omega = std::tan(M_PI * cutoff_hz / sample_hz);
        double omega2 = omega * omega;
        double sqrt2 = std::sqrt(2.0);
        double norm = 1.0 / (1.0 + sqrt2 * omega + omega2);
        a0 = T(omega2 * norm);
        a1 = T(2.0 * (omega2 * norm));
        a2 = a0;
        b1 = T(2.0 * (omega2 - 1.0) * norm);
        b2 = T((1.0 - sqrt2 * omega + omega2) * norm);
        x1 = x2 = T(initial);
        y1 = y2 = T(initial);
    }
    T filter(T x0) {
        T y0 = a0 * x0 + a1 * x1 + a2 * x2 - b1 * y1 - b2 * y2;
        x2 = x1; x1 = x0;
        y2 = y1; y1 = y0;
        return y0;
    }

private:
    T a0 = 1, a1 = 0, a2 = 0, b1 = 0, b2 = 0;
    T x1 = 0, x2 = 0, y1 = 0, y2 = 0;
};

//=============================================================================
// Per-joint diagnostics — the 8 trajectory_data blocks fic_control.h fills.
//=============================================================================
template <typename T>
struct JointDiagT {
    T planner_target = 0;  // block 0
    T q_current = 0;       // block 1
    T error = 0;           // block 2
    T q_target = 0;        // block 3
    T effort = 0;          // block 4
    T q_target_adm = 0;    // block 5
    T desired_effort = 0;  // block 6
    T current_effort = 0;  // block 7
};
using JointDiag = JointDiagT<double>;

//=============================================================================
// Cholesky solve of a symmetric positive-definite 6x6 system (row-major A), as H-PMP's
// (J J^T + lambda^2 I) w = J u. No Eigen on the MCU. False if A is not positive definite.
//=============================================================================
template <typename T>
inline bool solveSpd6(const T A[36], const T b[6], T x[6]) {
    T L[36] = {};
    for (int r = 0; r < 6; ++r) {
        for (int c = 0; c <= r; ++c) {
            T s = A[r * 6 + c];
            for (int k = 0; k < c; ++k) s -= L[r * 6 + k] * L[c * 6 + k];
            if (r == c) {
                if (!(s > T(0))) return false;
                L[r * 6 + r] = std::sqrt(s);
            } else {
                L[r * 6 + c] = s / L[c * 6 + c];
            }
        }
    }
    T y[6];
    for (int r = 0; r < 6; ++r) {
        T s = b[r];
        for (int k = 0; k < r; ++k) s -= L[r * 6 + k] * y[k];
        y[r] = s / L[r * 6 + r];
    }
    for (int r = 5; r >= 0; --r) {
        T s = y[r];
        for (int k = r + 1; k < 6; ++k) s -= L[k * 6 + r] * x[k];
        x[r] = s / L[r * 6 + r];
    }
    return true;
}

//=============================================================================
// Joint-space FIC for up to kMaxCoreJoints joints.
//=============================================================================
template <typename T>
class JointCore {
public:
    T dt = T(0.001);
    T deadband_joint_tau = T(1.5);
    // H-PMP's split drive (fic_control.h cartesianHpmp): task deadbands on the end-effector wrench part
    // of the torque gap, engaged at the value and released below kSplitRelease of it.
    T task_db_force = T(1.0);    // N
    T task_db_torque = T(0.1);   // N m
    static constexpr T kSplitRelease = T(0.25);
    static constexpr T kSplitLambda = T(0.01);
    // The robot's FIC law options (robots_config.yaml fic_law), applied by initJoint.
    bool restart_on_stall = true;
    T planner_max_hz = T(8.75);
    T planner_speed_margin = T(1.5708);
    // The robot's max joint speed, the joint planners' speed cap (rad/s; 0: none). Applied by
    // initJoint and, while running, by setJointSpeedLimit.
    T joint_speed_limit = T(0);
    void setJointSpeedLimit(T v) {
        joint_speed_limit = v;
        for (int i = 0; i < kMaxCoreJoints; ++i) plan_[i].setSpeedLimit(v);
    }

    void setJointLimits(int i, T lower, T upper) { lower_[i] = lower; upper_[i] = upper; }
    void setJointContinuous(int i, bool c) { continuous_[i] = c; }

    // initJointController + initJointPlanner + initJointAdmittance, in the order
    // FicControlNode::initJointControllers calls them. Returns false where the
    // original would have thrown.
    bool initJoint(int i, const JointParamsPOD& p, T q_current) {
        bool ok = true;
        SatController<T>& c = ctrl_[i];
        int n_s = p.n_saturations;
        int n_neg = p.n_neg_sat;
        if (!p.asymmetric && p.n_error_max > 0) {
            n_s = p.n_error_max * 2;
            n_neg = p.n_error_max;
        }
        if (n_s > kMaxSat) return false;
        c.reconfigure(n_s, p.asymmetric != 0, n_neg);
        ok &= c.set_x_b(p.error_max, p.n_error_max);
        ok &= c.setE_max(p.force_max, p.n_force_max);
        if (p.n_saturation_ratios > 0) {
            ok &= c.setS_Start(p.saturation_ratios, p.n_saturation_ratios);
        } else {
            const double d = 0.9;
            ok &= c.setS_Start(&d, 1);
        }
        ok &= c.setStiffness(p.stiffness);
        c.setRestartOnStall(restart_on_stall);

        Planner<T>& pl = plan_[i];
        pl.resetState(q_current);
        pl.setStiffness(p.planner_stiffness);
        pl.setVelDesired(p.vel_desired);
        pl.setAccMax(p.acc_max);
        pl.setDampingRatio(p.damping_ratio);
        pl.setFrequency(p.frequency);
        pl.setShape(planner_max_hz, planner_speed_margin);
        pl.setSpeedLimit(joint_speed_limit);
        pl.setWrapped(continuous_[i] ? 1 : 0);
        pl.setPosLimits(lower_[i], upper_[i]);

        Admittance<T>& a = adm_[i];
        ok &= a.setInvM_Kv(p.inv_m, p.adm_damping);
        a.setMinMax_dFlow(p.dflow[0], p.dflow[1]);
        a.setMinMax_Flow(p.flow[0], p.flow[1]);
        // As fic_control.h: pose limits are the controller's outer breakpoints.
        a.setMinMax_Pose(c.get_x_b()[0], c.get_x_b()[c.numSaturations() - 1]);
        a.resetState();
        engaged_[i] = false;
        return ok;
    }

    // Mode 1 — handleJointImpedanceControl
    T impedance(int i, T q_target, T q_current, JointDiagT<T>& d) {
        if (continuous_[i]) q_current = wrapToPi(q_current);
        else                q_target = clampv(q_target, lower_[i], upper_[i]);
        T error = q_target - q_current;
        if (continuous_[i]) error = wrapToPi(error);
        T effort = ctrl_[i].control(error);
        d.q_current = q_current;
        d.error = error;
        d.q_target = q_target;
        d.effort = effort;
        return effort;
    }

    // Mode 2 — handleJointImpedanceControlWithPlanner
    T impedanceWithPlanner(int i, T q_target, T q_current, JointDiagT<T>& d) {
        T planned = plan_[i].plan(q_target, dt);
        if (!continuous_[i]) planned = clampv(planned, lower_[i], upper_[i]);
        T effort = impedance(i, planned, q_current, d);
        d.planner_target = planned;
        return effort;
    }

    // Modes 3/4 and hapfic — handleJointAdmitanceControl.
    // adm_mode 1 = push-and-stay, 2 = haptic spring-return.
    // release_tau > 0: deadband with hysteresis (HAP-FIC), as fic_control.h.
    // drive (not NaN): the interaction torque already deadbanded by splitDrive (H-PMP).
    T admittance(int i, T& q_target, T q_current, T desired_effort,
                      T current_effort, int adm_mode, bool use_planner, JointDiagT<T>& d,
                      T release_tau = T(0), T drive = std::numeric_limits<T>::quiet_NaN()) {
        Admittance<T>& a = adm_[i];
        T ext_tau = desired_effort - current_effort;
        if (drive == drive) {                    // a drive given (not NaN): already deadbanded, split drive
            ext_tau = drive;
        } else if (release_tau > T(0)) {
            if (!engaged_[i] && std::fabs(ext_tau) >= deadband_joint_tau) engaged_[i] = true;
            else if (engaged_[i] && std::fabs(ext_tau) < release_tau) engaged_[i] = false;
            if (!engaged_[i]) ext_tau = T(0);
        } else if (std::fabs(ext_tau) < deadband_joint_tau) {
            ext_tau = T(0);
        }
        // No restart on a stall while the admittance moves the target (fic_control.h NoRestart).
        ctrl_[i].setRestartOnStall(false);
        a.setDesiredEffort(T(0));   // driven by the interaction torque alone (see fic_control.h)
        a.setCurrentEffort(-ext_tau);
        T adm_delta = a.computeDeltaPose(dt);

        T step_size = adm_delta - last_delta_[i];
        last_delta_[i] = adm_delta;

        T q_target_adm;
        T stable = wrapToPi(q_target);
        if (adm_mode == 1) {
            // Push-and-stay: the admittance integrates a change of angle, so it is bounded by the
            // joint's span, not its limits (a range leaving out zero made it jump at the start).
            // As fic_control.h (fixed there and here on 18 Oct 2026).
            const T span = upper_[i] - lower_[i];
            a.setMinMax_Pose(-span, span);
            offset_[i] += step_size;
            q_target_adm = stable + offset_[i];
        } else {
            offset_[i] = T(0);
            q_target_adm = stable + adm_delta;
        }

        if (continuous_[i]) {
            q_target_adm = wrapToPi(q_target_adm);
        } else {
            q_target_adm = clampv(q_target_adm, lower_[i], upper_[i]);
            if (q_target_adm == upper_[i] || q_target_adm == lower_[i]) {
                if (adm_mode == 1) offset_[i] -= step_size;
            }
        }

        T effort = use_planner ? impedanceWithPlanner(i, q_target_adm, q_current, d)
                                    : impedance(i, q_target_adm, q_current, d);
        d.q_target_adm = q_target_adm;
        d.desired_effort = desired_effort;
        d.current_effort = current_effort;
        ctrl_[i].setRestartOnStall(restart_on_stall);
        return effort;
    }

    T humanOffset(int i) const { return offset_[i]; }

    // H-PMP's split drive (fic_control.h cartesianHpmp): the torque gap u (per joint) split into its
    // end-effector wrench part, w = (J J^T + lambda^2 I)^-1 J u through the task deadband, and the rest,
    // u - J^T w through each joint's deadband (its own hysteresis, engaged_), recombined into the drive
    // that admittance() takes. jac: 6 x n, row-major (FIC-link's layout).
    void splitDrive(const float* jac, const T* u, int n, T* drive) {
        T A[36], b[6], w[6];
        for (int r = 0; r < 6; ++r) {
            T s = T(0);
            for (int c = 0; c < n; ++c) s += T(jac[r * n + c]) * u[c];
            b[r] = s;
            for (int q = 0; q < 6; ++q) {
                T a = (r == q) ? kSplitLambda * kSplitLambda : T(0);
                for (int c = 0; c < n; ++c) a += T(jac[r * n + c]) * T(jac[q * n + c]);
                A[r * 6 + q] = a;
            }
        }
        if (!solveSpd6(A, b, w)) for (int k = 0; k < 6; ++k) w[k] = T(0);
        const T fn = std::sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);
        const T tn = std::sqrt(w[3] * w[3] + w[4] * w[4] + w[5] * w[5]);
        if (!task_on_f_ && fn >= task_db_force) task_on_f_ = true;
        else if (task_on_f_ && fn < kSplitRelease * task_db_force) task_on_f_ = false;
        if (!task_on_t_ && tn >= task_db_torque) task_on_t_ = true;
        else if (task_on_t_ && tn < kSplitRelease * task_db_torque) task_on_t_ = false;
        for (int c = 0; c < n; ++c) {
            T jw = T(0), jwd = T(0);
            for (int r = 0; r < 6; ++r) {
                const T jrc = T(jac[r * n + c]);
                jw += jrc * w[r];
                if ((r < 3 && task_on_f_) || (r >= 3 && task_on_t_)) jwd += jrc * w[r];
            }
            const T un = u[c] - jw;
            bool& on = engaged_[c];
            if (!on && std::fabs(un) >= deadband_joint_tau) on = true;
            else if (on && std::fabs(un) < kSplitRelease * deadband_joint_tau) on = false;
            drive[c] = jwd + (on ? un : T(0));
        }
    }
    void resetSplit() { task_on_f_ = task_on_t_ = false; }

private:
    SatController<T> ctrl_[kMaxCoreJoints];
    Planner<T> plan_[kMaxCoreJoints];
    Admittance<T> adm_[kMaxCoreJoints];
    T lower_[kMaxCoreJoints] = {};
    T upper_[kMaxCoreJoints] = {};
    bool continuous_[kMaxCoreJoints] = {};
    T offset_[kMaxCoreJoints] = {};
    T last_delta_[kMaxCoreJoints] = {};
    bool engaged_[kMaxCoreJoints] = {};
    bool task_on_f_ = false, task_on_t_ = false;   // split drive: task deadband engaged (force, moment)

public:
    JointCore() { reset(); }

    // Back to the freshly-constructed state, in place (no large temporary —
    // this object is ~12 KB and the MCU stack is small).
    void reset() {
        for (int i = 0; i < kMaxCoreJoints; ++i) {
            ctrl_[i] = SatController<T>();
            plan_[i] = Planner<T>();
            adm_[i] = Admittance<T>();
            lower_[i] = -M_PI; upper_[i] = M_PI;
            continuous_[i] = false;
            offset_[i] = 0; last_delta_[i] = 0; engaged_[i] = false;
        }
        task_on_f_ = task_on_t_ = false;
    }
};

} // namespace core
} // namespace fic

#endif // FIC_JOINT_CORE_H
