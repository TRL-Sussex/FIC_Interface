#pragma once
#include <cmath>
#include <algorithm>
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry> // Quaternion & AngleAxis

//==============================//
// FIC Container Declaration
// T                - numeric type (float/double)
// n_dof            - number of degrees of freedom
// enable_admittance- include AdmittanceController per DOF
//==============================//

template <typename T>
class FIC {public:
    // Default: symmetric, N_s = 2, single-DOF, admittance disabled
    FIC()
      : N_s_(2)
      , N_negSat_(N_s_/2)
      , asymmetric_(false)
      , n_dof_(1)
      , enable_admittance_(false)
    {
        init_controllers();
    }

    // Fully configurable:
    //   N_s              = total number of saturation points
    //   asymmetric       = use asymmetric layout?
    //   N_negSat         = half-size in asymmetric mode
    //   n_dof            = how many DOFs/controllers to build
    //   enable_admittance= whether to build admittance controllers
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

    // Public API: compute FIC control effort for a given DOF
    T control(size_t idx_dof, T error) {
        return controllers_[idx_dof].control(idx_dof, error);
    }

    // Public API: generate a planned position for a given DOF
    T plan(size_t idx_dof, T posDesired, T dt) {
        return planners_[idx_dof].plan(posDesired, dt);
    }

    // Public API: compute compliant motion via admittance (runtime‐checked)
    T admittance(size_t idx_dof, T dt) {
        if (!enable_admittance_) {
            throw std::logic_error("Admittance is disabled in this FIC instance");
        }
        return admittance_controllers_[idx_dof].computeDeltaPose(dt);
    }

private:
    int    N_s_;               // total saturation points
    int    N_negSat_;          // “half” size in symmetric/asymmetric logic
    bool   asymmetric_;        // flag for asymmetric layout
    size_t n_dof_;             // number of DOFs/controllers
    bool   enable_admittance_; // runtime flag


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
    //==============================//
    // FIC::Controller nested class
    //==============================//
    template <typename TC = T>
    class Controller {

        // Default: 2 saturations per DOF
        T N_S = 2;

        // Number of negative saturations (default 1)
        T N_NegSat = 1;

        // Number of positive saturations computed from N_S - N_NegSat
        T N_PSat = N_S - N_NegSat;

        // Percentage of the saturation position where saturation starts (default 90%)
        // Start saturation ratio: per DOF, per saturation segment
        // Default: all set to 0.9 (must be configured)
        Eigen::VectorXd S_Start ;

        // Saturation forces: per DOF, per saturation segment
        // Default: all zero (must be configured)
        Eigen::VectorXd E_s ;
        // Saturation positions: per DOF, per saturation segment
        // Default: all zero (must be configured)
        Eigen::VectorXd x_b ;// vector of size of n_S

        // Linear stiffness values before saturation for negative and positive errors
        // K0(i,0): stiffness for negative error
        // K0(i,1): stiffness for positive error
        Eigen::VectorXd K0 ; //vector


        // Time step used in control computations
        //static constexpr T dt = 1e-3;
        //---------------- Private member variables ----------------//

        // Flag indicating whether the controller has been initialized
        bool isInitialised = false;

        // Previous position error
        TC error_old{};

        // Previous computed effort
        TC effort_old{};

        // Maximum error reached during last divergence phase
        TC error_max{};

        // Effort corresponding to error_max
        TC effort_max{};

        // Number of saturation regions for this DOF (can be set)
        int n_ = 0;

        //---------------- Private helper methods ----------------//

        // Computes the nonlinear force profile given the error and the DOF index
        TC getEffort(TC error) {
            // Most Negative Error Case
            if (error < x_b.coeff(0)) {
            return E_s.coeff(0);
            }
            // Most Positive Error Case
            else if (error > x_b.coeff(static_cast<int>(N_S) - 1)) {
            return E_s.coeff(static_cast<int>(N_S) - 1);
            }
            // Negative Error Linear Range
            else if (error < 0 && error > S_Start.coeff(N_NegSat - 1) * x_b.coeff(N_NegSat - 1)) {
            return K0(0) * error;
            }
            // Positive Error Linear Range
            else if (error > 0 && error < S_Start.coeff(N_NegSat) * x_b.coeff(N_NegSat)) {
            return K0(1) * error;
            }
            // Saturation bands. Band k blends from the force before it to E_s(k)
            // with a tanh that starts at the band's start point and ends at its
            // breakpoint x_b(k); from there to the next band's start the effort
            // holds E_s(k). So band k covers [start(k), start(next band)], and the
            // outermost band runs to x_b at the ends (beyond, the outermost force).
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
            // Fallback if no condition matched
            return T(0);
        }

        // Computes effort smoothly between two breakpoints using a tanh interpolation
        T getEffortValue(T S, T F_sat, T F_0, T xb, T xb0, T x) {
            T deltaF = F_sat - F_0;
            T tanhArg = ((2 * M_PI * (x - (S * (xb - xb0) + xb0))) / ((1 - S) * (xb - xb0))) - M_PI;
            return deltaF / 2.0 * (1.0 + std::tanh(tanhArg)) + F_0;
        }

        // Returns the sign of a value (-1, 0, or 1)
        int Sign(T value) { return (value > 0) - (value < 0); }

    public:
        //---------------- Public methods ----------------//

        // Default constructor (does not require arguments)
        Controller() {
                S_Start = Eigen::VectorXd::Constant(N_S, 0.9);
                E_s = Eigen::VectorXd::Zero(N_S);
                x_b = Eigen::VectorXd::Zero(N_S);// vector of size of n_S
                K0 = Eigen::VectorXd::Zero(2); //vector
        }
        Controller(T _N_s) 
            : N_S(_N_s){
            // Initialize the number of saturations
            N_NegSat = N_S/2;
            N_PSat = N_S - N_NegSat; // Compute positive saturation

            S_Start = Eigen::VectorXd::Constant(N_S, 0.9);
            E_s = Eigen::VectorXd::Zero(N_S);
            x_b = Eigen::VectorXd::Zero(N_S);// vector of size of n_S
            K0 = Eigen::VectorXd::Zero(2); //vector

        }

        Controller(T _N_s, T _N_negSat) //asymmetric saturation
            : N_S(_N_s), N_NegSat(_N_negSat){
            // Initialize the number of saturations
            N_PSat = N_S - N_NegSat; // Compute positive saturation

            S_Start = Eigen::VectorXd::Constant(N_S, 0.9);
            E_s = Eigen::VectorXd::Zero(N_S);
            x_b = Eigen::VectorXd::Zero(N_S);// vector of size of n_S
            K0 = Eigen::VectorXd::Zero(2); //vector
        }





        // Main control method
        // Computes the effort based on current error and internal state
        T control(size_t idx_dof, T error) {
            // Initialize if this is the first call
            if (!isInitialised) {
                isInitialised = true;
                error_old = 0;
                error_max = error;
                effort_max = 0;
                effort_old = 0;
            }

            T effort = 0;

            // Divergence phase: error growing
            if (std::fabs(error) > std::fabs(error_old)) {

                effort = getEffort(error);  

                error_max = error;
                effort_max = effort;
            }

            // Convergence phase: error shrinking
            else {
                // Update maximum error if error crosses zero or grows again.
                // Reset effort_max with it (to the divergence-curve value at the
                // new error): keeping the old peak effort would give
                // K_conv = |effort_max| / |error / 2| -> the full previous peak
                // effort at near-zero error, then a step down to K0*e on the
                // next divergence tick.
                if (std::fabs(error_max) < std::fabs(error) || Sign(error_max) != Sign(error)) {
                    error_max = error;
                    effort_max = getEffort(error);
                }
                // Compute convergence stiffness dynamically
                T error_mid = error_max / 2.0;
                T K_conv = std::fabs(effort_max) / std::max(std::fabs(error_mid), T(1e-5));
                effort = K_conv * (error - error_mid);
            }

            // Store state for next step
            error_old = error;
            effort_old = effort;

            return effort;
        }

        //---------------- Getters and setters ----------------//

        // Get saturation positions vector
        Eigen::VectorXd get_x_b() const { return x_b; }

        void set_x_b(const Eigen::VectorXd& x_b_in) {


            if (x_b_in.size() == 0)
                throw std::logic_error("Error max vector cannot be empty");

            // 1) Resize target storage so indexing is valid
            x_b.resize(N_S);

            // 2) Symmetric case?
            if (N_NegSat * 2 == N_S) {
                if ((size_t)N_NegSat != x_b_in.size())
                    throw std::logic_error("Symmetric: x_b_in size must be N_NegSat");

                // fill negative half (mirrored and negated)
                for (int i = 0; i < N_NegSat; ++i) {
                    // index N_NegSat-1-i is in [0..N_NegSat-1]
                    x_b(i) = -x_b_in.coeff(N_NegSat - 1 - i);
                }
                // fill positive half
                for (int i = N_NegSat; i < N_S; ++i) {
                    // i-N_NegSat is in [0..N_NegSat-1]
                    x_b(i) =  x_b_in.coeff(i - N_NegSat);
                }
            }
            else {
                // Asymmetric: x_b_in must be full length
                if ((size_t)N_S != x_b_in.size())
                    throw std::logic_error("Asymmetric: x_b_in size must be N_S");
                x_b = x_b_in;  // Eigen takes care of sizing
            }
        }


        // Get saturation effort vector
        Eigen::VectorXd getE_max() const { return E_s; }

        // Set saturation effort vector
        void setE_max(const Eigen::VectorXd& E_s_in) {

            if (E_s_in.size() == 0)
                throw std::logic_error("Effort max vector cannot be empty");

            // Make sure your storage vector is sized
            E_s.resize(N_S);

            // Check if symmetric or asymmetric
            if (N_NegSat * 2 == N_S) { // Symmetric
                if (E_s_in.size() != N_NegSat)
                    throw std::logic_error("Symmetric: E_s_in size must be N_NegSat");
                for (int i = 0; i < N_NegSat; ++i)
                    E_s(i) = -E_s_in.coeff(N_NegSat - 1 - i);
                for (int i = N_NegSat; i < N_S; ++i)
                    E_s(i) =  E_s_in.coeff(i - N_NegSat);
            } else { // Asymmetric
                if (E_s_in.size() != N_S)
                    throw std::logic_error("Asymmetric: E_s_in size must be N_S");
                E_s = E_s_in;
            }
        }



        // Get start saturation ratio vector
        Eigen::VectorXd getS_Start() const { return S_Start; }

        // Set start saturation ratio vector
        void setS_Start(const Eigen::VectorXd& S_Start_in) {
            // std::cout << "[DEBUG] setS_Start called. Input size: " 
            //       << S_Start_in.size() << "\n";
            if (S_Start_in.size() == 0)
            throw std::logic_error("Saturation ratios vector cannot be empty");
            if (S_Start_in.size() == 1) {
                S_Start = Eigen::VectorXd::Constant(N_S, std::clamp(S_Start_in.coeff(0), T(0.4), T(0.9)));
            } else {
                S_Start = S_Start_in;
                for (int i = 0; i < S_Start.size(); ++i) {
                    S_Start(i) = std::clamp(S_Start(i), T(0.4), T(0.9));
                }
            }

        }

        // Get linear stiffness vector
        Eigen::VectorXd getStiffness() const { return K0; }

        // Set linear stiffness vector
        void setStiffness(const Eigen::VectorXd& K0_in) {
            // std::cout << "[DEBUG] setStiffness called. Input size: " 
            //       << K0_in.size() << "\n";
            if (K0_in.minCoeff() <= 0.0)
            throw std::logic_error("Invalid stiffness: must be positive");
            K0 = K0_in;
        }
        
        // Get the number of saturations
        int getNumSaturations() const { return static_cast<int>(N_S); }

        // Set the number of saturations
        void setNumSaturations(int n) {
            if (n < 0)
            throw std::logic_error("Number of saturations must be non-negative");
            n_ = n;
        }

        void reconfigure(int new_N_S, bool asymmetric, int new_N_NegSat) {
            N_S = new_N_S;
            N_NegSat = asymmetric ? new_N_NegSat : (new_N_S / 2);
            N_PSat = N_S - N_NegSat;
            S_Start = Eigen::VectorXd::Constant(N_S, 0.9);
            E_s = Eigen::VectorXd::Zero(N_S);
            x_b = Eigen::VectorXd::Zero(N_S);
        }
    };

    //==============================//
    // FIC Planner Class
    // This nested template class implements a trajectory generator / motion planner.
    // It plans smooth position trajectories given desired positions over time,
    // considering velocity, acceleration, stiffness, and damping.
    //==============================//
    template <typename TP = T>
    class Planner {
        //---------------- Private member variables ----------------//

        // Minimum and maximum position limits per DOF
        //Eigen::Matrix<T, n_dof, 2> dof_Limits = Eigen::Matrix<T, n_dof, 2>::Zero(); //pull from urdf 

        // Linear stiffness coefficient (force per unit error)
        T stiffness_;

        // Desired velocity (max nominal velocity)
        T velDesired_;

        // Maximum allowed acceleration
        T accMax_;

        // Damping ratio (affects convergence behavior)
        T dampingRatio_;

        // Natural frequency of the planner's dynamics (Hz)
        T frequency_;
        T posLower_;
        T posUpper_;
        // Wrapping mode:
        // 0 = no wrap
        // 1 = wrap to [-pi, pi]
        // 2 = wrap to [0, 2pi]
        T wrapped_;

        // Flag indicating whether the planner has been initialized
        bool isInitialized_;

        // Last position error
        T lastPosError_;

        // Last divergence error (largest error since last convergence phase)
        T lastDivergencePosError_;

        // Last desired position used for planning
        T lastPosDesired_;

        // Current planned position
        T posPlanner_;

        // Current planned velocity
        T velPlanner_;

        // Current planned acceleration
        T accPlanner_;

        // Dynamically computed velocity limit (based on distance to goal)
        T stateVelMax_;

        // Dynamically computed acceleration limit (based on distance to goal)
        T stateAccMax_;
        T fd_;
        T omega_n_;
        T gainDamping_;

        //---------------- Private helper methods ----------------//

        // Returns the sign of a value (-1, 0, or +1)
        int Sign(T v) { return (v >= 0) - (v < 0); }

        T clamp(T x, T minX, T maxX) {
            return std::min(std::max(x, minX), maxX);
        }

        // Initializes the planner state the first time plan() is called
        void initializeState(T posDesired, T error) {
            isInitialized_ = true;
            lastPosError_ = 0;
            lastDivergencePosError_ = error;
            lastPosDesired_ = posPlanner_;
            stateVelMax_ = 0;
            stateAccMax_ = 0;
        }

        // distanceToGoal: posDesired - posPlanner_, wrapped for a continuous joint (plan()), so a goal
        // just across +-pi is a short move, not a 2*pi one.
        void updateVelocityAndAccelerationLimits(T distanceToGoal, T dt) {
            T dist = std::fabs(distanceToGoal);
            dist = std::max(T(0.0001), dist);

            if (dt <= 1e-4)
                fd_ = std::min(std::fabs(velDesired_ / dist), T(50.0));
            else
                fd_ = std::min(std::fabs(velDesired_ / dist), T(8.75));

            omega_n_ = fd_ * T(2.0) * T(M_PI);
            gainDamping_ = dampingRatio_ * T(2.0) * omega_n_;
            stiffness_ = omega_n_ * omega_n_;

            T vel_n = std::min(velDesired_, omega_n_ * dist);
            stateVelMax_ = T(1.5708) * vel_n;

            T a_max = stateVelMax_ * vel_n / dist;
            T An = omega_n_ * omega_n_ * dist;

            stateAccMax_ = std::min(a_max, An);
        }


        T profileLinear(T error, T effortMax) {
            T effort = stiffness_ * error;
            return clamp(effort, -effortMax, effortMax);
        }
        // Computes acceleration for the current error, including divergence/convergence behavior
        T calculateAcceleration(T error) {
            if (std::fabs(error) > std::fabs(lastPosError_)) {
                accPlanner_ = profileLinear(error, stateAccMax_);
                lastDivergencePosError_ = error;
            } else {
                if (std::fabs(lastDivergencePosError_) < std::fabs(error) ||
                    Sign(lastDivergencePosError_) != Sign(error)|| std::fabs(error-lastPosError_)<1e-5) { 
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
        //---------------- Public methods ----------------//

        // Default constructor initializes all parameters to sensible defaults
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

        // Main planning function for a single DOF
        // Updates position planner state over time
        T plan(T posDesired, T dt) {
            T error = posDesired - posPlanner_;
            if (wrapped_ != 0 && fabs(error) > M_PI){
                error = -Sign(error)*(2*M_PI - std::fabs(error));
            }

            if (dt<=0.005){
                
                if (!isInitialized_) {
                    initializeState(posDesired, error);
                }

                // How far the goal moved since the last recomputation, wrapped for a continuous
                // joint like the error above: unwrapped, a goal crossing +-pi looked like a 2*pi
                // move and the limits were recomputed for it (a 0.5 Hz planner that coasted
                // away from a goal just across the seam).
                T moved = posDesired - lastPosDesired_;
                if (wrapped_ != 0 && std::fabs(moved) > M_PI) {
                    moved = -Sign(moved) * (2 * M_PI - std::fabs(moved));
                }
                if (std::fabs(moved) > T(1e-3)) {
                    updateVelocityAndAccelerationLimits(error, dt);             
                    lastPosDesired_ = posDesired;  // reference for the next >1 mm check: small streamed steps accumulate
                }

                accPlanner_ = calculateAcceleration(error)-gainDamping_*velPlanner_;
                velPlanner_ = clamp(velPlanner_ + dt * accPlanner_, -stateVelMax_, stateVelMax_);
                posPlanner_ += dt * velPlanner_; 

            } 

            else {
                posPlanner_ = posDesired;// clampped tp the position limits of the joint is NOT wrapped 
            } 
            if (wrapped_ == 0) {
                posPlanner_ = std::clamp(posPlanner_, posLower_, posUpper_);// clampped tp the position limits of the joint is NOT  wrapped 
            }
            // Post-position wrapping
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

        //---------------- Getters and setters ----------------//

        T getStiffness() const { return stiffness_; }
        void setStiffness(T s) { stiffness_ = std::max(s, T(0.01)); }

        T getVelDesired() const { return velDesired_; }
        void setVelDesired(T v) { velDesired_ = std::max(v, T(0.01)); }

        T getAccMax() const { return accMax_; }
        void setAccMax(T a) { accMax_ = std::max(a, T(0.01)); }

        T getDampingRatio() const { return dampingRatio_; }
        void setDampingRatio(T d) { dampingRatio_ = std::max(d, T(0.0001)); }

        T getFrequency() const { return frequency_; }
        void setFrequency(T f) { frequency_ = std::max(f, T(0.0001)); }

        T getWrapped() const { return wrapped_; }
        void setWrapped(T w) { wrapped_ = w; }
                // Resets the planner state for scalar positions

        T getPosLower() const { return posLower_; }
        T getPosUpper() const { return posUpper_; } // defult limit of pi, -pi for each joint maybe

        void setPosLimits(T lower, T upper) {
            // sanity check: ensure lower <= upper
            if (lower > upper) {
                // swap if user passes reversed
                std::swap(lower, upper);
            }

            // store (allow -inf / +inf if you want unlimited)
            posLower_ = lower;
            posUpper_ = upper;
        }

        void resetState(T pos) {
            posPlanner_ = pos;
            velPlanner_ = 0;
            accPlanner_ = 0;
            isInitialized_ = false;
        }

    };

    //==============================//
    // FIC AdmittanceController Class
    // This nested template class implements an admittance controller:
    // It integrates applied force over time to compute a compliant motion.
    //==============================//
    template <typename TA = T>
    class AdmittanceController {
        //---------------- Private member variables ----------------//

        // Desired effort (force or torque) setpoint
        T desired_effort_;

        // Current measured effort (force or torque)
        T current_effort_;

        // Min/max allowed acceleration (delta flow) [min, max]
        Eigen::Vector2d minMax_dFlow_;

        // Min/max allowed velocity (flow) [min, max]
        Eigen::Vector2d minMax_Flow_;

        // Min/max allowed pose (position) [min, max]
        Eigen::Vector2d minMax_Pose_;

        // Inverse mass (1/mass) scaling factor for acceleration computation
        T invM_;

        T Kv_; // damping coefficient
        // Current integrated position
        T pose_;

        // Current integrated velocity
        T velocity_;

    public:
        //---------------- Public methods ----------------//

        // Constructor with defaults (same style as Controller and Planner)
        AdmittanceController() {
            desired_effort_ = 0;// gravity compensation
            current_effort_ = 0; 
            Kv_ = 10.0; // damping coefficient
            minMax_dFlow_ = Eigen::Vector2d(-10.0, 10.0);   // acceleration limits
            minMax_Flow_ = Eigen::Vector2d(-5.0, 5.0);      // velocity limits
            minMax_Pose_ = Eigen::Vector2d(-0.05, 0.05);  // position limits need to bound to xb
            invM_ = 1.0;                                     // inverse mass
            pose_ = 0;
            velocity_ = 0;
        }

        // /Eigen::Matrix<T> Admittance_Type = Eigen::Matrix<T::Zero(); //a

        // Sets the desired effort target
        void setDesiredEffort(T desired_effort) { desired_effort_ = desired_effort; }

        // Sets the current measured effort
        void setCurrentEffort(T current_effort) { current_effort_ = current_effort; }

        // Resets the internal position state
        void resetState() { pose_ = 0; velocity_ = 0; }

        // add setter for minMax_dFlow_, minMax_Flow_, minMax_Pose_ and invM_ not as vector but as T 
        void setMinMax_dFlow(T min, T max) {
            minMax_dFlow_ << min, max;
        }   
        void setMinMax_Flow(T min, T max) {
            minMax_Flow_ << min, max;
        }
        void setMinMax_Pose(T min, T max) {
            minMax_Pose_ << min, max;
        }
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
        

        // Main update function:
        // Integrates acceleration and velocity to compute new pose
        T computeDeltaPose(T dt) {
            // Compute effective net effort
            T effective_effort = desired_effort_ - current_effort_;

            // Compute acceleration (force * inverse mass)
            T acceleration = invM_ * (effective_effort - Kv_*velocity_);

            // Clamp acceleration to limits
            acceleration = std::clamp(
                acceleration,
                minMax_dFlow_[0],
                minMax_dFlow_[1]
            );

            // Integrate velocity with time step
            velocity_ += acceleration * dt;

            // Clamp velocity
            velocity_ = std::clamp(
                velocity_,
                minMax_Flow_[0],
                minMax_Flow_[1]
            );

            // Integrate position with time step
            pose_ += velocity_ * dt;

            // Clamp position
            pose_ = std::clamp(
                pose_,
                minMax_Pose_[0],
                minMax_Pose_[1]
            );

            // Anti-windup: when pose_ saturates against a bound, the velocity
            // still driving INTO that bound is non-physical (the wall absorbs
            // it). Zero only that component so the integrator settles AT the
            // bound instead of winding up (no snap-back on release/reversal).
            // Velocity pulling AWAY from the bound is preserved so motion can
            // resume immediately. Strictly dissipative -> passivity preserved.
            // Shared by joint and haptic admittance; only acts at saturation,
            // so unsaturated behaviour is byte-identical.
            if (pose_ >= minMax_Pose_[1] && velocity_ > T(0)) velocity_ = T(0);
            else if (pose_ <= minMax_Pose_[0] && velocity_ < T(0)) velocity_ = T(0);

            return pose_;
        }
    };


  // 3) typedef them for convenience
  using Ctr_t = Controller<T>;
  using Pln_t = Planner<T>;
  using Adm_t = AdmittanceController<T>;

  // 4) **The missing vector members**:
  std::vector<Ctr_t> controllers_;
  std::vector<Pln_t> planners_;
  std::vector<Adm_t> admittance_controllers_;


    // Getters for nested modules
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
