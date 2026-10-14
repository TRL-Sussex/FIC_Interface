#pragma once

#include <cmath>
#include <iostream>

/**
 * @brief A second-order digital Butterworth Low-Pass Filter.
 * * Implements the filter using the Bilinear Transform method for coefficient
 * calculation and Direct Form II Transposed for filtering.
 */
class ButterworthFilter {
public:
    /**
     * @brief Constructs the Butterworth filter.
     * * @param cutoff_freq The desired -3dB cutoff frequency (in Hz).
     * @param sample_rate The frequency at which the filter will be updated (in Hz).
     * @param initial_value The value used to initialize the filter's history 
     * to prevent large transients on startup.
     */
    ButterworthFilter(double cutoff_freq, double sample_rate, double initial_value = 0.0) {
        // --- Coefficient Calculation using Bilinear Transform (Frequency Pre-warping) ---
        double omega = tan(M_PI * cutoff_freq / sample_rate);
        double omega2 = omega * omega;
        double sqrt2 = std::sqrt(2.0);

        // Denominator of the transfer function for normalization
        double norm = 1.0 / (1.0 + sqrt2 * omega + omega2);
        
        // Numerator coefficients (a0, a1, a2)
        a0 = omega2 * norm;
        a1 = 2.0 * a0;
        a2 = a0;
        
        // Denominator coefficients (b1, b2)
        b1 = 2.0 * (omega2 - 1.0) * norm;
        b2 = (1.0 - sqrt2 * omega + omega2) * norm;

        // --- Initialize State Variables ---
        // Pre-load the history with the initial value to avoid initial transients
        x1 = x2 = initial_value;
        y1 = y2 = initial_value;

        std::cout << "Butterworth Filter Initialized: " 
                  << "Cutoff=" << cutoff_freq << "Hz, " 
                  << "Sample Rate=" << sample_rate << "Hz." 
                  << std::endl;
    }

    /**
     * @brief Filters a single input value (x0).
     * * Implements the difference equation:
     * y0 = a0*x0 + a1*x1 + a2*x2 - b1*y1 - b2*y2
     * * @param x0 The current input value (e.g., raw torque).
     * @return The filtered output value (e.g., smoothed torque).
     */
    double filter(double x0) {
        // Compute the current output (y0)
        double y0 = a0 * x0 + a1 * x1 + a2 * x2 - b1 * y1 - b2 * y2;

        // Update the state variables for the next sample
        x2 = x1; // x[n-2] = x[n-1]
        x1 = x0; // x[n-1] = x[n]
        y2 = y1; // y[n-2] = y[n-1]
        y1 = y0; // y[n-1] = y[n]

        return y0;
    }

private:
    // Filter coefficients
    double a0, a1, a2, b1, b2;
    
    // State variables (history)
    double x1, x2; // Previous inputs
    double y1, y2; // Previous outputs
};