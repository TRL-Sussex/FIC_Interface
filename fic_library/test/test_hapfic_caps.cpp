// HAP-FIC's set-point limit (fic_control.h): limitSetpoint keeps a joint set-point within a window
// of the reached position, wrapped for a continuous joint, and leaves it alone inside the window;
// setpointWindow is the admittance bound, never narrower than 0.1 rad.
//
// Needs Eigen and tinyxml2 (fic_control.h pulls in the XML loader).

#include <cmath>
#include <cstdio>

#include <fic/fic_control.h>

using FC = fic::ficControl<double>;
static int failures = 0;

static void check(bool ok, const char* what, double value) {
    std::printf("  %-62s %s (%.6g)\n", what, ok ? "ok" : "FAIL", value);
    if (!ok) ++failures;
}

int main() {
    std::printf("set-point limit\n");
    check(std::fabs(FC::limitSetpoint(0.5, 0.0, 0.1, false) - 0.1) < 1e-12, "far above: bound above q", FC::limitSetpoint(0.5, 0.0, 0.1, false));
    check(std::fabs(FC::limitSetpoint(-0.5, 0.2, 0.1, false) - 0.1) < 1e-12, "far below: bound below q", FC::limitSetpoint(-0.5, 0.2, 0.1, false));
    check(FC::limitSetpoint(0.05, 0.0, 0.1, false) == 0.05, "inside: unchanged", FC::limitSetpoint(0.05, 0.0, 0.1, false));
    // continuous: q just below pi, target just across -pi (0.0832 rad ahead the short way)
    const double qc = 3.1, tc = -3.1, lim = FC::limitSetpoint(tc, qc, 0.05, true);
    const double ahead = std::atan2(std::sin(lim - qc), std::cos(lim - qc));
    check(std::fabs(ahead - 0.05) < 1e-12 && lim <= M_PI && lim > -M_PI, "across +-pi: 0.05 ahead the short way, wrapped", ahead);
    check(FC::setpointWindow(0.02) == 0.1 && FC::setpointWindow(-0.4) == 0.4, "window: bound, at least 0.1 rad", FC::setpointWindow(0.02));

    std::printf(failures ? "FAILED (%d)\n" : "passed\n", failures);
    return failures ? 1 : 0;
}
