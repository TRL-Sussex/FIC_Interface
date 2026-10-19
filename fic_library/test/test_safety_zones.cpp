// Safety zones for the whole arm (fic/safety_zones.h): distances to each shape, keep-in and keep-out, the arm
// checked along its segments, the virtual wall, the tool target kept out, and the message format.
#include <cmath>
#include <cstdio>
#include <vector>

#include <fic/safety_zones.h>

using fic::safety::Shape;
using fic::safety::Zone;
using fic::safety::ZoneSet;
using V3 = Eigen::Vector3d;

static int failures = 0;
static void check(bool ok, const char* what, double v) {
    std::printf("  %-66s %s (%.6g)\n", what, ok ? "ok" : "FAIL", v);
    if (!ok) ++failures;
}

static Zone box(V3 c, V3 half, double yaw = 0.0, bool keep_in = false) {
    Zone z;
    z.shape = Shape::Box; z.centre = c; z.half = half; z.yaw = yaw; z.keep_in = keep_in;
    return z;
}
static Zone sphere(V3 c, double r, bool keep_in = false) {
    Zone z;
    z.shape = Shape::Sphere; z.centre = c; z.radius = r; z.keep_in = keep_in;
    return z;
}

int main() {
    using fic::safety::signedDistance;
    V3 n;

    std::printf("shapes\n");
    const Zone b = box(V3(0, 0, 0), V3(0.1, 0.2, 0.3));
    double d = signedDistance(b, V3(0.3, 0, 0), &n);
    check(std::abs(d - 0.2) < 1e-12 && n.isApprox(V3(1, 0, 0)), "box: 0.2 m out along +x, normal +x", d);
    d = signedDistance(b, V3(0.05, 0, 0), &n);
    check(std::abs(d + 0.05) < 1e-12 && n.isApprox(V3(1, 0, 0)), "box: inside, the nearest face 5 cm away", d);
    d = signedDistance(b, V3(0.2, 0.3, 0), &n);
    check(std::abs(d - std::sqrt(0.02)) < 1e-12, "box: past a corner edge, the distance to the edge", d);
    const Zone turned = box(V3(1, 0, 0), V3(0.1, 0.5, 0.1), M_PI / 2);
    d = signedDistance(turned, V3(1.4, 0, 0), &n);
    check(d < 0.0 && std::abs(d + 0.1) < 1e-9, "box turned 90 deg: its long side now along x", d);
    d = signedDistance(turned, V3(1, 0.3, 0), &n);
    check(std::abs(d - 0.2) < 1e-9 && n.isApprox(V3(0, 1, 0), 1e-9), "box turned 90 deg: its short side along y", d);

    const Zone s = sphere(V3(0, 0, 1), 0.2);
    d = signedDistance(s, V3(0, 0.5, 1), &n);
    check(std::abs(d - 0.3) < 1e-12 && n.isApprox(V3(0, 1, 0)), "sphere: 0.3 m from the surface", d);

    Zone cyl;
    cyl.shape = Shape::Cylinder; cyl.centre = V3(0.5, 0, 0); cyl.radius = 0.1; cyl.z_min = 0; cyl.z_max = 1;
    d = signedDistance(cyl, V3(0.8, 0, 0.5), &n);
    check(std::abs(d - 0.2) < 1e-12 && n.isApprox(V3(1, 0, 0)), "cylinder: beside it", d);
    d = signedDistance(cyl, V3(0.5, 0, 1.25), &n);
    check(std::abs(d - 0.25) < 1e-12 && n.isApprox(V3(0, 0, 1)), "cylinder: above it", d);
    d = signedDistance(cyl, V3(0.5, 0, 0.95), &n);
    check(std::abs(d + 0.05) < 1e-12 && n.isApprox(V3(0, 0, 1)), "cylinder: inside, near the top", d);

    Zone ell;   // an L: the notch at x > 0.5, y > 0.5 is outside
    ell.shape = Shape::Prism; ell.z_min = 0; ell.z_max = 0.5;
    ell.outline = {{0, 0}, {1, 0}, {1, 0.5}, {0.5, 0.5}, {0.5, 1}, {0, 1}};
    d = signedDistance(ell, V3(0.25, 0.25, 0.25), &n);
    check(std::abs(d + 0.25) < 1e-12, "drawn outline (an L): inside its corner", d);
    d = signedDistance(ell, V3(0.75, 0.75, 0.25), &n);
    check(std::abs(d - 0.25) < 1e-12, "drawn outline: the notch is outside", d);
    d = signedDistance(ell, V3(0.25, 0.25, 0.7), &n);
    check(std::abs(d - 0.2) < 1e-12 && n.isApprox(V3(0, 0, 1)), "drawn outline: above its height", d);
    d = signedDistance(ell, V3(1.0, 0.25, 0.25), &n);
    check(std::abs(d) < 1e-12 && n.isApprox(V3(1, 0, 0)), "drawn outline: on an edge, the normal points out", d);

    std::printf("keep-in and keep-out\n");
    ZoneSet zs;
    zs.zones = {sphere(V3(0, 0, 0), 0.5, true), sphere(V3(0.8, 0, 0), 0.5, true)};   // two keep-in, overlapping
    auto h = zs.ball(V3(0.4, 0, 0), 0.05);
    check(std::abs(h.clearance - 0.05) < 1e-12, "keep-in: inside their union, the clearer of the two counts", h.clearance);
    h = zs.ball(V3(-0.48, 0, 0), 0.05);
    check(std::abs(h.clearance + 0.03) < 1e-12 && h.dir.isApprox(V3(1, 0, 0)), "keep-in: 3 cm past, pushed back in", h.clearance);
    zs.zones.push_back(box(V3(0.4, 0, 0), V3(0.05, 0.05, 0.05)));                     // a keep-out inside them
    h = zs.ball(V3(0.4, 0, 0.08), 0.05);
    check(std::abs(h.clearance + 0.02) < 1e-12 && h.dir.isApprox(V3(0, 0, 1)) && h.zone == 2,
          "keep-out inside a keep-in: the keep-out is the nearer boundary", h.clearance);
    zs.zones[2].enabled = false;
    h = zs.ball(V3(0.4, 0, 0.08), 0.05);
    check(h.clearance > 0.0, "a zone switched off is not checked", h.clearance);

    std::printf("the arm\n");
    ZoneSet arm;
    arm.body.radius = 0.05; arm.body.tool_radius = 0.02;
    arm.zones = {sphere(V3(0.3, 0, 0.5), 0.1)};
    const std::vector<V3> pts = {V3(0, 0, 0.5), V3(0.6, 0, 0.5), V3(0.6, 0, 0.1)};   // elbow passes the sphere
    ZoneSet::BodyHit worst;
    auto past = arm.check(pts, &worst, 0.05);
    check(past.size() == 1 && past[0].segment == 0, "a link through a keep-out zone: that segment is past it", past.size());
    check(std::abs(worst.hit.clearance + 0.15) < 1e-9 && std::abs(worst.t - 0.5) < 1e-9,
          "the deepest point is mid-link, 15 cm in (10 cm radius + 5 cm link)", worst.hit.clearance);
    const std::vector<V3> tool_only = {V3(0.3, 0, 0.5)};
    past = arm.check(tool_only, &worst, 0.05);
    check(past.size() == 1 && std::abs(worst.hit.clearance + 0.12) < 1e-9, "the tool point alone, with its own radius", worst.hit.clearance);
    const std::vector<V3> clear = {V3(0, 0, 0.9), V3(0.6, 0, 0.9)};
    past = arm.check(clear, &worst, 0.05);
    check(past.empty() && std::abs(worst.hit.clearance - 0.25) < 1e-9, "an arm clear of it: nothing past, 25 cm to spare", worst.hit.clearance);

    // the wall: two points moving with their own 3-D joint (J = identity), the link's midpoint pushed out
    past = arm.check({V3(0, 0, 0.45), V3(0.6, 0, 0.45)}, &worst, 0.05);
    auto jac = [](int i) { Eigen::MatrixXd J = Eigen::MatrixXd::Zero(3, 6); J.block<3, 3>(0, 3 * i) = Eigen::Matrix3d::Identity(); return J; };
    const Eigen::VectorXd tau = arm.wallTorque(past, jac, 6, 2000.0);
    check(tau[2] < 0.0 && tau[5] < 0.0 && std::abs(tau[2] - tau[5]) < 1e-9,
          "wall: the link below the sphere is pushed down at both ends", tau[2]);
    check(std::abs(tau[2] + 0.5 * 2000.0 * 0.1) < 1e-9, "wall: 2000 N/m on 10 cm, half to each end", tau[2]);

    std::printf("the tool target\n");
    V3 p;
    bool found = arm.project(V3(0.32, 0, 0.5), 0.02, &p);
    h = arm.ball(p, 0.02);
    check(found && h.clearance >= 0.0 && h.clearance < 0.0011, "a target in a keep-out zone moves to its edge", h.clearance);
    ZoneSet ws;
    ws.zones = {box(V3(0, 0, 0.5), V3(0.5, 0.5, 0.5), 0.0, true), sphere(V3(0.4, 0, 0.5), 0.05)};
    found = ws.project(V3(0.7, 0.2, 0.5), 0.0, &p);
    h = ws.ball(p, 0.0);
    check(found && h.clearance >= 0.0 && std::abs(p.x() - 0.499) < 1e-9, "a target outside the workspace comes back to its edge", p.x());
    ws.zones[1] = sphere(V3(0.47, 0, 0.5), 0.1);       // a keep-out across the workspace's edge, the target behind it
    found = ws.project(V3(0.7, 0, 0.5), 0.0, &p);
    check(!found, "no clear place along the way back: reported, the caller keeps its last target", p.x());

    std::printf("message\n");
    ZoneSet a;
    a.body.radius = 0.06; a.body.tool_radius = 0.025; a.body.from_joint = 2;
    a.zones = {box(V3(0.1, 0.2, 0.3), V3(0.1, 0.1, 0.1), 0.3, true), sphere(V3(0, 0, 1), 0.2), cyl, ell};
    a.zones[1].enabled = false;
    ZoneSet r;
    const bool ok = r.parse(a.encode());
    check(ok && r.zones.size() == 4 && r.body.from_joint == 2 && std::abs(r.body.radius - 0.06) < 1e-12,
          "encode then parse: the same zones and arm", r.zones.size());
    check(r.zones[0].keep_in && !r.zones[1].enabled && r.zones[3].outline.size() == 6 && std::abs(r.zones[0].yaw - 0.3) < 1e-12,
          "keep-in, switched off, outline corners and turn kept", r.zones[3].outline.size());
    std::vector<double> m = a.encode();
    m.back() = std::nan("");
    std::string why;
    check(!r.parse(m, &why) && r.zones.size() == 4, "a malformed message is refused and changes nothing", r.zones.size());
    m = a.encode();
    m.pop_back();
    check(!r.parse(m, &why), "a truncated message is refused", 0);
    check(r.parse({1, 0.05, 0.03, 1, 0}) && !r.active(), "no zones: nothing is checked", r.zones.size());

    std::printf(failures ? "\n%d FAILED\n" : "\nall passed\n", failures);
    return failures ? 1 : 0;
}
