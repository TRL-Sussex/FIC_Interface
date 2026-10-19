// Safety zones for the whole arm: shapes the robot must stay inside (keep-in) or out of (keep-out), checked
// along the arm from a chosen joint to the tool, not only at the tool point. Used by fic_simulation and
// fic_hardware; set by robot{N}_safety/zones from the Studio's Safety page.
//
//   shapes     a box (turned about z), a sphere, an upright cylinder and a prism: an outline drawn on the
//              floor, raised between two heights
//   keep-in    the arm must stay inside; with several, inside any one of them (their union)
//   keep-out   the arm must stay outside every one
//   the arm    the joint frames from body.from_joint (0-based, the second joint by default: the first link
//              turns about its own axis and cannot be moved out of a zone) to the tool frame, joined by
//              segments; every segment is a capsule of body.radius, the tool point a ball of body.tool_radius.
//              Sample points every 4 cm along each segment are what is checked.
//
// A point's clearance is how far its ball is from the nearest boundary it must not cross (negative: past it),
// with the direction that takes it back. The nodes push a point that is past a boundary back with a virtual
// wall (wallTorque), refuse joint targets that would take the arm further in, keep the tool target out of the
// zones (project), and report the arm's worst clearance to the Studio, which warns and stops on it.
//
// Message (Float64MultiArray): [1, radius, tool_radius, from_joint, n, zone...], each zone
// [shape, keep_in, enabled, k, k parameters]:
//   box      cx cy cz hx hy hz yaw    (centre, half sizes, turn about z in rad)
//   sphere   cx cy cz r
//   cylinder cx cy r z_min z_max
//   prism    z_min z_max x0 y0 x1 y1 ... (at least three corners, in order)
#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace fic {
namespace safety {

enum class Shape { Box = 0, Sphere = 1, Cylinder = 2, Prism = 3 };

struct Zone {
    Shape shape = Shape::Box;
    bool keep_in = false;
    bool enabled = true;
    Eigen::Vector3d centre = Eigen::Vector3d::Zero();   // box, sphere (cylinder: x, y)
    Eigen::Vector3d half = Eigen::Vector3d::Zero();     // box half sizes
    double yaw = 0.0;                                   // box turn about z
    double radius = 0.0;                                // sphere, cylinder
    double z_min = 0.0, z_max = 0.0;                    // cylinder, prism
    std::vector<Eigen::Vector2d> outline;               // prism corners, in order
};

struct Body {
    double radius = 0.05;        // m, each segment of the arm
    double tool_radius = 0.03;   // m, the tool point
    int from_joint = 1;          // the first joint frame checked (0-based)
};

// Signed distance from p to the zone's surface (negative inside) and its gradient, the outward normal.
namespace detail {

// A shape raised between two heights: the distance in the plane (dxy, with its 2-D gradient) and along z
// (dz, gz = +-1) combined as for a box.
inline double extrude(double dxy, const Eigen::Vector2d& gxy, double dz, double gz, Eigen::Vector3d* n) {
    if (dxy > 0.0 && dz > 0.0) {
        const double d = std::hypot(dxy, dz);
        if (n) *n = Eigen::Vector3d(gxy.x() * dxy / d, gxy.y() * dxy / d, gz * dz / d);
        return d;
    }
    if (dxy >= dz) {
        if (n) *n = Eigen::Vector3d(gxy.x(), gxy.y(), 0.0);
        return dxy;
    }
    if (n) *n = Eigen::Vector3d(0.0, 0.0, gz);
    return dz;
}

inline double box(const Zone& z, const Eigen::Vector3d& p, Eigen::Vector3d* n) {
    const double c = std::cos(z.yaw), s = std::sin(z.yaw);
    const Eigen::Vector3d d = p - z.centre;
    const Eigen::Vector3d l(c * d.x() + s * d.y(), -s * d.x() + c * d.y(), d.z());   // in the box's frame
    const Eigen::Vector3d q = l.cwiseAbs() - z.half;
    Eigen::Vector3d g;
    double dist;
    const Eigen::Vector3d out = q.cwiseMax(0.0);
    if (out.squaredNorm() > 0.0) {
        dist = out.norm();
        g = out / dist;
    } else {
        int i = 0;
        dist = q.maxCoeff(&i);
        g = Eigen::Vector3d::Zero();
        g[i] = 1.0;
    }
    for (int i = 0; i < 3; ++i) g[i] *= (l[i] < 0.0 ? -1.0 : 1.0);
    if (n) *n = Eigen::Vector3d(c * g.x() - s * g.y(), s * g.x() + c * g.y(), g.z());
    return dist;
}

inline double sphere(const Zone& z, const Eigen::Vector3d& p, Eigen::Vector3d* n) {
    const Eigen::Vector3d d = p - z.centre;
    const double r = d.norm();
    if (n) *n = r > 1e-12 ? Eigen::Vector3d(d / r) : Eigen::Vector3d::UnitZ();
    return r - z.radius;
}

inline void heights(const Zone& z, double pz, double* dz, double* gz) {
    const double mid = 0.5 * (z.z_min + z.z_max), half = 0.5 * (z.z_max - z.z_min);
    *dz = std::abs(pz - mid) - half;
    *gz = pz < mid ? -1.0 : 1.0;
}

inline double cylinder(const Zone& z, const Eigen::Vector3d& p, Eigen::Vector3d* n) {
    const Eigen::Vector2d d(p.x() - z.centre.x(), p.y() - z.centre.y());
    const double r = d.norm();
    const Eigen::Vector2d gxy = r > 1e-12 ? Eigen::Vector2d(d / r) : Eigen::Vector2d::UnitX();
    double dz, gz;
    heights(z, p.z(), &dz, &gz);
    return extrude(r - z.radius, gxy, dz, gz, n);
}

// Signed distance to a closed outline in the plane (negative inside) and its gradient.
inline double outline(const std::vector<Eigen::Vector2d>& poly, const Eigen::Vector2d& p, Eigen::Vector2d* g) {
    double best = std::numeric_limits<double>::infinity();
    Eigen::Vector2d closest = p, edge_n = Eigen::Vector2d::UnitX();
    bool inside = false;
    const size_t m = poly.size();
    for (size_t i = 0, j = m - 1; i < m; j = i++) {
        const Eigen::Vector2d& a = poly[j];
        const Eigen::Vector2d& b = poly[i];
        const Eigen::Vector2d ab = b - a;
        const double len2 = ab.squaredNorm();
        const double t = len2 > 0.0 ? std::clamp((p - a).dot(ab) / len2, 0.0, 1.0) : 0.0;
        const Eigen::Vector2d c = a + t * ab;
        const double d = (p - c).norm();
        if (d < best) {
            best = d;
            closest = c;
            edge_n = len2 > 0.0 ? Eigen::Vector2d(Eigen::Vector2d(ab.y(), -ab.x()) / std::sqrt(len2)) : Eigen::Vector2d(Eigen::Vector2d::UnitX());
        }
        if ((a.y() > p.y()) != (b.y() > p.y()) && p.x() < a.x() + (p.y() - a.y()) * ab.x() / ab.y()) inside = !inside;
    }
    Eigen::Vector2d away = best > 1e-9 ? Eigen::Vector2d((p - closest) / best) : edge_n;
    if (best <= 1e-9) {
        // on the edge: the edge normal, turned outward (a point just outside along it must test outside)
        Eigen::Vector2d probe = p + 1e-6 * edge_n;
        bool in_probe = false;
        for (size_t i = 0, j = m - 1; i < m; j = i++) {
            const Eigen::Vector2d& a = poly[j];
            const Eigen::Vector2d& b = poly[i];
            if ((a.y() > probe.y()) != (b.y() > probe.y()) &&
                probe.x() < a.x() + (probe.y() - a.y()) * (b.x() - a.x()) / (b.y() - a.y()))
                in_probe = !in_probe;
        }
        away = in_probe ? Eigen::Vector2d(-edge_n) : edge_n;
        if (g) *g = away;
        return 0.0;
    }
    if (g) *g = inside ? Eigen::Vector2d(-away) : away;
    return inside ? -best : best;
}

inline double prism(const Zone& z, const Eigen::Vector3d& p, Eigen::Vector3d* n) {
    Eigen::Vector2d gxy;
    const double dxy = outline(z.outline, Eigen::Vector2d(p.x(), p.y()), &gxy);
    double dz, gz;
    heights(z, p.z(), &dz, &gz);
    return extrude(dxy, gxy, dz, gz, n);
}

}  // namespace detail

inline double signedDistance(const Zone& z, const Eigen::Vector3d& p, Eigen::Vector3d* normal = nullptr) {
    switch (z.shape) {
        case Shape::Box: return detail::box(z, p, normal);
        case Shape::Sphere: return detail::sphere(z, p, normal);
        case Shape::Cylinder: return detail::cylinder(z, p, normal);
        case Shape::Prism: return detail::prism(z, p, normal);
    }
    return std::numeric_limits<double>::infinity();
}

class ZoneSet {
public:
    std::vector<Zone> zones;
    Body body;

    struct Hit {
        double clearance = std::numeric_limits<double>::infinity();   // m; negative: past the boundary
        int zone = -1;                                                // index into zones
        Eigen::Vector3d dir = Eigen::Vector3d::Zero();                // the way back
    };
    struct BodyHit {
        Hit hit;
        int segment = -1;   // between body point segment and segment + 1 (the tool point alone: 0)
        double t = 0.0;     // where along it
    };

    bool active() const {
        for (const Zone& z : zones)
            if (z.enabled) return true;
        return false;
    }

    // A ball of radius rho at p against every enabled zone: the boundary it is nearest to crossing.
    Hit ball(const Eigen::Vector3d& p, double rho) const {
        Hit worst, in_best;
        bool any_in = false;
        for (int i = 0; i < static_cast<int>(zones.size()); ++i) {
            const Zone& z = zones[i];
            if (!z.enabled) continue;
            Eigen::Vector3d n;
            const double s = signedDistance(z, p, &n);
            if (z.keep_in) {
                const double c = -s - rho;
                if (!any_in || c > in_best.clearance) in_best = Hit{c, i, -n};
                any_in = true;
            } else {
                const double c = s - rho;
                if (c < worst.clearance) worst = Hit{c, i, n};
            }
        }
        if (any_in && in_best.clearance < worst.clearance) worst = in_best;
        return worst;
    }

    // The arm's points: the deepest hit of each segment past a zone (clearance < 0), and in *worst the
    // arm's worst clearance whatever its sign. points: the joint frames from body.from_joint, then the tool.
    std::vector<BodyHit> check(const std::vector<Eigen::Vector3d>& points, BodyHit* worst = nullptr,
                               double step = 0.04) const {
        std::vector<BodyHit> past;
        BodyHit w;
        const int m = static_cast<int>(points.size());
        auto visit = [&](int seg, double t, const Eigen::Vector3d& p, double rho, BodyHit& seg_best) {
            const Hit h = ball(p, rho);
            if (h.clearance < w.hit.clearance) w = BodyHit{h, seg, t};
            if (h.clearance < 0.0 && h.clearance < seg_best.hit.clearance) seg_best = BodyHit{h, seg, t};
        };
        if (m == 1) {
            BodyHit b;
            visit(0, 0.0, points[0], body.tool_radius, b);
            if (b.segment >= 0) past.push_back(b);
        }
        for (int k = 0; k + 1 < m; ++k) {
            BodyHit b;
            const double len = (points[k + 1] - points[k]).norm();
            const int steps = std::max(1, static_cast<int>(std::ceil(len / step)));
            for (int s = 0; s <= steps; ++s) {
                const double t = static_cast<double>(s) / steps;
                const bool tool = (k + 2 == m) && s == steps;
                visit(k, t, (1.0 - t) * points[k] + t * points[k + 1], tool ? body.tool_radius : body.radius, b);
            }
            if (b.segment >= 0) past.push_back(b);
        }
        if (worst) *worst = w;
        return past;
    }

    // Joint torques of a virtual wall of stiffness k (N/m) on every segment past a zone. jac(i) gives the
    // 3 x n position Jacobian of body point i (a point along a segment moves as the blend of its ends).
    template <typename JacobianOf>
    Eigen::VectorXd wallTorque(const std::vector<BodyHit>& past, JacobianOf jac, int n, double k) const {
        Eigen::VectorXd tau = Eigen::VectorXd::Zero(n);
        for (const BodyHit& b : past) {
            const Eigen::Vector3d f = k * (-b.hit.clearance) * b.hit.dir;
            Eigen::MatrixXd J = (1.0 - b.t) * jac(b.segment);
            if (b.t > 0.0) J += b.t * jac(b.segment + 1);
            tau += J.transpose() * f;
        }
        return tau;
    }

    // p moved the least way that keeps a ball of radius rho clear of every zone, by margin, into *out; false
    // if no clear place was found that way (zones that leave none, or two boundaries pushing it back and
    // forth): the caller keeps its last safe target.
    bool project(const Eigen::Vector3d& p, double rho, Eigen::Vector3d* out, double margin = 0.001,
                 int iterations = 20) const {
        Eigen::Vector3d q = p;
        for (int i = 0; i < iterations; ++i) {
            const Hit h = ball(q, rho);
            if (h.zone < 0 || h.clearance >= 0.0) {
                *out = q;
                return true;
            }
            q += (margin - h.clearance) * h.dir;
        }
        return false;
    }

    // From robot{N}_safety/zones (see the top of this file). False, and nothing changed, if malformed.
    bool parse(const std::vector<double>& m, std::string* why = nullptr) {
        auto bad = [&](const char* w) { if (why) *why = w; return false; };
        for (double v : m)
            if (!std::isfinite(v)) return bad("a value is not finite");
        if (m.size() < 5 || m[0] != 1.0) return bad("unknown format");
        Body b;
        b.radius = m[1];
        b.tool_radius = m[2];
        b.from_joint = static_cast<int>(m[3]);
        if (b.radius < 0.0 || b.radius > 0.5 || b.tool_radius < 0.0 || b.tool_radius > 0.5 || b.from_joint < 0)
            return bad("body radius or first joint out of range");
        const int n = static_cast<int>(m[4]);
        if (n < 0 || n > 64) return bad("too many zones");
        std::vector<Zone> out;
        size_t i = 5;
        for (int j = 0; j < n; ++j) {
            if (i + 4 > m.size()) return bad("truncated");
            Zone z;
            const int shape = static_cast<int>(m[i]);
            z.keep_in = m[i + 1] > 0.5;
            z.enabled = m[i + 2] > 0.5;
            const size_t k = static_cast<size_t>(m[i + 3]);
            i += 4;
            if (i + k > m.size()) return bad("truncated");
            const double* a = m.data() + i;
            switch (shape) {
                case 0:
                    if (k != 7) return bad("a box takes 7 values");
                    z.shape = Shape::Box;
                    z.centre = Eigen::Vector3d(a[0], a[1], a[2]);
                    z.half = Eigen::Vector3d(a[3], a[4], a[5]);
                    z.yaw = a[6];
                    if (z.half.minCoeff() <= 0.0) return bad("a box needs a positive size");
                    break;
                case 1:
                    if (k != 4) return bad("a sphere takes 4 values");
                    z.shape = Shape::Sphere;
                    z.centre = Eigen::Vector3d(a[0], a[1], a[2]);
                    z.radius = a[3];
                    if (z.radius <= 0.0) return bad("a sphere needs a positive radius");
                    break;
                case 2:
                    if (k != 5) return bad("a cylinder takes 5 values");
                    z.shape = Shape::Cylinder;
                    z.centre = Eigen::Vector3d(a[0], a[1], 0.0);
                    z.radius = a[2];
                    z.z_min = a[3];
                    z.z_max = a[4];
                    if (z.radius <= 0.0 || z.z_min >= z.z_max) return bad("a cylinder needs a radius and a height");
                    break;
                case 3:
                    if (k < 8 || k % 2 != 0) return bad("a prism takes two heights and at least three corners");
                    z.shape = Shape::Prism;
                    z.z_min = a[0];
                    z.z_max = a[1];
                    if (z.z_min >= z.z_max) return bad("a prism needs a height");
                    for (size_t c = 2; c + 1 < k; c += 2) z.outline.emplace_back(a[c], a[c + 1]);
                    break;
                default:
                    return bad("unknown shape");
            }
            i += k;
            out.push_back(z);
        }
        if (i != m.size()) return bad("trailing values");
        zones = std::move(out);
        body = b;
        return true;
    }

    std::vector<double> encode() const {
        std::vector<double> m = {1.0, body.radius, body.tool_radius, static_cast<double>(body.from_joint),
                                 static_cast<double>(zones.size())};
        for (const Zone& z : zones) {
            std::vector<double> a;
            switch (z.shape) {
                case Shape::Box: a = {z.centre.x(), z.centre.y(), z.centre.z(), z.half.x(), z.half.y(), z.half.z(), z.yaw}; break;
                case Shape::Sphere: a = {z.centre.x(), z.centre.y(), z.centre.z(), z.radius}; break;
                case Shape::Cylinder: a = {z.centre.x(), z.centre.y(), z.radius, z.z_min, z.z_max}; break;
                case Shape::Prism:
                    a = {z.z_min, z.z_max};
                    for (const auto& c : z.outline) { a.push_back(c.x()); a.push_back(c.y()); }
                    break;
            }
            m.insert(m.end(), {static_cast<double>(static_cast<int>(z.shape)), z.keep_in ? 1.0 : 0.0,
                               z.enabled ? 1.0 : 0.0, static_cast<double>(a.size())});
            m.insert(m.end(), a.begin(), a.end());
        }
        return m;
    }
};

}  // namespace safety
}  // namespace fic
