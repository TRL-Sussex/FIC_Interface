// H-PMP's tuning, as both nodes read it: fic::tuningKeyFor (which blocks a control mode reads,
// in fic_simulation and fic_hardware alike) and loadTuningSet's H-PMP block, with the fallback
// to modes 4 and 6 for a robot that has none. Needs tinyxml2:  see fic_library/test/CMakeLists.txt

#include <cstdio>
#include <fstream>
#include <string>

#include <fic/fic_param_loader.hpp>

static int g_fail = 0;
#define CHECK(c) do { if (!(c)) { std::printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); ++g_fail; } } while (0)

static std::string joint(int id, double kv) {
    return "<joint id=\"" + std::to_string(id) + "\"><controller><stiffness>200</stiffness></controller>"
           "<admittance><InvM>1.0</InvM><Damping>" + std::to_string(kv) + "</Damping></admittance></joint>";
}
static std::string cart(double k) {
    return "<cartesian><x><controller><stiffness>" + std::to_string(k) + "</stiffness></controller></x></cartesian>";
}
static std::string mode(int id, const char* profile, const std::string& body) {
    return "<mode id=\"" + std::to_string(id) + "\" profile=\"" + profile + "\">" + body + "</mode>";
}
static std::string write(const char* name, const std::string& robot_body) {
    const std::string path = std::string("/tmp/fic_test_") + name + ".xml";
    std::ofstream(path) << "<?xml version=\"1.0\"?><robots><robot name=\"R\">" << robot_body << "</robot></robots>";
    return path;
}

int main() {
    using fic::tuningKeyFor;
    std::printf("tuningKeyFor\n");
    for (int m = 1; m <= 9; ++m) {
        // H-PMP off: every mode reads itself (mode 9, simulation only, mode 1's joints)
        auto k = tuningKeyFor("R", 7, m, false);
        CHECK(k.joint_mode == (m == 9 ? 1 : m) && k.cart_mode == m && k.postural == (m >= 5 && m <= 8));
        // H-PMP on: its own block in the Cartesian modes 5-8 only, simulation and hardware alike
        auto h = tuningKeyFor("R", 7, m, true), hw = tuningKeyFor("R", 7, m, true, false);
        const bool on = m >= 5 && m <= 8;
        CHECK(fic::hpmpActive(m, true) == on && !fic::hpmpActive(m, false));
        CHECK(h == hw);
        CHECK(on ? (h.joint_mode == 100 && h.cart_mode == 100) : (h == k));
        // the MCU runs Cartesian modes on mode 4's joints; H-PMP still reads its own block
        auto mcu = tuningKeyFor("R", 7, m, true, true);
        CHECK(on ? mcu.joint_mode == 100 : mcu.joint_mode == (m > 4 ? 4 : m));
    }

    std::printf("loadTuningSet: no H-PMP block -> modes 4 and 6\n");
    const std::string layers = mode(4, "default", joint(0, 9.0) + joint(1, 13.7)) + mode(6, "default", cart(200.0)) +
                               mode(4, "x4", joint(0, 36.0) + joint(1, 54.8));
    auto key = tuningKeyFor("R", 2, 8, true);
    {
        auto t = fic::loadTuningSet(write("none", layers), key, "default");
        CHECK(t.hpmp_fallback && t.key == key);
        CHECK(t.joint_admittance[0].damping == 9.0 && t.joint_admittance[1].damping == 13.7);
        CHECK(t.cart.pos_axes[0].stiffness == 200.0);
        auto x4 = fic::loadTuningSet(write("none", layers), key, "x4");     // the profile's own mode 4
        CHECK(x4.hpmp_fallback && x4.joint_admittance[1].damping == 54.8 && x4.cart.pos_axes[0].stiffness == 200.0);
    }

    std::printf("loadTuningSet: H-PMP's own block\n");
    const std::string own = layers + mode(100, "default", joint(0, 20.0) + joint(1, 30.0) + cart(150.0)) +
                            mode(100, "x4", joint(0, 36.0) + joint(1, 54.8) + cart(150.0));
    {
        auto t = fic::loadTuningSet(write("own", own), key, "default");
        CHECK(!t.hpmp_fallback);
        CHECK(t.joint_admittance[0].damping == 20.0 && t.joint_admittance[1].damping == 30.0);
        CHECK(t.cart.pos_axes[0].stiffness == 150.0);
        auto x4 = fic::loadTuningSet(write("own", own), key, "x4");
        CHECK(!x4.hpmp_fallback && x4.joint_admittance[1].damping == 54.8);
        auto other = fic::loadTuningSet(write("own", own), key, "stiff");  // no block: default's H-PMP block
        CHECK(!other.hpmp_fallback && other.joint_admittance[0].damping == 20.0);
        // the control modes still read their own blocks
        auto m4 = fic::loadTuningSet(write("own", own), tuningKeyFor("R", 2, 4, true), "default");
        CHECK(m4.joint_admittance[0].damping == 9.0);
        auto m6 = fic::loadTuningSet(write("own", own), tuningKeyFor("R", 2, 6, false), "default");
        CHECK(m6.cart.pos_axes[0].stiffness == 200.0);
    }

    if (g_fail) { std::printf("%d check(s) failed\n", g_fail); return 1; }
    std::printf("all passed\n");
    return 0;
}
