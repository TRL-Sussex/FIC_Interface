#ifndef FIC_PARAM_LOADER_HPP
#define FIC_PARAM_LOADER_HPP

#include <tinyxml2.h>
#include <vector>
#include <string>
#include <sstream>
#include <stdexcept>
#include <limits>
#include <algorithm>
#include <iostream>

namespace fic {

// --- 1. Data Structures ---
enum class ParameterType { Joint, Cartesian, Postural };

struct ListParams {
    std::vector<double> error_max;
    std::vector<double> force_max;
    std::vector<double> saturation_ratios;
    std::vector<double> velocity;
    int n_saturations = 4;
    bool asymmetric = false;
    int n_neg_sat = 2;
};

struct ScalarParams {
    double stiffness = 0.0;
    double diff_effort_max = 0.0;
};

struct AdmittanceParams {
    double InvM = 1.0;
    double damping = 0.0;
    std::vector<double> dFlow;
    std::vector<double> flow;
    std::vector<double> pose;
};

struct PlannerParams {
    double Plannerstiffness = 0.0;
    double vel_desired = 0.0;
    double acc_max = 0.0;
    double damping_ratio = 0.0;
    double frequency = 0.0;
    double stiffness_Q = 0.0;
};

struct CartesianAxisParams {
    double stiffness = 0.0;
    double diff_effort_max = 0.0;
    std::vector<double> error_max;
    std::vector<double> force_max;
    std::vector<double> saturation_ratios;
    int n_saturations = 4;
    bool asymmetric = false;
    int n_neg_sat = 2;
    double stiffness_P = 0.0;
    double vel_desired = 0.0;
    double acc_max = 0.0;
    double damping_ratio = 0.0;
    double frequency = 0.0;
    AdmittanceParams admittance;
};

struct CartesianAllParams {
    CartesianAxisParams pos_axes[3]; // X, Y, Z
    CartesianAxisParams rot_axis;    // Q
};

// --- 2. Loader Class ---
class ParameterLoader {
public:
    static void loadJointParams(const std::string& xml_file, const std::string& robot_name, size_t id,
                                ScalarParams& s, ListParams& l, PlannerParams& p, AdmittanceParams& a,
                                int control_mode = 1, bool is_postural = false,
                                const std::string& profile_name = "default") {

        // --- 1. INJECT SAFE MEMORY DEFAULTS FIRST ---
        // If the XML tag doesn't exist, the node will use these instead of Segfaulting!
        s.stiffness = 100.0; s.diff_effort_max = 10.0;
        l.error_max = {0.05, 0.1}; l.force_max = {10.0, 20.0};
        l.saturation_ratios = {0.9}; l.velocity = {0.0};
        p.Plannerstiffness = 15000.0; p.vel_desired = 0.0; p.acc_max = 5000.0;
        p.damping_ratio = 0.01; p.frequency = 150.0;
        a.InvM = 1.0; a.damping = 10.0; a.dFlow = {-5.0, 5.0}; a.flow = {-1.0, 1.0}; a.pose = {-0.1, 0.1};

        // --- 2. OVERWRITE WITH XML (IF IT EXISTS) ---
        ParameterType type = is_postural ? ParameterType::Postural : ParameterType::Joint;
        loadRobotParametersFromXML(robot_name, type, id, s, l, p, a, xml_file, nullptr, control_mode, profile_name);
    }

    static void loadCartesianParams(const std::string& xml_file, const std::string& robot_name,
                                    CartesianAllParams& cart_out, int control_mode = 5,
                                    const std::string& profile_name = "default") {

        // --- 1. INJECT SAFE MEMORY DEFAULTS FIRST ---
        for(int i=0; i<3; ++i) {
            cart_out.pos_axes[i].stiffness = 100.0; cart_out.pos_axes[i].diff_effort_max = 10.0;
            cart_out.pos_axes[i].error_max = {0.05, 0.1}; cart_out.pos_axes[i].force_max = {10.0, 20.0};
            cart_out.pos_axes[i].saturation_ratios = {0.9}; cart_out.pos_axes[i].stiffness_P = 15000.0;
            cart_out.pos_axes[i].vel_desired = 0.0; cart_out.pos_axes[i].acc_max = 5000.0;
            cart_out.pos_axes[i].damping_ratio = 0.01; cart_out.pos_axes[i].frequency = 150.0;
            cart_out.pos_axes[i].admittance.InvM = 1.0; cart_out.pos_axes[i].admittance.damping = 10.0;
            cart_out.pos_axes[i].admittance.dFlow = {-5.0, 5.0}; cart_out.pos_axes[i].admittance.flow = {-1.0, 1.0};
            cart_out.pos_axes[i].admittance.pose = {-0.1, 0.1};
        }
        cart_out.rot_axis = cart_out.pos_axes[0]; // Copy defaults to rotation axis

        // --- 2. OVERWRITE WITH XML (IF IT EXISTS) ---
        ScalarParams s; ListParams l; PlannerParams p; AdmittanceParams a;
        loadRobotParametersFromXML(robot_name, ParameterType::Cartesian, 0, s, l, p, a, xml_file, &cart_out, control_mode, profile_name);
    }

    static void loadHardcodedParams(size_t num_joints, std::vector<ScalarParams>& joint_scalars,
                                    std::vector<ListParams>& joint_lists, std::vector<PlannerParams>& joint_planners,
                                    std::vector<AdmittanceParams>& joint_admittance, CartesianAllParams& cart_params) {
        joint_scalars.assign(num_joints, {400.0, 50.0}); 
        joint_admittance.assign(num_joints, {1.0, 100.0, {0.05, 0.1}, {5.0, 39.0}, {5.0, 39.0}});
        for (size_t i = 0; i < num_joints; ++i) {
            joint_lists[i] = {{0.05, 0.1, 0.2}, {20.0, 40.0, 60.0}, {0.8, 0.9, 1.0}, {0.5, 1.0, 1.5}, 6, false, 3};
            // acc_max 5000 as config.xml: it caps the planner since FIC 5.11.0, and the 0.5 here
            // before had no effect.
            joint_planners[i] = {150.0, 0.3, 5000.0, 1.0, 1000.0, 0.0};
        }
        for (int i = 0; i < 3; ++i) {
            cart_params.pos_axes[i] = {500.0, 100.0, {0.02, 0.05, 0.1}, {10.0, 20.0, 30.0}, {0.7, 0.9, 1.0}, 6, false, 3, 200.0, 0.1, 5000.0, 1.0, 1000.0, {}};
        }
        cart_params.rot_axis = {50.0, 10.0, {0.1, 0.2}, {5.0, 10.0}, {1.0, 1.0}, 4, false, 2, 50.0, 0.5, 5000.0, 1.0, 1000.0, {}};
    }

    // Whether the robot has a <mode id="control_mode"> block for this profile or, failing
    // that, for "default" (the block the loaders would read).
    static bool hasModeElement(const std::string& xml_file, const std::string& robot_name,
                               int control_mode, const std::string& profile_name) {
        tinyxml2::XMLDocument doc;
        if (doc.LoadFile(xml_file.c_str()) != tinyxml2::XML_SUCCESS) return false;
        auto* robots = doc.FirstChildElement("robots");
        if (!robots) return false;
        for (auto* robot = robots->FirstChildElement("robot"); robot; robot = robot->NextSiblingElement("robot")) {
            const char* name = robot->Attribute("name");
            if (name && robot_name == name) return findModeElement(robot, control_mode, profile_name) != nullptr;
        }
        return false;
    }

private:
    static void parseList(tinyxml2::XMLElement* parent, const std::string& tag_name, std::vector<double>& out_list) {
        if (auto* e = parent->FirstChildElement(tag_name.c_str())) {
            if (const char* text = e->GetText()) {
                out_list.clear(); // Only clear if we actually found text to replace it with
                std::stringstream ss(text);
                std::string tok;
                while (std::getline(ss, tok, ',')) {
                    try { out_list.push_back(std::stod(tok)); }
                    catch (...) { /* Ignore bad formats, rely on defaults */ }
                }
            }
        }
    }

    static void loadCartesianAxisParameters(tinyxml2::XMLElement* axis_el, CartesianAxisParams& axis_params) {
        // Controller params now live under a <controller> sub-block, mirroring
        // <planner>/<admittance> nesting. Fall back to direct children for any
        // older config file that has not been migrated yet.
        tinyxml2::XMLElement* ctrl = axis_el->FirstChildElement("controller");
        tinyxml2::XMLElement* ctrl_src = ctrl ? ctrl : axis_el;
        if (auto* e = ctrl_src->FirstChildElement("stiffness")) e->QueryDoubleText(&axis_params.stiffness);
        if (auto* e = ctrl_src->FirstChildElement("diff_effort_max")) e->QueryDoubleText(&axis_params.diff_effort_max);
        parseList(ctrl_src, "error_max", axis_params.error_max);
        parseList(ctrl_src, "force_max", axis_params.force_max);
        parseList(ctrl_src, "saturation_ratios", axis_params.saturation_ratios);
        if (auto* e = ctrl_src->FirstChildElement("n_saturations")) e->QueryIntText(&axis_params.n_saturations);
        if (auto* e = ctrl_src->FirstChildElement("n_neg_sat")) e->QueryIntText(&axis_params.n_neg_sat);
        if (auto* e = ctrl_src->FirstChildElement("asymmetric")) {
            if (const char* t = e->GetText()) {
                std::string s(t);
                axis_params.asymmetric = (s == "true" || s == "1" || s == "yes");
            }
        }

        if (auto* pl = axis_el->FirstChildElement("planner")) {
            if (auto* e = pl->FirstChildElement("stiffness_P")) e->QueryDoubleText(&axis_params.stiffness_P);
            if (auto* e = pl->FirstChildElement("vel_desired")) e->QueryDoubleText(&axis_params.vel_desired);
            if (auto* e = pl->FirstChildElement("acc_max")) e->QueryDoubleText(&axis_params.acc_max);
            if (auto* e = pl->FirstChildElement("damping_ratio")) e->QueryDoubleText(&axis_params.damping_ratio);
            if (auto* e = pl->FirstChildElement("frequency")) e->QueryDoubleText(&axis_params.frequency);
        }
        if (auto* adm = axis_el->FirstChildElement("admittance")) {
            if (auto* e = adm->FirstChildElement("InvM")) e->QueryDoubleText(&axis_params.admittance.InvM);
            if (auto* e = adm->FirstChildElement("Damping")) e->QueryDoubleText(&axis_params.admittance.damping);
            parseList(adm, "dFlow", axis_params.admittance.dFlow);
            parseList(adm, "Flow", axis_params.admittance.flow);
            parseList(adm, "Pose", axis_params.admittance.pose);
        }
    }

    // Find a <mode id="N" profile="P"> element under a robot. Falls back to the
    // "default" profile if the requested profile does not exist for that mode.
    static tinyxml2::XMLElement* findModeElement(tinyxml2::XMLElement* robot,
                                                 int control_mode,
                                                 const std::string& profile_name) {
        tinyxml2::XMLElement* fallback = nullptr;
        for (auto* m = robot->FirstChildElement("mode"); m; m = m->NextSiblingElement("mode")) {
            if (m->IntAttribute("id") != control_mode) continue;
            const char* prof = m->Attribute("profile");
            std::string p = prof ? prof : "default";
            if (p == profile_name) return m;
            if (p == "default") fallback = m;
        }
        return fallback;
    }

    static void loadRobotParametersFromXML(const std::string& robot_name, ParameterType type, size_t id,
                                          ScalarParams& scalars, ListParams& lists, PlannerParams& planner,
                                          AdmittanceParams& admittance, const std::string& xml_file,
                                          CartesianAllParams* cartesian_out, int control_mode,
                                          const std::string& profile_name = "default") {
        tinyxml2::XMLDocument doc;
        // Load failures were previously silent and produced zero-velocity planner defaults
        // that manifested as joints sagging under gravity. Log loudly so the cause is visible.
        if (doc.LoadFile(xml_file.c_str()) != tinyxml2::XML_SUCCESS) {
            std::cerr << "[fic] config.xml load FAILED at " << xml_file
                      << " — planner will use defaults (vel_desired=0)\n";
            return;
        }

        auto* robots = doc.FirstChildElement("robots");
        if (!robots) {
            std::cerr << "[fic] config.xml has no <robots> root at " << xml_file << "\n";
            return;
        }

        for (auto* robot = robots->FirstChildElement("robot"); robot; robot = robot->NextSiblingElement("robot")) {
            if (std::string(robot->Attribute("name")) != robot_name) continue;

            tinyxml2::XMLElement* search_root = robot;

            // 1. DYNAMIC ROUTING
            if (type != ParameterType::Postural) {
                search_root = findModeElement(robot, control_mode, profile_name);
                if (!search_root) {
                    std::cerr << "[fic] no <mode id=\"" << control_mode
                              << "\" profile=\"" << profile_name << "\"> for robot "
                              << robot_name << " — planner defaults (vel_desired=0)\n";
                    return;
                }
            }

            if (type == ParameterType::Cartesian) {
                auto* cart = search_root->FirstChildElement("cartesian");
                if (!cart) return; // No cartesian block in this mode? Safely return.
                
                std::vector<std::string> axes = {"x", "y", "z", "q"};
                std::vector<CartesianAxisParams*> targets = {&cartesian_out->pos_axes[0], &cartesian_out->pos_axes[1], &cartesian_out->pos_axes[2], &cartesian_out->rot_axis};
                for (size_t i = 0; i < axes.size(); ++i) {
                    if (auto* axis_el = cart->FirstChildElement(axes[i].c_str())) {
                        loadCartesianAxisParameters(axis_el, *targets[i]);
                    }
                }
                return;
            } else {
                const char* tag = "joint";
                if (type == ParameterType::Postural) tag = "postural";
                
                auto* el = search_root->FirstChildElement(tag);
                if (!el && type == ParameterType::Postural) el = search_root->FirstChildElement("Postural");
                if (!el) return; // No joints in this mode? Safely return.

                for (; el; el = el->NextSiblingElement(tag)) {
                    if (el->IntAttribute("id") == (int)id || type == ParameterType::Postural) {

                        // Controller params now live under <controller>. Fall
                        // back to direct children so an un-migrated file still
                        // loads (pre-controller-nest layout).
                        tinyxml2::XMLElement* ctrl = el->FirstChildElement("controller");
                        tinyxml2::XMLElement* ctrl_src = ctrl ? ctrl : el;

                        if (auto* e = ctrl_src->FirstChildElement("stiffness")) e->QueryDoubleText(&scalars.stiffness);
                        if (auto* e = ctrl_src->FirstChildElement("diff_effort_max")) e->QueryDoubleText(&scalars.diff_effort_max);

                        parseList(ctrl_src, "error_max", lists.error_max);
                        parseList(ctrl_src, "force_max", lists.force_max);
                        parseList(ctrl_src, "saturation_ratios", lists.saturation_ratios);
                        parseList(ctrl_src, "velocity", lists.velocity);
                        if (auto* e = ctrl_src->FirstChildElement("n_saturations")) e->QueryIntText(&lists.n_saturations);
                        if (auto* e = ctrl_src->FirstChildElement("n_neg_sat")) e->QueryIntText(&lists.n_neg_sat);
                        if (auto* e = ctrl_src->FirstChildElement("asymmetric")) {
                            if (const char* t = e->GetText()) {
                                std::string s(t);
                                lists.asymmetric = (s == "true" || s == "1" || s == "yes");
                            }
                        }

                        if (auto* pl = el->FirstChildElement("planner")) {
                            if (auto* e = pl->FirstChildElement("stiffness")) e->QueryDoubleText(&planner.Plannerstiffness);
                            if (auto* e = pl->FirstChildElement("vel_desired")) e->QueryDoubleText(&planner.vel_desired);
                            if (auto* e = pl->FirstChildElement("acc_max")) e->QueryDoubleText(&planner.acc_max);
                            if (auto* e = pl->FirstChildElement("damping_ratio")) e->QueryDoubleText(&planner.damping_ratio);
                            if (auto* e = pl->FirstChildElement("frequency")) e->QueryDoubleText(&planner.frequency);
                        }

                        if (auto* adm = el->FirstChildElement("admittance")) {
                            if (auto* e = adm->FirstChildElement("InvM")) e->QueryDoubleText(&admittance.InvM);
                            if (auto* e = adm->FirstChildElement("Damping")) e->QueryDoubleText(&admittance.damping);
                            parseList(adm, "dFlow", admittance.dFlow);
                            parseList(adm, "Flow", admittance.flow);
                            parseList(adm, "Pose", admittance.pose);
                        }
                        return;
                    }
                }
            }
        }
    }
};

// --- 3. A whole tuning set, read in one go ---
// Reading config.xml takes tens of ms (one parse per joint plus a Cartesian
// pass), far longer than a 1 ms control tick. The nodes therefore read the
// tuning once into a TuningSet, off the control thread, and keep using the
// set they have until a new one is ready; re-seeding a controller for a new
// target only copies from the set already in memory.

// What the set was read for. A different key means a different controller
// structure (robot, joint count, mode), so a set for an old key cannot be used.
struct TuningKey {
    std::string robot;
    size_t n_joints = 0;
    int joint_mode = 0;     // control_mode passed to loadJointParams
    int cart_mode = 0;      // control_mode passed to loadCartesianParams
    bool postural = false;  // also read the null-space postural block (at cart_mode)

    bool operator==(const TuningKey& o) const {
        return robot == o.robot && n_joints == o.n_joints && joint_mode == o.joint_mode &&
               cart_mode == o.cart_mode && postural == o.postural;
    }
    bool operator!=(const TuningKey& o) const { return !(*this == o); }
};

// H-PMP (switched on in the Cartesian modes 6-8) has a tuning block of its own,
// <mode id="100" name="H-PMP">: its joint layer in <joint> (FIC controller, planner and
// admittance per joint) and its Cartesian layer in <cartesian> (FIC controller and planner
// per axis). 100 is not a control mode: no node is ever put in it. A robot with no H-PMP
// block runs H-PMP on mode 4's joints and mode 6's Cartesian block, as it did before the
// block existed (see loadTuningSet).
constexpr int kHpmpTuningMode = 100;
constexpr int kHpmpFallbackJointMode = 4;
constexpr int kHpmpFallbackCartMode = 6;

// H-PMP runs when it is switched on in a Cartesian mode (5-8): on mode 5
// its task layer tracks the goal without its planner.
inline bool hpmpActive(int control_mode, bool hpmp_enabled) {
    return hpmp_enabled && control_mode >= 5 && control_mode <= 8;
}

// The tuning a node reads for a control mode. fic_simulation and fic_hardware both take it
// from here, so every mode, H-PMP included, reads the same blocks in simulation and on the
// robot. `mcu`: fic_hardware's inner loop on the MCU, which runs every Cartesian mode on the
// joint admittance-haptic layer, so it needs mode 4's joints.
inline TuningKey tuningKeyFor(const std::string& robot, size_t n_joints, int control_mode,
                              bool hpmp_enabled, bool mcu = false) {
    TuningKey k;
    k.robot = robot;
    k.n_joints = n_joints;
    if (hpmpActive(control_mode, hpmp_enabled)) {
        k.joint_mode = k.cart_mode = kHpmpTuningMode;
    } else {
        // mode 9 (Cartesian IK, simulation only) drives mode 1's joint impedance
        k.joint_mode = (mcu && control_mode > 4) ? 4 : (control_mode == 9 ? 1 : control_mode);
        k.cart_mode = control_mode;
    }
    k.postural = control_mode >= 5 && control_mode <= 8;   // the robot-wide null-space block
    return k;
}

struct TuningSet {
    TuningKey key;
    std::string profile;
    bool hpmp_fallback = false;   // a H-PMP set read from modes 4 and 6: the robot has no H-PMP block
    std::vector<ScalarParams> joint_scalars;
    std::vector<ListParams> joint_lists;
    std::vector<PlannerParams> joint_planners;
    std::vector<AdmittanceParams> joint_admittance;
    CartesianAllParams cart;
    ScalarParams postural_scalars;
    ListParams postural_lists;
    PlannerParams postural_planner;
    AdmittanceParams postural_admittance;
};

inline TuningSet loadTuningSet(const std::string& xml_file, const TuningKey& key, const std::string& profile) {
    TuningSet t;
    t.key = key;
    t.profile = profile;
    t.joint_scalars.resize(key.n_joints);
    t.joint_lists.resize(key.n_joints);
    t.joint_planners.resize(key.n_joints);
    t.joint_admittance.resize(key.n_joints);
    int joint_mode = key.joint_mode, cart_mode = key.cart_mode;
    if ((joint_mode == kHpmpTuningMode || cart_mode == kHpmpTuningMode) &&
        !ParameterLoader::hasModeElement(xml_file, key.robot, kHpmpTuningMode, profile)) {
        if (joint_mode == kHpmpTuningMode) joint_mode = kHpmpFallbackJointMode;
        if (cart_mode == kHpmpTuningMode) cart_mode = kHpmpFallbackCartMode;
        t.hpmp_fallback = true;
    }
    for (size_t i = 0; i < key.n_joints; ++i) {
        ParameterLoader::loadJointParams(xml_file, key.robot, i,
            t.joint_scalars[i], t.joint_lists[i], t.joint_planners[i], t.joint_admittance[i],
            joint_mode, false, profile);
    }
    ParameterLoader::loadCartesianParams(xml_file, key.robot, t.cart, cart_mode, profile);
    if (key.postural) {
        ParameterLoader::loadJointParams(xml_file, key.robot, 0,
            t.postural_scalars, t.postural_lists, t.postural_planner, t.postural_admittance,
            cart_mode, /*is_postural=*/true, profile);
    }
    return t;
}

} // namespace fic

#endif