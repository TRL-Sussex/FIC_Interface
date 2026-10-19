// Read-only: the arm's state and every device's safety status (which actuator tripped on what, latched until
// cleared). Commands nothing; uses only the TCP API (no real-time session, no servoing change).
//
//   safety_read [--ip 192.168.1.10]

#include <iostream>
#include <string>

#include <BaseClientRpc.h>
#include <DeviceConfigClientRpc.h>
#include <DeviceManagerClientRpc.h>
#include <KDetailedException.h>
#include <RouterClient.h>
#include <SessionManager.h>
#include <TransportClientTcp.h>
#include <ActuatorConfig.pb.h>

namespace k_api = Kinova::Api;

int main(int argc, char** argv) {
    std::string ip = "192.168.1.10";
    for (int i = 1; i + 1 < argc; ++i) if (std::string(argv[i]) == "--ip") ip = argv[i + 1];
    auto transport = new k_api::TransportClientTcp();
    auto router = new k_api::RouterClient(transport, [](k_api::KError e) { std::cout << "error: " << e.toString(); });
    transport->connect(ip, 10000);
    auto info = k_api::Session::CreateSessionInfo();
    info.set_username("admin");
    info.set_password("admin");
    info.set_session_inactivity_timeout(60000);
    info.set_connection_inactivity_timeout(2000);
    auto session = new k_api::SessionManager(router);
    int rc = 0;
    try {
        session->CreateSession(info);
        k_api::Base::BaseClient base(router);
        const auto st = base.GetArmState();
        std::cout << "arm state: " << k_api::Common::ArmState_Name(st.active_state()) << "\n";
        k_api::DeviceManager::DeviceManagerClient dm(router);
        k_api::DeviceConfig::DeviceConfigClient dc(router);
        const auto devs = dm.ReadAllDevices();
        for (const auto& d : devs.device_handle()) {
            const bool actuator = d.device_type() == k_api::Common::BIG_ACTUATOR ||
                                  d.device_type() == k_api::Common::MEDIUM_ACTUATOR ||
                                  d.device_type() == k_api::Common::SMALL_ACTUATOR;
            const auto list = dc.GetAllSafetyInformation(d.device_identifier());
            int shown = 0;
            for (const auto& s : list.information()) {
                if (s.status() == k_api::Common::SAFETY_STATUS_NORMAL) continue;
                const uint32_t id = s.handle().identifier();
                std::string name = actuator && k_api::ActuatorConfig::SafetyIdentifierBankA_IsValid(id)
                                       ? k_api::ActuatorConfig::SafetyIdentifierBankA_Name(
                                             static_cast<k_api::ActuatorConfig::SafetyIdentifierBankA>(id))
                                       : "id " + std::to_string(id);
                std::cout << "  device " << d.device_identifier() << " (" << k_api::Common::DeviceTypes_Name(d.device_type())
                          << ", order " << d.order() << "): " << name << " -> "
                          << k_api::Common::SafetyStatusValue_Name(s.status()) << "\n";
                ++shown;
            }
            if (!shown)
                std::cout << "  device " << d.device_identifier() << " (" << k_api::Common::DeviceTypes_Name(d.device_type())
                          << ", order " << d.order() << "): all " << list.information_size() << " safeties normal\n";
        }
    } catch (k_api::KDetailedException& e) {
        std::cout << "API error: " << e.what() << "\n";
        rc = 1;
    }
    session->CloseSession();
    router->SetActivationStatus(false);
    transport->disconnect();
    return rc;
}
