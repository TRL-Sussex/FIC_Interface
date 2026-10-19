#ifndef FIC_MCU_LINK_HPP
#define FIC_MCU_LINK_HPP

// Linux end of FIC-link (fic_link.h): a UDP client for an FIC inner-loop MCU.
//
// fic_control owns one of these. probe() is how the system decides, per robot
// load, whether an MCU is there and can drive this robot; everything after that
// is fire-and-forget sends from the control loop plus a receive thread that
// keeps the latest STATE. Nothing here needs real-time scheduling — lateness on
// this side only slows the outer loops, the MCU keeps the arm stable.

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <mutex>
#include <optional>
#include <random>
#include <string>
#include <thread>

#include "fic_link.h"

namespace fic {

class McuLink {
public:
    using Clock = std::chrono::steady_clock;

    McuLink() = default;
    McuLink(const McuLink&) = delete;
    McuLink& operator=(const McuLink&) = delete;
    ~McuLink() { close(); }

    // "host" or "host:port". Resolves, opens a connected UDP socket and starts
    // the receive thread. Returns false (with lastError()) on failure.
    bool open(const std::string& address) {
        close();
        std::string host = address;
        uint16_t port = fic::link::kDefaultMcuPort;
        const auto colon = address.rfind(':');
        if (colon != std::string::npos) {
            host = address.substr(0, colon);
            try { port = static_cast<uint16_t>(std::stoi(address.substr(colon + 1))); }
            catch (...) { error_ = "bad port in '" + address + "'"; return false; }
        }
        addrinfo hints{};
        hints.ai_family = AF_INET;
        hints.ai_socktype = SOCK_DGRAM;
        addrinfo* res = nullptr;
        if (getaddrinfo(host.c_str(), std::to_string(port).c_str(), &hints, &res) != 0 || !res) {
            error_ = "cannot resolve '" + host + "'";
            return false;
        }
        fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (fd_ < 0) { freeaddrinfo(res); error_ = std::strerror(errno); return false; }
        const bool ok = ::connect(fd_, res->ai_addr, res->ai_addrlen) == 0;
        freeaddrinfo(res);
        if (!ok) { error_ = std::strerror(errno); ::close(fd_); fd_ = -1; return false; }
        timeval tv{0, 100000};   // 100 ms: lets the rx thread notice close()
        setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        address_ = host + ":" + std::to_string(port);
        running_ = true;
        rx_ = std::thread([this] { rxLoop(); });
        return true;
    }

    void close() {
        running_ = false;
        if (rx_.joinable()) rx_.join();
        if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
        std::lock_guard<std::mutex> l(m_);
        have_state_ = false;
        hello_.reset();
    }

    bool isOpen() const { return fd_ >= 0; }
    const std::string& address() const { return address_; }
    const std::string& lastError() const { return error_; }

    // HELLO → HELLO_ACK. Returns the MCU's capabilities, or nullopt if nothing
    // answered within timeout_ms (split over `attempts` sends).
    std::optional<fic::link::HelloAck> probe(int timeout_ms = 300, int attempts = 3) {
        if (fd_ < 0) return std::nullopt;
        std::mt19937 rng(static_cast<uint32_t>(Clock::now().time_since_epoch().count()));
        fic::link::Hello h{static_cast<uint32_t>(rng())};
        {
            std::lock_guard<std::mutex> l(m_);
            hello_.reset();
            hello_nonce_ = h.nonce;
        }
        const auto per_try = std::chrono::milliseconds(std::max(1, timeout_ms / std::max(1, attempts)));
        for (int a = 0; a < attempts; ++a) {
            sendFixed(fic::link::MsgType::Hello, h);
            std::unique_lock<std::mutex> l(m_);
            if (cv_.wait_for(l, per_try, [this] { return hello_.has_value(); })) return hello_;
        }
        return std::nullopt;
    }

    bool sendConfig(const fic::link::Config& c) { return sendFixed(fic::link::MsgType::Config, c); }
    bool sendParams(const fic::link::Params& p) { return sendFixed(fic::link::MsgType::Params, p); }
    // The topic plant (a simulated arm that cannot step in lockstep): this link is the plant too.
    bool sendSimStep(const fic::link::SimStepMsg& m) {
        uint8_t buf[fic::link::kMaxDatagram];
        const size_t n = fic::link::encodeSimStep(m, ++tx_seq_, buf, sizeof(buf));
        return n && sendRaw(buf, n);
    }
    bool latestSimTorque(fic::link::SimTorqueMsg& out, double& age_s) const {
        std::lock_guard<std::mutex> l(m_);
        if (!have_sim_torque_) return false;
        out = sim_torque_;
        age_s = std::chrono::duration<double>(Clock::now() - sim_torque_time_).count();
        return true;
    }

    // E-STOP straight to the inner loop (latched there like a CMD's e-stop flag; no CONFIG needed).
    bool sendEstop() { return sendFixed(fic::link::MsgType::Estop, fic::link::EstopMsg{1}); }

    bool sendCmd(const fic::link::CmdMsg& m) {
        uint8_t buf[fic::link::kMaxDatagram];
        const size_t n = fic::link::encodeCmd(m, ++tx_seq_, buf, sizeof(buf));
        return n && sendRaw(buf, n);
    }

    // Latest STATE and its age. False until the first STATE arrives.
    bool latestState(fic::link::StateMsg& out, double& age_s) const {
        std::lock_guard<std::mutex> l(m_);
        if (!have_state_) return false;
        out = state_;
        age_s = std::chrono::duration<double>(Clock::now() - state_time_).count();
        return true;
    }

    // STATE packets per second, over the last ~1 s window.
    double stateRateHz() const { return rate_hz_.load(); }
    uint32_t lastTxSeq() const { return tx_seq_; }

private:
    int fd_ = -1;
    std::string address_;
    std::string error_;
    std::atomic<bool> running_{false};
    std::thread rx_;
    std::atomic<uint32_t> tx_seq_{0};

    mutable std::mutex m_;
    std::condition_variable cv_;
    uint32_t hello_nonce_ = 0;
    std::optional<fic::link::HelloAck> hello_;
    fic::link::StateMsg state_{};
    bool have_state_ = false;
    fic::link::SimTorqueMsg sim_torque_{};
    Clock::time_point sim_torque_time_{};
    bool have_sim_torque_ = false;
    Clock::time_point state_time_{};
    std::atomic<double> rate_hz_{0.0};

    template <typename V>
    bool sendFixed(fic::link::MsgType t, const V& v) {
        uint8_t buf[fic::link::kMaxDatagram];
        const size_t n = fic::link::encodeFixed(t, v, ++tx_seq_, buf, sizeof(buf));
        return n && sendRaw(buf, n);
    }

    bool sendRaw(const uint8_t* buf, size_t n) {
        if (fd_ < 0) return false;
        // Never block the control loop; a dropped datagram is superseded next tick.
        return ::send(fd_, buf, n, MSG_DONTWAIT) == static_cast<ssize_t>(n);
    }

    void rxLoop() {
        uint8_t buf[2048];
        uint32_t count = 0;
        auto window = Clock::now();
        while (running_) {
            const ssize_t n = ::recv(fd_, buf, sizeof(buf), 0);
            const auto now = Clock::now();
            if (now - window >= std::chrono::seconds(1)) {
                rate_hz_ = count / std::chrono::duration<double>(now - window).count();
                count = 0;
                window = now;
            }
            if (n <= 0) continue;
            fic::link::Reader r;
            if (!r.open(buf, static_cast<size_t>(n))) continue;
            if (r.type() == fic::link::MsgType::State) {
                fic::link::StateMsg s;
                if (!fic::link::decodeState(r, s)) continue;
                std::lock_guard<std::mutex> l(m_);
                state_ = s;
                state_time_ = now;
                have_state_ = true;
                ++count;
            } else if (r.type() == fic::link::MsgType::SimTorque) {
                fic::link::SimTorqueMsg t;
                if (!fic::link::decodeSimTorque(r, t)) continue;
                std::lock_guard<std::mutex> l(m_);
                sim_torque_ = t;
                sim_torque_time_ = now;
                have_sim_torque_ = true;
            } else if (r.type() == fic::link::MsgType::HelloAck) {
                fic::link::HelloAck a;
                if (!fic::link::decodeFixed(r, a)) continue;
                std::lock_guard<std::mutex> l(m_);
                if (a.nonce == hello_nonce_) {
                    a.fw_name[sizeof(a.fw_name) - 1] = '\0';
                    hello_ = a;
                    cv_.notify_all();
                }
            }
        }
    }
};

} // namespace fic

#endif // FIC_MCU_LINK_HPP
