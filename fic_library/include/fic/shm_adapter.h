#ifndef FIC_SHM_ADAPTER_H
#define FIC_SHM_ADAPTER_H

// The PC inner loop's adapter (fic_inner_loop, Adapter::Shm): a vendor driver's shared memory
// (fic/shared_params.h) in place of an arm on the MCU's network. The driver writes the arm's position and
// torque and applies commanded_torque at its own rate; this adapter turns that into the inner loop's
// ArmFeedback / ArmCommand, keeping the contract the drivers already rely on:
//
//   * nothing commanded (NaN) until the inner loop torques the arm: a Kinova driver waits in position mode
//     for the first finite command before it enters torque mode;
//   * after torque, the last torque is held (libfranka holds the last command anyway and ignores the stop
//     flag), and as soon as the inner loop no longer torques the arm (its ramp is over, or a fault / a new
//     CONFIG stopped it) stop_torque_mode = 1, so a Kinova driver leaves torque mode; it is cleared when
//     torque is wanted again;
//   * attach only: never initSharedMemory() on an existing segment (it would reset the driver's fields);
//   * a driver that leaves (alive -> 0) makes the feedback invalid and the commands NaN again, so a driver
//     that comes back never picks up an old torque; a recreated segment (libfranka's reflex recovery) is
//     followed by checkRemap().
//
// Linux only (POSIX shared memory); the inner loop itself stays portable.

#include <cmath>
#include <limits>

#include <sys/mman.h>

#include "inner_loop.h"
#include "shared_params.h"

namespace fic {
namespace mcu {

class ShmAdapter {
public:
    // seg: a block to use as it is (tests); nullptr: map the feedback segment in attach().
    explicit ShmAdapter(SharedFeedback* seg = nullptr) : seg_(seg), own_(seg == nullptr) {}
    ~ShmAdapter() { unmap(); }
    ShmAdapter(const ShmAdapter&) = delete;
    ShmAdapter& operator=(const ShmAdapter&) = delete;

    bool attach() {
        if (own_) {
            unmap();
            seg_ = fic::attachFeedbackSegment(&inode_);
        }
        resetDriverState();
        adoptRunningTorque();
        return seg_ != nullptr;
    }

    // The segment was recreated under the same name (a driver's cleanup + init): map the new one.
    // Cheap enough to call every few hundred ms.
    void checkRemap() {
        if (!own_) return;
        const ino_t now = fic::feedbackSegmentInode();
        if (now != 0 && now != inode_) attach();
    }

    // The driver is running and has written a real sample since it (re)started.
    bool ready() const { return seg_ && alive_ && sampled_; }

    // Feedback unchanged (bit for bit, every joint) for this many reads is frozen: a hung or killed driver
    // whose `alive` stayed 1. Real sensors are never bit-steady; 200 ms leaves room for a 100 Hz bridge.
    static constexpr int kFrozenTicks = 200;

    ArmFeedback read(int n) {
        poll(n);
        ArmFeedback fb;
        if (!ready()) return fb;                       // valid = false
        const int m = n < static_cast<int>(kMaxJoints) ? n : static_cast<int>(kMaxJoints);
        bool all_zero = true, same = true;
        for (int i = 0; i < m; ++i) {
            fb.q[i] = seg_->joint_positions[i].load(std::memory_order_acquire);
            fb.tau[i] = seg_->torque[i].load(std::memory_order_acquire);
            all_zero = all_zero && fb.q[i] == 0.0f;
            same = same && fb.q[i] == prev_q_[i] && fb.tau[i] == prev_tau_[i];
            prev_q_[i] = fb.q[i];
            prev_tau_[i] = fb.tau[i];
        }
        if (all_zero) {                                // the driver re-initialised the segment
            resetDriverState();
            return ArmFeedback{};
        }
        frozen_ = same ? frozen_ + 1 : 0;
        if (frozen_ >= kFrozenTicks) return ArmFeedback{};   // frozen: invalid (the loop faults after 50)
        fb.valid = true;
        for (int k = 0; k < 6; ++k) fb.wrench[k] = seg_->tool_wrench[k].load(std::memory_order_acquire);
        if (n < static_cast<int>(kGripperFeedbackSlot)) {
            fb.gripper_pos = seg_->joint_positions[kGripperFeedbackSlot].load(std::memory_order_acquire);
            fb.gripper_current = seg_->torque[kGripperFeedbackSlot].load(std::memory_order_acquire);
        }
        return fb;
    }

    // released: the inner loop has finished its ramp (InnerLoop::released()). idle_tau (optional): what to
    // apply once the inner loop no longer torques the arm (its gravity at the current pose: the old idle
    // baseline, for a driver that stays in torque mode); without it the last torque is held.
    void write(const ArmCommand& c, int n, bool released, const float* idle_tau = nullptr) {
        if (!seg_) return;
        const int m = n < static_cast<int>(kMaxJoints) ? n : static_cast<int>(kMaxJoints);
        // NaN back in the commands after we torqued: the driver re-initialised the segment. Never write a
        // torque over it; wait for its first sample again.
        if (torqued_ && !(seg_->commanded_torque[0].load(std::memory_order_acquire) ==
                          seg_->commanded_torque[0].load(std::memory_order_acquire)))
            resetDriverState();
        if (c.torque_mode && ready()) {
            for (int i = 0; i < m; ++i) {
                last_[i] = c.tau[i];
                seg_->commanded_torque[i].store(c.tau[i], std::memory_order_release);
            }
            torqued_ = true;
            if (stop_sent_) {
                seg_->stop_torque_mode.store(0, std::memory_order_release);
                stop_sent_ = false;
            }
        } else if (!torqued_) {
            const float nan = std::numeric_limits<float>::quiet_NaN();
            for (int i = 0; i < m; ++i) seg_->commanded_torque[i].store(nan, std::memory_order_release);
        } else {
            // Not torquing any more, after torque: a finished ramp (released), or a fault or a new CONFIG.
            // Either way the driver is told to leave torque mode now, with the last torque held meanwhile
            // (a driver that ignores the flag, libfranka, holds it anyway).
            for (int i = 0; i < m; ++i) {
                if (idle_tau) last_[i] = idle_tau[i];
                seg_->commanded_torque[i].store(last_[i], std::memory_order_release);
            }
            if (!stop_sent_) {
                seg_->stop_torque_mode.store(1, std::memory_order_release);
                stop_sent_ = true;
            }
            (void)released;
        }
        if (c.gripper_width >= 0.0f)
            seg_->commanded_gripper_width.store(c.gripper_width, std::memory_order_release);
        std::atomic_thread_fence(std::memory_order_release);
    }

private:
    SharedFeedback* seg_ = nullptr;
    bool own_ = false;
    ino_t inode_ = 0;
    bool alive_ = false, sampled_ = false, torqued_ = false, stop_sent_ = false;
    float last_[kMaxJoints] = {};
    float prev_q_[kMaxJoints] = {}, prev_tau_[kMaxJoints] = {};
    int frozen_ = 0;

    void resetDriverState() {
        alive_ = sampled_ = torqued_ = stop_sent_ = false;
        frozen_ = 0;
        for (uint32_t i = 0; i < kMaxJoints; ++i) last_[i] = prev_q_[i] = prev_tau_[i] = 0.0f;
    }

    // Attaching while a driver applies finite commands (the inner loop restarted under a torquing arm): take
    // them over as held torque, so the next write hands the arm back with the stop flag instead of NaN.
    void adoptRunningTorque() {
        if (!seg_ || seg_->alive.load(std::memory_order_acquire) != 1) return;
        for (uint32_t i = 0; i < kMaxJoints; ++i) {
            const float t = seg_->commanded_torque[i].load(std::memory_order_acquire);
            if (!(t == t)) {
                if (i == 0) return;                    // NaN: nothing commanded
                break;
            }
            last_[i] = t;
        }
        torqued_ = true;
    }

    // alive and the first-sample latch: the zero fill of initSharedMemory() is not a sample.
    void poll(int n) {
        if (!seg_) return;
        const bool alive = seg_->alive.load(std::memory_order_acquire) == 1;
        if (!alive) {
            if (alive_) resetDriverState();            // the driver left: NaN again, wait for a new first sample
            return;
        }
        alive_ = true;
        if (!sampled_) {
            const int m = n < static_cast<int>(kMaxJoints) ? n : static_cast<int>(kMaxJoints);
            for (int i = 0; i < m && !sampled_; ++i)
                sampled_ = seg_->joint_positions[i].load(std::memory_order_acquire) != 0.0f;
        }
    }

    void unmap() {
        if (own_ && seg_) munmap(seg_, sizeof(SharedFeedback));
        if (own_) seg_ = nullptr;
    }
};

}  // namespace mcu
}  // namespace fic

#endif  // FIC_SHM_ADAPTER_H
