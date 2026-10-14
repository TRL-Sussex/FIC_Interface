#ifndef SHARED_XYZ_H
#define SHARED_XYZ_H

#include <atomic>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <cerrno>
#include <cstdint>
#include <new>        // placement new
#include <limits>
#include <iostream>
#include <algorithm>

// Shared memory object names
#define SHARED_FEEDBACK_NAME        "/robot_feedback_memory"
#define SHARED_DESIRED_JOINTS_NAME  "/desired_joint_angles_memory"

// --------------------------------------
// Sizing & versioning
// --------------------------------------
// Compile-time cap on how many joints the shared-memory structs reserve. The
// runtime n_joints field below says how many of those slots are active.
// Bumping this requires all processes that map the segments (fic_hardware +
// every vendor driver bridge) to be rebuilt against the new header.
//
// 16 comfortably covers every arm we support today (7-DOF Franka / Kinova /
// KUKA) and leaves headroom for dual-arm or 14-DOF platforms without being
// profligate (struct is ~400 bytes, a rounding error in a mmap page).
inline constexpr uint32_t kMaxJoints = 16;

// Bump this when SharedFeedback / SharedDesiredJointState layout changes.
// A mismatched version (or mismatched mapped size) triggers a recreate so
// upgraded processes don't silently alias old segments.
inline constexpr uint32_t kShmLayoutVersion = 3;
inline constexpr uint32_t kShmMagic = 0x46494353u;  // "FICS"

// --------------------------------------
// Feedback: flags + current/commanded IO
// --------------------------------------
struct SharedFeedback {
  // Versioning header — must stay first so we can sanity-check a mapped segment
  // without dereferencing past its actual size.
  std::atomic<uint32_t> magic{kShmMagic};
  std::atomic<uint32_t> version{kShmLayoutVersion};
  std::atomic<uint32_t> n_joints{0};          // active joint count (written by fic_hardware)
  std::atomic<uint32_t> alive{0};             // 1 while writer is running

  std::atomic<float> torque[kMaxJoints];            // measured joint torques
  std::atomic<float> joint_positions[kMaxJoints];   // measured joint positions
  std::atomic<float> commanded_torque[kMaxJoints];  // commanded torques (writer sets, reader consumes)
  std::atomic<float> tool_wrench[6];                // {Fx,Fy,Fz, Tx,Ty,Tz}

  std::atomic<float> commanded_gripper_width;       // desired width in meters

  // fic_hardware sets this to 1 after torque ramp-down completes, telling the
  // driver to exit torque mode and switch back to position/single-level servoing.
  std::atomic<uint32_t> stop_torque_mode{0};
};

// --------------------------------------
// Measured gripper state — rides in the last, otherwise unused joint slot
// --------------------------------------
// A driver bridge with a gripper writes the gripper's MEASURED state here:
//
//   joint_positions[kGripperFeedbackSlot]  measured finger opening in metres,
//                                          same convention as
//                                          commanded_gripper_width: 0 = closed,
//                                          ~0.085 = fully open (Robotiq 2F-85)
//   torque[kGripperFeedbackSlot]           gripper motor current, as the
//                                          vendor API reports it (Kinova: mA)
//
// Fingers that stop short of the commanded width while the current stays up
// mean the gripper has closed on something.
//
// Existing fields are reused, so sizeof(SharedFeedback) and kShmLayoutVersion
// do NOT change: bridges built against the older header keep working, they
// just never write the slot. Only defined for robots with fewer than
// kGripperFeedbackSlot joints; on a bigger robot the slot is a real joint and
// must never be read as a gripper.
//
// "No gripper feedback" sentinel: initSharedMemory() / cleanupSharedMemory()
// zero every slot and a bridge without a gripper never writes this one, so a
// bit-exact (0.0f, 0.0f) pair means "no gripper feedback". Readers must ignore
// the slot until they have once seen a pair that is not both 0.0f, and then
// latch: afterwards (0.0f, 0.0f) is a genuine reading (a fully closed gripper
// reads width 0; its current is normally nonzero, but an idle motor may
// report 0). Re-arm the latch when `alive` drops to 0 — a bridge's
// cleanupSharedMemory() does that as it exits, zeroing the slot again.
inline constexpr uint32_t kGripperFeedbackSlot = kMaxJoints - 1;

// Desired joint angles (set by ROS 2)
struct SharedDesiredJointState {
  std::atomic<uint32_t> magic{kShmMagic};
  std::atomic<uint32_t> version{kShmLayoutVersion};
  std::atomic<uint32_t> n_joints{0};
  std::atomic<float> desired_joint_angles[kMaxJoints];
};

// Global pointers (inline so header can be included in multiple TUs)
inline SharedFeedback* shared_feedback       = nullptr;
inline SharedDesiredJointState* shared_desired_joints = nullptr;
// Track whether this process created the segments (so only we unlink)
inline bool g_feedback_creator = false;
inline bool g_desired_creator  = false;

// Helper: create-or-open a shm segment of the expected size. If an existing
// segment has a different byte-size (layout change across an upgrade), it's
// unlinked and recreated — otherwise we'd either mmap short and SIGBUS on
// access past the old bounds, or silently read a struct the other end never
// wrote.
inline int open_or_create(const char* name, bool& created, size_t bytes) {
  created = false;

  // Try exclusive create first.
  int fd = shm_open(name, O_CREAT | O_EXCL | O_RDWR, 0666);
  if (fd >= 0) {
    created = true;
    if (ftruncate(fd, static_cast<off_t>(bytes)) != 0) {
      close(fd);
      shm_unlink(name);
      return -1;
    }
    return fd;
  }
  if (errno != EEXIST) return -1;

  // Segment exists. Check size; recreate if stale.
  fd = shm_open(name, O_RDWR, 0666);
  if (fd < 0) return -1;

  struct stat st{};
  if (fstat(fd, &st) == 0 && static_cast<size_t>(st.st_size) != bytes) {
    std::cerr << "[shared_params] existing segment " << name
              << " has size " << st.st_size << " bytes, expected " << bytes
              << " — recreating. If another process is still mapped, restart it.\n";
    close(fd);
    shm_unlink(name);
    fd = shm_open(name, O_CREAT | O_EXCL | O_RDWR, 0666);
    if (fd < 0) return -1;
    created = true;
    if (ftruncate(fd, static_cast<off_t>(bytes)) != 0) {
      close(fd);
      shm_unlink(name);
      return -1;
    }
  }
  return fd;
}

// --------------------------------------
// Initialization of shared memory
// --------------------------------------
inline void initSharedMemory() {
  // --- Feedback ---
  int fd1 = open_or_create(SHARED_FEEDBACK_NAME, g_feedback_creator, sizeof(SharedFeedback));
  if (fd1 < 0) { std::cerr << "Failed to open/create " << SHARED_FEEDBACK_NAME << "\n"; return; }

  void* p1 = mmap(nullptr, sizeof(SharedFeedback), PROT_READ | PROT_WRITE, MAP_SHARED, fd1, 0);
  close(fd1);
  if (p1 == MAP_FAILED) { std::cerr << "Failed to mmap feedback\n"; return; }

  shared_feedback = static_cast<SharedFeedback*>(p1);
  if (g_feedback_creator) {
    // Value-init atomics safely
    new (shared_feedback) SharedFeedback{};
  } else {
    // Sanity check: if a peer process created this segment with a stale layout
    // version (shouldn't happen after the size-guard in open_or_create, but
    // belt-and-braces), warn loudly.
    const uint32_t m = shared_feedback->magic.load(std::memory_order_acquire);
    const uint32_t v = shared_feedback->version.load(std::memory_order_acquire);
    if (m != kShmMagic || v != kShmLayoutVersion) {
      std::cerr << "[shared_params] feedback segment magic/version mismatch "
                   "(got magic=" << m << " version=" << v
                << ", expected magic=" << kShmMagic
                << " version=" << kShmLayoutVersion << "). "
                   "Stop all processes using the shared memory and delete "
                   "/dev/shm/robot_feedback_memory before restarting.\n";
    }
  }

  // Reset fields for a clean session. Zero the entire reserved range — cheap
  // and avoids leaving stale values in slots the current robot doesn't use.
  for (uint32_t i = 0; i < kMaxJoints; ++i) {
    shared_feedback->torque[i].store(0.0f, std::memory_order_relaxed);
    shared_feedback->joint_positions[i].store(0.0f, std::memory_order_relaxed);
    // NaN sentinel so readers can detect "no commanded torque yet"
    shared_feedback->commanded_torque[i].store(std::numeric_limits<float>::quiet_NaN(),
                                               std::memory_order_relaxed);
  }
  for (int i = 0; i < 6; ++i) {
    shared_feedback->tool_wrench[i].store(0.0f, std::memory_order_relaxed);
  }

  // Fully open gripper on startup so it doesn't snap shut before commands arrive
  shared_feedback->commanded_gripper_width.store(0.08f, std::memory_order_relaxed);

  shared_feedback->stop_torque_mode.store(0, std::memory_order_relaxed);

  shared_feedback->alive.store(1, std::memory_order_release);

  // --- Desired joints ---
  int fd2 = open_or_create(SHARED_DESIRED_JOINTS_NAME, g_desired_creator, sizeof(SharedDesiredJointState));
  if (fd2 < 0) { std::cerr << "Failed to open/create " << SHARED_DESIRED_JOINTS_NAME << "\n"; return; }

  void* p2 = mmap(nullptr, sizeof(SharedDesiredJointState), PROT_READ | PROT_WRITE, MAP_SHARED, fd2, 0);
  close(fd2);
  if (p2 == MAP_FAILED) { std::cerr << "Failed to mmap desired_joints\n"; return; }

  shared_desired_joints = static_cast<SharedDesiredJointState*>(p2);
  if (g_desired_creator) {
    new (shared_desired_joints) SharedDesiredJointState{};
  }
  for (uint32_t i = 0; i < kMaxJoints; ++i) {
    shared_desired_joints->desired_joint_angles[i].store(0.0f, std::memory_order_relaxed);
  }
}

// --------------------------------------
// Tell readers how many joints the writer actually drives. Call from the
// hardware node once num_joints_ is known; drivers read this to know how many
// torque/position slots are meaningful. Safe to call more than once.
// --------------------------------------
inline void publishJointCount(uint32_t n) {
  if (n > kMaxJoints) {
    std::cerr << "[shared_params] requested n_joints=" << n
              << " exceeds kMaxJoints=" << kMaxJoints
              << ". Clamping. Bump kMaxJoints in shared_params.h and rebuild.\n";
    n = kMaxJoints;
  }
  if (shared_feedback)       shared_feedback->n_joints.store(n, std::memory_order_release);
  if (shared_desired_joints) shared_desired_joints->n_joints.store(n, std::memory_order_release);
}

// Read the active joint count (0 = writer hasn't published yet).
inline uint32_t readJointCount() {
  if (!shared_feedback) return 0;
  return shared_feedback->n_joints.load(std::memory_order_acquire);
}

// --------------------------------------
// Cleanup shared memory
// --------------------------------------
inline void cleanupSharedMemory() {
  if (shared_feedback) {
    // Tell readers the writer is gone, and clear values
    shared_feedback->alive.store(0, std::memory_order_release);
    for (uint32_t i = 0; i < kMaxJoints; ++i) {
      shared_feedback->torque[i].store(0.0f, std::memory_order_relaxed);
      shared_feedback->joint_positions[i].store(0.0f, std::memory_order_relaxed);
      shared_feedback->commanded_torque[i].store(0.0f, std::memory_order_relaxed);
    }
    for (int i = 0; i < 6; ++i) {
      shared_feedback->tool_wrench[i].store(0.0f, std::memory_order_relaxed);
    }

    // Clear gripper command and stop flag
    shared_feedback->commanded_gripper_width.store(0.0f, std::memory_order_relaxed);
    shared_feedback->stop_torque_mode.store(0, std::memory_order_relaxed);

    std::atomic_thread_fence(std::memory_order_release);

    munmap(shared_feedback, sizeof(SharedFeedback));
    shared_feedback = nullptr;

    if (g_feedback_creator) {
      shm_unlink(SHARED_FEEDBACK_NAME);
    }
  }

  if (shared_desired_joints) {
    for (uint32_t i = 0; i < kMaxJoints; ++i) {
      shared_desired_joints->desired_joint_angles[i].store(0.0f, std::memory_order_relaxed);
    }
    std::atomic_thread_fence(std::memory_order_release);

    munmap(shared_desired_joints, sizeof(SharedDesiredJointState));
    shared_desired_joints = nullptr;

    if (g_desired_creator) {
      shm_unlink(SHARED_DESIRED_JOINTS_NAME);
    }
  }
}

#endif // SHARED_XYZ_H
