// Encodes Kortex frames with fic/kortex_min.h and decodes a Feedback frame, in the
// current directory. Driven by kortex_crosscheck.py (see there).
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fic/kortex_min.h>
using namespace fic::kortex;
static void dump(const char* name, const uint8_t* b, size_t n) {
  FILE* f = std::fopen(name, "wb"); std::fwrite(b, 1, n, f); std::fclose(f);
}
int main(int argc, char** argv) {
  uint8_t pl[2048], fr[2100];
  if (argc > 1 && !std::strcmp(argv[1], "encode")) {
    size_t n = encodeCreateSession(pl, sizeof pl, "admin", "admin", 60000, 2000);
    dump("create_session.bin", fr, encodeRequest(fr, sizeof fr, CreateSession, 0, 1, 0, pl, n));
    n = encodeServoingMode(pl, sizeof pl, LOW_LEVEL);
    dump("servoing.bin", fr, encodeRequest(fr, sizeof fr, SetServoingMode, 0x1234, 2, 0, pl, n));
    n = encodeControlMode(pl, sizeof pl, TORQUE);
    dump("control.bin", fr, encodeRequest(fr, sizeof fr, SetControlMode, 0x1234, 3, 5, pl, n));
    float pos[7], tau[7];
    for (int i = 0; i < 7; ++i) { pos[i] = -170.25f + 51.5f * i; tau[i] = -12.5f + 3.75f * i; }
    tau[3] = 0.0f;                                   // proto3 default: must still decode as 0
    n = encodeCommand(pl, sizeof pl, 0xABCDEF01u, 7, pos, tau, 42.5f, 12.0f, 100.0f);
    dump("command.bin", fr, encodeRequest(fr, sizeof fr, Refresh, 0x0BAD, 0x7777, 0, pl, n));
    std::printf("encoded\n");
    return 0;
  }
  // decode feedback.bin -> print fields
  static uint8_t buf[8192];
  FILE* f = std::fopen("feedback.bin", "rb"); size_t len = std::fread(buf, 1, sizeof buf, f); std::fclose(f);
  Header h; const uint8_t* p; size_t pn;
  if (!decodeFrame(buf, len, h, p, pn)) { std::printf("FRAME DECODE FAILED\n"); return 1; }
  Feedback fb;
  if (!decodeFeedback(p, pn, fb)) { std::printf("FEEDBACK DECODE FAILED\n"); return 1; }
  std::printf("hdr %u %u %u %u %u %u\n", h.frameType(), h.sessionId(), h.messageId(), h.functionUid(), h.errorCode(), h.deviceId());
  std::printf("fb %u %u %u %u %d %d\n", fb.frame_id, fb.arm_state, fb.base_fault_bank_a, fb.base_fault_bank_b, fb.n_actuators, fb.has_gripper ? 1 : 0);
  std::printf("wrench"); for (float x : fb.tool_wrench) std::printf(" %.9g", x); std::printf("\n");
  for (int i = 0; i < fb.n_actuators; ++i)
    std::printf("act %u %u %u %.9g %.9g %.9g %.9g\n", fb.act[i].command_id, fb.act[i].status_flags, fb.act[i].fault_bank_a,
                fb.act[i].position, fb.act[i].velocity, fb.act[i].torque, fb.act[i].current_motor);
  std::printf("grip %.9g %.9g\n", fb.gripper_position, fb.gripper_current);
  return 0;
}
