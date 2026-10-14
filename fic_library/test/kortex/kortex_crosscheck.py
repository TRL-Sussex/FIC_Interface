#!/usr/bin/env python3
"""Cross-check fic/kortex_min.h (the MCU's Kortex codec) against Kinova's
official kortex_api message classes, in both directions:

  MCU encoder -> official decoder: CreateSession, SetServoingMode,
      SetControlMode, BaseCyclic.Command (7 actuators + gripper)
  official encoder -> MCU decoder: a full BaseCyclic.Feedback frame

  g++ -std=c++17 -O2 -I../../include kortex_codec_tool.cpp -o kortex_codec_tool
  cd <scratch dir> && /path/to/kortex_codec_tool encode
  PYTHONPATH=~/kortex_py PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python \
      python3 /path/to/kortex_crosscheck.py /path/to/kortex_codec_tool
"""
import random, struct, subprocess, sys
f32 = lambda x: struct.unpack('f', struct.pack('f', float(x)))[0]
from kortex_api.autogen.messages import Frame_pb2, Session_pb2, Base_pb2, ActuatorConfig_pb2, BaseCyclic_pb2
fails = 0
def expect(name, cond):
    global fails
    print(("  ok   " if cond else "  FAIL ") + name); fails += 0 if cond else 1
def frame(path):
    f = Frame_pb2.Frame(); f.ParseFromString(open(path, 'rb').read()); return f
def hdr(f):
    h = f.header
    return dict(ver=h.frame_info >> 28, type=(h.frame_info >> 24) & 0xF, dev=(h.frame_info >> 16) & 0xFF,
                sess=h.message_info >> 16, msg=h.message_info & 0xFFFF,
                sver=h.service_info >> 28, fn=h.service_info & 0x0FFFFFFF, plen=h.payload_info & 0xFFFFFF)

print("MCU encoder -> official decoder")
f = frame('create_session.bin'); h = hdr(f)
m = Session_pb2.CreateSessionInfo(); m.ParseFromString(f.payload)
expect("CreateSession header", h == dict(ver=1, type=1, dev=0, sess=0, msg=1, sver=1, fn=0x10001, plen=len(f.payload)))
expect("CreateSessionInfo fields", (m.username, m.password, m.session_inactivity_timeout, m.connection_inactivity_timeout) == ('admin', 'admin', 60000, 2000))
f = frame('servoing.bin'); h = hdr(f)
m = Base_pb2.ServoingModeInformation(); m.ParseFromString(f.payload)
expect("SetServoingMode header", h == dict(ver=1, type=1, dev=0, sess=0x1234, msg=2, sver=1, fn=0x20098, plen=len(f.payload)))
expect("ServoingMode = LOW_LEVEL", m.servoing_mode == Base_pb2.LOW_LEVEL_SERVOING)
f = frame('control.bin'); h = hdr(f)
m = ActuatorConfig_pb2.ControlModeInformation(); m.ParseFromString(f.payload)
expect("SetControlMode header (device 5)", h == dict(ver=1, type=1, dev=5, sess=0x1234, msg=3, sver=1, fn=0xa0007, plen=len(f.payload)))
expect("ControlMode = TORQUE", m.control_mode == ActuatorConfig_pb2.TORQUE)
f = frame('command.bin'); h = hdr(f)
c = BaseCyclic_pb2.Command(); c.ParseFromString(f.payload)
expect("Refresh header", h == dict(ver=1, type=1, dev=0, sess=0x0BAD, msg=0x7777, sver=1, fn=0x30001, plen=len(f.payload)))
expect("Command.frame_id", c.frame_id == 0xABCDEF01)
expect("7 actuators", len(c.actuators) == 7)
expect("actuator fields", all(a.command_id == 0xABCDEF01 and abs(a.position - (-170.25 + 51.5*i)) < 1e-4 and
       abs(a.torque_joint - (0.0 if i == 3 else -12.5 + 3.75*i)) < 1e-6 and a.velocity == 0 and a.flags == 0
       for i, a in enumerate(c.actuators)))
g = c.interconnect.gripper_command.motor_cmd[0]; expect("interconnect command_id = frame_id", c.interconnect.command_id.identifier == 0xABCDEF01); expect("one gripper motor_cmd", len(c.interconnect.gripper_command.motor_cmd) == 1)
expect("gripper motor_cmd", (g.position, g.velocity, g.force) == (42.5, 12.0, 100.0))
ref = BaseCyclic_pb2.Command(); ref.CopyFrom(c)
expect("re-serialized by official lib parses identically", BaseCyclic_pb2.Command.FromString(ref.SerializeToString()) == c)

print("official encoder -> MCU decoder")
rnd = random.Random(7)
F = lambda: struct.unpack('f', struct.pack('f', rnd.uniform(-500, 500)))[0]
fb = BaseCyclic_pb2.Feedback(); fb.frame_id = 0x12345678
fb.base.active_state = 6; fb.base.fault_bank_a = 0x11; fb.base.fault_bank_b = 0x22
fb.base.arm_voltage = F(); fb.base.imu_acceleration_x = F(); fb.base.commanded_tool_pose_theta_z = F()
W = [F() for _ in range(6)]
(fb.base.tool_external_wrench_force_x, fb.base.tool_external_wrench_force_y, fb.base.tool_external_wrench_force_z,
 fb.base.tool_external_wrench_torque_x, fb.base.tool_external_wrench_torque_y, fb.base.tool_external_wrench_torque_z) = W
A = []
for i in range(7):
    a = fb.actuators.add()
    vals = dict(command_id=rnd.getrandbits(32), status_flags=rnd.getrandbits(32), fault_bank_a=rnd.getrandbits(8),
                position=F(), velocity=F(), torque=F(), current_motor=F())
    for k, v in vals.items(): setattr(a, k, v)
    a.voltage = F(); a.temperature_core = F(); a.warning_bank_b = 3
    A.append(vals)
m0 = fb.interconnect.gripper_feedback.motor.add(); m0.position = F(); m0.current_motor = F(); m0.velocity = F()
fr = Frame_pb2.Frame()
fr.header.frame_info = (1 << 28) | (3 << 24) | (0 << 16)
fr.header.message_info = (0x0BAD << 16) | 0x7777
fr.header.service_info = (1 << 28) | 0x30001
fr.payload = fb.SerializeToString(); fr.header.payload_info = len(fr.payload)
open('feedback.bin', 'wb').write(fr.SerializeToString())
print("  (feedback frame is %d bytes)" % len(fr.SerializeToString()))
out = subprocess.run([sys.argv[1] if len(sys.argv) > 1 else './kortex_codec_tool'], capture_output=True, text=True).stdout.split('\n')
L = {l.split()[0]: l.split()[1:] for l in out if l and l.split()[0] in ('hdr', 'fb', 'wrench', 'grip')}
acts = [l.split()[1:] for l in out if l.startswith('act ')]
expect("header", L['hdr'] == ['3', str(0x0BAD), str(0x7777), str(0x30001), '0', '0'])
expect("frame_id / arm_state / base faults / count / gripper", L['fb'] == [str(0x12345678), '6', '17', '34', '7', '1'])
expect("tool wrench", all(f32(x) == f32(w) for x, w in zip(L['wrench'], W)))
ok = len(acts) == 7
for a, v in zip(acts, A):
    ok &= [int(a[0]), int(a[1]), int(a[2])] == [v['command_id'], v['status_flags'], v['fault_bank_a']]
    ok &= all(f32(x) == f32(v[k]) for x, k in zip(a[3:], ('position', 'velocity', 'torque', 'current_motor')))
expect("7 actuators: ids, flags, faults, position/velocity/torque/current (exact)", ok)
expect("gripper position/current", [f32(x) for x in L['grip']] == [f32(m0.position), f32(m0.current_motor)])
print("ALL OK" if not fails else "%d FAILED" % fails)
