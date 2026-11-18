#!/usr/bin/env python3
"""WIP: runs the hub in-process (MuJoCo) and the patched MCB, and prints what the
firmware believes (its odometry over the vision link) next to sim truth, plus
joint positions and world yaw rates. Paths are still this session's scratch
paths: make them repo-relative and fold it into run.sh as --compare."""
import os, subprocess, sys, time
sys.path.insert(0, "/home/user/aruw-sim/hw-emulator/python")
import hwemu
RT = "/tmp/claude-0/rt2"
os.system(f"rm -rf {RT}")
A = "/home/user/aruw-sim/assets/sentry"
hub = hwemu.Hub(f"{A}/sim.yaml", f"{A}/devices.yaml", sys.argv[1] if len(sys.argv) > 1 else "mujoco", RT)
hub.start()
env = dict(os.environ, HWEMU_RUNTIME_DIR=RT, HWEMU_PROCESS="mcb")
mcb = subprocess.Popen(["/tmp/claude-0/mcb/aruw-mcb-project/build/sim-hwemu/scons-debug/TARGET_SENTRY_ACHLYS/aruw-mcb.elf"],
                       env=env, stdout=open("/tmp/claude-0/mcb_run.log", "w"), stderr=subprocess.STDOUT)
s = hub.sim
names = ["major_yaw_motor_encoder", "minor_yaw_encoder", "minor_pitch_encoder", "flywheel_left_encoder", "agitator_encoder", "lf_wheel_encoder"]
t_end = time.time() + float(sys.argv[2] if len(sys.argv) > 2 else 25)
while time.time() < t_end:
    time.sleep(2.5)
    st = "  ".join(f"{n.split('_encoder')[0]}={s.encoder(n).read().position:+.2f}" for n in names)
    import json
    odo = None
    try:
        for line in open(f"{RT}/vision.jsonl"):
            if '"odometry"' in line: odo = line
    except FileNotFoundError: pass
    if odo:
        m = json.loads(odo)["msg"]
        k = ["x","y","z","roll","pitch","yaw"]
        ch = dict(zip(k, m.get("chassis", [0]*6))); tu = dict(zip(k, (m.get("turrets") or [[0]*6])[0]))
        catt = s.attitude("chassis_attitude").read(); watt = s.attitude("widow_attitude").read()
        print(f"   MCB says: chassis yaw={ch.get('yaw',0):+.2f} xy=({ch.get('x',0):+.2f},{ch.get('y',0):+.2f})  turret yaw={tu.get('yaw',0):+.2f} pitch={tu.get('pitch',0):+.2f}")
        print(f"   sim truth: chassis yaw={catt.yaw:+.2f}                    turret yaw={watt.yaw:+.2f} pitch={watt.pitch:+.2f}")
    cg = s.gyro("chassis_gyro").read().z
    mg = s.gyro("major_ism330a_gyro").read().z
    wg = s.gyro("widow_gyro").read().z
    print(f"t={s.time():6.2f} {st}  wz chassis={cg:+.2f} major={mg:+.2f} widow={wg:+.2f}", flush=True)
mcb.terminate(); mcb.wait(); hub.stop()
print(hub.status())
