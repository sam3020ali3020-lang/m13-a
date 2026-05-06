#!/usr/bin/env python3
"""Debug lockstep using pymavlink — guaranteed correct MAVLink framing.

Environment variables (with defaults derived from repo layout):
    PX4_SITL_BIN  - path to px4 binary
                    (default: <repo>/AndroidApp/app/src/main/cpp/PX4-Autopilot/
                              build/px4_sitl_default/bin/px4)
    PX4_SITL_CWD  - PX4 working directory (default: dirname(dirname(PX4_SITL_BIN)))
    ACADOS_LIB    - acados lib dir (default: <repo>/acados-main/lib)
"""
import socket, struct, time, sys, os, subprocess, signal
from pathlib import Path
from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink

ACTUATOR_ID = 93

_SCRIPT_DIR = Path(__file__).resolve().parent
_M13_ROOT = _SCRIPT_DIR.parents[2]  # <repo>/6DOF_v4_pure/sitl/_dev_tools -> repo
_DEFAULT_PX4_BIN = _M13_ROOT / "AndroidApp/app/src/main/cpp/PX4-Autopilot/build/px4_sitl_default/bin/px4"
_DEFAULT_ACADOS_LIB = _M13_ROOT / "acados-main/lib"

def main():
    PX4_BIN = os.environ.get("PX4_SITL_BIN", str(_DEFAULT_PX4_BIN))
    PX4_CWD = os.environ.get("PX4_SITL_CWD",
                             str(Path(PX4_BIN).resolve().parent.parent))
    
    for f in ('parameters.bson', 'parameters_backup.bson'):
        p = os.path.join(PX4_CWD, f)
        if os.path.exists(p): os.remove(p)

    print("[DBG] Listening on TCP 4560...")
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    srv.bind(('127.0.0.1', 4560))
    srv.listen(1)
    srv.settimeout(60)

    print("[DBG] Starting PX4...")
    env = os.environ.copy()
    env['PX4_SYS_AUTOSTART'] = '22003'
    env['PX4_SIM_MODEL'] = 'none'
    acados_lib = os.environ.get("ACADOS_LIB", str(_DEFAULT_ACADOS_LIB))
    env['LD_LIBRARY_PATH'] = acados_lib + ':' + env.get('LD_LIBRARY_PATH', '')
    px4 = subprocess.Popen(
        [PX4_BIN, '-s', 'etc/init.d-posix/rcS', '-d'],
        env=env, cwd=PX4_CWD,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        preexec_fn=os.setsid)
    print(f"[DBG] PX4 PID={px4.pid}")

    conn, addr = srv.accept()
    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    srv.close()
    print(f"[DBG] PX4 connected from {addr}")

    # Create pymavlink MAVLink instance for encoding
    class DummyFile:
        def write(self, data): pass
    mav = mavlink.MAVLink(DummyFile(), srcSystem=1, srcComponent=1)
    mav.robust_parsing = True

    # Read initial data from PX4
    conn.settimeout(3)
    try:
        init_data = conn.recv(4096)
        print(f"[DBG] Initial rx: {len(init_data)} bytes")
    except socket.timeout:
        print("[DBG] No initial data")

    # Lockstep: send HIL_SENSOR using pymavlink, wait for response
    dt_us = 4000  # 4ms per step (250Hz)
    t_us = 0
    total_rx = 0
    got_actuators = False
    step_with_data = 0

    print(f"[DBG] Starting lockstep (dt={dt_us}us, 250Hz)...")
    for step in range(10000):
        t_us += dt_us

        # Build HIL_SENSOR with pymavlink
        msg = mav.hil_sensor_encode(
            time_usec=t_us,
            xacc=0.0, yacc=0.0, zacc=-9.80665,
            xgyro=0.0, ygyro=0.0, zgyro=0.0,
            xmag=0.2, ymag=0.0, zmag=0.4,
            abs_pressure=1013.25, diff_pressure=0.0,
            pressure_alt=0.0, temperature=25.0,
            fields_updated=0x1FFF, id=0)
        buf = msg.pack(mav)
        conn.sendall(buf)

        # Wait for response
        conn.settimeout(2.0)
        try:
            data = conn.recv(4096)
            if data:
                total_rx += len(data)
                step_with_data = step
                # Parse with pymavlink
                mav_recv = mavlink.MAVLink(DummyFile())
                mav_recv.robust_parsing = True
                try:
                    parsed_msgs = mav_recv.parse_buffer(data)
                    if parsed_msgs:
                        for m in parsed_msgs:
                            mtype = m.get_type()
                            if step < 10 or step % 500 == 0 or mtype != 'HEARTBEAT':
                                print(f"[DBG] Step {step} (t={t_us/1e6:.3f}s): "
                                      f"RX {mtype} (id={m.get_msgId()})")
                            if m.get_msgId() == ACTUATOR_ID:
                                got_actuators = True
                                print(f"[DBG] *** ACTUATOR CONTROLS at step {step}!")
                                print(f"[DBG]   controls={m.controls[:4]}")
                except Exception as e:
                    print(f"[DBG] Parse error: {e}")
                    # Try raw parse
                    for i in range(len(data)):
                        if data[i] == 0xFD and i+10 < len(data):
                            mid = data[i+7] | (data[i+8]<<8) | (data[i+9]<<16)
                            print(f"[DBG]   raw msg_id={mid}")
        except socket.timeout:
            if step < 10 or step % 500 == 0:
                print(f"[DBG] Step {step} (t={t_us/1e6:.2f}s): TIMEOUT rx_total={total_rx}")

        if got_actuators:
            print(f"[DBG] SUCCESS! Exiting.")
            break

    print(f"\n[DBG] Summary: steps={step+1}, total_rx={total_rx}, "
          f"actuators={got_actuators}, last_data_step={step_with_data}")

    # Print PX4 output
    print("\n--- PX4 output (last 30 lines) ---")
    os.killpg(os.getpgid(px4.pid), signal.SIGTERM)
    try:
        out = px4.communicate(timeout=5)[0].decode('utf-8', errors='replace')
        for line in out.strip().split('\n')[-30:]:
            print(f"  PX4| {line}")
    except: pass
    conn.close()

if __name__ == '__main__':
    main()
