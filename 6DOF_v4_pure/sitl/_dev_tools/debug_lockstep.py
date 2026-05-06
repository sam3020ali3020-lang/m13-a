#!/usr/bin/env python3
"""Focused lockstep debug: one HIL_SENSOR at a time, wait for PX4 reply.

Environment variables (with defaults derived from repo layout):
    PX4_SITL_BIN  - path to px4 binary
                    (default: <repo>/AndroidApp/app/src/main/cpp/PX4-Autopilot/
                              build/px4_sitl_default/bin/px4)
    PX4_SITL_CWD  - PX4 working directory (default: dirname(dirname(PX4_SITL_BIN)))
    ACADOS_LIB    - acados lib dir (default: <repo>/acados-main/lib)
"""
import socket, struct, time, sys, os, subprocess, signal
from pathlib import Path

_SCRIPT_DIR = Path(__file__).resolve().parent
_M13_ROOT = _SCRIPT_DIR.parents[2]  # <repo>/6DOF_v4_pure/sitl/_dev_tools -> repo
_DEFAULT_PX4_BIN = _M13_ROOT / "AndroidApp/app/src/main/cpp/PX4-Autopilot/build/px4_sitl_default/bin/px4"
_DEFAULT_ACADOS_LIB = _M13_ROOT / "acados-main/lib"

MAVLINK_STX_V2 = 0xFD
HIL_SENSOR_ID = 107
HIL_ACTUATOR_ID = 93
CRC_EXTRA_HIL_SENSOR = 108

def x25_crc(data, crc=0xFFFF):
    for b in data:
        tmp = b ^ (crc & 0xFF)
        tmp ^= (tmp << 4) & 0xFF
        crc = (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)
        crc &= 0xFFFF
    return crc

_seq = [0]
def build_hil_sensor(time_usec):
    payload = struct.pack('<Q3f3f3fffffIB',
        time_usec,
        0.0, 0.0, -9.80665,
        0.0, 0.0, 0.0,
        0.2, 0.0, 0.4,
        1013.25, 0.0, 0.0, 25.0,
        0x1FFF, 0)
    trimmed = payload.rstrip(b'\x00')
    header = struct.pack('<BBBBBBBHB', MAVLINK_STX_V2, len(trimmed), 0, 0,
        _seq[0] & 0xFF, 1, 1, HIL_SENSOR_ID & 0xFFFF, (HIL_SENSOR_ID >> 16) & 0xFF)
    _seq[0] += 1
    crc = x25_crc(header[1:] + payload)
    crc = x25_crc(bytes([CRC_EXTRA_HIL_SENSOR]), crc)
    return header + trimmed + struct.pack('<H', crc)

def parse_messages(data):
    """Extract MAVLink v2 messages from raw bytes."""
    msgs = []
    buf = bytearray(data)
    while True:
        idx = buf.find(bytes([0xFD]))
        if idx < 0: break
        if idx > 0: del buf[:idx]
        if len(buf) < 10: break
        plen = buf[1]
        total = 10 + plen + 2
        if len(buf) < total: break
        msg_id = buf[7] | (buf[8] << 8) | (buf[9] << 16)
        payload = bytes(buf[10:10+plen])
        msgs.append((msg_id, payload))
        del buf[:total]
    return msgs

def main():
    PX4_BIN = os.environ.get("PX4_SITL_BIN", str(_DEFAULT_PX4_BIN))
    PX4_CWD = os.environ.get("PX4_SITL_CWD",
                             str(Path(PX4_BIN).resolve().parent.parent))
    
    # Clean params
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

    # Start PX4
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

    # Read initial PX4 data
    conn.settimeout(2)
    try:
        init_data = conn.recv(4096)
        print(f"[DBG] Initial rx: {len(init_data)} bytes")
        for mid, pl in parse_messages(init_data):
            print(f"[DBG]   msg_id={mid} len={len(pl)}")
    except socket.timeout:
        print("[DBG] No initial data from PX4")

    # Lockstep loop: send one HIL_SENSOR, wait for response
    dt_us = 4000  # 4ms = 250Hz (standard PX4 IMU rate)
    t_us = 0
    total_rx = 0
    got_actuators = False
    
    print(f"[DBG] Starting lockstep loop (dt={dt_us}us)...")
    for step in range(5000):  # 5000 steps @ 4ms = 20s sim time
        t_us += dt_us
        msg = build_hil_sensor(t_us)
        conn.sendall(msg)

        # Wait for PX4 to respond (proper lockstep)
        conn.settimeout(2.0)
        try:
            data = conn.recv(4096)
            if data:
                total_rx += len(data)
                for mid, pl in parse_messages(data):
                    if mid == HIL_ACTUATOR_ID:
                        if not got_actuators:
                            got_actuators = True
                            print(f"\n[DBG] *** FIRST ACTUATOR CONTROLS at step {step} "
                                  f"(t={t_us/1e6:.3f}s) payload_len={len(pl)}")
                            if len(pl) >= 24:
                                vals = struct.unpack(f'<Q{min((len(pl)-8)//4, 16)}f', pl[:min(len(pl), 72)])
                                print(f"[DBG]   time_usec={vals[0]}")
                                print(f"[DBG]   controls={list(vals[1:5])}")
                    elif step < 10 or step % 500 == 0:
                        print(f"[DBG] Step {step}: msg_id={mid} len={len(pl)}")
        except socket.timeout:
            if step < 10 or step % 200 == 0:
                print(f"[DBG] Step {step} (t={t_us/1e6:.2f}s): TIMEOUT (no PX4 response) total_rx={total_rx}")

        if got_actuators and step > 100:
            print(f"[DBG] Got actuator controls! Exiting after step {step}")
            break

    # Print PX4 stdout
    print(f"\n[DBG] Total bytes received from PX4: {total_rx}")
    print(f"[DBG] Got actuator controls: {got_actuators}")
    print("\n--- PX4 stdout (last 50 lines) ---")
    os.killpg(os.getpgid(px4.pid), signal.SIGTERM)
    try:
        stdout = px4.communicate(timeout=5)[0].decode('utf-8', errors='replace')
        for line in stdout.strip().split('\n')[-50:]:
            print(f"  PX4| {line}")
    except: pass
    conn.close()

if __name__ == '__main__':
    main()
