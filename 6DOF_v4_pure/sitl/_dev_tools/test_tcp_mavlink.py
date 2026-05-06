#!/usr/bin/env python3
"""Minimal TCP MAVLink test — verify PX4 simulator_mavlink receives our messages."""
import socket
import struct
import time
import sys

# MAVLink v2 constants
MAVLINK_STX_V2 = 0xFD
MAVLINK_MSG_ID_HIL_SENSOR = 107
CRC_EXTRA_HIL_SENSOR = 108

def x25_crc(data, crc=0xFFFF):
    for b in data:
        tmp = b ^ (crc & 0xFF)
        tmp ^= (tmp << 4) & 0xFF
        crc = (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)
        crc &= 0xFFFF
    return crc

def build_hil_sensor(time_usec, seq=0):
    """Build minimal HIL_SENSOR with reasonable values."""
    payload = struct.pack('<Q3f3f3fffffIB',
        time_usec,
        0.0, 0.0, -9.80665,   # accel (body, stationary = gravity in z)
        0.0, 0.0, 0.0,         # gyro
        0.2, 0.0, 0.4,         # mag (Gauss)
        1013.25,                # abs_pressure (mbar)
        0.0,                    # diff_pressure
        0.0,                    # pressure_alt (m)
        25.0,                   # temperature
        0x1FFF,                 # fields_updated (all)
        0)                      # sensor_id = 0

    # Trim trailing zeros (MAVLink v2 payload truncation)
    trimmed = payload.rstrip(b'\x00')
    trimmed_len = len(trimmed)

    # MAVLink v2 header
    header = struct.pack('<BBBBBBBHB',
        MAVLINK_STX_V2, trimmed_len,
        0, 0,           # flags
        seq & 0xFF,     # sequence
        1, 1,           # sys_id, comp_id
        MAVLINK_MSG_ID_HIL_SENSOR & 0xFFFF,
        (MAVLINK_MSG_ID_HIL_SENSOR >> 16) & 0xFF)

    # CRC over header[1:] + FULL payload + crc_extra
    crc_data = header[1:] + payload
    crc = x25_crc(crc_data)
    crc = x25_crc(bytes([CRC_EXTRA_HIL_SENSOR]), crc)

    msg = header + trimmed + struct.pack('<H', crc)
    return msg

def main():
    host, port = '127.0.0.1', 4560
    
    print(f"[TEST] Listening on TCP {host}:{port}...")
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    server.bind((host, port))
    server.listen(1)
    server.settimeout(120.0)

    try:
        conn, addr = server.accept()
    except socket.timeout:
        print("[TEST] FAIL: No connection within 120s")
        return 1
    
    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    conn.setblocking(True)
    server.close()
    print(f"[TEST] PX4 connected from {addr}")

    # Check for initial heartbeat from PX4
    conn.settimeout(5.0)
    try:
        initial = conn.recv(4096)
        print(f"[TEST] Received {len(initial)} bytes from PX4 (heartbeat?): {initial[:20].hex()}...")
    except socket.timeout:
        print("[TEST] No initial data from PX4 within 5s")

    # Send HIL_SENSOR messages and check for response
    seq = 0
    t_usec = 1000  # start at 1ms (not 0, to avoid any edge case)
    
    for i in range(200):
        msg = build_hil_sensor(t_usec, seq)
        try:
            sent = conn.send(msg)
            if i < 5 or i % 50 == 0:
                print(f"[TEST] Sent HIL_SENSOR #{i}: {len(msg)} bytes, "
                      f"t_usec={t_usec}, sent={sent}, "
                      f"first_bytes={msg[:6].hex()}")
        except Exception as e:
            print(f"[TEST] Send error at #{i}: {e}")
            break
        
        seq = (seq + 1) & 0xFF
        t_usec += 10000  # advance 10ms per step

        # Check for response
        conn.settimeout(0.2)
        try:
            resp = conn.recv(4096)
            if resp:
                print(f"[TEST] *** RECEIVED {len(resp)} bytes from PX4 at step {i}!")
                # Parse for any MAVLink message
                for j in range(len(resp)):
                    if resp[j] == 0xFD and j + 10 < len(resp):
                        plen = resp[j+1]
                        msg_id = resp[j+7] | (resp[j+8] << 8) | (resp[j+9] << 16)
                        print(f"[TEST]   MAVLink v2 msg: id={msg_id}, payload_len={plen}")
                    elif resp[j] == 0xFE and j + 6 < len(resp):
                        plen = resp[j+1]
                        msg_id = resp[j+5]
                        print(f"[TEST]   MAVLink v1 msg: id={msg_id}, payload_len={plen}")
        except socket.timeout:
            if i < 5 or i % 50 == 0:
                print(f"[TEST] No response at step {i}")

        time.sleep(0.01)  # 10ms between messages

    conn.close()
    print("[TEST] Done")
    return 0

if __name__ == '__main__':
    sys.exit(main())
