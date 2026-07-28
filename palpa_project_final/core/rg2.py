"""RG2 하드웨어 계층 — 상수, 너비↔각도 환산, 순수 소켓 Modbus TCP."""
# 자동 분리: grip_web.py → core/ (내용 동일, 위치만 이동)

import math
import socket
import struct
import threading
import time

# ── RG2 상수 ────────────────────────────────────────────────────────────────
L1, L3 = 0.108505, 0.055
THETA1, THETA3 = 1.41371, 0.76794
DY = -0.0144
MAX_WIDTH_MM = 110.0
CONTROL_GRIP = 16
G = 9.80665


def width_mm_to_angle_deg(width_mm):
    w = width_mm / 1000.0
    x = ((w / 2.0) - DY - L1 * math.cos(THETA1)) / L3
    x = max(-1.0, min(1.0, x))
    return math.degrees(math.acos(x) - THETA3)


# ── 순수 소켓 Modbus TCP ────────────────────────────────────────────────────
class ModbusTCP:
    def __init__(self, host, port=502, unit=65, timeout=1.0):
        self.host, self.port, self.unit, self.timeout = host, port, unit, timeout
        self.sock = None
        self.tid = 0

    def connect(self):
        self.sock = socket.create_connection((self.host, self.port), self.timeout)

    def close(self):
        try:
            if self.sock:
                self.sock.close()
        except Exception:
            pass
        self.sock = None

    def _recv(self, n):
        buf = b''
        while len(buf) < n:
            chunk = self.sock.recv(n - len(buf))
            if not chunk:
                raise IOError('연결 끊김')
            buf += chunk
        return buf

    def _txn(self, pdu, unit=None):
        u = self.unit if unit is None else unit
        self.tid = (self.tid + 1) & 0xFFFF
        self.sock.sendall(struct.pack('>HHHB', self.tid, 0, len(pdu) + 1, u) + pdu)
        _, _, length, _ = struct.unpack('>HHHB', self._recv(7))
        body = self._recv(length - 1)
        if body[0] & 0x80:
            raise IOError(f'Modbus 예외 code={body[1]}')
        return body

    def read_holding(self, address, count, unit=None):
        body = self._txn(struct.pack('>BHH', 0x03, address, count), unit)
        bc = body[1]
        return list(struct.unpack('>' + 'H' * (bc // 2), body[2:2 + bc]))

    def write_registers(self, address, values, unit=None):
        pdu = struct.pack('>BHHB', 0x10, address, len(values), len(values) * 2)
        for v in values:
            pdu += struct.pack('>H', int(v) & 0xFFFF)
        self._txn(pdu, unit)
