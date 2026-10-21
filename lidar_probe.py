#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
lidar_probe.py — เครื่องมือหา protocol ของ lidar ที่ไม่รู้รุ่น (เช่นแกะจากหุ่นยนต์ดูดฝุ่น) บน Raspberry Pi 4

การต่อสาย (ยืนยันขาด้วยมัลติมิเตอร์ก่อนทุกครั้ง):
  lidar V -> Pi 5V  (pin 2)        lidar G -> Pi GND (pin 6)
  lidar T -> Pi RXD GPIO15 (pin 10)   ** สัญญาณต้องไม่เกิน 3.3V ถ้าวัดได้ 5V ให้ใช้ตัวแบ่งแรงดัน **
  (ถ้าจะใช้คำสั่ง cmdprobe) lidar RX -> Pi TXD GPIO14 (pin 8)
  มอเตอร์ +/- -> จ่ายไฟแยก หรือผ่าน MOSFET ที่คุมด้วย GPIO18 (ใช้ --motor-pin 18)
  ห้ามต่อมอเตอร์เข้า GPIO ตรง

ติดตั้ง:
  sudo apt install -y python3-serial python3-gpiozero

คำสั่ง:
  python3 lidar_probe.py scan                        # ไล่ baud rate หา protocol อัตโนมัติ (เริ่มที่นี่)
  python3 lidar_probe.py dump  -b 115200             # ดูข้อมูลดิบเป็น hex
  python3 lidar_probe.py live  -b 115200 -p neato    # ถอดค่าระยะ real-time เมื่อรู้ protocol แล้ว
  python3 lidar_probe.py cmdprobe -b 115200          # ส่งคำสั่งแบบ RPLIDAR/YDLIDAR (กรณี lidar ไม่ส่งข้อมูลเอง)
  python3 lidar_probe.py analyze captures/raw_115200.bin   # วิเคราะห์ไฟล์ที่บันทึกไว้

ตัวเลือกมอเตอร์ (ใช้ได้ทุกคำสั่ง):
  --motor-pin 18 --duty 60            # หมุนมอเตอร์ด้วย PWM
  --target-rpm 300                    # (คำสั่ง live) ปรับ duty อัตโนมัติให้ได้รอบที่ต้องการ
"""

import argparse
import os
import sys
import time
from collections import Counter, defaultdict

try:
    import serial
except ImportError:
    serial = None

DEFAULT_PORT = "/dev/serial0"
DEFAULT_BAUDS = [115200, 230400, 128000, 153600, 256000, 57600, 460800, 38400, 19200, 9600]


# =====================================================================
# มอเตอร์ (ไม่บังคับ)
# =====================================================================
class Motor:
    def __init__(self, pin, duty):
        self.dev = None
        self._duty = duty
        if pin is None:
            return
        try:
            from gpiozero import PWMOutputDevice
        except ImportError:
            print("[motor] ไม่พบ gpiozero: sudo apt install python3-gpiozero")
            return
        self.dev = PWMOutputDevice(pin, frequency=1000, initial_value=duty / 100.0)
        print(f"[motor] GPIO{pin} PWM 1kHz duty {duty:.0f}%")

    def set(self, duty):
        self._duty = max(0.0, min(100.0, duty))
        if self.dev:
            self.dev.value = self._duty / 100.0

    @property
    def duty(self):
        return self._duty

    def stop(self):
        if self.dev:
            self.dev.value = 0
            self.dev.close()
            self.dev = None


# =====================================================================
# ตัวถอด protocol ที่รู้จัก
# แต่ละตัวคืนค่า (packets, bad_checksum_count, consumed_bytes)
# packet = (offset, rpm หรือ None, [(angle_deg, distance_mm, strength), ...])
# =====================================================================

# ---- 1) Neato XV-11 / Xiaomi LDS: 22 byte, 0xFA + index 0xA0..0xF9, 115200 baud ----
def neato_checksum(p):
    chk = 0
    for i in range(10):
        chk = (chk << 1) + (p[2 * i] | (p[2 * i + 1] << 8))
    chk = (chk & 0x7FFF) + (chk >> 15)
    return chk & 0x7FFF


def decode_neato(buf):
    out, bad, i, n = [], 0, 0, len(buf)
    while i + 22 <= n:
        if buf[i] == 0xFA and 0xA0 <= buf[i + 1] <= 0xF9:
            p = buf[i:i + 22]
            if neato_checksum(p) == (p[20] | (p[21] << 8)):
                rpm = (p[2] | (p[3] << 8)) / 64.0
                base = (p[1] - 0xA0) * 4
                pts = []
                for k in range(4):
                    d0, d1, s0, s1 = p[4 + 4 * k:8 + 4 * k]
                    if d1 & 0x80:  # invalid flag
                        continue
                    pts.append((float(base + k), d0 | ((d1 & 0x3F) << 8), s0 | (s1 << 8)))
                out.append((i, rpm, pts))
                i += 22
                continue
            bad += 1
        i += 1
    return out, bad, i


# ---- 2) LDROBOT LD06/LD19/LD14 (และ OEM หุ่นดูดฝุ่น): 47 byte, 0x54 0x2C, CRC8 poly 0x4D ----
def _crc8_table(poly):
    t = []
    for i in range(256):
        c = i
        for _ in range(8):
            c = ((c << 1) ^ poly) & 0xFF if c & 0x80 else (c << 1) & 0xFF
        t.append(c)
    return t


CRC8_LD = _crc8_table(0x4D)


def crc8_ld(data):
    crc = 0
    for b in data:
        crc = CRC8_LD[(crc ^ b) & 0xFF]
    return crc


def decode_ldrobot(buf):
    out, bad, i, n = [], 0, 0, len(buf)
    while i + 47 <= n:
        if buf[i] == 0x54 and buf[i + 1] == 0x2C:
            p = buf[i:i + 47]
            if crc8_ld(p[:46]) == p[46]:
                rpm = (p[2] | p[3] << 8) / 6.0  # deg/s -> rpm
                start = (p[4] | p[5] << 8) / 100.0
                end = (p[42] | p[43] << 8) / 100.0
                step = ((end - start) % 360.0) / 11.0
                pts = []
                for k in range(12):
                    o = 6 + 3 * k
                    dist = p[o] | p[o + 1] << 8
                    if dist:
                        pts.append(((start + step * k) % 360.0, dist, p[o + 2]))
                out.append((i, rpm, pts))
                i += 47
                continue
            bad += 1
        i += 1
    return out, bad, i


# ---- 3) YDLIDAR X2/X4 style: 0xAA 0x55, CT, LSN, FSA, LSA, CS(XOR), LSN x 2 byte ----
def decode_ydlidar(buf):
    out, bad, i, n = [], 0, 0, len(buf)
    while i + 10 <= n:
        if buf[i] == 0xAA and buf[i + 1] == 0x55:
            lsn = buf[i + 3]
            length = 10 + 2 * lsn
            if lsn == 0:
                bad += 1
                i += 1
                continue
            if i + length > n:
                break  # แพ็กเก็ตยังมาไม่ครบ เก็บไว้รอบหน้า
            p = buf[i:i + length]
            cs = p[8] | p[9] << 8
            x = 0
            for j in range(0, length, 2):
                if j != 8:
                    x ^= p[j] | p[j + 1] << 8
            if x == cs:
                fsa = ((p[4] | p[5] << 8) >> 1) / 64.0
                lsa = ((p[6] | p[7] << 8) >> 1) / 64.0
                span = (lsa - fsa) % 360.0
                pts = []
                for k in range(lsn):
                    d = (p[10 + 2 * k] | p[11 + 2 * k] << 8) / 4.0
                    if d:
                        ang = fsa + (span * k / (lsn - 1) if lsn > 1 else 0.0)
                        pts.append((ang % 360.0, d, 0))
                out.append((i, None, pts))
                i += length
                continue
            bad += 1
        i += 1
    return out, bad, i


# ---- 4) RPLIDAR standard scan: 5 byte/จุด (ต้องส่งคำสั่ง A5 20 ก่อน, ใช้ใน live/cmdprobe เท่านั้น) ----
def decode_rplidar(buf):
    out, bad, i, n = [], 0, 0, len(buf)
    pts = []
    while i + 5 <= n:
        b0, b1 = buf[i], buf[i + 1]
        if ((b0 ^ (b0 >> 1)) & 1) and (b1 & 1):
            ang = ((b1 >> 1) | (buf[i + 2] << 7)) / 64.0
            d = (buf[i + 3] | buf[i + 4] << 8) / 4.0
            if d:
                pts.append((ang % 360.0, d, b0 >> 2))
            i += 5
            continue
        bad += 1
        i += 1
    if pts:
        out.append((0, None, pts))
    return out, bad, i


PROTOCOLS = {
    "neato":   ("Neato XV-11 / Xiaomi LDS (0xFA, 22B, ปกติ 115200)", decode_neato, 22),
    "ldrobot": ("LDROBOT LD06/LD19/LD14 (0x54 0x2C, 47B, ปกติ 230400 หรือ 115200)", decode_ldrobot, 47),
    "ydlidar": ("YDLIDAR X2/X4 style (0xAA 0x55, ยาวไม่คงที่, ปกติ 115200 หรือ 128000)", decode_ydlidar, None),
}
LIVE_PROTOCOLS = dict(PROTOCOLS, rplidar=("RPLIDAR A1/A2 standard scan", decode_rplidar, 5))


# =====================================================================
# วิเคราะห์ข้อมูลดิบแบบไม่รู้ protocol
# =====================================================================
def generic_stats(buf):
    n = len(buf)
    if n == 0:
        return dict(n=0, cands=[], printable=0.0, zero_ff=0.0, unique=0, period=None, header=None)
    cnt = Counter(buf)
    printable = (sum(cnt[c] for c in range(32, 127)) + cnt[10] + cnt[13]) / n
    zero_ff = (cnt[0] + cnt[0xFF]) / n

    pos = defaultdict(list)
    for idx, b in enumerate(buf):
        pos[b].append(idx)

    cands = []
    for b, pl in pos.items():
        if len(pl) < 10:
            continue
        gaps = Counter(pl[j + 1] - pl[j] for j in range(len(pl) - 1))
        g, c = max(((g, c) for g, c in gaps.items() if 4 <= g <= 600), key=lambda t: t[1], default=(0, 0))
        if not g or c < 5:
            continue
        cover = c * g / n  # สัดส่วนของสตรีมที่ byte นี้โผล่ซ้ำทุก g byte (~1.0 = น่าจะเป็น header)
        nxt = Counter(buf[p + 1] for p in pl if p + 1 < n)
        nb, nc = nxt.most_common(1)[0]
        cands.append((cover, b, g, nb, nc / len(pl)))
    cands.sort(reverse=True)

    # ความยาวแพ็กเก็ตที่น่าจะเป็น = ระยะซ้ำที่ byte เด่นๆ ใช้ร่วมกันมากที่สุด
    strong = [c for c in cands if c[0] > 0.5]
    period = Counter(c[2] for c in strong).most_common(1)[0][0] if strong else None

    # header byte แรก: byte ที่ซ้ำตามจังหวะ และ byte ถัดไปก็เป็น header candidate (เช่น CE AB)
    header = None
    if period:
        same = [c for c in strong if c[2] == period]
        sb = {c[1] for c in same}
        firsts = [c for c in same if c[3] in sb and c[4] > 0.8 and c[3] != c[1]]

        # byte ที่ตามด้วย index เพิ่มทีละ 1 ในแต่ละแพ็กเก็ต (แบบ FA A0, FA A1, ...) = header ชัดเจนที่สุด
        def incr_ratio(b):
            pl = [q for q in pos[b] if q + period + 1 < n and buf[q + period] == b]
            if len(pl) < 5:
                return 0.0
            ok = sum(1 for q in pl if (buf[q + period + 1] - buf[q + 1]) & 0xFF == 1)
            return ok / len(pl)

        inc = [c for c in same if incr_ratio(c[1]) > 0.6]
        header = (firsts or same)[0][1]
        if inc:
            ib = inc[0][1]
            pair = [c for c in firsts if c[3] == ib]  # ถ้ามี byte คงที่นำหน้า เช่น CE AB 01..
            header = bytes([pair[0][1], ib]) if pair else bytes([ib])
        elif firsts:
            header = bytes([firsts[0][1], firsts[0][3]])
        else:
            header = bytes([header])
    return dict(n=n, cands=cands[:6], printable=printable, zero_ff=zero_ff,
                unique=len(cnt), period=period, header=header)


def known_results(buf):
    res = {}
    for key, (name, fn, plen) in PROTOCOLS.items():
        pkts, bad, _ = fn(buf)
        good = len(pkts)
        if plen:
            frac = good * plen / max(1, len(buf))
        else:
            frac = sum(10 + 2 * len(p[2]) for p in pkts) / max(1, len(buf))
        rpms = [p[1] for p in pkts if p[1]]
        npts = sum(len(p[2]) for p in pkts)
        sample = [pt for p in pkts[:50] for pt in p[2]][:8]
        res[key] = dict(name=name, good=good, bad=bad, frac=min(frac, 1.0),
                        rpm=(sum(rpms) / len(rpms)) if rpms else None, npts=npts, sample=sample)
    return res


def score(buf):
    kr = known_results(buf)
    best_key = max(kr, key=lambda k: kr[k]["frac"])
    if kr[best_key]["good"] >= 3 and kr[best_key]["frac"] > 0.3:
        return 1.0 + kr[best_key]["frac"], best_key, kr
    gs = generic_stats(buf)
    g = gs["cands"][0][0] if gs["cands"] else 0.0
    if gs["printable"] > 0.9:
        g = max(g, 0.8)
    return min(g, 0.99), None, kr


def hexdump(buf, width=16, limit=None):
    data = buf if limit is None else buf[:limit]
    lines = []
    for off in range(0, len(data), width):
        chunk = data[off:off + width]
        hx = " ".join(f"{b:02X}" for b in chunk)
        asc = "".join(chr(b) if 32 <= b < 127 else "." for b in chunk)
        lines.append(f"{off:06X}  {hx:<{width * 3}} {asc}")
    return "\n".join(lines)


def report(buf, label="", seconds=None):
    print("=" * 70)
    print(f"รายงาน {label}")
    rate = f", {len(buf) / seconds:.0f} byte/s" if seconds else ""
    print(f"ข้อมูล {len(buf)} byte{rate}")
    if not buf:
        print("ไม่มีข้อมูลเข้าเลย → เช็ค: ไฟเลี้ยง V/G, สาย T เข้า RX(pin 10), เปิด UART แล้วหรือยัง,")
        print("  มอเตอร์หมุนหรือยัง (บางรุ่นส่งเฉพาะตอนหมุน), หรือรุ่นนี้ต้องรับคำสั่งก่อน (ลอง cmdprobe)")
        return

    kr = known_results(buf)
    print("\n[protocol ที่รู้จัก]")
    for key, r in kr.items():
        mark = "✔" if r["good"] >= 3 and r["frac"] > 0.3 else " "
        rpm = f", rpm≈{r['rpm']:.0f}" if r["rpm"] else ""
        print(f" {mark} {key:8s} แพ็กเก็ตถูกต้อง {r['good']:5d} | checksum ผิด {r['bad']:5d} | "
              f"ครอบคลุม {r['frac'] * 100:5.1f}%{rpm}")
        if mark == "✔":
            print(f"     {r['name']}")
            print(f"     จุดที่ถอดได้ {r['npts']} จุด ตัวอย่าง (มุม°, ระยะmm, ความแรง):")
            for a, d, s in r["sample"]:
                print(f"       {a:7.2f}°  {d:8.1f} mm  {s}")

    gs = generic_stats(buf)
    print("\n[วิเคราะห์โครงสร้างแบบทั่วไป]")
    print(f" byte ไม่ซ้ำกัน {gs['unique']}/256 | 0x00/0xFF {gs['zero_ff'] * 100:.0f}% | "
          f"ตัวอักษรอ่านได้ {gs['printable'] * 100:.0f}%")
    if gs["printable"] > 0.9:
        print(" → ดูเหมือนส่งเป็นข้อความ ASCII:")
        print("   " + buf[:300].decode("ascii", "replace").replace("\n", "\n   "))
    if gs["zero_ff"] > 0.6:
        print(" → 0x00/0xFF เยอะผิดปกติ: baud น่าจะผิด, สายสัญญาณผิดขา หรือไม่มีสัญญาณ")
    if gs["cands"]:
        print(" byte ที่ซ้ำเป็นจังหวะ (header candidate):")
        print("   byte  ทุกๆ(byte)  ครอบคลุม  byte ถัดไปที่พบบ่อย")
        for cover, b, g, nb, nr in gs["cands"]:
            print(f"   0x{b:02X}   {g:5d}      {cover * 100:5.1f}%    0x{nb:02X} ({nr * 100:.0f}%)")
    if gs["period"]:
        hdr = gs["header"]
        print(f" → ความยาวแพ็กเก็ตน่าจะเป็น {gs['period']} byte, header น่าจะเป็น {hdr.hex(' ').upper()}")
        start = buf.find(hdr)
        print(f"\n ตัวอย่างแพ็กเก็ต เรียงให้ตรงกันที่ header {hdr.hex(' ').upper()}:")
        p = gs["period"]
        for k in range(6):
            seg = buf[start + k * p:start + (k + 1) * p]
            if len(seg) < p:
                break
            print("   " + " ".join(f"{b:02X}" for b in seg))
        print(" (ดูคอลัมน์ที่ค่าคงที่ = header/ชนิดข้อมูล, ค่าเพิ่มทีละขั้น = index/มุม, ท้ายสุดมัก = checksum)")
    print("\n hex 128 byte แรก:")
    print(hexdump(buf, limit=128))


# =====================================================================
# Serial helpers
# =====================================================================
def need_serial():
    if serial is None:
        sys.exit("ไม่พบ pyserial: sudo apt install python3-serial")


def open_port(port, baud):
    need_serial()
    try:
        return serial.Serial(port, baud, timeout=0.1)
    except serial.SerialException as e:
        sys.exit(f"เปิดพอร์ต {port} ไม่ได้: {e}\n"
                 "เช็ค: raspi-config เปิด serial hardware แล้ว, ปิด serial console, user อยู่ในกลุ่ม dialout")


def capture(port, baud, seconds):
    ser = open_port(port, baud)
    time.sleep(0.05)
    ser.reset_input_buffer()
    data = bytearray()
    t0 = time.time()
    while time.time() - t0 < seconds:
        data += ser.read(4096)
    ser.close()
    return bytes(data)


# =====================================================================
# คำสั่ง
# =====================================================================
def cmd_scan(args, motor):
    bauds = [int(b) for b in args.bauds.split(",")] if args.bauds else DEFAULT_BAUDS
    os.makedirs(args.outdir, exist_ok=True)
    if motor.dev:
        print("[motor] รอมอเตอร์หมุนคงที่ 2 วินาที...")
        time.sleep(2)
    print(f"ไล่ทดสอบ {len(bauds)} baud rate ที่ {args.port}, ครั้งละ {args.duration}s\n")
    results = []
    for baud in bauds:
        data = capture(args.port, baud, args.duration)
        path = os.path.join(args.outdir, f"raw_{baud}.bin")
        with open(path, "wb") as f:
            f.write(data)
        sc, key, _ = score(data)
        tag = f"พบ {key}" if key else ("ข้อความ ASCII" if data and generic_stats(data)["printable"] > 0.9 else "")
        print(f" {baud:7d} baud: {len(data):7d} byte  คะแนน {sc:4.2f}  {tag}")
        results.append((sc, baud, key, data))

    if not any(r[3] for r in results):
        report(b"", "ทุก baud")
        return
    results.sort(key=lambda r: r[0], reverse=True)
    sc, baud, key, data = results[0]
    print(f"\nbaud ที่น่าจะถูกที่สุด: {baud}")
    report(data, f"@ {baud} baud", args.duration)
    print("\nขั้นต่อไป:")
    if key:
        print(f"  python3 {sys.argv[0]} live -b {baud} -p {key}")
    else:
        print("  ยังไม่ตรง protocol ที่รู้จัก → ดูตาราง header/ตัวอย่างแพ็กเก็ตด้านบน")
        print(f"  ส่งไฟล์ {os.path.join(args.outdir, f'raw_{baud}.bin')} มาให้วิเคราะห์ต่อได้")
        print("  ถ้าคะแนนต่ำทุก baud ลอง cmdprobe (บางรุ่นต้องรับคำสั่งก่อนถึงจะส่งข้อมูล)")


def cmd_dump(args, motor):
    ser = open_port(args.port, args.baud)
    ser.reset_input_buffer()
    out = open(args.save, "wb") if args.save else None
    total, off = 0, 0
    print(f"dump @ {args.baud} baud (Ctrl+C เพื่อหยุด)")
    try:
        t0 = time.time()
        while args.duration <= 0 or time.time() - t0 < args.duration:
            d = ser.read(args.width)
            if not d:
                continue
            if out:
                out.write(d)
            hx = " ".join(f"{b:02X}" for b in d)
            asc = "".join(chr(b) if 32 <= b < 127 else "." for b in d)
            print(f"{off:08X}  {hx:<{args.width * 3}} {asc}")
            off += len(d)
            total += len(d)
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        if out:
            out.close()
            print(f"บันทึก {total} byte ลง {args.save}")


def cmd_live(args, motor):
    name, fn, _ = LIVE_PROTOCOLS[args.proto]
    ser = open_port(args.port, args.baud)
    ser.reset_input_buffer()
    if args.proto == "rplidar":
        ser.write(b"\xA5\x20")
        desc = ser.read(7)
        print("RPLIDAR descriptor:", desc.hex(" "))
    print(f"live: {name} @ {args.baud} baud (Ctrl+C เพื่อหยุด)")
    csv = open(args.csv, "w") if args.csv else None
    if csv:
        csv.write("time,angle_deg,distance_mm,strength\n")

    buf = bytearray()
    scan = {}
    rpms = []
    npts = 0
    nbad = 0
    t_print = time.time()
    try:
        while True:
            buf += ser.read(4096)
            pkts, bad, used = fn(bytes(buf))
            nbad += bad
            del buf[:used]
            now = time.time()
            for _, rpm, pts in pkts:
                if rpm:
                    rpms.append(rpm)
                for a, d, s in pts:
                    scan[int(round(a)) % 360] = d
                    npts += 1
                    if csv:
                        csv.write(f"{now:.3f},{a:.2f},{d:.1f},{s}\n")

            if now - t_print >= 0.5:
                rpm = sum(rpms) / len(rpms) if rpms else None
                if args.target_rpm and rpm and motor.dev:
                    motor.set(motor.duty + args.kp * (args.target_rpm - rpm))

                def at(a):
                    for da in (0, 1, -1, 2, -2):
                        v = scan.get((a + da) % 360)
                        if v:
                            return f"{v:6.0f}"
                    return "     -"

                near = min(((d, a) for a, d in scan.items() if d > 0), default=None)
                rpm_s = f"{rpm:5.0f}" if rpm else "    -"
                duty_s = f" duty {motor.duty:4.1f}%" if motor.dev else ""
                near_s = f"{near[0]:.0f}mm@{near[1]}°" if near else "-"
                print(f"rpm {rpm_s}{duty_s} | {npts / (now - t_print):5.0f} จุด/s | bad {nbad:4d} | "
                      f"0°{at(0)} 90°{at(90)} 180°{at(180)} 270°{at(270)} mm | ใกล้สุด {near_s}")
                rpms, npts, nbad, t_print = [], 0, 0, now
    except KeyboardInterrupt:
        pass
    finally:
        if args.proto == "rplidar":
            ser.write(b"\xA5\x25")
        ser.close()
        if csv:
            csv.close()
            print(f"บันทึกจุดลง {args.csv}")


def cmd_cmdprobe(args, motor):
    """ส่งคำสั่งของ lidar ที่ต้องสั่งก่อนถึงจะส่งข้อมูล (ต้องต่อขา RX ของ lidar เข้า TXD pin 8)"""
    ser = open_port(args.port, args.baud)
    probes = [
        ("RPLIDAR stop",        b"\xA5\x25", 0),
        ("RPLIDAR GET_INFO",    b"\xA5\x50", 27),
        ("RPLIDAR GET_HEALTH",  b"\xA5\x52", 10),
        ("YDLIDAR stop",        b"\xA5\x65", 0),
        ("YDLIDAR GET_INFO",    b"\xA5\x90", 27),
        ("YDLIDAR GET_HEALTH",  b"\xA5\x92", 10),
    ]
    print(f"cmdprobe @ {args.baud} baud (ต้องต่อ lidar RX ← Pi TXD pin 8)\n")
    found = []
    for name, cmd, expect in probes:
        ser.reset_input_buffer()
        ser.write(cmd)
        time.sleep(0.3 if expect else 0.1)
        resp = ser.read(max(expect, 64))
        hit = resp.find(b"\xA5\x5A")
        flag = "✔ ตอบกลับแบบ A5 5A" if hit >= 0 else ""
        print(f" {name:20s} ส่ง {cmd.hex(' ')} → {resp[:32].hex(' ') or '(เงียบ)'} {flag}")
        if hit >= 0:
            found.append(name)
    for name, cmd in (("RPLIDAR SCAN", b"\xA5\x20"), ("YDLIDAR SCAN", b"\xA5\x60")):
        ser.reset_input_buffer()
        ser.write(cmd)
        time.sleep(1.5)
        data = ser.read(20000)
        stop = b"\xA5\x25" if name.startswith("RP") else b"\xA5\x65"
        ser.write(stop)
        print(f"\n {name}: ได้ข้อมูล {len(data)} byte")
        if data:
            report(data, f"หลังส่ง {name}")
    ser.close()
    if found:
        print("\nlidar ตอบคำสั่ง:", ", ".join(found))
        if any(f.startswith("RPLIDAR") for f in found):
            print(f"ลอง: python3 {sys.argv[0]} live -b {args.baud} -p rplidar")


def cmd_analyze(args, motor):
    with open(args.file, "rb") as f:
        data = f.read()
    report(data, args.file)


# =====================================================================
def main():
    ap = argparse.ArgumentParser(description="ทดสอบหา protocol lidar ที่ไม่รู้รุ่น")
    ap.add_argument("--port", default=DEFAULT_PORT, help=f"serial port (default {DEFAULT_PORT}, USB = /dev/ttyUSB0)")
    ap.add_argument("--motor-pin", type=int, default=None, help="GPIO ที่ต่อ MOSFET คุมมอเตอร์ (เช่น 18)")
    ap.add_argument("--duty", type=float, default=60.0, help="PWM duty เริ่มต้น %% (default 60)")
    sub = ap.add_subparsers(dest="cmd", required=True)

    s = sub.add_parser("scan", help="ไล่ baud rate หา protocol อัตโนมัติ")
    s.add_argument("--bauds", help="รายการ baud คั่นด้วย comma")
    s.add_argument("--duration", type=float, default=3.0, help="วินาทีต่อ baud")
    s.add_argument("--outdir", default="captures")

    d = sub.add_parser("dump", help="แสดงข้อมูลดิบเป็น hex")
    d.add_argument("-b", "--baud", type=int, default=115200)
    d.add_argument("--width", type=int, default=22, help="byte ต่อบรรทัด (ตั้งเท่าความยาวแพ็กเก็ตจะดูง่าย)")
    d.add_argument("--duration", type=float, default=0, help="วินาที (0 = จนกด Ctrl+C)")
    d.add_argument("--save", help="บันทึกข้อมูลดิบลงไฟล์ .bin")

    l = sub.add_parser("live", help="ถอดค่าระยะ real-time")
    l.add_argument("-b", "--baud", type=int, default=115200)
    l.add_argument("-p", "--proto", choices=list(LIVE_PROTOCOLS), required=True)
    l.add_argument("--target-rpm", type=float, help="ปรับ duty อัตโนมัติให้ได้รอบนี้ (ต้องใช้ --motor-pin)")
    l.add_argument("--kp", type=float, default=0.05, help="gain ปรับ duty ต่อ rpm ที่ผิด")
    l.add_argument("--csv", help="บันทึกจุดลงไฟล์ CSV")

    c = sub.add_parser("cmdprobe", help="ส่งคำสั่ง RPLIDAR/YDLIDAR ทดสอบ")
    c.add_argument("-b", "--baud", type=int, default=115200)

    a = sub.add_parser("analyze", help="วิเคราะห์ไฟล์ .bin ที่บันทึกไว้")
    a.add_argument("file")

    args = ap.parse_args()
    motor = Motor(args.motor_pin, args.duty)
    try:
        {"scan": cmd_scan, "dump": cmd_dump, "live": cmd_live,
         "cmdprobe": cmd_cmdprobe, "analyze": cmd_analyze}[args.cmd](args, motor)
    finally:
        motor.stop()


if __name__ == "__main__":
    main()
