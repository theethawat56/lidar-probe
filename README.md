# lidar-probe

เครื่องมือหา protocol ของ lidar ที่ไม่รู้รุ่น (เช่น lidar ที่แกะมาจากหุ่นยนต์ดูดฝุ่น) บน Raspberry Pi 4

ไล่ baud rate อัตโนมัติ ลองถอดตาม protocol ที่พบบ่อย และถ้าไม่ตรงรุ่นไหน จะวิเคราะห์หาความยาวแพ็กเก็ตและ header จากข้อมูลดิบให้

| Protocol | Header | ความยาว | Baud ปกติ |
|---|---|---|---|
| Neato XV-11 / Xiaomi LDS | `FA` + index `A0–F9` | 22 B | 115200 |
| LDROBOT LD06 / LD19 / LD14 | `54 2C` | 47 B | 230400 / 115200 |
| YDLIDAR X2 / X4 style | `AA 55` | ไม่คงที่ | 115200 / 128000 |
| 3irobotix Delta-2A / 2B / 2G (หุ่นดูดฝุ่นหลายยี่ห้อ) | `AA 00 5B 01 61 AD` | 93 B (26 จุด) | 115200 |
| RPLIDAR A1/A2 (ต้องส่งคำสั่งก่อน) | `A5 5A` | 5 B/จุด | 115200 |

## การต่อสาย

ยืนยันขาด้วยมัลติมิเตอร์ก่อนทุกครั้ง

| Lidar | Raspberry Pi 4 |
|---|---|
| V (ไฟเลี้ยง) | 5V (pin 2) |
| G | GND (pin 6) |
| T (TX) | RXD GPIO15 (pin 10) — **สัญญาณต้องไม่เกิน 3.3V** ถ้า 5V ใช้ตัวแบ่งแรงดัน |
| RX (ถ้ามี, ใช้กับ `cmdprobe`) | TXD GPIO14 (pin 8) |
| มอเตอร์ + / − | จ่ายไฟแยก หรือผ่าน MOSFET ที่คุมด้วย GPIO18 — **ห้ามต่อ GPIO ตรง** |

## ติดตั้งบน Pi 4

```bash
git clone https://github.com/theethawat56/lidar-probe.git
cd lidar-probe

sudo apt install -y python3-serial python3-gpiozero python3-matplotlib python3-numpy
sudo usermod -aG dialout $USER                       # แล้ว logout/login ใหม่

# ถ้าต่อผ่านขา GPIO UART (ถ้าใช้ USB-UART ข้ามส่วนนี้ได้)
sudo raspi-config                                    # Interface → Serial Port: login shell = No, hardware = Yes
echo "dtoverlay=disable-bt" | sudo tee -a /boot/firmware/config.txt
sudo reboot
```

## วิธีใช้

```bash
python3 lidar_probe.py scan                          # เริ่มที่นี่: ไล่ baud rate หา protocol อัตโนมัติ
python3 lidar_probe.py --motor-pin 18 scan           # หมุนมอเตอร์ผ่าน MOSFET ที่ GPIO18 ระหว่างทดสอบ
python3 lidar_probe.py dump -b 115200                # ดูข้อมูลดิบเป็น hex
python3 lidar_probe.py live -b 115200 -p neato       # ถอดค่าระยะ real-time
python3 lidar_probe.py live -b 115200 -p neato --csv scan.csv --motor-pin 18 --target-rpm 300
python3 lidar_probe.py plot -b 115200 -p neato       # ภาพ 2D real-time (ต้องมีหน้าจอ/VNC)
python3 lidar_probe.py plot -b 115200 -p neato --png scan.png   # ไม่มีจอ (SSH) → บันทึกเป็นรูป
python3 lidar_probe.py plot -p neato --file captures/raw_115200.bin  # วาดจากไฟล์ที่บันทึกไว้
python3 lidar_probe.py cmdprobe -b 115200            # สำหรับรุ่นที่ต้องรับคำสั่งก่อนถึงจะส่งข้อมูล
python3 lidar_probe.py analyze captures/raw_115200.bin
python3 lidar_probe.py --port /dev/ttyUSB0 scan      # ถ้าต่อผ่าน USB-UART
```

## ภาพ 2D

ตัวอย่างสำหรับ lidar Delta (ตัวที่แกะจากหุ่นดูดฝุ่น):

```bash
python3 lidar_probe.py plot -b 115200 -p delta --max-range 3000       # ภาพ real-time
python3 lidar_probe.py plot -b 115200 -p delta --png scan.png         # บันทึกเป็นรูป
python3 lidar_probe.py plot -p delta --file lidar.bin --speed 0.2     # จำลองภาพเคลื่อนไหวจากไฟล์ (ไม่ต้องต่อ lidar)
python3 lidar_probe.py plot -p delta --file lidar.bin --png room.png  # ภาพนิ่งทุกจุดจากไฟล์
python3 lidar_probe.py live -b 115200 -p delta                        # ตัวเลขระยะ + rpm
```


`plot` วาดจุดแบบ polar: 0° ชี้ขึ้น หมุนตามเข็มนาฬิกา สีตามระยะ เส้นสีแดงคือมุมที่ lidar กำลังวัดอยู่
`--speed` ใช้กับ `--file` เพื่อปรับความเร็วการจำลอง (1.0 = เวลาจริง 300 rpm, 0.2 = ช้าลง 5 เท่า)
ปรับได้ด้วย `--max-range 4000` (mm) และ `--persist 1.0` (วินาทีที่จุดค้างบนจอ)

ถ้าใช้ Pi ผ่าน SSH ไม่มีจอ จะบันทึก `scan.png` อัตโนมัติ หรือดูแบบ real-time ได้ด้วย
VNC (`sudo raspi-config` → Interface → VNC) หรือ `ssh -X`

## เคล็ดลับ

- lidar หุ่นดูดฝุ่นหลายรุ่นจะส่งข้อมูลเฉพาะตอนมอเตอร์หมุนด้วยความเร็วที่ถูกต้อง (~300 rpm)
- ถ้า `scan` ไม่เจอ protocol ที่รู้จัก ให้ดูตาราง header candidate และแพ็กเก็ตตัวอย่างในรายงาน:
  คอลัมน์ที่ค่าคงที่ = header, ค่าเพิ่มทีละขั้น = index/มุม, byte ท้ายสุดมักเป็น checksum
- ไฟล์ดิบของทุก baud ถูกบันทึกไว้ใน `captures/` เพื่อวิเคราะห์ต่อภายหลังได้
