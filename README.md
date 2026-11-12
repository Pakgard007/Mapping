# 3DMapping-INC31-KMUTT — 1Mapping

Workspace ROS 2 (Humble) สำหรับ **ทำแผนที่ 3 มิติ (point cloud map)** ด้วยรถกอล์ฟ
โดยใช้ LiDAR Ouster OS-1-128 + LIO-SAM และใช้ GNSS RTK (u-blox ZED-F9R + NTRIP)
ช่วยยึดแผนที่เข้ากับพิกัดโลก

แผนที่ที่ได้ (`GlobalMap.pcd`) นำไปใช้ต่อใน workspace `2Localize_get_path`
สำหรับหาตำแหน่งรถ (NDT localization) และขับตามเส้นทาง

```text
Ouster OS-1-128 ─┐ /ouster/points, /ouster/imu
                 ├──► LIO-SAM ──► แผนที่ + เส้นทาง ──► save_map ──► ~/Downloads/LOAM/*.pcd
u-blox ZED-F9R ──┤ /ublox_gps_node/fix
   ▲ /rtcm       │
NTRIP client ────┘
CAN (STM32, KEYA) ──► golfcart_low_level ──► /speed_status (ความเร็วล้อ ใช้ช่วย LIO-SAM)
```

## โครงสร้างโปรเจกต์

| โฟลเดอร์ / ไฟล์ | หน้าที่ |
|---|---|
| `src/LIO-SAM-ROS-Humble-main/` | LIO-SAM (แพ็กเกจ `lio_sam`) ปรับให้ใช้กับ Ouster 128 ช่อง, รับ GPS แบบ `NavSatFix` ตรงๆ และรวมความเร็วล้อจาก CAN |
| `src/mapping_rviz_panel/` | หน้าต่าง RViz สำหรับทำแผนที่ มีปุ่มเปิด/ปิดแต่ละขั้นตอน ปุ่มเซฟแผนที่ และโมเดลรถกอล์ฟขนาดจริง |
| `src/ouster-ros/` | ไดรเวอร์ Ouster LiDAR (จาก [ouster-lidar/ouster-ros](https://github.com/ouster-lidar/ouster-ros)) |
| `src/ublox/` | ไดรเวอร์ GNSS u-blox (จาก [KumarRobotics/ublox](https://github.com/KumarRobotics/ublox)) |
| `src/ntrip_client/` | NTRIP client ดึงค่าแก้ RTCM3 จาก VRS caster ส่งให้ u-blox (`/rtcm`) |
| `src/golfcart_low_level/` | สะพาน ROS 2 ↔ CAN ไปยังบอร์ด STM32 (คันเร่ง/เบรก) และไดรเวอร์มอเตอร์พวงมาลัย KEYA — ดู [README ของแพ็กเกจ](src/golfcart_low_level/README.md) |
| `firmware/stm32_motorbrake_car_shield/` | เฟิร์มแวร์ STM32G474 สำหรับควบคุมความเร็วและเบรก (STM32CubeIDE) |
| `docs/` | เอกสารประกอบ: ตัวอย่างการสั่งเบรก, ภาพรวมระบบ low-level, สไลด์การเก็บแผนที่ |
| `192.168.1-metadata.json` | metadata ของ LiDAR Ouster (OS-1-128, IP 192.168.1.100) |
| `start_cart.sh` | สคริปต์เปิดระบบขับรถทั้งชุดตามลำดับ (ใช้ร่วมกับ `2Localize_get_path`) |
| `cart_env.sh` | `source` เพื่อเตรียม shell สำหรับขับรถ และตรวจว่าแพ็กเกจมาจาก workspace ไหน |

## ฮาร์ดแวร์

- LiDAR: Ouster OS-1-128 (1024×10 Hz) ต่อผ่าน LAN ที่ `192.168.1.100`
- GNSS: u-blox ZED-F9R (C102-F9R) ผ่าน USB (`/dev/ttyACM*`) + NTRIP RTK
- CAN: InnoMaker USB2CAN เป็น `can0` ที่ 250 kbit/s
- คอมพิวเตอร์บนรถ: Jetson / Ubuntu 22.04 + ROS 2 Humble

## ติดตั้งและ build

```bash
# ROS 2 Humble ต้องติดตั้งไว้ก่อน
git clone git@github.com:Pakgard007/Mapping.git ~/1Mapping
cd ~/1Mapping
rosdep install --from-paths src --ignore-src -r -y
sudo apt install -y python3-can can-utils python3-aenum   # สำหรับ golfcart_low_level
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

> โฟลเดอร์ `build/`, `install/`, `log/` ไม่ได้อยู่ใน git ต้อง build เองหลัง clone

## วิธีทำแผนที่

### แบบใช้หน้าต่าง Mapping Panel (แนะนำ)

```bash
source /opt/ros/humble/setup.bash
source ~/1Mapping/install/setup.bash
ros2 launch mapping_rviz_panel mapping_panel.launch.py          # theme:=false = หน้าตา RViz แบบเดิม
```

ในแผง **Mapping** กดเปิดทีละขั้น แต่ละขั้นจะเปิดใน terminal ของตัวเอง:

1. **LiDAR Ouster** — `ros2 launch ouster_ros driver.launch.py`
2. **LIO-SAM** — `ros2 launch mapping_rviz_panel lio_sam.launch.py`
3. **Low-level** — ระบบ CAN ของรถ (ให้ได้ความเร็วล้อ `/speed_status`)
4. **GPS u-blox** — `ros2 launch ublox_gps ublox_gps_node_zed_f9r-launch.py`
5. **NTRIP (RTK)** — `ros2 launch ntrip_client ntrip_client-launch.py`

ขับรถเก็บข้อมูลให้ทั่วพื้นที่ แล้วกดปุ่ม **เซฟแผนที่** แผนที่จะถูกบันทึกไว้ที่ `~/Downloads/LOAM/`

> ⚠️ แผนที่อยู่ในหน่วยความจำจนกว่าจะเซฟ ถ้าหยุด LIO-SAM ก่อนเซฟต้องเริ่มทำใหม่ทั้งหมด

### แบบสั่งเอง

```bash
ros2 launch ouster_ros driver.launch.py
ros2 launch lio_sam run.launch.py                     # มี RViz ของ LIO-SAM ในตัว
ros2 launch ublox_gps ublox_gps_node_zed_f9r-launch.py
ros2 launch ntrip_client ntrip_client-launch.py

# เซฟแผนที่ (resolution 0.2 m)
ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap "{resolution: 0.2, destination: Downloads/LOAM}"   # path สัมพัทธ์จาก $HOME
# ⚠️ โฟลเดอร์ปลายทางจะถูกลบแล้วสร้างใหม่ทุกครั้งที่เซฟ ย้ายแผนที่เก่าออกก่อน
```

## การตั้งค่าที่สำคัญ

| ไฟล์ | ตั้งค่าอะไร |
|---|---|
| `src/LIO-SAM-ROS-Humble-main/config/params.yaml` | topic ของ sensor, การใช้ GPS (`gpsCovThreshold`, `gpsAlign*`), ที่เซฟแผนที่ (`savePCDDirectory`), extrinsic LiDAR–IMU |
| `src/ntrip_client/config/ntrip.yaml` | host / port / mountpoint / บัญชี NTRIP และพิกัดเริ่มต้นของพื้นที่ |
| `src/mapping_rviz_panel/cart/cart.yaml` | ขนาดรถกอล์ฟและตำแหน่ง LiDAR บนรถ (ใช้วาดโมเดลใน RViz) |
| `src/golfcart_low_level/config/` | ค่าพารามิเตอร์ CAN, พวงมาลัย, wheelbase |

### บัญชี NTRIP

username/password ของ NTRIP **ไม่ได้อยู่ใน git** ให้สร้างไฟล์ `src/ntrip_client/config/ntrip.local.yaml`
(ไฟล์นี้อยู่ใน `.gitignore`) แล้ว build ใหม่ — launch จะโหลดไฟล์นี้ทับ `ntrip.yaml` ให้เอง

```yaml
ntrip_client:
  ros__parameters:
    username: "<username>"
    password: "<password>"
```

แก้ config แล้วต้อง `colcon build --packages-select <ชื่อแพ็กเกจ>` ใหม่ (หรือใช้ `--symlink-install`)

## ใช้คู่กับ 2Localize_get_path

ทั้ง `1Mapping` และ `2Localize_get_path` มีแพ็กเกจ `golfcart_low_level`, `ouster_ros`, `ublox_gps`
ซ้ำกัน และใน ROS 2 **workspace ที่ source ทีหลังจะถูกใช้**

- **ทำแผนที่** → source `1Mapping` ทีหลัง
- **ขับรถ / localization** → source `2Localize_get_path` ทีหลัง หรือใช้ `source ~/1Mapping/cart_env.sh`

เปิดระบบขับรถทั้งชุด (CAN → low level → Ouster → GNSS → localization → waypoint follower):

```bash
bash ~/1Mapping/start_cart.sh              # sensor + localization อย่างเดียว รถไม่ขยับ
bash ~/1Mapping/start_cart.sh --dry-run    # เปิด follower แต่ไม่ส่ง /cmd_vel
bash ~/1Mapping/start_cart.sh --drive      # รถจะวิ่งจริง — เตรียมมือไว้ที่ปุ่มหยุดฉุกเฉิน
bash ~/1Mapping/start_cart.sh --help       # ดูตัวเลือกทั้งหมด
```

## แก้ปัญหาเบื้องต้น

- **ไม่มี `/ouster/points`** — `ping 192.168.1.100` และตรวจว่าการ์ดแลนอยู่วง `192.168.1.x`
- **`ublox_gps_node_zed_f9r-launch.py` not found** — source workspace สลับลำดับ ดูหัวข้อด้านบน
- **ไม่เจอ GPS** — `ls /dev/ttyACM*` และ `dmesg | tail -20`
- **ไม่ได้ RTK fix** — ตรวจว่า `/rtcm` มีข้อมูลเข้า (`ros2 topic hz /rtcm`) และบัญชี NTRIP ยังใช้ได้
- **CAN ไม่ขึ้น** — `bash src/golfcart_low_level/scripts/setup_can0.sh can0 250000`

## เครดิต

- [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM) — Tixiao Shan et al.
- [ouster-ros](https://github.com/ouster-lidar/ouster-ros) — Ouster
- [ublox](https://github.com/KumarRobotics/ublox) — KumarRobotics

ไลเซนส์ของแต่ละแพ็กเกจเป็นไปตามไฟล์ `LICENSE` ในโฟลเดอร์ของแพ็กเกจนั้น
