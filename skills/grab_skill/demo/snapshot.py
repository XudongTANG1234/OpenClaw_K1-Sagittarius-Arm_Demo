"""
snapshot.py — 读取当前机械臂位姿并保存

用法:
  python snapshot.py                    # 交互式输入动作名和路径点编号
  python snapshot.py "pick left block"  # 指定动作名，追加一个路径点

流程:
  1. 先用物理按键把机械臂摆到目标位置并锁定
  2. 运行此脚本
  3. 脚本以极低速发送 0x0B，读取 0x06 响应中的真实编码器位置
  4. 立即再发一次 0x0B 把臂锁在读到的位置
  5. 保存到 sequences.json
"""

import json
import math
import os
import serial
import struct
import sys
import time

SERIAL_PORT    = "/dev/ttyACM1"
BAUD_RATE      = 1000000
SEQUENCES_FILE = os.path.join(os.path.dirname(__file__), "sequences.json")
PI             = math.pi

GRIPPER_OPEN   =  0.0
GRIPPER_CLOSE  = -0.068


# ── 串口包构造 ────────────────────────────────────────────────

def _checksum(buf: bytearray) -> int:
    return sum(buf[3: 3 + buf[2]]) & 0xFF


def make_set_vel(vel: int) -> bytes:
    """CMD_SET_SERVO_VEL (0x0D)"""
    buf = bytearray([0x55, 0xAA, 4, 0x01, 0x0D,
                     vel & 0xFF, (vel >> 8) & 0xFF])
    buf.append(_checksum(buf))
    buf.append(0x7D)
    return bytes(buf)


def make_0x0B(joints_rad) -> bytes:
    """CMD_CONTROL_ALL_DEGREE_CB (0x0B) — 运动 + 请求位置反馈"""
    buf = bytearray([0x55, 0xAA, 14, 0x01, 0x0B])
    for j in joints_rad:
        d = int(round(1800.0 / PI * j))
        d = max(-32768, min(32767, d))
        buf.append(d & 0xFF)
        buf.append((d >> 8) & 0xFF)
    buf.append(_checksum(buf))
    buf.append(0x7D)
    return bytes(buf)


# ── 解析 0x06 响应帧 ─────────────────────────────────────────

def parse_response(data: bytes):
    """
    在接收缓冲区中查找合法的 0x06 0x01 位置帧，返回 (joints[6], gripper_raw)。
    帧格式：55 AA 10 01 06 [j1l j1h ... j6l j6h grl grh] cs 7D  (21 bytes)
    """
    i = 0
    while i <= len(data) - 21:
        if data[i] != 0x55 or data[i+1] != 0xAA:
            i += 1
            continue
        length = data[i+2]           # 应为 0x10 = 16
        total  = length + 5          # 21
        if i + total > len(data):
            break
        frame = data[i: i + total]
        if frame[3] != 0x01 or frame[4] != 0x06 or frame[-1] != 0x7D:
            i += 1
            continue
        cs = sum(frame[3: 3 + length]) & 0xFF
        if cs != frame[-2]:
            i += 1
            continue
        # 解码 6 个关节 + 1 个夹爪（各 2 字节，有符号小端）
        values = []
        for k in range(7):
            raw = struct.unpack_from('<h', frame, 5 + 2 * k)[0]
            values.append(raw / 1800.0 * PI)
        return values[:6], values[6]   # joints, gripper_angle
        i += 1
    return None, None


# ── 主逻辑 ───────────────────────────────────────────────────

def read_position(ser: serial.Serial):
    """发送极低速 0x0B，等待 0x06 响应，返回 joints[6] 和 gripper_raw。"""
    ser.reset_input_buffer()

    # 速度设为 1（极低），机械臂几乎不动
    ser.write(make_set_vel(1))
    time.sleep(0.05)
    ser.reset_input_buffer()

    # 发送 0x0B（目标全零），触发臂回报当前真实编码器位置
    ser.write(make_0x0B([0.0] * 6))

    # 等待 0x06 响应（臂会先回报当前位置，再极缓慢地向零位运动）
    time.sleep(0.12)
    data = ser.read(ser.in_waiting or 64)

    joints, gripper_raw = parse_response(data)

    if joints is None:
        # 若未收到，多等一次
        time.sleep(0.15)
        data += ser.read(ser.in_waiting or 64)
        joints, gripper_raw = parse_response(data)

    return joints, gripper_raw


def lock_at_position(ser: serial.Serial, joints):
    """发送 0x0B 将机械臂锁在读取到的位置，恢复正常速度。"""
    ser.write(make_0x0B(joints))
    time.sleep(0.05)
    ser.write(make_set_vel(500))   # 恢复正常速度
    time.sleep(0.05)


def load_sequences():
    if os.path.exists(SEQUENCES_FILE):
        with open(SEQUENCES_FILE, "r") as f:
            return json.load(f)
    return {}


def save_sequences(seqs):
    os.makedirs(os.path.dirname(SEQUENCES_FILE), exist_ok=True)
    with open(SEQUENCES_FILE, "w") as f:
        json.dump(seqs, f, indent=2, ensure_ascii=False)


def main():
    # ── 打开串口 ──
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.5)
    except serial.SerialException as e:
        print(f"无法打开串口 {SERIAL_PORT}：{e}")
        sys.exit(1)
    time.sleep(0.1)

    print("正在读取机械臂当前位置...")
    joints, gripper_raw = read_position(ser)

    if joints is None:
        print("未收到位置响应，请确认：")
        print("  1. 机械臂已通过物理按键摆好并锁定")
        print("  2. 没有其他程序（如 teach.py）占用串口")
        ser.close()
        sys.exit(1)

    # ── 锁定在读取位置 ──
    lock_at_position(ser, joints)
    ser.close()

    # ── 显示结果 ──
    print("\n读取成功！")
    print("─" * 55)
    for i, j in enumerate(joints):
        print(f"  J{i+1}: {j:+.4f} rad  ({math.degrees(j):+8.3f}°)")

    # 夹爪：根据 gripper_raw 判断开/关
    # raw ≈ 0 时为开，raw 较大负值时为关（参考 arm_set_gripper_linear_position 编码）
    gripper_linear = -abs(gripper_raw) * 0.026 / PI if gripper_raw != 0 else 0.0
    gripper_linear = max(GRIPPER_CLOSE, min(GRIPPER_OPEN, gripper_linear))
    g_str = "开 (OPEN)" if gripper_linear > -0.01 else f"关 (CLOSE，约 {gripper_linear:.3f} m)"
    print(f"  夹爪: {g_str}")
    print("─" * 55)

    # ── 询问夹爪状态（用户确认） ──
    g_input = input("\n夹爪状态确认 (o=开/c=关，回车保持自动判断): ").strip().lower()
    if g_input == "o":
        gripper_save = GRIPPER_OPEN
    elif g_input == "c":
        gripper_save = GRIPPER_CLOSE
    else:
        gripper_save = GRIPPER_OPEN if gripper_linear > -0.01 else GRIPPER_CLOSE

    # ── 询问保存方式 ──
    if len(sys.argv) > 1:
        action_name = " ".join(sys.argv[1:])
    else:
        action_name = input("\n保存为动作名称（如 pick left block）: ").strip()
        if not action_name:
            print("未输入名称，不保存。")
            sys.exit(0)

    wait_time = input("到达此路径点后等待时间（秒，默认 2.0）: ").strip()
    try:
        wait_time = float(wait_time) if wait_time else 2.0
    except ValueError:
        wait_time = 2.0

    waypoint = {
        "joints":  [float(j) for j in joints],
        "gripper": gripper_save,
        "wait":    wait_time,
    }

    seqs = load_sequences()
    if action_name in seqs:
        choice = input(f"\n动作 '{action_name}' 已存在（{len(seqs[action_name])} 个路径点），"
                       f"追加/覆盖/取消？(a=追加 / o=覆盖 / 回车取消): ").strip().lower()
        if choice == "a":
            seqs[action_name].append(waypoint)
            print(f"已追加，现共 {len(seqs[action_name])} 个路径点。")
        elif choice == "o":
            seqs[action_name] = [waypoint]
            print("已覆盖为 1 个路径点。")
        else:
            print("已取消。")
            sys.exit(0)
    else:
        seqs[action_name] = [waypoint]
        print(f"已新建动作 '{action_name}'，1 个路径点。")

    save_sequences(seqs)
    print(f"保存至：{SEQUENCES_FILE}")


if __name__ == "__main__":
    main()
