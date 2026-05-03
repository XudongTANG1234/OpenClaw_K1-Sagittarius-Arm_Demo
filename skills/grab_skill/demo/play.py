"""
play.py — 动作回放脚本

用法:
  python play.py pick left block
  python play.py --list
"""

import importlib.util
import json
import os
import sys
import time

import numpy as np

SO_PATH = "../sagittarius_sdk_python/pysagittarius.so"
SEQUENCES_FILE = os.path.join(os.path.dirname(__file__), "sequences.json")


def load_pysagittarius():
    spec = importlib.util.spec_from_file_location("pysagittarius", SO_PATH)
    ps = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(ps)
    return ps


def load_sequences():
    if not os.path.exists(SEQUENCES_FILE):
        print(f"找不到序列文件：{SEQUENCES_FILE}")
        print("请先用 teach.py 录制动作。")
        sys.exit(1)
    with open(SEQUENCES_FILE, "r") as f:
        return json.load(f)


def play_sequence(arm, sequence, action_name):
    print(f"\n执行动作：'{action_name}'（共 {len(sequence)} 个路径点）\n")
    for i, wp in enumerate(sequence):
        joints  = np.array(wp["joints"],  dtype=np.float32)
        gripper = float(wp["gripper"])
        wait    = float(wp.get("wait", 2.0))

        g_str = "开" if gripper >= -0.001 else "关"
        j_str = "  ".join(f"J{j+1}:{joints[j]:+.3f}" for j in range(6))
        print(f"  [{i+1}/{len(sequence)}] 夹爪={g_str}  {j_str}")

        arm.SetAllServoRadian(joints)
        time.sleep(wait)
        arm.arm_set_gripper_linear_position(gripper)
        time.sleep(1.0)

    print(f"\n动作 '{action_name}' 执行完成。\n")


def main():
    if len(sys.argv) < 2:
        print("用法: python play.py <动作名称>")
        print("      python play.py --list")
        sys.exit(1)

    seqs = load_sequences()

    if sys.argv[1] == "--list":
        print("\n已保存的动作：")
        for name, wps in seqs.items():
            print(f"  '{name}'  —  {len(wps)} 个路径点")
        print()
        return

    action_name = " ".join(sys.argv[1:])
    if action_name not in seqs:
        print(f"找不到动作 '{action_name}'")
        print("已有动作：", list(seqs.keys()))
        sys.exit(1)

    ps  = load_pysagittarius()
    arm = ps.SagittariusArmReal("/dev/ttyACM1", 1000000, 500, 5)
    arm.SetFreeAfterDestructor(False)
    arm.ControlTorque("lock")
    time.sleep(0.5)

    play_sequence(arm, seqs[action_name], action_name)


if __name__ == "__main__":
    main()
