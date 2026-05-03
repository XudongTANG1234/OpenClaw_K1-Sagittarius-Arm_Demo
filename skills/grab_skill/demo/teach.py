"""
teach.py — tkinter 滑块示教脚本

用法:
  python teach.py pick left block
  python teach.py --list
"""

import importlib.util
import json
import os
import sys
import time
import threading
import tkinter as tk
from tkinter import ttk, messagebox

import numpy as np

SO_PATH = "../sagittarius_sdk_python/pysagittarius.so"
SEQUENCES_FILE = os.path.join(os.path.dirname(__file__), "sequences.json")
GRIPPER_OPEN  =  0.0
GRIPPER_CLOSE = -0.068

LIMITS_LO = [-2.0, -1.57, -1.48, -2.9, -1.8, -3.1]
LIMITS_HI  = [ 2.0,  1.4,   1.8,  2.9,  1.6,  3.1]


def load_pysagittarius():
    spec = importlib.util.spec_from_file_location("pysagittarius", SO_PATH)
    ps = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(ps)
    return ps


def load_sequences():
    if os.path.exists(SEQUENCES_FILE):
        with open(SEQUENCES_FILE, "r") as f:
            return json.load(f)
    return {}


def save_sequences(seqs):
    os.makedirs(os.path.dirname(SEQUENCES_FILE), exist_ok=True)
    with open(SEQUENCES_FILE, "w") as f:
        json.dump(seqs, f, indent=2)


class TeachApp:
    def __init__(self, root, arm, action_name, init_joints=None):
        self.root = root
        self.arm  = arm
        self.action_name = action_name
        self.waypoints = []
        self.gripper = GRIPPER_OPEN
        self._init_joints = init_joints if init_joints is not None else [0.0] * 6

        # 上次发送给机械臂的关节角（避免重复发送）
        self._last_sent = [float(self._init_joints[i]) for i in range(6)]
        self._send_lock = threading.Lock()

        root.title(f"示教模式 — {action_name}")
        root.resizable(False, False)
        self._build_ui()

        # 定时刷新（限制发送频率到 20Hz）
        self._schedule_send()

    # ── UI 构建 ──────────────────────────────────────────────

    def _build_ui(self):
        root = self.root
        PAD = dict(padx=8, pady=4)

        # ── 标题 ──
        tk.Label(root, text=f"动作：{self.action_name}",
                 font=("Helvetica", 13, "bold")).grid(
            row=0, column=0, columnspan=3, pady=(10, 4))

        # ── 关节滑块 ──
        self.sliders = []
        self.angle_labels = []

        joint_frame = tk.LabelFrame(root, text="关节控制", padx=8, pady=6)
        joint_frame.grid(row=1, column=0, columnspan=3, sticky="ew", **PAD)

        for i in range(6):
            tk.Label(joint_frame, text=f"J{i+1}", width=3,
                     font=("Courier", 11, "bold")).grid(row=i, column=0)

            var = tk.DoubleVar(value=float(self._init_joints[i]))
            sl = tk.Scale(
                joint_frame, variable=var,
                from_=LIMITS_LO[i], to=LIMITS_HI[i],
                resolution=0.01, orient=tk.HORIZONTAL,
                length=420, showvalue=False,
                command=lambda val, idx=i: self._on_slider(idx, val),
            )
            sl.grid(row=i, column=1, padx=4)

            init_val = float(self._init_joints[i])
            init_deg = init_val * 180.0 / 3.14159265
            lbl = tk.Label(joint_frame,
                           text=f"{init_val:+.3f} rad  ({init_deg:+6.1f}°)",
                           width=22, anchor="w", font=("Courier", 10))
            lbl.grid(row=i, column=2)

            self.sliders.append(var)
            self.angle_labels.append(lbl)

        # ── 夹爪 ──
        grip_frame = tk.LabelFrame(root, text="夹爪", padx=8, pady=6)
        grip_frame.grid(row=2, column=0, columnspan=3, sticky="ew", **PAD)

        self.grip_btn = tk.Button(
            grip_frame, text="夹爪：开  OPEN",
            width=20, height=2, bg="#4CAF50", fg="white",
            font=("Helvetica", 11, "bold"),
            command=self._toggle_gripper)
        self.grip_btn.pack(side=tk.LEFT, padx=4)

        # ── 操作按钮 ──
        btn_frame = tk.Frame(root)
        btn_frame.grid(row=3, column=0, columnspan=3, **PAD)

        tk.Button(btn_frame, text="归零 (Home)", width=14,
                  command=self._home).pack(side=tk.LEFT, padx=4)
        tk.Button(btn_frame, text="✔  记录路径点", width=16,
                  bg="#2196F3", fg="white", font=("Helvetica", 10, "bold"),
                  command=self._record).pack(side=tk.LEFT, padx=4)
        tk.Button(btn_frame, text="✘  删除最后", width=14,
                  bg="#f44336", fg="white",
                  command=self._delete_last).pack(side=tk.LEFT, padx=4)
        tk.Button(btn_frame, text="💾  保存退出", width=14,
                  bg="#FF9800", fg="white", font=("Helvetica", 10, "bold"),
                  command=self._save_quit).pack(side=tk.LEFT, padx=4)

        # ── 路径点列表 ──
        wp_frame = tk.LabelFrame(root, text="已记录路径点", padx=8, pady=6)
        wp_frame.grid(row=4, column=0, columnspan=3, sticky="ew", **PAD)

        self.wp_text = tk.Text(wp_frame, height=8, width=72,
                               font=("Courier", 9), state=tk.DISABLED,
                               bg="#f5f5f5")
        sb = tk.Scrollbar(wp_frame, command=self.wp_text.yview)
        self.wp_text.configure(yscrollcommand=sb.set)
        self.wp_text.pack(side=tk.LEFT, fill=tk.BOTH)
        sb.pack(side=tk.RIGHT, fill=tk.Y)

        # ── 状态栏 ──
        self.status_var = tk.StringVar(value="就绪")
        tk.Label(root, textvariable=self.status_var,
                 anchor="w", fg="#555").grid(
            row=5, column=0, columnspan=3, sticky="ew", padx=8, pady=(2, 8))

    # ── 回调 ─────────────────────────────────────────────────

    def _on_slider(self, idx, val):
        deg = float(val) * 180.0 / 3.14159265
        self.angle_labels[idx].config(
            text=f"{float(val):+.3f} rad  ({deg:+6.1f}°)")
        # 标记需要发送（实际发送在定时器里做，限频）
        with self._send_lock:
            self._last_sent[idx] = None  # 强制下次发送

    def _toggle_gripper(self):
        if self.gripper >= -0.001:
            self.gripper = GRIPPER_CLOSE
            self.grip_btn.config(text="夹爪：关  CLOSE", bg="#f44336")
        else:
            self.gripper = GRIPPER_OPEN
            self.grip_btn.config(text="夹爪：开  OPEN",  bg="#4CAF50")
        threading.Thread(target=self.arm.arm_set_gripper_linear_position,
                         args=(self.gripper,), daemon=True).start()
        self.status_var.set(f"夹爪 {'关闭' if self.gripper < -0.001 else '打开'}")

    def _home(self):
        for var in self.sliders:
            var.set(0.0)
        with self._send_lock:
            self._last_sent = [None] * 6
        self.status_var.set("已归零")

    def _record(self):
        joints = [float(v.get()) for v in self.sliders]
        self.waypoints.append({
            "joints":  joints,
            "gripper": float(self.gripper),
            "wait":    2.0,
        })
        self._refresh_wp_list()
        self.status_var.set(f"已记录路径点 [{len(self.waypoints)}]")

    def _delete_last(self):
        if self.waypoints:
            self.waypoints.pop()
            self._refresh_wp_list()
            self.status_var.set("已删除最后一个路径点")

    def _save_quit(self):
        if not self.waypoints:
            if not messagebox.askyesno("提示", "没有路径点，确定不保存直接退出？"):
                return
        else:
            seqs = load_sequences()
            seqs[self.action_name] = self.waypoints
            save_sequences(seqs)
            messagebox.showinfo("保存成功",
                f"已保存 '{self.action_name}'\n共 {len(self.waypoints)} 个路径点\n\n{SEQUENCES_FILE}")
        self.arm.ControlTorque("free")
        self.root.destroy()

    # ── 定时发送（20 Hz）────────────────────────────────────

    def _schedule_send(self):
        self._do_send()
        self.root.after(50, self._schedule_send)

    def _do_send(self):
        joints = np.array([float(v.get()) for v in self.sliders], dtype=np.float32)
        with self._send_lock:
            changed = any(self._last_sent[i] != joints[i] for i in range(6))
            if changed:
                self._last_sent = joints.tolist()
        if changed:
            threading.Thread(target=self.arm.SetAllServoRadian,
                             args=(joints,), daemon=True).start()

    # ── 路径点列表刷新 ──────────────────────────────────────

    def _refresh_wp_list(self):
        self.wp_text.config(state=tk.NORMAL)
        self.wp_text.delete("1.0", tk.END)
        for i, wp in enumerate(self.waypoints):
            j = wp["joints"]
            g = "开" if wp["gripper"] >= -0.001 else "关"
            line = (f"[{i+1:2d}] 夹爪={g}  " +
                    "  ".join(f"J{k+1}:{j[k]:+.3f}" for k in range(6)) + "\n")
            self.wp_text.insert(tk.END, line)
        self.wp_text.config(state=tk.DISABLED)
        self.wp_text.see(tk.END)


# ── 主程序 ──────────────────────────────────────────────────

def main():
    if len(sys.argv) < 2:
        print("用法: python teach.py <动作名称>")
        print("      python teach.py --list")
        sys.exit(1)

    if sys.argv[1] == "--list":
        seqs = load_sequences()
        if not seqs:
            print("尚无录制动作。")
        else:
            for name, wps in seqs.items():
                print(f"  '{name}'  —  {len(wps)} 个路径点")
        return

    action_name = " ".join(sys.argv[1:])

    OBSERVATION_JOINTS = np.array([
        -1.6493361431346414,
        -0.03490658503988659,
        0.1466076571675237,
        -0.05235987755982988,
        -1.075122819228507,
        -0.024434609527920613,
    ], dtype=np.float32)

    ps  = load_pysagittarius()
    arm = ps.SagittariusArmReal("/dev/ttyACM1", 1000000, 500, 5)
    arm.SetFreeAfterDestructor(False)
    arm.ControlTorque("lock")
    time.sleep(0.5)
    arm.SetAllServoRadian(OBSERVATION_JOINTS)
    time.sleep(1.5)

    root = tk.Tk()
    app  = TeachApp(root, arm, action_name, init_joints=OBSERVATION_JOINTS)
    root.protocol("WM_DELETE_WINDOW", app._save_quit)
    root.mainloop()


if __name__ == "__main__":
    main()
