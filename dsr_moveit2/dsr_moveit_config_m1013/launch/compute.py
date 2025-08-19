#!/usr/bin/env python3
import subprocess
import re
import numpy as np

def run_tf2_echo(parent, child):
    """ros2 run tf2_ros tf2_echo 실행해서 첫 transform 좌표 리턴"""
    cmd = ["ros2", "run", "tf2_ros", "tf2_echo", parent, child]
    # 표준출력 스트림 잡기
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

    xyz = None
    for line in proc.stdout:
        m = re.search(r"Translation:.*x: ([\-\d\.]+).*y: ([\-\d\.]+).*z: ([\-\d\.]+)", line)
        if m:
            xyz = np.array([float(m.group(1)), float(m.group(2)), float(m.group(3))])
            proc.kill()
            break
    return xyz

def main():
    parent = "robotiq_85_base_link"
    left   = "robotiq_85_left_finger_tip_link"
    right  = "robotiq_85_right_finger_tip_link"

    pL = run_tf2_echo(parent, left)
    pR = run_tf2_echo(parent, right)

    if pL is None or pR is None:
        print("[ERR] 좌표를 못 읽어옴. TF가 안올라왔을 가능성")
        return

    p  = 0.5*(pL + pR)
    print(f"left : {pL}")
    print(f"right: {pR}")
    print(f"mid  : {p}")

    print(f'<origin xyz="{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}" rpy="0 0 0"/>')

if __name__ == "__main__":
    main()
