import rclpy
import time
from rclpy.logging import get_logger

# ================================
# Robot Config
# ================================
ROBOT_ID    = ""
ROBOT_MODEL = "m1013"

import DR_init
DR_init.__dsr__id    = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

logger = get_logger('amovel_compliance_test')

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('amovel_compliance_test_node', namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            movej, movel, amovel,
            task_compliance_ctrl, release_compliance_ctrl,
            set_velx, set_accx, set_robot_mode,
            posj, posx,
            DR_BASE, DR_TOOL,
            MOVE_MODE_ABSOLUTE,
            MOVE_REFERENCE_BASE,
            BLENDING_SPEED_TYPE_DUPLICATE,
            ROBOT_MODE_AUTONOMOUS
        )
    except ImportError as e:
        logger.error(f"DSR_ROBOT2 import failed: {e}")
        return

    logger.info("=== amovel + task_compliance_ctrl MOTION_HOLD Test Start ===")

    # ================================
    # Basic Robot Setup
    # ================================
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    set_velx(30, 20)
    set_accx(60, 40)

    velx = [50, 50]
    accx = [100, 100]

    # ================================
    # Home & Target Poses
    # ================================
    home = posj(0, 0, 90, 0, 90, 0)

    approach = posx(370, 670, 500, 0, 180, 0)   # 접근 위치
    press    = posx(370, 670, 300, 0, 180, 0)   # 접촉(Force 작용 영역)

    # ================================
    # Move to Home
    # ================================
    movej(home, vel=60, acc=60)
    time.sleep(2.0)

    # ================================
    # Task Compliance ON
    # ================================
    stiffness = [3000, 3000, 300, 200, 200, 200]   # Z축만 낮게
    task_compliance_ctrl(stiffness, DR_TOOL)

    time.sleep(1.0)

    # ================================
    # Repeated Test Loop
    # ================================
    test_count = 0

    while rclpy.ok():
        test_count += 1
        logger.warn(f"[TEST] Iteration: {test_count}")

        # ----------------------------
        # 1️⃣ 비동기 직선 접근 (핵심)
        # ----------------------------
        amovel(
            press,
            velx,
            accx,
            0.0,
            MOVE_MODE_ABSOLUTE,
            MOVE_REFERENCE_BASE,
            BLENDING_SPEED_TYPE_DUPLICATE
        )

        # ✅ 일부러 모션 종료 안 기다림 (HOLD 트리거 가능 구간)
        time.sleep(0.1)

        # ----------------------------
        # 2️⃣ 비동기 복귀 명령 중첩
        # ----------------------------
        amovel(
            approach,
            velx,
            accx,
            0.0,
            MOVE_MODE_ABSOLUTE,
            MOVE_REFERENCE_BASE,
            BLENDING_SPEED_TYPE_DUPLICATE
        )

        time.sleep(1.0)

        # ----------------------------
        # 3️⃣ 동기 복귀 (기준선)
        # ----------------------------
        movel(approach, velx, accx)
        time.sleep(1.0)

        # 5회마다 잠깐 정지
        if test_count % 5 == 0:
            logger.info("Short pause...")
            time.sleep(0.1)

    # ================================
    # Compliance OFF & Shutdown
    # ================================
    release_compliance_ctrl()
    movej(home, vel=60, acc=60)

    logger.info("=== Test Finished ===")
    rclpy.shutdown()

if __name__ == "__main__":
    main()
