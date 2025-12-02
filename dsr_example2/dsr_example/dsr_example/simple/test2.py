import rclpy
from rclpy.logging import get_logger
import time

ROBOT_ID    = ""
ROBOT_MODEL = "m1013"

import DR_init
DR_init.__dsr__id    = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

logger = get_logger("force_ref_compare_test")


def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node("force_ref_compare_test", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            movej,
            posj,
            set_robot_mode,
            check_force_condition,
            DR_AXIS_Z,
            DR_BASE,
            DR_TOOL,
            ROBOT_MODE_AUTONOMOUS,
            get_tool_force,
            task_compliance_ctrl,
            set_desired_force,
            DR_FC_MOD_REL,
            release_compliance_ctrl,
            release_force,
            set_ref_coord,
        )
    except ImportError as e:
        print(f"Import Error: {e}")
        return

    set_robot_mode(ROBOT_MODE_AUTONOMOUS)

    # TOOL Z축이 BASE X축을 보도록 90도 회전한 자세
    p_test = posj(0, 0, 90, 0, 90, 0)

    logger.info("검증 자세로 이동 (TOOL Z축이 눕혀진 상태)")
    movej(p_test, vel=50, acc=50)

    stiffness = [3000, 3000, 300, 200, 200, 200]   # Z축만 낮게
    # compliance_ctrl, set_desired_force는 base 기준
    # set_ref_coord(DR_TOOL)
    # task_compliance_ctrl(stiffness)
    time.sleep(1.0)
    set_desired_force(
        [0, 0, -20, 0, 0, 0],
        [0, 0, 1, 0, 0, 0],  
        DR_FC_MOD_REL         # mod: 상대 힘 제어
    )
    try:
        while rclpy.ok():
            f_base = get_tool_force(DR_BASE)
            f_tool = get_tool_force(DR_TOOL)

            ret_base = check_force_condition(
                DR_AXIS_Z,
                min=0.0,
                max=10.0,
                ref=DR_BASE
            )

            ret_tool = check_force_condition(
                DR_AXIS_Z,
                min=0,
                max=10,
                ref=DR_TOOL
            )

            logger.info(
                f"""
                [FORCE]
                BASE  Fz = {f_base[2]:6.2f} N
                TOOL  Fz = {f_tool[2]:6.2f} N

                [COND]ㄴ
                BASE_Z = {ret_base}
                TOOL_Z = {ret_tool}
                """
            )

            rclpy.spin_once(node, timeout_sec=1.0)
        release_compliance_ctrl()

    except KeyboardInterrupt:
        logger.info("사용자 종료 요청")
        release_compliance_ctrl()

    finally:
        # release_force(1.0)
        # release_compliance_ctrl()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
