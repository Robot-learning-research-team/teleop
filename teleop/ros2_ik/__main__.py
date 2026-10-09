# This file is based on SpesRobotics/teleop (https://github.com/SpesRobotics/teleop),
# licensed under the Apache License 2.0.
#
# Modifications, 2026:
#   - Added --lock-z / --z-height: lock the end-effector Z to a fixed base-frame
#     height for planar pushing.
#   - Added --lock-orientation: lock the tool orientation to the start pose
#     ("pointing down") captured at launch.
#   - Added a time gate (send_interval = 0.10 s, 10 Hz) in the teleop callback so
#     the phone command rate matches what the robot can physically execute,
#     preventing the robot from jumping to only the last pose.
#
# The original Apache 2.0 LICENSE and copyright notices are retained; see LICENSE.

import argparse
import threading
import time

from teleop import Teleop

# provera source-a ROS2
try:
    import rclpy
    from std_msgs.msg import String
except ImportError:
    raise ImportError(
        "ROS2 is not sourced. Please source ROS2 before running this script."
    )

try:
    from teleop.utils.jacobi_robot_ros import JacobiRobotROS
except ImportError:
    raise ImportError(
        "JacobiRobotROS is not available. Please install the teleop with [utils] extra."
    )


# parsiranje argumenata iz terminala
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--omit-current-pose", action="store_true", help="Omit usage of current pose"
    )
    parser.add_argument("--host", type=str, default="0.0.0.0", help="Host address")
    parser.add_argument("--port", type=int, default=4443, help="Port number")
    parser.add_argument(
        "--natural-position",
        nargs=3,
        type=float,
        default=[0.0, 0.0, 0.0],
        help="Natural position of the phone",
    )
    parser.add_argument(
        "--natural-orientation",
        nargs=3,
        type=float,
        default=[0.0, -45, 0.0],
        help="Natural orientation of the phone (in degrees)",
    )
    parser.add_argument(
        "--ee-link",
        type=str,
        default="end_effector",
        help="End effector name (e.g., 'panda_hand')",
    )
    parser.add_argument(
        "--joint-names",
        nargs="+",
        default=None,
        help="List of joint names",
    )
    # --- Z-lock (planarno guranje) ---
    parser.add_argument(
        "--lock-z",
        action="store_true",
        help="Zakljucaj Z osu (planarno guranje). Ako se izostavi, Z se prati slobodno.",
    )
    parser.add_argument(
        "--z-height",
        type=float,
        default=None,
        help="Visina Z u base frame-u [m] na koju se zakljucava. "
        "Ako se izostavi, uzima se trenutni Z robota na startu.",
    )
    # ---------------------------------
    # --- Orientation-lock (alat uperen u pod) ---
    parser.add_argument(
        "--lock-orientation",
        action="store_true",
        help="Zakljucaj orijentaciju alata. Jog-uj robota u cistu 'uperen u pod' "
        "pozu PRE pokretanja skripte — ta orijentacija se hvata i drzi.",
    )
    # --------------------------------------------
    parser.add_argument(
        "--ros-args",
        nargs=argparse.REMAINDER,
        help="Arguments to pass to ROS",
        default=[],
    )

    args = parser.parse_args()

    rclpy.init(args=["--ros-args"] + args.ros_args)

    # kreiranje ROS cvorova, Teleop i Robot objekata
    node = rclpy.create_node("teleop")
    gripper_publisher = node.create_publisher(String, "/gripper_command", 1)
    teleop = Teleop(
        host=args.host,
        port=args.port,
        natural_phone_orientation_euler=args.natural_orientation,
        natural_phone_position=args.natural_position,
    )
    robot = JacobiRobotROS(
        node,
        ee_link=args.ee_link,
        joint_names=args.joint_names,
    )

    # postavljanje pocetne referentne poze
    robot.reset_joint_states()
    ee_pose = robot.get_ee_pose()
    teleop.set_pose(ee_pose)

    # --- Z-lock: odredi visinu na koju se zakljucava ---
    lock_z = args.lock_z
    z_lock_value = args.z_height if args.z_height is not None else float(ee_pose[2, 3])
    if lock_z:
        node.get_logger().info(
            f"Z-lock UKLJUCEN: Z = {z_lock_value:.4f} m (base frame)"
        )
    else:
        node.get_logger().info("Z-lock iskljucen (Z se prati slobodno).")
    # ---------------------------------------------------

    # --- Orientation-lock: uhvati fiksnu rotaciju sa starta ---
    lock_orientation = args.lock_orientation
    R_fixed = ee_pose[:3, :3].copy()  # rotacioni blok startne (down) poze
    if lock_orientation:
        node.get_logger().info(
            "Orientation-lock UKLJUCEN: orijentacija zakljucana na startnu "
            "(uperen u pod). Jog-uj u cistu down pozu PRE starta."
        )
    else:
        node.get_logger().info("Orientation-lock iskljucen.")
    # ----------------------------------------------------------

    last_send_time = 0.0
    send_interval = 0.10

    # kada telefon posalje novi paket
    def teleop_pose_callback(pose, params):
        nonlocal teleop
        nonlocal node
        nonlocal robot
        nonlocal last_send_time

        gripper_publisher.publish(String(data=params["gripper"]))

        if not robot.are_joint_states_received():
            return

        if not params["move"]:
            return

        current_time = time.time()
        if current_time - last_send_time >= send_interval:
            # --- Lock: zakljucaj Z i/ili orijentaciju neposredno pre slanja ---
            if lock_z or lock_orientation:
                pose = pose.copy()  # kopija, da ne mutiramo interni akumulator Teleop-a
                if lock_z:
                    pose[2, 3] = z_lock_value
                if lock_orientation:
                    pose[:3, :3] = R_fixed
            # -----------------------------------------------------------------
            robot.servo_to_pose(pose, send_interval)
            last_send_time = current_time

    # pokretanje niti i izvrsavanje
    teleop.subscribe(teleop_pose_callback)

    # start the ros2 node in a separate thread
    t = threading.Thread(
        target=rclpy.spin,
        args=(node,),
        daemon=True,
    )
    t.start()

    teleop.run()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
