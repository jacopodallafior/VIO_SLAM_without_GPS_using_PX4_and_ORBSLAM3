import sys
import argparse
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # abilita 3D

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleOdometry


def read_bag(bag_path):
    reader = SequentialReader()
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions('', '')
    reader.open(storage_options, converter_options)

    slam_t, slam_p = [], []
    px4_t, px4_p = [], []

    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        t = float(t_ns) * 1e-9  # bag time [s]

        if topic == "/orb_slam3/pose":
            msg = deserialize_message(data, PoseStamped)
            p = msg.pose.position
            slam_t.append(t)
            slam_p.append([p.x, p.y, p.z])

        elif topic == "/fmu/out/vehicle_odometry":
            msg = deserialize_message(data, VehicleOdometry)
            px4_t.append(t)
            px4_p.append([msg.position[0], msg.position[1], msg.position[2]])  # NED

    return (np.array(slam_t), np.array(slam_p),
            np.array(px4_t),  np.array(px4_p))


def align_origin(P):
    return P - P[0]


def time_match(slam_t, slam_p, px4_t, px4_p):
    idx = np.searchsorted(px4_t, slam_t)
    idx = np.clip(idx, 1, len(px4_t) - 1)

    left = idx - 1
    right = idx
    choose_right = (np.abs(px4_t[right] - slam_t) < np.abs(px4_t[left] - slam_t))
    nn = np.where(choose_right, right, left)

    print("[DEBUG] unique matched px4 indices:", len(np.unique(nn)), "/", len(nn))
    return slam_p, px4_p[nn], slam_t


def ned_to_enu(P_ned):
    # NED [N,E,D] -> ENU [E,N,U]
    N = P_ned[:, 0]
    E = P_ned[:, 1]
    D = P_ned[:, 2]
    return np.stack([E, N, -D], axis=1)


def best_yaw_rotation_2d(A_xy, B_xy):
    # trova theta che minimizza || R*A - B || in LS (no scala)
    ax, ay = A_xy[:, 0], A_xy[:, 1]
    bx, by = B_xy[:, 0], B_xy[:, 1]
    num = np.sum(ax * by - ay * bx)
    den = np.sum(ax * bx + ay * by)
    theta = np.arctan2(num, den)
    c, s = np.cos(theta), np.sin(theta)
    R = np.array([[c, -s],
                  [s,  c]])
    return theta, R


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag", help="path rosbag2 folder")
    ap.add_argument("--enu", action="store_true",
                    help="convert PX4 NED -> ENU before plotting")
    ap.add_argument("--yaw_align", action="store_true",
                    help="estimate constant yaw offset and rotate ORB in XY to match PX4")
    ap.add_argument("--max_points", type=int, default=2000,
                    help="decimate to at most this many matched samples for plotting")
    args = ap.parse_args()

    bag_path = args.bag
    slam_t, slam_p, px4_t, px4_p = read_bag(bag_path)

    print("[INFO] slam samples:", len(slam_p))
    print("[INFO] px4  samples:", len(px4_p))
    if len(slam_t) > 0:
        print(f"[INFO] slam time range: {slam_t[0]:.3f} -> {slam_t[-1]:.3f} (dt={slam_t[-1]-slam_t[0]:.3f}s)")
    if len(px4_t) > 0:
        print(f"[INFO] px4  time range: {px4_t[0]:.3f} -> {px4_t[-1]:.3f} (dt={px4_t[-1]-px4_t[0]:.3f}s)")

    if len(slam_p) < 10 or len(px4_p) < 10:
        print("Not enough samples. Check that /orb_slam3/pose and /fmu/out/vehicle_odometry are in the bag.")
        sys.exit(1)

    print("[DEBUG] px4 position std:", np.std(px4_p, axis=0))
    print("[DEBUG] px4 position min:", np.min(px4_p, axis=0))
    print("[DEBUG] px4 position max:", np.max(px4_p, axis=0))

    slam_p_m, px4_p_m, t_m = time_match(slam_t, slam_p, px4_t, px4_p)

    # opzionale: PX4 NED -> ENU
    if args.enu:
        px4_p_m = ned_to_enu(px4_p_m)

    # origin align
    slam_a = align_origin(slam_p_m)
    px4_a  = align_origin(px4_p_m)

    # opzionale: yaw align (ruota ORB in XY per matchare PX4)
    if args.yaw_align:
        theta, R = best_yaw_rotation_2d(slam_a[:, :2], px4_a[:, :2])
        slam_xy = (R @ slam_a[:, :2].T).T
        slam_a[:, 0] = slam_xy[:, 0]
        slam_a[:, 1] = slam_xy[:, 1]
        print(f"[INFO] yaw offset = {theta:.4f} rad ({np.degrees(theta):.2f} deg)")

    # decimazione per plot leggibile
    N = len(t_m)
    step = max(1, N // max(1, args.max_points))
    slam_a = slam_a[::step]
    px4_a  = px4_a[::step]
    t_m    = t_m[::step]

    tt = t_m - t_m[0]

    # ---- FIGURE 1: XY ----
    plt.figure(figsize=(8, 8))
    plt.plot(px4_a[:, 0], px4_a[:, 1], 'g-', label="PX4 XY")
    plt.plot(slam_a[:, 0], slam_a[:, 1], 'r--', label="ORB-SLAM3 XY")
    plt.axis("equal")
    plt.grid(True)
    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.legend()
    plt.title("XY trajectory (origin-aligned)")

    # ---- FIGURE 2: X(t) ----
    plt.figure()
    plt.plot(tt, px4_a[:, 0], label="PX4 X")
    plt.plot(tt, slam_a[:, 0], label="ORB X")
    plt.legend()
    plt.grid(True)
    plt.xlabel("t [s]")
    plt.ylabel("X [m]")
    plt.title("X vs time (origin-aligned)")

    # ---- FIGURE 3: Y(t) ----
    plt.figure()
    plt.plot(tt, px4_a[:, 1], label="PX4 Y")
    plt.plot(tt, slam_a[:, 1], label="ORB Y")
    plt.legend()
    plt.grid(True)
    plt.xlabel("t [s]")
    plt.ylabel("Y [m]")
    plt.title("Y vs time (origin-aligned)")

    # ---- FIGURE 4: Z(t) ----
    plt.figure()
    plt.plot(tt, px4_a[:, 2], label="PX4 Z")
    plt.plot(tt, slam_a[:, 2], label="ORB Z")
    plt.legend()
    plt.grid(True)
    plt.xlabel("t [s]")
    plt.ylabel("Z [m]")
    plt.title("Z vs time (origin-aligned)")

    # ---- FIGURE 5: 3D ----
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(px4_a[:, 0], px4_a[:, 1], px4_a[:, 2], label="PX4 vehicle_odometry")
    ax.plot(slam_a[:, 0], slam_a[:, 1], slam_a[:, 2], label="ORB-SLAM3 pose")
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")
    ax.legend()
    ax.set_title("3D trajectory (origin-aligned)")

    plt.show()


if __name__ == "__main__":
    main()
