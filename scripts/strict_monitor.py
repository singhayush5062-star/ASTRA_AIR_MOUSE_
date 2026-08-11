#!/usr/bin/env python3
import csv
import os
import subprocess

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from std_msgs.msg import Bool

OUT_DIR = os.environ.get("STRICT_MONITOR_OUT", "/tmp/strict_monitor")
os.makedirs(OUT_DIR, exist_ok=True)

state_csv = os.path.join(OUT_DIR, "state_pose_completion.csv")
events_log = os.path.join(OUT_DIR, "events.log")
pub_log = os.path.join(OUT_DIR, "setpoint_publishers.log")
node_log = os.path.join(OUT_DIR, "nodes_snapshot.log")

rospy.init_node("strict_mission_monitor", anonymous=True)

state_msg = None
pose_msg = None
completion_msg = None


def state_cb(msg):
    global state_msg
    state_msg = msg


def pose_cb(msg):
    global pose_msg
    pose_msg = msg


def completion_cb(msg):
    global completion_msg
    completion_msg = msg


rospy.Subscriber("/mavros/state", State, state_cb, queue_size=100)
rospy.Subscriber("/mavros/local_position/pose", PoseStamped, pose_cb, queue_size=100)
rospy.Subscriber("/exploration_completed", Bool, completion_cb, queue_size=20)

start = rospy.Time.now().to_sec()
max_runtime = float(os.environ.get("STRICT_MONITOR_RUNTIME", "230"))
rate = rospy.Rate(10)

# Position-near-static detector
last_ref = None
last_ref_t = None
static_threshold_xy = 0.20
static_threshold_z = 0.08
static_long_sec = 25.0
static_reported = False

# Premature landing detector
max_seen_z = -1e9
near_ground_threshold = 0.20
near_ground_sec = 6.0
near_ground_start = None
premature_reported = False

last_mode = None
last_armed = None
last_completion = None
last_pub_snapshot_t = 0.0

with open(state_csv, "w", newline="") as f_csv, open(events_log, "w") as f_evt, open(pub_log, "w") as f_pub, open(node_log, "w") as f_nodes:
    w = csv.writer(f_csv)
    w.writerow([
        "ros_time",
        "elapsed",
        "armed",
        "mode",
        "connected",
        "x",
        "y",
        "z",
        "exploration_completed",
    ])

    f_evt.write("Strict monitor started\n")
    f_evt.flush()

    while (not rospy.is_shutdown()) and (rospy.Time.now().to_sec() - start) < max_runtime:
        now = rospy.Time.now().to_sec()
        elapsed = now - start

        armed = state_msg.armed if state_msg else ""
        mode = state_msg.mode if state_msg else ""
        connected = state_msg.connected if state_msg else ""
        x = pose_msg.pose.position.x if pose_msg else ""
        y = pose_msg.pose.position.y if pose_msg else ""
        z = pose_msg.pose.position.z if pose_msg else ""
        completed = completion_msg.data if completion_msg else False

        w.writerow([now, elapsed, armed, mode, connected, x, y, z, completed])

        if mode != last_mode and mode != "":
            f_evt.write("[%8.3fs] MODE_CHANGE: %s -> %s\n" % (elapsed, str(last_mode), str(mode)))
            last_mode = mode
        if armed != last_armed and armed != "":
            f_evt.write("[%8.3fs] ARMED_CHANGE: %s -> %s\n" % (elapsed, str(last_armed), str(armed)))
            last_armed = armed
        if completed != last_completion:
            f_evt.write("[%8.3fs] EXPLORATION_COMPLETED: %s\n" % (elapsed, str(completed)))
            last_completion = completed

        if pose_msg:
            try:
                max_seen_z = max(max_seen_z, float(z))
            except Exception:
                pass

            if last_ref is None:
                last_ref = (x, y, z)
                last_ref_t = now
            else:
                try:
                    dx = abs(float(x) - float(last_ref[0]))
                    dy = abs(float(y) - float(last_ref[1]))
                    dz = abs(float(z) - float(last_ref[2]))
                except Exception:
                    dx = dy = dz = 999.0

                if dx <= static_threshold_xy and dy <= static_threshold_xy and dz <= static_threshold_z:
                    static_dur = now - last_ref_t
                    if static_dur >= static_long_sec and not static_reported:
                        f_evt.write(
                            "[%8.3fs] STATIC_ALERT: near (%.2f, %.2f, %.2f) for %.1fs\n"
                            % (elapsed, float(last_ref[0]), float(last_ref[1]), float(last_ref[2]), static_dur)
                        )
                        static_reported = True
                else:
                    last_ref = (x, y, z)
                    last_ref_t = now
                    static_reported = False

            if max_seen_z >= 1.0:
                try:
                    zf = float(z)
                except Exception:
                    zf = 999.0

                if zf <= near_ground_threshold:
                    if near_ground_start is None:
                        near_ground_start = now
                    elif (now - near_ground_start) >= near_ground_sec and not premature_reported:
                        f_evt.write(
                            "[%8.3fs] PREMATURE_LAND_ALERT: z<=%.2fm for %.1fs after max_z=%.2f\n"
                            % (elapsed, near_ground_threshold, (now - near_ground_start), max_seen_z)
                        )
                        premature_reported = True
                else:
                    near_ground_start = None

        if (now - last_pub_snapshot_t) >= 5.0:
            last_pub_snapshot_t = now
            try:
                info = subprocess.check_output(
                    "bash -lc 'rostopic info /mavros/setpoint_position/local'",
                    shell=True,
                    stderr=subprocess.STDOUT,
                    timeout=4,
                    text=True,
                )
                f_pub.write("\n=== t+%.1fs ===\n%s\n" % (elapsed, info))
            except Exception as e:
                f_pub.write("\n=== t+%.1fs ===\nERROR: %s\n" % (elapsed, str(e)))

            try:
                nodes = subprocess.check_output(
                    "bash -lc 'rosnode list | sort'",
                    shell=True,
                    stderr=subprocess.STDOUT,
                    timeout=4,
                    text=True,
                )
                f_nodes.write("\n=== t+%.1fs ===\n%s\n" % (elapsed, nodes))
            except Exception as e:
                f_nodes.write("\n=== t+%.1fs ===\nERROR: %s\n" % (elapsed, str(e)))

            f_pub.flush()
            f_nodes.flush()

        f_evt.flush()
        rate.sleep()

print("strict monitor finished")
print(state_csv)
print(events_log)
print(pub_log)
print(node_log)
