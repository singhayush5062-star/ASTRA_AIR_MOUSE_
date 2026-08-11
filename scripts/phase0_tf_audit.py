#!/usr/bin/env python3
import time
import rospy
import tf2_ros


def main():
    rospy.init_node("phase0_tf_audit", anonymous=True)

    out_path = rospy.get_param("~out", "/tmp/phase0_audit/tf_audit.log")
    duration = float(rospy.get_param("~duration", 120.0))
    period = float(rospy.get_param("~period", 0.5))

    # `base_link` belongs to MAVROS/PX4's separate vehicle TF tree.  FUEL and
    # FAST-LIO operate in the localization tree below, where `camera_init` is
    # the FAST-LIO world frame and `body` is its odometry child.
    targets = [
        ("map", "camera_init"),
        ("camera_init", "body"),
        ("map", "body"),
    ]

    buf = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
    tf2_ros.TransformListener(buf)

    deadline = time.time() + duration
    with open(out_path, "w") as f:
        f.write("# ros_time,source,target,ok,latency_sec,error\n")
        while not rospy.is_shutdown() and time.time() < deadline:
            now = rospy.Time.now()
            for source, target in targets:
                ok = True
                err = ""
                latency = -1.0
                try:
                    tr = buf.lookup_transform(source, target, rospy.Time(0), rospy.Duration(0.2))
                    # Static transforms conventionally carry a zero stamp.  It
                    # means "valid for all time", not "published at epoch";
                    # treating it as an age produces a false stale-TF alarm.
                    latency = (
                        0.0
                        if tr.header.stamp == rospy.Time(0)
                        else max(0.0, (now - tr.header.stamp).to_sec())
                    )
                except Exception as exc:
                    ok = False
                    err = str(exc).replace("\n", " ")
                f.write(f"{now.to_sec():.6f},{source},{target},{int(ok)},{latency:.6f},{err}\n")
            f.flush()
            time.sleep(period)


if __name__ == "__main__":
    main()
