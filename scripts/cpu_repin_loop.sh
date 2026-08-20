#!/bin/bash
# Periodically re-applies CPU-core pinning to the NIDAR sim/flight-control processes.
#
# A one-shot `taskset -pc <cores> <pid>` only pins the single thread it's called on, and only
# catches threads that exist at the moment it runs. gzserver in particular spawns worker threads
# continuously (physics stepping, sensor plugins, ROS transport) after startup, so a one-time pin
# gets left behind as new, unpinned threads appear -- confirmed empirically: `ps -eLo pid,psr` on a
# freshly-pinned gzserver still showed threads executing on cores well outside the target pair
# within seconds. cgroups (cpuset) would enforce this properly for a process's whole thread/child
# tree automatically, but /sys/fs/cgroup is read-only in this environment even with root, so this
# loop is the practical alternative: keep re-pinning on an interval so newly spawned threads get
# caught quickly instead of drifting indefinitely.
#
# Started in the background by test_takeoff.sh once all target processes are up; killed by the
# next test_takeoff.sh invocation's cleanup step, same as every other process it starts.

pin_all_threads() {
    local pattern="$1" cores="$2" pid tid
    for pid in $(pgrep -f "$pattern"); do
        for tid in /proc/"$pid"/task/*; do
            [ -d "$tid" ] || continue
            taskset -pc "$cores" "$(basename "$tid")" >/dev/null 2>&1
        done
    done
}

while true; do
    pin_all_threads "gzserver" "2,3"
    pin_all_threads "bin/px4" "4,5"
    pin_all_threads "mavros_node" "6,7"
    pin_all_threads "fastlio_mapping" "0,1"
    pin_all_threads "flight_envelope_guard.py" "6,7"
    pin_all_threads "relay_odometry.py" "6,7"
    sleep 3
done
