#!/usr/bin/env python3
"""End the simulation when the flight is over -- crashed, terminated, or wedged.

    python3 tools/flightlog/watchdog.py [--no-kill] [--tilt-deg 60] [--stall-wall-s 60]

WHY THIS IS NEEDED. A PX4 SITL crash does not stop the simulation, it DEADLOCKS it. PX4 and
Gazebo run in lockstep: Gazebo will not advance a physics step until PX4 returns actuator
outputs, and PX4 will not step until Gazebo delivers sensors. When the FailureDetector triggers
flight termination the controller stops producing output, both processes park in futex_wait,
and the pair sits there burning wall-clock at RTF ~0.005 with `[simulator_mavlink] poll timeout`
scrolling. Nothing times out on its own. Observed directly: the clock advanced 476.2 -> 479.4 s
across nine wall minutes after an fd_pitch trip at t=475.

So the vehicle crashing and the simulation ending are two different events, and something has
to connect them. This does, and -- importantly -- it flushes and packs the recording BEFORE
tearing anything down, because a bundle is only worth having if it survives the failure it
documents.

DETECTORS (any one fires):
  tilt        roll/pitch beyond --tilt-deg held for --tilt-hold  (mirrors PX4 FD_FAIL_P/R)
  statustext  PX4 announcing termination/failsafe on /mavros/statustext/recv
  stall       /clock advancing < --stall-sim-s over --stall-wall-s of WALL time = deadlock
  disarm      armed -> disarmed after having flown; the mission is over either way

All timing is WALL clock. Sim time is precisely what stops being trustworthy in the case this
exists to catch, so no rospy.Timer and no rospy.sleep anywhere in this file.
"""
import argparse, math, os, re, subprocess, sys, threading, time

import rospy
from mavros_msgs.msg import State, StatusText
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# PX4 announces the end of a flight in words before the vehicle stops moving.
FATAL_TEXT = re.compile(
    r'attitude failure|triggering terminate|flight termination|failsafe|critical|'
    r'land immediately|manual control lost|emergency', re.I)

# Mirrors the cleanup in scripts/test_takeoff.sh. Orphans from a half-killed stack are not
# harmless here: a surviving fastlio_mapping publishes a SECOND solution onto
# /Fast_LIO/odometry and the next run looks like a SLAM divergence.
KILLALL = ['rosmaster', 'rosout', 'roslaunch', 'gzserver', 'gzclient', 'px4',
           'mavros_node', 'rostopic', 'px4-simulator_mavlink']
PKILL_F = ['flight_envelope_guard.py', 'relay_odometry.py', 'exploration_node', 'traj_server',
           'waypoint_generator', 'fast_lio', 'FAST_LIO', 'cpu_repin_loop.sh', 'rviz',
           'mission_telemetry_logger.py', 'mission_manager.py', 'entry_detection_module.py',
           'robot_state_publisher', 'static_transform_publisher']


class Watchdog(object):
    def __init__(self, a):
        self.a = a
        self.fired = threading.Event()
        self.reason = None
        self.t_start = time.time()
        # "Has this vehicle actually flown?" gates the destructive detectors so that a stack
        # still starting up, or a Gazebo deliberately paused from the GUI to inspect something,
        # is never torn down. Set from altitude OR from having been armed; --assume-flown is
        # for attaching to a simulation that is ALREADY wedged, where neither can be observed.
        self.flew = bool(a.assume_flown)
        self.armed = False
        self.tilt_since = None
        self.clock_s = None
        self.clock_mark = (time.time(), None)

        rospy.Subscriber('/mavros/local_position/odom', Odometry, self._odom, queue_size=20)
        rospy.Subscriber('/mavros/state', State, self._state, queue_size=20)
        rospy.Subscriber('/mavros/statustext/recv', StatusText, self._text, queue_size=50)
        rospy.Subscriber('/clock', Clock, self._clock, queue_size=20)

        t = threading.Thread(target=self._stall_loop, name='watchdog-stall')
        t.daemon = True
        t.start()

    # ------------------------------------------------------------------ detectors
    def _odom(self, m):
        q = m.pose.pose.orientation
        # Angle between the body z axis and world vertical -- one number covering roll+pitch,
        # which is what PX4's FailureDetector actually trips on.
        cz = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, cz))))
        if m.pose.pose.position.z > 0.8:
            self.flew = True
        now = time.time()
        if tilt > self.a.tilt_deg and self.flew:
            self.tilt_since = self.tilt_since or now
            if now - self.tilt_since >= self.a.tilt_hold:
                self.trip('tilt %.0f deg > %.0f for %.1fs' % (tilt, self.a.tilt_deg, self.a.tilt_hold))
        else:
            self.tilt_since = None

    def _state(self, m):
        if m.armed:
            self.flew = True
        if self.armed and not m.armed and self.flew:
            self.trip('disarmed after flight (mode %s)' % m.mode)
        self.armed = m.armed

    def _text(self, m):
        if FATAL_TEXT.search(m.text or ''):
            self.trip('PX4 statustext: %s' % m.text.strip()[:120])

    def _clock(self, m):
        self.clock_s = m.clock.to_sec()

    def _stall_loop(self):
        """The deadlock detector, and the only one that catches a wedged lockstep."""
        while not self.fired.wait(5.0):
            now, (t0, c0) = time.time(), self.clock_mark
            if now - self.t_start < self.a.grace or self.clock_s is None:
                self.clock_mark = (now, self.clock_s)
                continue
            if c0 is None:
                self.clock_mark = (now, self.clock_s)
                continue
            if now - t0 >= self.a.stall_wall_s:
                advanced = self.clock_s - c0
                if advanced < self.a.stall_sim_s and self.flew:
                    self.trip('sim clock advanced %.2f s in %.0f s wall (RTF %.4f) = lockstep '
                              'deadlock' % (advanced, now - t0, advanced / (now - t0)))
                self.clock_mark = (now, self.clock_s)

    # ------------------------------------------------------------------ response
    def trip(self, reason):
        if self.fired.is_set():
            return
        self.reason = reason
        self.fired.set()
        rospy.logwarn('[watchdog] FIRED: %s', reason)

    def respond(self):
        stamp = time.strftime('%Y-%m-%d %H:%M:%S')
        print('\n' + '=' * 74)
        print('[watchdog] %s  ENDING RUN: %s' % (stamp, self.reason))
        print('=' * 74)

        # 1. Flush and pack FIRST. Tearing the stack down destroys /tmp/fuel.log's writer and
        #    leaves the recorder with no chance to write its final state; the whole point of
        #    stopping the sim is to keep what it produced.
        subprocess.call(['pkill', '-INT', '-f', 'flightlog/record.py'])
        time.sleep(3.0)
        try:
            subprocess.call([sys.executable, os.path.join(REPO, 'tools', 'flightlog', 'pack.py'),
                             '--fuel-log', '/tmp/fuel.log'])
        except Exception as exc:
            print('[watchdog] pack failed: %s' % exc)

        with open('/tmp/watchdog_reason.txt', 'w') as f:
            f.write('%s\n%s\n' % (stamp, self.reason))

        if self.a.no_kill:
            print('[watchdog] --no-kill set; leaving the simulation running.')
            return
        print('[watchdog] tearing down the simulation stack...')
        subprocess.call(['killall', '-9'] + KILLALL,
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        for name in PKILL_F:
            subprocess.call(['pkill', '-9', '-f', name],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        print('[watchdog] done. Bundle is under logs/runs/, reason in /tmp/watchdog_reason.txt')


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--tilt-deg', type=float, default=60.0, help='PX4 FD_FAIL_P/R default')
    ap.add_argument('--tilt-hold', type=float, default=0.3, help='PX4 FD_FAIL_P_TTRI default')
    ap.add_argument('--stall-wall-s', type=float, default=60.0)
    ap.add_argument('--stall-sim-s', type=float, default=0.5,
                    help='less sim time than this over --stall-wall-s means deadlocked')
    ap.add_argument('--grace', type=float, default=90.0,
                    help='wall seconds before detectors arm, so startup cannot trip them')
    ap.add_argument('--assume-flown', action='store_true',
                    help='treat the vehicle as having flown; use when attaching to a sim that '
                         'is already wedged, where takeoff can no longer be observed')
    ap.add_argument('--no-kill', action='store_true',
                    help='flush and pack, but leave the simulation running')
    a = ap.parse_args(rospy.myargv()[1:])

    rospy.init_node('nidar_watchdog', anonymous=True, disable_signals=True)
    w = Watchdog(a)
    print('[watchdog] armed: tilt>%.0fdeg/%.1fs, statustext, clock-stall <%.1fs sim per %.0fs '
          'wall, disarm. Grace %.0fs.'
          % (a.tilt_deg, a.tilt_hold, a.stall_sim_s, a.stall_wall_s, a.grace))
    while not w.fired.wait(1.0):
        if rospy.is_shutdown():
            return 0
    w.respond()
    return 0


if __name__ == '__main__':
    sys.exit(main())
