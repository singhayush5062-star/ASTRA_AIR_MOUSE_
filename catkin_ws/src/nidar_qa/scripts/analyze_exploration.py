#!/usr/bin/env python3
"""Live mission monitor + revisit analysis.

Answers two questions a log grep cannot:

  1. HOW MUCH IS LEFT.  /sdf_map/coverage carries the map's own free/occupied/unknown counts
     against the arena's measured navigable area, so progress is a real fraction rather than a
     guess from the published point clouds (which only carry OCCUPIED cells).

  2. WHY IT RE-MAPS.  Revisiting is not automatically waste: reaching new space usually means
     flying back through space already mapped. What matters is whether coverage MOVES while
     the vehicle is somewhere it has already been. So every revisit is scored by the coverage
     gained during it:
         PRODUCTIVE  transit through known space on the way to new space
         WASTED      time spent where nothing new was learned
     and separately, every viewpoint FUEL commits to is checked against the ones it already
     committed to, because re-targeting a place it has already finished is a different failure
     (a frontier that will not die) from merely flying through it.

Usage:  python3 catkin_ws/src/nidar_qa/scripts/analyze_exploration.py [seconds]   (default: until Ctrl-C)
Writes a JSON summary next to the log for later comparison.
"""
import json
import math
import os
import sys
import time

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64MultiArray, String

CELL = 1.0          # visit-grid cell, metres
REVISIT_GAP = 20.0  # seconds away before returning counts as a revisit, not as loitering
TARGET_SAME = 1.0   # metres: two viewpoints this close are "the same place"


class Analyzer(object):
    def __init__(self):
        self.t0 = None
        self.cov = None            # latest coverage message
        self.cov_hist = []         # (t, free_area, pct)
        self.pos = None
        self.visits = {}           # cell -> [(t_enter, t_exit, free_area_at_enter)]
        self.cur_cell = None
        self.cur_enter = None
        self.revisits = []         # dicts
        self.targets = []          # (t, x, y)
        self.repeat_targets = []
        self.state = ''
        self.completed = False
        self.path_len = 0.0
        self.last_xy = None

        rospy.Subscriber('/sdf_map/coverage', Float64MultiArray, self.cov_cb, queue_size=2)
        rospy.Subscriber('/Fast_LIO/odometry', Odometry, self.odom_cb, queue_size=20)
        rospy.Subscriber('/exploration/next_view', PoseStamped, self.view_cb, queue_size=20)
        rospy.Subscriber('/edm/mission_state', String, self.state_cb, queue_size=5)
        rospy.Subscriber('/exploration_completed', Bool, self.done_cb, queue_size=1)

    # ---- callbacks ------------------------------------------------------------------
    def cov_cb(self, m):
        d = m.data
        if len(d) < 9:
            return
        self.cov = {'free_cells': d[0], 'occ_cells': d[1], 'unk_cells': d[2],
                    'free_area': d[3], 'known_area': d[4], 'unknown_area': d[5],
                    'arena_area': d[6], 'pct': d[7], 'left': d[8]}
        self.cov_hist.append((self.now(), d[3], d[7]))

    def state_cb(self, m):
        if m.data != self.state:
            print("  [state] %s -> %s   (t+%.0fs)" % (self.state or 'INIT', m.data, self.now()))
            self.state = m.data

    def done_cb(self, m):
        if m.data and not self.completed:
            self.completed = True
            print("  [FUEL] exploration COMPLETE at t+%.0fs" % self.now())

    def view_cb(self, m):
        t = self.now()
        x, y = m.pose.position.x, m.pose.position.y
        for (pt, px, py) in self.targets:
            if math.hypot(x - px, y - py) < TARGET_SAME and t - pt > REVISIT_GAP:
                self.repeat_targets.append(
                    {'t': t, 'x': x, 'y': y, 'first_t': pt, 'gap': t - pt,
                     'pct': self.cov['pct'] if self.cov else None})
                break
        self.targets.append((t, x, y))

    def odom_cb(self, m):
        if self.t0 is None:
            self.t0 = rospy.Time.now().to_sec()
        t = self.now()
        p = m.pose.pose.position
        self.pos = (p.x, p.y)
        if self.last_xy is not None:
            self.path_len += math.hypot(p.x - self.last_xy[0], p.y - self.last_xy[1])
        self.last_xy = (p.x, p.y)

        cell = (int(math.floor(p.x / CELL)), int(math.floor(p.y / CELL)))
        if cell == self.cur_cell:
            return
        # left a cell: close it out
        if self.cur_cell is not None:
            self.visits.setdefault(self.cur_cell, []).append(
                (self.cur_enter, t, self.enter_area))
        # entering a cell we have been in before, after a real absence?
        prev = self.visits.get(cell)
        area_now = self.cov['free_area'] if self.cov else 0.0
        if prev:
            last_exit = prev[-1][1]
            if t - last_exit > REVISIT_GAP:
                self.revisits.append({'cell': cell, 't': t, 'gap': t - last_exit,
                                      'area_at_prev_exit': prev[-1][2],
                                      'area_now': area_now, 'n_prev': len(prev)})
        self.cur_cell = cell
        self.cur_enter = t
        self.enter_area = area_now

    def now(self):
        if self.t0 is None:
            return 0.0
        return rospy.Time.now().to_sec() - self.t0

    # ---- reporting ------------------------------------------------------------------
    def score_revisits(self):
        """A revisit is productive if coverage grew while the vehicle was back in that cell.

        Scored against the coverage 20 s AFTER the revisit rather than instantaneously,
        because the point of flying through known space is what you reach on the far side.
        """
        out = []
        for r in self.revisits:
            after = None
            for (t, area, _) in self.cov_hist:
                if t >= r['t'] + 20.0:
                    after = area
                    break
            if after is None and self.cov_hist:
                after = self.cov_hist[-1][1]
            gain = (after - r['area_now']) if after is not None else 0.0
            r['gain_m2'] = gain
            r['productive'] = gain > 0.5
            out.append(r)
        return out

    def report(self):
        print("\n" + "=" * 78)
        print("MISSION REPORT   t+%.0fs" % self.now())
        print("=" * 78)
        if self.cov:
            c = self.cov
            print("COVERAGE")
            print("  arena navigable area      %8.1f m2   (measured off the mesh)"
                  % c['arena_area'])
            print("  mapped free               %8.1f m2   = %.1f%%" % (c['free_area'], c['pct']))
            print("  mapped as wall/obstacle   %8.1f m2" % (c['known_area'] - c['free_area']))
            print("  LEFT TO MAP               %8.1f m2" % c['left'])
            print("  unknown cells remaining   %8d" % int(c['unk_cells']))
        else:
            print("COVERAGE: no /sdf_map/coverage received - is FUEL running?")
        print("  distance flown            %8.1f m" % self.path_len)
        print("  exploration completed     %s" % ("YES" if self.completed else "no"))

        rev = self.score_revisits()
        prod = [r for r in rev if r['productive']]
        waste = [r for r in rev if not r['productive']]
        print("\nREVISITS  (returning to a 1 m cell after >%.0fs away)" % REVISIT_GAP)
        print("  total %d:  %d productive (coverage grew), %d wasted (nothing new learned)"
              % (len(rev), len(prod), len(waste)))
        if waste:
            print("  worst wasted revisits:")
            for r in sorted(waste, key=lambda r: -r['n_prev'])[:8]:
                print("    cell %-12s visit #%d, %.0fs after the last, coverage %+.2f m2"
                      % (str(r['cell']), r['n_prev'] + 1, r['gap'], r['gain_m2']))

        print("\nREPEAT TARGETS  (FUEL committing again to a viewpoint within %.1f m)" % TARGET_SAME)
        print("  %d of %d viewpoints were repeats" % (len(self.repeat_targets), len(self.targets)))
        for r in self.repeat_targets[:8]:
            print("    (%.2f, %.2f) re-targeted %.0fs later, coverage %s%%"
                  % (r['x'], r['y'], r['gap'],
                     ("%.1f" % r['pct']) if r['pct'] is not None else "n/a"))

        print("\nVERDICT")
        if not rev and not self.repeat_targets:
            print("  no revisiting observed in this window.")
        elif len(waste) > len(prod):
            print("  MOST REVISITS WERE UNPRODUCTIVE. Coverage did not move while the vehicle")
            print("  was back in space it had already mapped - this is the re-scanning you saw.")
        else:
            print("  Revisits were mostly transit: coverage grew each time the vehicle passed")
            print("  back through mapped space, which is normal for a single-entrance arena.")
        if self.repeat_targets:
            print("  %d repeat VIEWPOINTS: FUEL re-committed to places it had already been sent"
                  % len(self.repeat_targets))
            print("  to. That is a frontier not dying, not merely a transit.")

        out = {'coverage': self.cov, 'path_len_m': self.path_len,
               'completed': self.completed, 'duration_s': self.now(),
               'revisits': rev, 'repeat_targets': self.repeat_targets,
               'n_targets': len(self.targets),
               'coverage_history': self.cov_hist[-500:]}
        path = '/tmp/exploration_analysis.json'
        with open(path, 'w') as f:
            json.dump(out, f, indent=1, default=str)
        print("\n  full data: %s" % path)
        print("=" * 78)


def main():
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 1e9
    rospy.init_node('analyze_exploration', anonymous=True, disable_signals=True)
    a = Analyzer()
    print("analyze_exploration: watching. Ctrl-C for the report.")
    deadline = time.time() + dur
    last = 0
    try:
        while time.time() < deadline and not rospy.is_shutdown():
            time.sleep(1.0)
            if time.time() - last > 30:
                last = time.time()
                if a.cov:
                    print("  t+%4.0fs  %5.1f%% mapped  %6.1f m2 left  %3d revisits  "
                          "%3d repeat targets  %s"
                          % (a.now(), a.cov['pct'], a.cov['left'], len(a.revisits),
                             len(a.repeat_targets), a.state))
                else:
                    print("  t+%4.0fs  waiting for /sdf_map/coverage   %s" % (a.now(), a.state))
    except KeyboardInterrupt:
        pass
    a.report()
    return 0


if __name__ == '__main__':
    sys.exit(main())
