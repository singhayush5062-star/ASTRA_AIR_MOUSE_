#!/usr/bin/env python3
"""Re-apply the /mission/stop_exploration handshake. Idempotent: safe to run any time.

This exists because entry_detection_module.py (now nidar_mission/scripts/mission_manager.py)
and fast_exploration_fsm.cpp have twice been
rewritten back to a pre-handshake state (2026-09-09 09:29:36 and 10:49:15, both times within
the same second, both times only these two files). Each revert silently cost a test run, so
recovery needs to be one command rather than a re-derivation. Run it, then rebuild:

    python3 scripts/apply_handshake_fix.py && catkin build exploration_manager
"""
import os, sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EDM = os.path.join(ROOT, 'catkin_ws/src/nidar_mission/scripts/mission_manager.py')
FSM = os.path.join(ROOT, 'catkin_ws/src/fuel/fuel_planner/exploration_manager/src/fast_exploration_fsm.cpp')
HDR = os.path.join(ROOT, 'catkin_ws/src/fuel/fuel_planner/exploration_manager/include/exploration_manager/fast_exploration_fsm.h')

changed = []

def edit(path, marker, subs):
    """Apply (old, new) pairs to `path` unless `marker` is already present."""
    s = open(path).read()
    if marker in s:
        print('  already present: %s' % os.path.basename(path))
        return
    for old, new in subs:
        if old not in s:
            sys.exit('FAILED: anchor not found in %s:\n%s' % (path, old[:200]))
        s = s.replace(old, new, 1)
    open(path, 'w').write(s)
    changed.append(path)
    print('  patched: %s' % os.path.basename(path))

print('header:')
edit(HDR, 'stop_requested_', [
 ('#include <std_msgs/Empty.h>', '#include <std_msgs/Empty.h>\n#include <std_msgs/Bool.h>'),
 ('  ros::Subscriber trigger_sub_, odom_sub_;',
  '  ros::Subscriber trigger_sub_, odom_sub_, stop_sub_;'),
 ('  bool exploration_completed_sent_;',
  '  bool exploration_completed_sent_;\n'
  '  // Set by /mission/stop_exploration. The mission layer may decide exploration is\n'
  '  // over before FUEL\'s own frontier logic does (coverage plateau); this is the\n'
  '  // handshake that lets it stop us WITHOUT both of us driving /planning/pos_cmd.\n'
  '  bool stop_requested_;'),
 ('  void triggerCallback(const nav_msgs::PathConstPtr& msg);',
  '  void triggerCallback(const nav_msgs::PathConstPtr& msg);\n'
  '  void stopExplorationCallback(const std_msgs::BoolConstPtr& msg);'),
])

print('fsm.cpp:')
_sub = '  odom_sub_ = nh.subscribe("/odom_world", 1, &FastExplorationFSM::odometryCallback, this);'
_trig = 'void FastExplorationFSM::triggerCallback(const nav_msgs::PathConstPtr& msg) {'
edit(FSM, 'stopExplorationCallback', [
 (_sub, _sub + '\n'
  '  // The mission layer (EDM) decides the run is over on a coverage plateau, which can happen\n'
  '  // while our own frontier logic still has work queued. Without this handshake the EDM flips\n'
  '  // to RETURN and starts publishing /planning/pos_cmd while we are still publishing it too --\n'
  '  // measured on runs 20260909_082915 and 20260909_101738. On the latter the plateau fired at\n'
  '  // t=505.4 s and we did not finish until t=594.5 s: 89 s of both of us driving the topic,\n'
  '  // coverage frozen at 99.24 %, then FAST-LIO diverged at t=600 and PX4 failsafed the vehicle\n'
  '  // down 9.1 m from the pad. One publisher at a time.\n'
  '  stop_sub_ = nh.subscribe("/mission/stop_exploration", 1,\n'
  '                           &FastExplorationFSM::stopExplorationCallback, this);'),
 ('  exploration_completed_sent_ = false;\n}',
  '  exploration_completed_sent_ = false;\n  stop_requested_ = false;\n}'),
 (_trig,
  'void FastExplorationFSM::stopExplorationCallback(const std_msgs::BoolConstPtr& msg) {\n'
  '  if (!msg->data || stop_requested_) return;\n'
  '  stop_requested_ = true;\n'
  '  // Go straight to FINISH and skip the re-check budget. The re-checks exist to survive a\n'
  '  // MOMENTARY frontier gap; this is not that -- the mission layer has decided on evidence we\n'
  '  // do not have (a coverage plateau) that there is nothing left worth flying to, so retrying\n'
  '  // exploration planning would only keep us publishing trajectories the mission no longer\n'
  '  // wants. FINISH then publishes /exploration_completed exactly once, which is the signal the\n'
  '  // EDM already waits on before it takes over /planning/pos_cmd.\n'
  '  finish_recheck_count_ = finish_recheck_max_;\n'
  '  ROS_WARN("[FSM] stop requested by mission layer: ending exploration and handing over.");\n'
  '  if (state_ != FINISH) transitState(FINISH, "StopRequest");\n'
  '}\n\n' + _trig),
 (_trig + '\n  if (msg->poses[0].pose.position.z < -0.1) return;\n',
  _trig + '\n  if (msg->poses[0].pose.position.z < -0.1) return;\n'
  '  // Once the mission layer has called a halt, a stray trigger must not restart us. The\n'
  '  // waypoint_generator node self-triggers on this same topic (see below), so this is a real\n'
  '  // path, not a hypothetical one -- and re-arming here would put us back to publishing\n'
  '  // /planning/pos_cmd underneath the EDM\'s return leg.\n'
  '  if (stop_requested_) {\n'
  '    ROS_WARN_THROTTLE(5.0, "[FSM] ignoring trigger: exploration was stopped by the mission.");\n'
  '    return;\n'
  '  }\n'),
])

print('mission_manager.py:')
edit(EDM, 'pub_stop_exploration', [
 ('        self.plateau_fired = False            # one-shot latch',
  '        self.plateau_fired = False            # one-shot latch\n'
  '        self.plateau_stop_sent_time = None    # when we asked FUEL to stop\n'
  '        # If FUEL never answers the stop request we must still come home rather than orbit the\n'
  '        # arena forever, so give up on the handshake after this long and transition anyway.\n'
  '        self.plateau_stop_timeout_s = rospy.get_param(\n'
  "            '/nidar/exploration_completion/stop_handshake_timeout_s', 15.0)"),
 ("        self.pub_pos_cmd = rospy.Publisher('/planning/pos_cmd', PositionCommand, queue_size=10)",
  "        self.pub_pos_cmd = rospy.Publisher('/planning/pos_cmd', PositionCommand, queue_size=10)\n"
  '        # Handshake out of exploration. FUEL transits to FINISH on this and then publishes the\n'
  '        # latched /exploration_completed we already subscribe to; see coverage_cb.\n'
  "        self.pub_stop_exploration = rospy.Publisher('/mission/stop_exploration', Bool,\n"
  '                                                    queue_size=1, latch=True)'),
 ('''        if would and self.plateau_force:
            self.plateau_fired = True
            rospy.logwarn("[EDM] coverage plateau (%.2f%%, +%.2f m2 / %.0f s) -- ending "
                          "exploration and returning.", pct, growth, self.plateau_window_s)
            self.transition_to(MissionState.RETURN)
''',
  '''        if would and self.plateau_force:
            self.plateau_fired = True
            self.request_end_of_exploration(
                "coverage plateau (%.2f%%, +%.2f m2 / %.0f s)"
                % (pct, growth, self.plateau_window_s))
'''),
 ('    def transition_to(self, new_state):',
  '''    def request_end_of_exploration(self, reason):
        """Ask FUEL to stop, instead of transitioning to RETURN underneath it.

        Both endings the mission layer decides on its own -- the coverage plateau and the
        mission clock -- used to call transition_to(RETURN) directly. That leaves FUEL in
        EXEC_TRAJ, still publishing /planning/pos_cmd, while run_return() publishes the same
        topic. Run 20260909_101738 is what that costs: the plateau fired at t=505.4 s but FUEL
        did not finish until t=594.5 s, so for 89 s both drove the topic, coverage sat frozen at
        99.24%, FAST-LIO then diverged at t=600 and PX4 failsafed the vehicle down 9.1 m from
        the pad. Run 20260909_082915 failed the same way for 140 s.

        FUEL answers on the latched /exploration_completed, which completed_cb already handles;
        the watchdog in control_loop transitions anyway if that answer never comes.
        """
        if self.state != MissionState.EXPLORATION:
            return
        self.plateau_stop_sent_time = rospy.Time.now()
        rospy.logwarn("[EDM] %s -- asking FUEL to stop exploring (waiting up to %.0f s for the "
                      "handover).", reason, self.plateau_stop_timeout_s)
        self.pub_stop_exploration.publish(Bool(data=True))

    def transition_to(self, new_state):'''),
 ('''                self.clock_forced_return = True
                rospy.logwarn("[EDM] mission clock %.0f s of %.0f s; %.0f s reserved to fly "
                              "home. Ending exploration and returning.",
                              mission_s, self.clock_limit_s, self.clock_return_margin_s)
                self.transition_to(MissionState.RETURN)''',
  '''                self.clock_forced_return = True
                if self.state == MissionState.EXPLORATION:
                    # Same handshake as the plateau: FUEL owns /planning/pos_cmd right now.
                    self.request_end_of_exploration(
                        "mission clock %.0f s of %.0f s, %.0f s reserved to fly home"
                        % (mission_s, self.clock_limit_s, self.clock_return_margin_s))
                else:
                    # ENTRY_SEARCH / ENTRY_CONFIRMATION: FUEL has not been triggered yet, so
                    # there is no second publisher to hand over from.
                    rospy.logwarn("[EDM] mission clock %.0f s of %.0f s; %.0f s reserved to fly "
                                  "home. Returning.",
                                  mission_s, self.clock_limit_s, self.clock_return_margin_s)
                    self.transition_to(MissionState.RETURN)'''),
 ('        if self.state == MissionState.TAKEOFF:',
  '''        # --- stop-request handshake watchdog ------------------------------------------------
        # completed_cb normally makes this transition when FUEL answers. If it never does, we
        # must not orbit the arena indefinitely -- but we also must not take over pos_cmd early,
        # so this waits the full timeout before forcing it.
        if (self.plateau_stop_sent_time is not None and
                self.state == MissionState.EXPLORATION and
                (rospy.Time.now() - self.plateau_stop_sent_time).to_sec()
                >= self.plateau_stop_timeout_s):
            rospy.logerr("[EDM] FUEL did not acknowledge the stop request within %.0f s; "
                         "returning anyway.", self.plateau_stop_timeout_s)
            self.plateau_stop_sent_time = None
            self.transition_to(MissionState.RETURN)

        if self.state == MissionState.TAKEOFF:'''),
])

print('\n%d file(s) patched.' % len(changed))
if changed:
    print('Rebuild now:  catkin build exploration_manager')
