#!/usr/bin/env python3
"""Render the derived config files from the central mission_config.yaml.

Three consumers in this stack cannot read the ROS parameter server: FAST-LIO reads its
own YAML by path, FUEL reads roslaunch <param> literals, and Gazebo reads static world
XML. This script pushes the relevant values from mission_config.yaml into those files.

It edits values IN PLACE with targeted substitutions rather than rewriting the files,
because those files carry a lot of hard-won explanatory comments that must survive.

Usage:
    rosrun nidar_config apply_mission_config.py            # apply
    rosrun nidar_config apply_mission_config.py --check    # report drift, change nothing
"""

import argparse
import math
import os
import re
import sys

import yaml

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..'))
CONFIG = os.path.join(REPO, 'catkin_ws', 'src', 'nidar_config', 'config', 'mission_config.yaml')

# The file FAST-LIO actually loads, per launch/fast_lio/nidar_mapping.launch:
#     <rosparam command="load" file=".../config/fast_lio/nidar_sim.yaml" />
# This pointed at catkin_ws/src/FAST_LIO/config/velodyne.yaml until 2026-09-06, which is
# upstream's stock config and is loaded by nothing in this stack. Every regeneration since
# the sim config was introduced wrote blind / extrinsic_T / pcd_save_en into a file no node
# reads, so the generator reported success while the live values silently drifted: it was
# still carrying the iris-era extrinsic_T of [0, 0, 0.12] after the whole X500 migration.
FASTLIO_YAML = os.path.join(REPO, 'config', 'fast_lio', 'nidar_sim.yaml')
# The flight vehicle model. The lidar mount, its standoff mast and the sensor's range window
# are all generated into this file from nidar.lidar, so that raising the lidar cannot leave the
# mast floating in mid air or the FAST-LIO extrinsic pointing at the old height.
VEHICLE_SDF = os.path.join(REPO, 'simulation', 'PX4-Autopilot-v1.14.3', 'Tools', 'simulation',
                           'gazebo-classic', 'sitl_gazebo-classic', 'models', 'x500_vlp16',
                           'x500_vlp16.sdf')

# x500 geometry the lidar mount is constrained by, measured from x500.sdf and its prop meshes.
# UPDATED 2026-09-06 for the shrunk airframe: hubs (+-0.174) -> (+-0.120) and the prop
# collision changed from a 0.2792 m box to a 0.100 m-radius cylinder disc.
PROP_COLL_TIP, PROP_COLL_TOP = 0.2697, 0.0625    # cylinder discs: hypot(.12,.12) + 0.100
PROP_VIS_TIP, PROP_VIS_TOP = 0.2546, 0.0694      # visual meshes: this is the binding pair
TAN15 = 0.2679491924311227                       # tan of the lidar's lowest ring, -15 deg
PLATE_TOP = 0.032                                # top of base_link_collision_0, where the mast foots
LIDAR_BODY_LEN = 0.0717                          # the sensor cylinder's own height

FUEL_XML = os.path.join(REPO, 'catkin_ws', 'src', 'fuel', 'fuel_planner',
                        'exploration_manager', 'launch', 'algorithm.xml')
FUEL_LAUNCH = os.path.join(REPO, 'launch', 'nidar_fuel_upstream.launch')
GUARD_YAML = os.path.join(REPO, 'catkin_ws', 'src', 'nidar_config', 'config', 'flight_envelope_guard.yaml')
WORLD = os.path.join(REPO, 'nidar_competition.world')


def validate(cfg):
    """Reject config values that mean something different from what they say.

    Every check here exists because the value was silently reinterpreted downstream and cost a
    debugging session. A comment would not have caught any of them.
    """
    errors, warnings = [], []
    planner = cfg['planner']

    # SDFMap::clearAndInflateLocalMap does `inf_step = ceil(obstacles_inflation / resolution)`
    # and then inflates a full cube of +-inf_step cells, so any inflation that is not an exact
    # multiple of the map resolution is rounded UP to the next whole cell.
    # 2026-09-05: 0.42 with resolution 0.10 delivered 0.50 m (+19%), which sealed the only
    # corridor out of the arena entry pocket and deadlocked exploration for two sessions.
    res = float(planner['map_resolution'])
    infl = float(planner['obstacles_inflation'])
    steps = infl / res
    if abs(steps - round(steps)) > 1e-9:
        actual = math.ceil(steps) * res
        lo, hi = math.floor(steps) * res, math.ceil(steps) * res
        errors.append(
            'planner.obstacles_inflation %.3f is not a multiple of planner.map_resolution %.3f\n'
            '    -> FUEL will inflate by %.3f m (%d cells, %+.0f%% more than requested)\n'
            '    -> use %.2f (%d cells) or %.2f (%d cells) instead'
            % (infl, res, actual, math.ceil(steps), 100.0 * (actual - infl) / infl,
               lo, int(math.floor(steps)), hi, int(math.ceil(steps))))

    # The planner's ONLY notion of vehicle size is obstacles_inflation, and the vehicle does not
    # fly where the planner puts it -- FAST-LIO/EKF position error against Gazebo ground truth
    # measured 101 mm median / 212 mm p90 (ulog 12_23_19) and 160 / 314 mm (ulog 14_58_08).
    # So the inflation has to cover the airframe PLUS that error, not just the airframe.
    #
    # 2026-09-06: inflation was 0.400 against a 0.386 m collision radius -- a 14 mm margin
    # against a 101 mm error. The result was 50.8% of one flight with the propeller disc inside
    # a wall, the vehicle jammed in a 2 m strip for 226 s, coverage frozen for 406 s, and two
    # flights lost to prop strikes. This single assertion would have caught it before either.
    MIN_CLEARANCE_MARGIN = 0.10          # the measured MEDIAN localisation error
    rad = float(cfg['vehicle']['collision_radius'])
    margin = infl - rad
    if margin < MIN_CLEARANCE_MARGIN:
        errors.append(
            'planner.obstacles_inflation %.4f leaves only %.0f mm over vehicle.collision_radius\n'
            '    %.4f, but measured localisation error is 101 mm median / 212 mm p90.\n'
            '    -> the planner will route the airframe into gaps it does not fit and the\n'
            '       propellers will strike walls; this is not a tuning issue, it is geometry\n'
            '    -> raise obstacles_inflation to at least %.2f, or shrink the vehicle.\n'
            '       NOTE arina_nidar DISCONNECTS above 0.45 inflation (0.6%% reachable at\n'
            '       0.50), so above that the answer must be a smaller vehicle.'
            % (infl, margin * 1000, rad, math.ceil((rad + MIN_CLEARANCE_MARGIN) * 100) / 100.0))

    # The lidar mount height is a hard geometric constraint, not a preference: below this the
    # -15 deg ring is cut by the propellers. Those returns are not merely lost - a blade sliced
    # at 200 Hz appears to FAST-LIO as a wall at 0.3 m that is there on one scan and gone on the
    # next, which is worse than a dropout. Checked against the prop VISUAL meshes, which reach
    # higher (z 0.0694) than the collision boxes (0.0604) and so bind first.
    lid = cfg['lidar']
    mount = float(lid['mount_height'])
    floor = max(PROP_VIS_TOP + PROP_VIS_TIP * TAN15, PROP_COLL_TOP + PROP_COLL_TIP * TAN15)
    if mount <= floor:
        errors.append(
            'lidar.mount_height %.4f is at or below the propeller floor %.4f\n'
            '    -> the -15 deg ring crosses the prop plane at r = %.3f m, inside the blade\n'
            '       sweep (r = 0.070 .. 0.270), so the lower rings are chopped by the blades\n'
            '    -> use at least %.3f' % (mount, floor, (mount - PROP_VIS_TOP) / TAN15,
                                          math.ceil(floor * 1000 + 5) / 1000.0))
    if mount - LIDAR_BODY_LEN / 2.0 <= PLATE_TOP:
        errors.append(
            'lidar.mount_height %.4f puts the sensor body into the centre plate (top %.3f)\n'
            '    -> the standoff mast would have negative length'
            % (mount, PLATE_TOP))

    # A blind radius below the sensor's own floor is not wrong, but it is not doing anything
    # either, and someone will later "fix" the wrong one of the two.
    if float(lid['blind']) < float(lid['min_range']):
        warnings.append(
            'lidar.blind %.3f is below lidar.min_range %.3f: the Gazebo sensor never returns\n'
            '    anything that close, so the blind radius has no effect. Harmless, but lower\n'
            '    min_range too if the intent was to see closer.'
            % (float(lid['blind']), float(lid['min_range'])))

    # THE ALTITUDE DATUM, checked across every file that carries a copy of it.
    # This exists because the rangefinder mount offset and the spawn height are each written
    # into four different files in three different languages, and on 2026-09-06 two of them
    # (FAST-LIO's hardcoded 0.05 / 0.1) were left at iris values through the whole X500
    # migration. Nothing failed loudly: FAST-LIO simply published Z 0.15 m high, which put
    # FUEL's replan start point above its own map ceiling and stalled the kinodynamic search on
    # 99.2% of attempts. The vehicle showed it as hovering for up to 125 s at a time.
    rf = cfg.get('rangefinder')
    if rf is not None:
        off = float(rf['mount_offset'])
        # (a) the vehicle SDF's tfmini mount pose
        veh = VEHICLE_SDF
        if os.path.exists(veh):
            txt = open(veh).read()
            m = re.search(r'<uri>model://tfmini_lidar</uri>.*?<pose>([-0-9.eE ]+)</pose>',
                          txt, re.S)
            if m:
                sdf_z = -float(m.group(1).split()[2])
                if abs(sdf_z - off) > 1e-3:
                    errors.append(
                        'rangefinder.mount_offset %.4f disagrees with the tfmini mount in %s '
                        '(%.4f below base_link)\n'
                        '    -> FAST-LIO would convert range to altitude with the wrong offset'
                        % (off, os.path.basename(veh), sdf_z))
        # (b) PX4's EKF2_RNG_POS_Z in the airframe file
        af = os.path.join(REPO, 'simulation', 'PX4-Autopilot-v1.14.3', 'ROMFS', 'px4fmu_common',
                          'init.d-posix', 'airframes', '1025_gazebo-classic_x500_vlp16')
        if os.path.exists(af):
            m = re.search(r'^param set-default EKF2_RNG_POS_Z\s+([0-9.]+)', open(af).read(), re.M)
            if m and abs(float(m.group(1)) - off) > 1e-3:
                errors.append(
                    'rangefinder.mount_offset %.4f disagrees with EKF2_RNG_POS_Z %.4f in the '
                    'airframe\n'
                    '    -> PX4 would place the sensor at the wrong point and the vehicle would\n'
                    '       over- or under-climb by exactly the difference'
                    % (off, float(m.group(1))))
        # (c) FAST-LIO's camera_init datum must be the spawn height, same as the guard's
        #     spawn_world_z and the world->map TF z, both of which are generated below.
        fl = FASTLIO_YAML
        if os.path.exists(fl):
            txt = open(fl).read()
            m = re.search(r'^\s*camera_init_world_z:\s*([0-9.]+)', txt, re.M)
            spawn = round(float(cfg['launch_pad']['thickness'])
                          + float(cfg['vehicle']['belly_clearance']), 3)
            if m and abs(float(m.group(1)) - spawn) > 1e-3:
                warnings.append(
                    'FAST-LIO camera_init_world_z %.3f != spawn height %.3f (will be regenerated)'
                    % (float(m.group(1)), spawn))

    # Both base_link and the LIDAR must sit inside the map slab, with real margin. Either one
    # falling outside is a silent, high-cost failure: the vehicle outside stalls A* entirely,
    # the lidar outside throws away most of every scan. Both have happened.
    _mount = float(lid['mount_height'])
    _cru = float(cfg['vehicle']['cruise_altitude_world']) - (
        float(cfg['launch_pad']['thickness']) + float(cfg['vehicle']['belly_clearance']))
    _c = _cru + _mount / 2.0
    _h = float(planner['box_z_halfspan'])
    for _what, _z in (('base_link', _cru), ('the lidar', _cru + _mount)):
        _m = min(_z - (_c - _h), (_c + _h) - _z)
        if _m < 0.05:
            errors.append(
                'map slab [%.3f, %.3f] leaves %s only %+.3f m of margin\n'
                '    -> %s'
                % (_c - _h, _c + _h, _what, _m,
                   'A* cannot expand from a start outside the box'
                   if _what == 'base_link' else
                   'most of every scan would be discarded before the map sees it'))

    # Advisory, not an error: the fixed-altitude architecture (see
    # PLANNING_DOCS/fixed_altitude_2d_flight_architecture_2026-09-03.md) deliberately runs a
    # z box thinner than the inflation cube. Stated out loud so nobody rediscovers it as a bug.
    z_thick = 2.0 * float(planner['box_z_halfspan'])
    if z_thick < 2 * infl:
        warnings.append(
            'planner box_z span %.2f m is thinner than the inflation cube (2 x %.2f = %.2f m):\n'
            '    every obstacle blocks every z layer, so planning is effectively 2D with no\n'
            '    vertical escape from a lateral pinch. Intended for fixed-altitude flight.'
            % (z_thick, infl, 2 * infl))

    # Every XML file this stack loads, checked for well-formedness.
    # An XML comment may not contain "--". Twice on 2026-09-06 an explanatory comment written
    # into iris_vlp16_cam.sdf used " -- " as punctuation, which silently made the vehicle model
    # unparseable; Gazebo then spawned nothing and the visible symptom was
    # "MAVROS failed to connect to PX4", several layers away from the actual edit. Two seconds
    # of parsing here beats another ten-minute flight that dies at startup.
    import glob
    import xml.etree.ElementTree as ET
    xml_files = [FUEL_XML, FUEL_LAUNCH, WORLD]
    xml_files += glob.glob(os.path.join(REPO, 'simulation', 'custom_models', '*', '*.sdf'))
    xml_files += glob.glob(os.path.join(
        REPO, 'simulation', 'PX4-Autopilot-v1.14.3', 'Tools', 'simulation', 'gazebo-classic',
        'sitl_gazebo-classic', 'models', 'iris_vlp16_cam', '*.sdf'))
    # The vehicle this stack actually flies, plus the frame it includes. These are generated
    # into, so a malformed edit here is exactly the failure that once presented as
    # "MAVROS failed to connect to PX4".
    for name in ('x500', 'x500_vlp16'):
        xml_files.append(os.path.join(
            REPO, 'simulation', 'PX4-Autopilot-v1.14.3', 'Tools', 'simulation',
            'gazebo-classic', 'sitl_gazebo-classic', 'models', name, name + '.sdf'))
    for path in xml_files:
        if not os.path.exists(path):
            continue
        try:
            ET.parse(path)
        except ET.ParseError as exc:
            errors.append('%s is not well-formed XML: %s\n'
                          '    -> if the line is inside a comment, check for a "--"'
                          % (os.path.relpath(path, REPO), exc))

    return errors, warnings


class Editor(object):
    """Applies regex value substitutions to a file and reports what changed."""

    def __init__(self, path):
        self.path = path
        self.name = os.path.relpath(path, REPO)
        self.missing = not os.path.exists(path)
        self.text = '' if self.missing else open(path).read()
        self.original = self.text
        self.changes = []

    def sub(self, pattern, new_value, label, flags=re.M):
        """Replace group(1) of `pattern` with new_value. Pattern must have exactly one group.

        Matched with re.M by default so that '^' anchors to a line start; several patterns rely
        on that to avoid matching a commented-out copy of the same key (algorithm.xml keeps
        old values around inside <!-- --> blocks).

        Pass flags=re.M | re.S for XML patterns that have to reach across lines from a
        containing element to the value inside it, e.g. from <link name="velodyne_link"> to
        its <pose>. DOTALL is opt-in rather than global precisely because the line-oriented
        patterns above depend on '.' NOT crossing lines.
        """
        m = re.search(pattern, self.text, flags)
        if not m:
            self.changes.append(('MISS', label, None, None))
            return
        old = m.group(1)
        if old.strip() == str(new_value).strip():
            return
        start, end = m.span(1)
        self.text = self.text[:start] + str(new_value) + self.text[end:]
        self.changes.append(('SET', label, old, new_value))

    def flush(self, check_only):
        if self.missing:
            print('  [SKIP] %s does not exist' % self.name)
            return 0
        applied = [c for c in self.changes if c[0] == 'SET']
        missed = [c for c in self.changes if c[0] == 'MISS']
        if not applied and not missed:
            print('  [OK]   %s already matches' % self.name)
            return 0
        print('  %s' % self.name)
        for _, label, old, new in applied:
            print('     %-34s %s -> %s' % (label, old, new))
        for _, label, _, _ in missed:
            print('     %-34s PATTERN NOT FOUND (file changed shape?)' % label)
        if not check_only and self.text != self.original:
            open(self.path, 'w').write(self.text)
        return len(applied)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--check', action='store_true',
                    help='report what would change without writing')
    args = ap.parse_args()

    cfg = yaml.safe_load(open(CONFIG))['nidar']
    arena_key = cfg['arena']['active']
    arena = cfg['arenas'][arena_key]
    lidar = cfg['lidar']
    planner = cfg['planner']
    flight = cfg['flight']
    guard = cfg['guard']
    pad = cfg['launch_pad']

    print('mission_config.yaml -> derived files   (arena: %s)' % arena_key)
    print('%s' % ('-' * 66))

    errors, warnings = validate(cfg)
    for w in warnings:
        print('  [NOTE] %s' % w)
    if errors:
        for e_msg in errors:
            print('  [FAIL] %s' % e_msg)
        print('%s' % ('-' * 66))
        print('Refusing to generate: the values above do not mean what they say downstream.')
        return 1

    total = 0

    # --- FAST-LIO -----------------------------------------------------------
    # camera_init z origin = the height base_link sits at when FAST-LIO initialises, which is
    # the spawn height. This is the SAME number already written into world_to_map_tf below.
    spawn_z = round(pad['thickness'] + cfg['vehicle']['belly_clearance'], 3)
    cruise_ci = round(float(cfg['vehicle']['cruise_altitude_world']) - spawn_z, 3)
    zhalf = float(planner['box_z_halfspan'])
    vhalf = float(planner['viewpoint_z_halfband'])

    # Format without trailing ".0" so re-running the generator is a no-op rather than
    # churning "0" into "0.0" on every invocation.
    def num(v):
        return ('%g' % v) if isinstance(v, float) else str(v)

    mount = float(lidar['mount_height'])
    e = Editor(FASTLIO_YAML)
    e.sub(r'^\s*blind:\s*([0-9.]+)', num(lidar['blind']), 'preprocess.blind')
    e.sub(r'^\s*scan_line:\s*([0-9]+)', lidar['beams'], 'preprocess.scan_line')
    e.sub(r'^\s*det_range:\s*([0-9.]+)', num(lidar['max_range']), 'mapping.det_range')
    e.sub(r'^\s*pcd_save_en:\s*(\w+)',
          'true' if lidar['pcd_save_en'] else 'false', 'pcd_save.pcd_save_en')
    # DERIVED, not configured: the extrinsic is the mount pose, by definition. Keeping a
    # second hand-maintained copy is what let the SDF say 0.20 while FAST-LIO said 0.12.
    e.sub(r'extrinsic_T:\s*\[([^\]]*)\]', ' 0, 0, %s' % num(mount), 'mapping.extrinsic_T')
    # The altitude datum. FAST-LIO publishes z = range + mount_offset - camera_init_world_z, so
    # these two decide what height the entire stack believes it is at.
    rng = cfg['rangefinder']
    e.sub(r'^\s*tfmini_mount_offset:\s*([0-9.]+)', num(rng['mount_offset']),
          'mapping.tfmini_mount_offset')
    e.sub(r'^\s*camera_init_world_z:\s*([0-9.]+)', num(spawn_z),
          'mapping.camera_init_world_z')
    e.sub(r'^\s*tfmini_range_min:\s*([0-9.]+)', num(rng['range_min']),
          'mapping.tfmini_range_min')
    e.sub(r'^\s*tfmini_range_max:\s*([0-9.]+)', num(rng['range_max']),
          'mapping.tfmini_range_max')
    total += e.flush(args.check)

    # --- vehicle SDF: lidar mount, its standoff mast, and the sensor range window --------
    # The mast is what holds the lidar at mount_height. It runs from the top of the centre
    # plate to the bottom of the sensor body, so its length and centre both follow the mount:
    #     length = mount - LIDAR_BODY_LEN/2 - PLATE_TOP
    #     centre (body frame) = PLATE_TOP + length/2
    # and the <pose> it is written with is in velodyne_link's frame, hence centre - mount.
    # It is visual-only, deliberately: giving it mass and collision would move the CoM and add
    # contact geometry overlapping base_link, for no gain, since the mast sits at r = 0.008 and
    # is inside the sensor's own min_range so it can never be seen by the lidar anyway.
    mast_len = mount - LIDAR_BODY_LEN / 2.0 - PLATE_TOP
    mast_pose_z = PLATE_TOP + mast_len / 2.0 - mount
    e = Editor(VEHICLE_SDF)
    e.sub(r'<link name="velodyne_link">.*?<pose>([-0-9.eE ]+)</pose>',
          '0 0 %s 0 0 0' % num(mount), 'velodyne_link pose', re.M | re.S)
    e.sub(r'<visual name="velodyne_mast_visual">\s*<pose>([-0-9.eE ]+)</pose>',
          '0 0 %s 0 0 0' % num(round(mast_pose_z, 6)), 'mast pose', re.M | re.S)
    e.sub(r'<visual name="velodyne_mast_visual">.*?<length>([0-9.]+)</length>',
          num(round(mast_len, 6)), 'mast length', re.M | re.S)
    e.sub(r'<sensor type="ray" name="velodyne-VLP16">.*?<range>\s*<min>([0-9.]+)</min>',
          num(lidar['min_range']), 'ray range min', re.M | re.S)
    e.sub(r'<sensor type="ray" name="velodyne-VLP16">.*?<range>.*?<max>([0-9.]+)</max>',
          num(lidar['max_range']), 'ray range max', re.M | re.S)
    e.sub(r'gazebo_ros_laser_controller.*?<min_range>([0-9.]+)</min_range>',
          num(lidar['min_range']), 'plugin min_range', re.M | re.S)
    e.sub(r'gazebo_ros_laser_controller.*?<max_range>([0-9.]+)</max_range>',
          num(lidar['max_range']), 'plugin max_range', re.M | re.S)
    total += e.flush(args.check)

    # --- FUEL algorithm.xml -------------------------------------------------
    # '^\s*<param' anchors to a real parameter line: algorithm.xml keeps superseded values
    # in <!-- ... --> blocks, and without the anchor the commented copy matches first.
    P = r'^\s*<param name="%s"\s+value="([0-9.]+)"'
    e = Editor(FUEL_XML)
    e.sub(P % 'sdf_map/resolution', planner['map_resolution'], 'sdf_map/resolution')
    e.sub(P % 'sdf_map/obstacles_inflation',
          planner['obstacles_inflation'], 'sdf_map/obstacles_inflation')
    e.sub(P % 'optimization/dist0', planner['dist0'], 'optimization/dist0')
    e.sub(P % 'perception_utils/top_angle',
          lidar['vertical_half_fov_rad'], 'perception_utils/top_angle')
    e.sub(P % 'exploration/target_switch_margin',
          planner['target_switch_margin'], 'exploration/target_switch_margin')
    e.sub(P % 'exploration/target_reached_dist',
          planner['target_reached_dist'], 'exploration/target_reached_dist')
    e.sub(P % 'exploration/target_match_dist',
          planner['target_match_dist'], 'exploration/target_match_dist')
    # These four are the camera_init altitudes that went stale on the iris -> X500 migration.
    # See the block comment on planner.box_z_halfspan in mission_config.yaml.
    e.sub(P % 'map_ros/arena_area_m2', arena['measured']['free_area_m2'],
          'map_ros/arena_area_m2')
    e.sub(P % 'optimization/z_cruise', round(cruise_ci, 3), 'optimization/z_cruise')
    e.sub(P % 'frontier/min_candidate_z', round(cruise_ci - vhalf, 3),
          'frontier/min_candidate_z')
    e.sub(P % 'frontier/max_candidate_z', round(cruise_ci + vhalf, 3),
          'frontier/max_candidate_z')
    e.sub(P % 'fsm/finish_recheck_interval',
          planner['finish_recheck_interval'], 'fsm/finish_recheck_interval')
    e.sub(P % 'fsm/finish_recheck_max',
          planner['finish_recheck_max'], 'fsm/finish_recheck_max')
    total += e.flush(args.check)

    # --- FUEL launch (map box + speed limits) -------------------------------
    # The exploration box is expressed in camera_init, whose relationship to world is
    # set by the spawn pose: xw = -y_ci, yw = x_ci + pad.center.y. Inverting, a world
    # bound [wmin, wmax] maps to camera_init x in [wmin - pad.y, wmax - pad.y] and
    # camera_init y in [-wmax, -wmin]. The effective envelope is the arena minus the
    # guard margin, matching flight_envelope_guard.py's own derivation.
    m = guard['boundary_margin']
    ex, ey = guard['explore'], guard['explore']
    off = pad['center']['y']
    # 2026-09-05: the lateral pair below used to omit this, i.e. it assumed the pad sits at
    # world x = 0. Displacing the pad 2 m sideways then slid the whole search volume 2 m with
    # it, leaving 2.7 m of arena outside the box -- never explored, and never reported as
    # missed (FUEL only counted it as "OutMap" viewpoint rejections). Both axes must be
    # referenced to the pad, because camera_init is planted at the pad.
    off_x = pad['center']['x']
    # Resting height of the vehicle origin above world z=0 when parked on the pad.
    # Used by both the world->map TF here and the guard's spawn_world_z below; they are
    # the same quantity and must not be allowed to drift apart.
    spawn_z = round(pad['thickness'] + cfg['vehicle']['belly_clearance'], 3)
    e = Editor(FUEL_LAUNCH)
    e.sub(r'name="box_min_x"\s+value="(-?[0-9.]+)"',
          round(ey['y_min'] + m - off, 3), 'box_min_x (camera_init)')
    e.sub(r'name="box_max_x"\s+value="(-?[0-9.]+)"',
          round(ey['y_max'] - m - off, 3), 'box_max_x (camera_init)')
    e.sub(r'name="box_min_y"\s+value="(-?[0-9.]+)"',
          round(off_x - (ex['x_max'] - m), 3), 'box_min_y (camera_init)')
    e.sub(r'name="box_max_y"\s+value="(-?[0-9.]+)"',
          round(off_x - (ex['x_min'] + m), 3), 'box_max_y (camera_init)')
    # Every FUEL altitude lives in camera_init, whose origin is base_link at FAST-LIO init,
    # i.e. the spawn height. Derive them all from one world-frame number rather than letting
    # five camera_init literals go stale the next time the airframe changes.
    # The map slab is centred on the MIDPOINT between base_link and the lidar, because both
    # have to be inside it: the vehicle so A* can expand from its own cell, the lidar because
    # it is what fills the map. See planner.box_z_halfspan in mission_config.yaml for the
    # measured capture fractions behind this.
    slab_c = round(cruise_ci + float(lidar['mount_height']) / 2.0, 3)
    e.sub(r'name="box_min_z"\s+value="([0-9.]+)"',
          round(slab_c - zhalf, 3), 'box_min_z (camera_init)')
    e.sub(r'name="box_max_z"\s+value="([0-9.]+)"',
          round(slab_c + zhalf, 3), 'box_max_z (camera_init)')
    e.sub(r'name="traj_server/z_cruise"\s+value="([0-9.]+)"',
          round(cruise_ci, 3), 'traj_server/z_cruise (camera_init)')
    e.sub(r'name="max_vel"\s+value="([0-9.]+)"', flight['max_vel'], 'max_vel')
    e.sub(r'name="max_acc"\s+value="([0-9.]+)"', flight['max_acc'], 'max_acc')
    # The world->map static TF encodes the same spawn pose the guard's camera_to_world uses.
    # These two must never disagree, which is exactly why they are generated together.
    e.sub(r'(?s)name="world_to_map_tf"\s*\n\s*args="([^"]*)"',
          '%s %s %s 1.5707963 0 0 world map' % (pad['center']['x'], pad['center']['y'], spawn_z),
          'world_to_map_tf args')
    total += e.flush(args.check)

    # --- Envelope guard -----------------------------------------------------
    # The guard runs one envelope at a time; the mission envelope is the permissive one
    # that must contain the pad, so it is what gets written until the phase-aware guard
    # (mission FSM phase) lands.
    mi = guard['mission']
    e = Editor(GUARD_YAML)
    e.sub(r'spawn_world_x:\s*(-?[0-9.]+)', pad['center']['x'], 'spawn_world_x')
    e.sub(r'spawn_world_y:\s*(-?[0-9.]+)', pad['center']['y'], 'spawn_world_y')
    e.sub(r'spawn_world_z:\s*(-?[0-9.]+)', spawn_z, 'spawn_world_z')
    e.sub(r'world_x_min:\s*(-?[0-9.]+)', mi['x_min'], 'world_x_min')
    e.sub(r'world_x_max:\s*(-?[0-9.]+)', mi['x_max'], 'world_x_max')
    e.sub(r'world_y_min:\s*(-?[0-9.]+)', mi['y_min'], 'world_y_min')
    e.sub(r'world_y_max:\s*(-?[0-9.]+)', mi['y_max'], 'world_y_max')
    e.sub(r'world_z_min:\s*([0-9.]+)', guard['z_min'], 'world_z_min')
    e.sub(r'world_z_max:\s*([0-9.]+)', guard['z_max'], 'world_z_max')
    e.sub(r'boundary_margin:\s*([0-9.]+)', guard['boundary_margin'], 'boundary_margin')
    e.sub(r'boundary_margin_z:\s*([0-9.]+)', guard['boundary_margin_z'], 'boundary_margin_z')
    # rad/s. Kept in lockstep with nidar/flight so the guard cannot slew faster than the rate
    # the rest of the stack (and FAST-LIO) is sized for.
    e.sub(r'max_yaw_rate:\s*([0-9.]+)', flight['max_yaw_rate'], 'max_yaw_rate')
    total += e.flush(args.check)

    # --- Gazebo world -------------------------------------------------------
    ap_ = arena['model_pose']
    survivors_cfg = cfg.get('survivors', {}) or {}
    if survivors_cfg.get('enabled', False):
        # Both keys are supported for backwards compatibility with an earlier
        # actor-based generator; new configs use mesh_uri.
        mesh_uri = survivors_cfg.get('mesh_uri',
                                     survivors_cfg.get('skin_uri',
                                     'file:///usr/share/gazebo-11/media/models/sitting.dae'))
        # sitting.dae's mesh origin is at the pelvis (measured Z range -0.859..+0.857),
        # not at the feet, so a pose z=0 sinks the character half a metre. mesh_z_offset
        # lifts the visual so feet touch the ground. Default 0.52 assumes a Z scale of 0.6
        # (see mesh_scale below); recompute if you change either.
        mesh_z_offset = float(survivors_cfg.get('mesh_z_offset', 0.52))
        # Visual scale (x, y, z). Z<1 compresses the figure vertically to simulate
        # a sitting posture without needing an SDF <actor> and skeletal animation.
        scale = survivors_cfg.get('mesh_scale', [1.0, 1.0, 0.6])
        if isinstance(scale, (int, float)):
            scale = [float(scale), float(scale), float(scale)]
        mesh_scale = '%g %g %g' % (float(scale[0]), float(scale[1]), float(scale[2]))
        coll_half = float(survivors_cfg.get('collision_half_extent', 0.15))
        coll_height = float(survivors_cfg.get('collision_height', 0.50))
        placements = survivors_cfg.get('placements', []) or []
        survivors_block = '\n'.join(
            SURVIVOR_BLOCK.format(name=s['name'],
                                  x=s['x'], y=s['y'], yaw=s.get('yaw', 0.0),
                                  mesh_uri=mesh_uri,
                                  mesh_z_offset=mesh_z_offset,
                                  mesh_scale=mesh_scale,
                                  coll_diameter=coll_half * 2.0,
                                  coll_height=coll_height,
                                  half_height=coll_height / 2.0)
            for s in placements
        )
        if not survivors_block:
            survivors_block = '    <!-- survivors enabled but no placements listed -->'
    else:
        survivors_block = '    <!-- survivors disabled in mission_config.yaml -->'
    world = WORLD_TEMPLATE.format(
        arena_model=arena['model_name'],
        ax=ap_['x'], ay=ap_['y'], az=ap_['z'],
        aroll=ap_['roll'], apitch=ap_['pitch'], ayaw=ap_['yaw'],
        pad_block=(PAD_BLOCK.format(pad_model=pad['model_name'],
                                    px=pad['center']['x'], py=pad['center']['y'])
                   if pad.get('enabled', True) else
                   '    <!-- launch pad disabled in mission_config.yaml -->'),
        survivors_block=survivors_block,
        config_rel=os.path.relpath(CONFIG, REPO),
    )
    if os.path.exists(WORLD) and open(WORLD).read() == world:
        print('  [OK]   %s already matches' % os.path.relpath(WORLD, REPO))
    else:
        print('  %s' % os.path.relpath(WORLD, REPO))
        print('     %-34s arena=%s pad=%s' % ('regenerated', arena['model_name'],
                                              pad['model_name']))
        total += 1
        if not args.check:
            open(WORLD, 'w').write(world)

    print('%s' % ('-' * 66))
    if args.check:
        print('%d value(s) would change. Run without --check to apply.' % total)
        return 1 if total else 0
    print('%d value(s) applied.' % total)
    print('\nReminders:')
    print('  * Gazebo needs the model path: simulation/custom_models must be on')
    print('    GAZEBO_MODEL_PATH (scripts/setup_env.sh handles this).')
    print('  * Values consumed straight off the ROS param server (EDM, mission')
    print('    manager, detector) need no regeneration - they read the YAML directly.')
    return 0


PAD_BLOCK = """    <!-- The pad model self-centres (its own origin is the pad centre), so this pose is
         simply launch_pad.center from mission_config.yaml. -->
    <include>
      <name>launch_pad</name>
      <uri>model://{pad_model}</uri>
      <pose>{px} {py} 0 0 0 0</pose>
    </include>"""

# One SDF static <model> per survivor. The mesh is stand.dae from
# /usr/share/gazebo-11/media/models loaded as a plain <visual>, rendered at
# its bind pose (standing). This is more robust than SDF <actor>, which the
# Gazebo classic renderer refuses to draw without a <animation> block that
# references a skeleton -- causing "only 1 of 6 survivors visible" on
# 2026-09-11, run 20260911_094635.
#
# A small box collision keeps the drone from flying through the mannequin
# (which would be geometry-invisible without a collision), but does not
# defeat the drone's own inflation margin.
SURVIVOR_BLOCK = """    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} 0 0 0 {yaw}</pose>
      <link name="body">
        <visual name="visual">
          <pose>0 0 {mesh_z_offset} 0 0 0</pose>
          <geometry>
            <mesh>
              <uri>{mesh_uri}</uri>
              <scale>{mesh_scale}</scale>
            </mesh>
          </geometry>
        </visual>
        <collision name="collision">
          <pose>0 0 {half_height} 0 0 0</pose>
          <geometry>
            <box>
              <size>{coll_diameter} {coll_diameter} {coll_height}</size>
            </box>
          </geometry>
        </collision>
      </link>
    </model>"""

WORLD_TEMPLATE = """<?xml version="1.0" ?>
<!-- GENERATED FILE - do not edit by hand.
     Rendered from {config_rel} by nidar_config/scripts/apply_mission_config.py.
     Change the arena or pad there and re-run the generator. -->
<sdf version="1.6">
  <world name="nidar_competition">

    <include>
      <uri>model://sun</uri>
    </include>

    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <name>{arena_model}</name>
      <uri>model://{arena_model}</uri>
      <pose>{ax} {ay} {az} {aroll} {apitch} {ayaw}</pose>
    </include>

{pad_block}

{survivors_block}

    <physics name="default_physics" default="true" type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>

  </world>
</sdf>
"""


if __name__ == '__main__':
    sys.exit(main())
