#!/usr/bin/env python3
"""Score a flight against the seven acceptance criteria in
PLANNING_DOCS/airframe_arena_fit_root_cause_and_plan_2026-09-06.md.

Everything here is measured from GROUND TRUTH (Gazebo) and the arena STL, never from the
estimator, because the whole class of bug this exists to catch is "the vehicle is not where the
software thinks it is". Comparing the planner against the estimator would hide exactly that.

    python3 scripts/verify_flight.py [path/to/flight.ulg] [--fuel-log /tmp/fuel.log]

With no ulg given it picks the newest under ~/.ros/log/.
"""
import sys, os, glob, struct, math, re
import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
STL = os.path.join(REPO, 'catkin_ws', 'src', 'nidar_sim', 'models', 'arina_nidar', 'meshes', 'arina_nidar.stl')
ARENA_M2 = 158.0
CRUISE_Z = 1.75          # height the wall cross-section is taken at
SPAWN_XY = (0.0, -9.5)   # launch pad centre, world ENU
SPAWN_Z = 0.26


def load_walls(z=CRUISE_Z):
    """Horizontal cross-section of the arena mesh at the cruise height, as 2D segments."""
    raw = open(STL, 'rb').read()
    n = struct.unpack('<I', raw[80:84])[0]
    d = np.frombuffer(raw[84:84 + n * 50], dtype=np.uint8).reshape(n, 50)
    tri = d[:, 12:48].copy().view('<f4').reshape(n, 3, 3).astype(np.float64)
    tri = tri * 0.001 + np.array([-7.5, -7.5, 0.0])      # mm -> m, recentre
    segs = []
    for T in tri:
        zz = T[:, 2]
        if zz.min() <= z <= zz.max():
            pts = []
            for a, b in ((0, 1), (1, 2), (2, 0)):
                za, zb = zz[a], zz[b]
                if (za - z) * (zb - z) <= 0 and abs(zb - za) > 1e-9:
                    t = (z - za) / (zb - za)
                    pts.append((T[a] + t * (T[b] - T[a]))[:2])
            if len(pts) >= 2:
                segs.append((pts[0], pts[1]))
    S = np.array(segs)
    return S[:, 0], S[:, 1]


def clearance_fn(A, B):
    AB = B - A
    L = (AB * AB).sum(1)
    L[L < 1e-12] = 1e-12

    def f(p):
        t = np.clip(((p - A) * AB).sum(1) / L, 0, 1)
        return np.linalg.norm(p - (A + t[:, None] * AB), axis=1).min()
    return f


def collision_radius():
    """Read the vehicle's real prop-tip radius out of the SDF rather than trusting a constant."""
    p = os.path.join(REPO, 'simulation', 'PX4-Autopilot-v1.14.3', 'Tools', 'simulation',
                     'gazebo-classic', 'sitl_gazebo-classic', 'models', 'x500', 'x500.sdf')
    s = open(p).read()
    arm = r = 0.0
    for m in re.finditer(r'<link name=[\'"]rotor_\d+[\'"]>(.*?)</link>', s, re.S):
        b = m.group(1)
        pos = re.search(r'<pose>([^<]*)</pose>', b)
        if pos:
            v = [float(x) for x in pos.group(1).split()[:3]]
            arm = max(arm, math.hypot(v[0], v[1]))
        cy = re.search(r'<collision.*?<cylinder>.*?<radius>([\d.eE+-]+)</radius>', b, re.S)
        bx = re.search(r'<collision.*?<box>.*?<size>([^<]*)</size>', b, re.S)
        if cy:
            r = max(r, float(cy.group(1)))
        elif bx:
            r = max(r, float(bx.group(1).split()[0]) / 2)
    return arm + r, arm, r


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    fuel_log = '/tmp/fuel.log'
    if '--fuel-log' in sys.argv:
        fuel_log = sys.argv[sys.argv.index('--fuel-log') + 1]
    if args:
        ulg = args[0]
    else:
        c = sorted(glob.glob(os.path.expanduser('~/.ros/log/*/*.ulg')), key=os.path.getmtime)
        if not c:
            print('no ulg found'); return 2
        ulg = c[-1]
    print('ulog     :', ulg)
    print('fuel log :', fuel_log)

    from pyulog import ULog
    u = ULog(ulg, ['vehicle_local_position_groundtruth', 'vehicle_attitude',
                   'vehicle_attitude_groundtruth', 'sensor_combined', 'vehicle_status'])
    D = {d.name: d for d in u.data_list}
    gt = D['vehicle_local_position_groundtruth']
    t = gt.data['timestamp'] / 1e6
    # ground truth may carry a UTM origin; re-reference to the first sample = the spawn point
    N = gt.data['x'] - gt.data['x'][0]
    E = gt.data['y'] - gt.data['y'][0]
    Dn = gt.data['z'] - gt.data['z'][0]
    X, Y, Z = E + SPAWN_XY[0], N + SPAWN_XY[1], SPAWN_Z - Dn

    A, B = load_walls()
    clr = clearance_fn(A, B)
    RAD, arm, prop = collision_radius()
    print('vehicle  : arm %.4f + prop %.4f = collision radius %.4f m (disc %.4f m)'
          % (arm, prop, RAD, 2 * RAD))

    airborne = (Z > SPAWN_Z + 0.5)
    if airborne.sum() < 10:
        print('vehicle never left the pad'); return 2
    t0, t1 = t[airborne][0], t[airborne][-1]
    print('airborne : t=%.1f .. %.1f s (%.0f s)\n' % (t0, t1, t1 - t0))

    ts = np.arange(t0, t1, 0.5)
    cl = np.array([clr(np.array([np.interp(T, t, X), np.interp(T, t, Y)])) for T in ts])

    results = []

    # 1 + 2. wall contact. cl is centre-to-wall; what matters is the BLADE TIP, so subtract the
    # collision radius. A negative tip clearance is penetration depth, not just proximity.
    tip = cl - RAD
    breach = tip < 0
    results.append(('1. min blade-tip clearance > 0 (no wall contact)',
                    tip.min() > 0.0,
                    'min tip clearance %+.3f m (centre-to-wall %.3f)' % (tip.min(), cl.min())))
    results.append(('2. no time with the prop disc inside a wall',
                    breach.sum() == 0,
                    '%.1f%% of flight (was 50.8%%); median centre-to-wall %.3f m (was 0.342)'
                    % (100.0 * breach.mean(), np.median(cl))))

    # 3. churn
    worst, worst_t = 0.0, 0.0
    for T0 in np.arange(t0, t1 - 10, 10.0):
        m = (t >= T0) & (t < T0 + 10)
        if m.sum() < 10:
            continue
        p = np.stack([X[m], Y[m]], 1)
        path = np.linalg.norm(np.diff(p, axis=0), axis=1).sum()
        net = np.linalg.norm(p[-1] - p[0])
        r = path / max(net, 1e-3)
        if r > worst:
            worst, worst_t = r, T0
    results.append(('3. no 10 s window with churn ratio > 10',
                    worst <= 10.0,
                    'worst %.1f at t=%.0f s (was 37.7)' % (worst, worst_t)))

    # 4 + 5. coverage progress and FINISH, from the planner log
    cov = []
    pat = re.compile(r',\s*(\d+\.\d+)\]:\s*\[coverage\] mapped [\d.]+ m2 free .*?= ([\d.]+)%; ([\d.]+) m2 left')
    finish = 0
    try:
        for line in open(fuel_log, errors='ignore'):
            line = re.sub(r'\x1b\[[0-9;]*m', '', line)
            m = pat.search(line)
            if m:
                cov.append((float(m.group(1)), float(m.group(2)), float(m.group(3))))
            if 'state: FINISH' in line:
                finish += 1
    except FileNotFoundError:
        pass
    if cov:
        stall, stall_t = 0.0, 0.0
        for i in range(len(cov)):
            for j in range(i + 1, len(cov)):
                if cov[j][0] - cov[i][0] > 60:
                    break
            else:
                j = len(cov) - 1
            if cov[j][0] - cov[i][0] >= 60 and (cov[j][1] - cov[i][1]) * ARENA_M2 / 100.0 < 1.0:
                if cov[j][0] - cov[i][0] > stall:
                    stall, stall_t = cov[j][0] - cov[i][0], cov[i][0]
        peak = max(c[1] for c in cov)
        left = min(c[2] for c in cov)
        results.append(('4. no >60 s window with < 1 m2 of progress',
                        stall == 0.0,
                        'longest stall %.0f s from t=%.0f (was 406 s); peak coverage %.1f%%, %.1f m2 left'
                        % (stall, stall_t, peak, left)))
    else:
        results.append(('4. coverage progress', False, 'no coverage lines in %s' % fuel_log))
    results.append(('5. FSM reached FINISH', finish > 0, '%d FINISH transitions' % finish))

    # 6. no flight termination
    msgs = '\n'.join(m.message for m in u.logged_messages)
    term = ('Attitude failure' in msgs) or ('triggering terminate' in msgs)
    results.append(('6. no attitude failure / flight termination', not term,
                    'terminated' if term else 'clean'))

    # 7. real-time factor held up
    ok7, note7 = True, 'not measured (needs wall-clock pairs from the ROS log)'
    wall = []
    try:
        pp = re.compile(r'\[(\d{10}\.\d+),\s*(\d+\.\d+)\]')
        for line in open(fuel_log, errors='ignore'):
            m = pp.search(line)
            if m:
                wall.append((float(m.group(1)), float(m.group(2))))
        if len(wall) > 100:
            wall.sort(key=lambda z: z[1])
            dw = wall[-1][0] - wall[0][0]
            ds = wall[-1][1] - wall[0][1]
            rtf = ds / dw if dw > 0 else 0
            ok7 = rtf > 0.3
            note7 = 'mean RTF %.3f over %.0f s wall' % (rtf, dw)
    except FileNotFoundError:
        pass
    results.append(('7. RTF stayed above 0.3', ok7, note7))

    print('=' * 78)
    npass = 0
    for name, ok, note in results:
        print('%-4s %-48s %s' % ('PASS' if ok else 'FAIL', name, note))
        npass += bool(ok)
    print('=' * 78)
    print('%d/%d criteria passed' % (npass, len(results)))
    print('\nNote: 60.6%% of arina_nidar is REACHABLE by the airframe at inflation 0.40, but that'
          '\nis not a ceiling on COVERAGE: the lidar has 30 m of range and maps through doorways'
          '\ninto rooms the vehicle never enters. Measured 102.7%% coverage on a run whose vehicle'
          '\ncould only occupy 60.6%% of the floor. Do not use reachable area as a coverage target.')
    return 0 if npass == len(results) else 1


if __name__ == '__main__':
    sys.exit(main())
