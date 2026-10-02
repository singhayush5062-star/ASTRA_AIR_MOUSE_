import { useEffect, useRef } from 'react';
import { useSimulationStore } from '@/store';
import type { MissionEvent, MissionState } from '@/types';

// ─── Helpers ───────────────────────────────────────────────────

const randomBetween = (min: number, max: number) => Math.random() * (max - min) + min;

let eventIdCounter = 0;
const makeSimEvent = (msg: string, level: MissionEvent['level'] = 'INFO'): MissionEvent => ({
  id: `sim-evt-${++eventIdCounter}`,
  timestamp: Date.now(),
  message: `[SIM] ${msg}`,
  level,
});

// ─── Simulation Sequence ──────────────────────────────────────

const SIM_SEQUENCE: Array<{
  delay: number;
  state: MissionState;
  event: string;
  level: MissionEvent['level'];
}> = [
  { delay: 0,     state: 'INIT',              event: 'GAZEBO SCENARIO_01 INITIALIZED', level: 'INFO' },
  { delay: 3000,  state: 'TAKEOFF',           event: 'TAKEOFF COMMAND ISSUED', level: 'INFO' },
  { delay: 6000,  state: 'LOCALIZATION',      event: 'FAST-LIO2 LOCALIZATION ACQUIRING...', level: 'INFO' },
  { delay: 9000,  state: 'EXPLORE',           event: 'FAST-LIO2 LOCKED — EXPLORATION STARTED', level: 'SUCCESS' },
  { delay: 15000, state: 'EXPLORE',           event: 'FRONTIER F03 DETECTED — NAVIGATING', level: 'INFO' },
  { delay: 22000, state: 'EXPLORE',           event: 'ROOM R01 ENTERED', level: 'INFO' },
  { delay: 28000, state: 'SURVIVOR_DETECTED', event: 'SURVIVOR #01 DETECTED — CONFIDENCE 91.2%', level: 'SUCCESS' },
  { delay: 30000, state: 'CONTINUE_EXPLORE',  event: 'GRID A3 ASSIGNED — EXPLORATION RESUMED', level: 'INFO' },
  { delay: 38000, state: 'EXPLORE',           event: 'FRONTIER F07 DETECTED', level: 'INFO' },
  { delay: 45000, state: 'SURVIVOR_DETECTED', event: 'SURVIVOR #02 DETECTED — CONFIDENCE 87.6%', level: 'SUCCESS' },
  { delay: 47000, state: 'CONTINUE_EXPLORE',  event: 'GRID C5 ASSIGNED — EXPLORATION RESUMED', level: 'INFO' },
  { delay: 60000, state: 'RETURN',            event: 'EXPLORATION COMPLETE — RETURNING TO ORIGIN', level: 'INFO' },
  { delay: 70000, state: 'LAND',              event: 'LANDING INITIATED', level: 'INFO' },
  { delay: 75000, state: 'COMPLETE',          event: 'MISSION COMPLETE — 2 SURVIVORS FOUND', level: 'SUCCESS' },
];

/**
 * useSimulationMockProvider — Drives simulated telemetry, subsystems, and mission events
 * strictly for the Simulation Dashboard. Never affects the Hardware Dashboard.
 */
export function useSimulationMockProvider() {
  const timersRef = useRef<ReturnType<typeof setTimeout>[]>([]);
  const intervalsRef = useRef<ReturnType<typeof setInterval>[]>([]);

  useEffect(() => {
    const sim = useSimulationStore.getState;

    // Boot Gazebo & PX4 SITL simulation
    const bootTimer = setTimeout(() => {
      const s = sim();
      s.setSubsystemHealth('ros2',     { status: 'CONNECTED' });
      s.setSubsystemHealth('px4',      { status: 'CONNECTED' });
      s.setSubsystemHealth('imu',      { status: 'ONLINE' });
      s.setSubsystemHealth('lidar',    { status: 'ONLINE' });
      s.setSubsystemHealth('camera',   { status: 'ONLINE' });
      s.setSubsystemHealth('fastlio2', { status: 'TRACKING' });
      s.setSubsystemHealth('yolo',     { status: 'READY' });
      s.setSubsystemHealth('planner',  { status: 'RUNNING' });
      s.setSimConfig({
        gazeboConnected: true,
        ros2Connected: true,
        px4SitlConnected: true,
        state: 'RUNNING',
      });
      s.setDrone({ connectionStatus: 'CONNECTED', mode: 'OFFBOARD' });
      s.addEvent(makeSimEvent('GAZEBO SIMULATION CONNECTED', 'SUCCESS'));
      s.addEvent(makeSimEvent('PX4 SITL HEARTBEAT LOCKED', 'SUCCESS'));
      s.addEvent(makeSimEvent('ROS 2 BRIDGE ONLINE', 'SUCCESS'));
    }, 1000);
    timersRef.current.push(bootTimer);

    // Sim telemetry at 20 Hz
    let t = 0;
    const simTelInterval = setInterval(() => {
      const s = sim();
      if (s.simConfig.state !== 'RUNNING') return;

      t += 0.05 * (s.simConfig.speed || 1);
      const radius = 3 + Math.sin(t * 0.3) * 2;
      const x = parseFloat((Math.cos(t * 0.4) * radius).toFixed(3));
      const y = parseFloat((Math.sin(t * 0.4) * radius).toFixed(3));
      const z = parseFloat((1.5 + Math.sin(t * 0.7) * 0.3).toFixed(2));

      s.setDrone({
        position: { x, y, z },
        attitude: {
          roll:  parseFloat((Math.sin(t * 1.2) * 0.08).toFixed(3)),
          pitch: parseFloat((Math.cos(t * 0.9) * 0.06).toFixed(3)),
          yaw:   parseFloat(((t * 0.4) % (2 * Math.PI)).toFixed(3)),
        },
        velocity: {
          x: parseFloat((-Math.sin(t * 0.4) * radius * 0.4).toFixed(2)),
          y: parseFloat((Math.cos(t * 0.4) * radius * 0.4).toFixed(2)),
          z: parseFloat((Math.cos(t * 0.7) * 0.3).toFixed(2)),
        },
        battery: {
          voltage:    parseFloat((16.8 - t * 0.005).toFixed(2)),
          current:    parseFloat((8 + Math.sin(t) * 0.5).toFixed(2)),
          percentage: parseFloat(Math.max(0, 100 - t * 0.1).toFixed(1)),
        },
      });
    }, 50);
    intervalsRef.current.push(simTelInterval);

    // Sim mission timer at 1 Hz
    const simTimerInterval = setInterval(() => {
      const s = sim();
      if (s.simConfig.state === 'RUNNING') {
        s.setMissionTimer(s.missionTimer + 1);
      }
    }, 1000);
    intervalsRef.current.push(simTimerInterval);

    // Sim mission sequence
    SIM_SEQUENCE.forEach(step => {
      const timer = setTimeout(() => {
        const s = sim();
        if (s.simConfig.state === 'STOPPED') return;
        s.setMissionState(step.state);
        s.addEvent(makeSimEvent(step.event, step.level));
        s.setAutonomy({ state: step.state });
        s.advanceMissionPhase();
      }, step.delay + 1500);
      timersRef.current.push(timer);
    });

    // Sim survivors detection
    timersRef.current.push(setTimeout(() => {
      sim().addSurvivor({
        id: 1, gridLabel: 'A3',
        position: { x: 2.1, y: 3.4, z: 0.5 },
        confidence: 91.2, status: 'CONFIRMED', detectedAt: Date.now(),
      });
      sim().setAutonomy({ currentTarget: 'SURVIVOR #01', reason: 'Survivor detected in corridor A3 by YOLO' });
    }, 28000));

    timersRef.current.push(setTimeout(() => {
      sim().addSurvivor({
        id: 2, gridLabel: 'C5',
        position: { x: -3.7, y: 5.1, z: 0.5 },
        confidence: 87.6, status: 'CONFIRMED', detectedAt: Date.now(),
      });
      sim().setAutonomy({ currentTarget: 'SURVIVOR #02', reason: 'Survivor localized in room C5' });
    }, 45000));

    // Sim autonomy updates
    const autonomyInterval = setInterval(() => {
      const s = sim();
      if (s.simConfig.state !== 'RUNNING') return;
      const targets = ['FRONTIER F03', 'FRONTIER F07', 'FRONTIER F11', 'CORRIDOR C2', 'ROOM R03'];
      const reasons = [
        'Unexplored frontier detected by FUEL planner',
        'Highest information gain frontier selected',
        'Nearest reachable frontier target',
        'Returning from dead-end corridor',
        'Corridor junction reached — splitting frontier',
      ];
      s.setAutonomy({
        currentTarget: targets[Math.floor(Math.random() * targets.length)],
        distanceToTarget: parseFloat(randomBetween(0.5, 8).toFixed(1)),
        reason: reasons[Math.floor(Math.random() * reasons.length)],
      });
    }, 5000);
    intervalsRef.current.push(autonomyInterval);

    return () => {
      timersRef.current.forEach(clearTimeout);
      intervalsRef.current.forEach(clearInterval);
    };
  }, []);
}

// Backwards-compatible alias
export const useMockProviders = useSimulationMockProvider;
