import { useEffect } from 'react';
import { useSimulationStore, type ArenaInfo } from '@/store';
import { ManagedWebSocket } from '@/services/websocket';
import { API_BASE, WS_BASE } from '@/services/api';
import type {
  AutonomyInfo, DroneState, MissionEvent, MissionPhase, MissionState, OccupancyGridMeta,
  SimulationState, Survivor, SystemHealth,
} from '@/types';

// Backend base URLs: see services/api.ts.
export const SIM_API_BASE = API_BASE;

interface TelemetryPayload {
  drone: DroneState;
  mission_state: MissionState;
  mission_timer: number;
  mission_phases?: MissionPhase[];
  health: SystemHealth;
  autonomy?: AutonomyInfo;
  survivors?: Survivor[];
  sim?: {
    state: SimulationState;
    scenario?: string;
    gazeboConnected: boolean;
    px4SitlConnected: boolean;
    ros2Connected: boolean;
  };
}

interface WsEnvelope<T> {
  type: string;
  payload: T;
}

const STALE_MS = 3000;          // no telemetry for this long = backend unreachable
const TRAJ_MIN_STEP_M = 0.05;   // record the flown path at 5 cm resolution

interface MapPayload {
  meta: OccupancyGridMeta | null;
  encoding?: 'rle' | 'raw';
  rle?: number[];
  data?: number[];
  timestamp: number;
}

/** /map_2d arrives run-length encoded: [value, count, value, count, ...]. */
function decodeMap(p: MapPayload): Int8Array | number[] | null {
  if (!p.meta) return null;
  if (p.encoding !== 'rle' || !p.rle) return p.data ?? null;
  const out = new Int8Array(p.meta.width * p.meta.height);
  let i = 0;
  for (let k = 0; k + 1 < p.rle.length; k += 2) {
    out.fill(p.rle[k], i, i + p.rle[k + 1]);
    i += p.rle[k + 1];
  }
  return out;
}

/**
 * useSimulationBackend — drives the Simulation Dashboard from the GCS backend, which reads the
 * real NIDAR simulation (Gazebo + PX4 SITL + FAST-LIO2 + FUEL + mission layer). Replaces the
 * browser-side useSimulationMockProvider; the store, and therefore every panel, is unchanged.
 */
export function useSimulationBackend() {
  useEffect(() => {
    const sim = useSimulationStore.getState;
    // The store outlives this page (navigating away and back); the backend resends its backlog
    // on every connect, so remember what is already on the timeline.
    const seenEvents = new Set<string>(sim().events.map(e => `${e.id}@${e.timestamp}`));
    let lastTelemetry = 0;
    let lastSimState: SimulationState | null = null;
    let backendDownReported = false;

    fetch(`${API_BASE}/api/simulation/arena`)
      .then(r => (r.ok ? r.json() : null))
      .then((a: ArenaInfo | null) => { if (a) sim().setArena(a); })
      .catch(() => { /* backend offline: the map draws without the grid until it is back */ });

    const telemetry = new ManagedWebSocket(`${WS_BASE}/simulation`, 'Simulation');
    const events = new ManagedWebSocket(`${WS_BASE}/events`, 'Events');
    const mapWs = new ManagedWebSocket(`${WS_BASE}/map`, 'Map');

    const unsubTelemetry = telemetry.subscribe((raw) => {
      const msg = raw as WsEnvelope<TelemetryPayload>;
      if (msg.type !== 'telemetry' || !msg.payload) return;
      const p = msg.payload;
      const s = sim();
      lastTelemetry = Date.now();
      backendDownReported = false;

      s.setDrone(p.drone);
      s.setMissionState(p.mission_state);
      s.setMissionTimer(p.mission_timer);
      if (p.health) s.setHealth(p.health);
      if (p.autonomy) s.setAutonomy(p.autonomy);
      if (p.survivors) s.setSurvivors(p.survivors);
      if (p.mission_phases) s.setMissionPhases(p.mission_phases);
      if (p.sim) {
        const st = p.sim.state;
        // A new run (or a reset) starts a fresh flown path.
        if (st !== lastSimState && (st === 'STARTING' || st === 'STOPPED')) s.clearTrajectory();
        lastSimState = st;
        s.setSimConfig({
          state: st,
          gazeboConnected: p.sim.gazeboConnected,
          px4SitlConnected: p.sim.px4SitlConnected,
          ros2Connected: p.sim.ros2Connected,
          // while a run is up, show the scenario it is actually flying
          ...(st !== 'STOPPED' && p.sim.scenario ? { scenario: p.sim.scenario } : {}),
        });
      }

      const { x, y } = p.drone.position;
      if (x !== null && y !== null) {
        const traj = sim().trajectory;
        const last = traj[traj.length - 1];
        if (!last || Math.hypot(x - last.x, y - last.y) >= TRAJ_MIN_STEP_M) {
          s.addTrajectoryPoint({ x, y, timestamp: Date.now() });
        }
      }
    });

    const unsubEvents = events.subscribe((raw) => {
      const msg = raw as WsEnvelope<MissionEvent>;
      if (msg.type !== 'event' || !msg.payload) return;
      // id + timestamp: ids restart from 1 when the backend restarts
      const key = `${msg.payload.id}@${msg.payload.timestamp}`;
      if (seenEvents.has(key)) return;
      seenEvents.add(key);
      sim().addEvent(msg.payload);
    });

    const unsubMap = mapWs.subscribe((raw) => {
      const msg = raw as WsEnvelope<MapPayload | null>;
      if (msg.type !== 'map') return;
      const data = msg.payload ? decodeMap(msg.payload) : null;
      sim().setMap(msg.payload && msg.payload.meta && data
        ? { meta: msg.payload.meta, data, timestamp: msg.payload.timestamp }
        : null);
    });

    telemetry.connect();
    events.connect();
    mapWs.connect();

    const watchdog = setInterval(() => {
      if (Date.now() - lastTelemetry < STALE_MS || backendDownReported) return;
      backendDownReported = true;
      const s = sim();
      s.setSimConfig({ gazeboConnected: false, px4SitlConnected: false, ros2Connected: false });
      s.setDrone({ connectionStatus: 'DISCONNECTED' });
      s.addEvent({
        id: `ui-${Date.now()}`, timestamp: Date.now(), level: 'ERROR',
        message: 'GCS BACKEND UNREACHABLE — start it with catkin_ws/src/nidar_gcs/scripts/start_gcs.sh',
      });
    }, 1000);

    return () => {
      clearInterval(watchdog);
      unsubTelemetry();
      unsubEvents();
      unsubMap();
      telemetry.disconnect();
      events.disconnect();
      mapWs.disconnect();
    };
  }, []);
}
