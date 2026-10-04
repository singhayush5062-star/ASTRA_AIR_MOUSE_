import { useEffect } from 'react';
import { useHardwareStore, type ArenaInfo, type HWState } from '@/store';
import { ManagedWebSocket } from '@/services/websocket';
import { apiUrl, WS_BASE } from '@/services/api';
import type {
  AutonomyInfo, DroneState, MissionEvent, MissionPhase, MissionState, OccupancyGridMeta,
  Survivor, SystemHealth,
} from '@/types';

interface HardwarePayload {
  connected: boolean;
  status: 'DISCONNECTED' | 'CONNECTING' | 'CONNECTED' | 'ERROR';
  error: string | null;
  drone: DroneState;
  health: SystemHealth;
  mission_state: MissionState;
  mission_timer: number;
  mission_phases?: MissionPhase[];
  autonomy?: AutonomyInfo;
  survivors?: Survivor[];
}

interface MapPayload {
  meta: OccupancyGridMeta | null;
  encoding?: 'rle' | 'raw';
  rle?: number[];
  data?: number[];
  timestamp: number;
}

interface WsEnvelope<T> {
  type: string;
  payload: T;
}

const TRAJ_MIN_STEP_M = 0.05;

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
 * useHardwareBackend — drives the Hardware Dashboard from the GCS backend's link to the real
 * drone (MAVLink over T12 / SiK / USB / Wi-Fi, plus the Jetson's ROS master when reachable):
 * telemetry, mission phase, health and survivors at 10 Hz, the live 2D map and the event
 * timeline. Connecting and commands stay in the store (connectDrone / sendCommand).
 */
export function useHardwareBackend() {
  useEffect(() => {
    const hw = useHardwareStore.getState;
    const seenEvents = new Set<string>(hw().events.map(e => `${e.id}@${e.timestamp}`));
    let lastConnected = false;

    fetch(apiUrl('/api/hardware/arena'))
      .then(r => (r.ok ? r.json() : null))
      .then((a: ArenaInfo | null) => { if (a) hw().setArena(a); })
      .catch(() => { /* backend offline: the map draws without the grid */ });

    const telemetry = new ManagedWebSocket(`${WS_BASE}/hardware`, 'Hardware');
    const events = new ManagedWebSocket(`${WS_BASE}/hardware/events`, 'HardwareEvents');
    const mapWs = new ManagedWebSocket(`${WS_BASE}/hardware/map`, 'HardwareMap');

    const unsubTelemetry = telemetry.subscribe((raw) => {
      const msg = raw as WsEnvelope<HardwarePayload>;
      if ((msg.type !== 'telemetry' && msg.type !== 'status') || !msg.payload) return;
      const p = msg.payload;
      const s = hw();

      // Link state: the backend owns it once connected. CONNECTING / ERROR belong to the
      // connect flow in the store and are left alone unless the backend says CONNECTED.
      let next: HWState['hwConnectionState'] = s.hwConnectionState;
      if (p.status === 'CONNECTED') {
        next = p.drone?.connectionStatus === 'CONNECTED' ? 'CONNECTED' : 'DEGRADED';
      } else if (s.hwConnectionState === 'CONNECTED' || s.hwConnectionState === 'DEGRADED') {
        next = 'DISCONNECTED';  // backend restarted or links closed elsewhere
      }
      if (next !== s.hwConnectionState) s.setConnectionState(next, next === 'DISCONNECTED' ? null : s.connectionError);
      if (!p.connected) {
        lastConnected = false;
        return;
      }
      if (!lastConnected) s.clearTrajectory();  // a fresh connection starts a fresh path
      lastConnected = true;

      s.setDrone(p.drone);
      s.setMissionState(p.mission_state);
      s.setMissionTimer(p.mission_timer);
      if (p.health) s.setHealth(p.health);
      if (p.autonomy) s.setAutonomy(p.autonomy);
      if (p.survivors) s.setSurvivors(p.survivors);
      if (p.mission_phases) s.setMissionPhases(p.mission_phases);
      if (s.emergencyAborted !== (p.mission_state === 'ABORT')) {
        s.setEmergencyAborted(p.mission_state === 'ABORT');
      }

      const { x, y } = p.drone.position;
      if (x !== null && y !== null) {
        const traj = hw().trajectory;
        const last = traj[traj.length - 1];
        if (!last || Math.hypot(x - last.x, y - last.y) >= TRAJ_MIN_STEP_M) {
          s.addTrajectoryPoint({ x, y, timestamp: Date.now() });
        }
      }
    });

    const unsubEvents = events.subscribe((raw) => {
      const msg = raw as WsEnvelope<MissionEvent>;
      if (msg.type !== 'event' || !msg.payload) return;
      const key = `${msg.payload.id}@${msg.payload.timestamp}`;
      if (seenEvents.has(key)) return;
      seenEvents.add(key);
      hw().addEvent(msg.payload);
    });

    const unsubMap = mapWs.subscribe((raw) => {
      const msg = raw as WsEnvelope<MapPayload | null>;
      if (msg.type !== 'map') return;
      const data = msg.payload ? decodeMap(msg.payload) : null;
      hw().setMap(msg.payload && msg.payload.meta && data
        ? { meta: msg.payload.meta, data, timestamp: msg.payload.timestamp }
        : null);
    });

    telemetry.connect();
    events.connect();
    mapWs.connect();

    return () => {
      unsubTelemetry();
      unsubEvents();
      unsubMap();
      telemetry.disconnect();
      events.disconnect();
      mapWs.disconnect();
    };
  }, []);
}
