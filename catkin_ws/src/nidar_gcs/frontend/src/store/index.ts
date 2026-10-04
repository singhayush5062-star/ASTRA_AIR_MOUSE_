import { create } from 'zustand';
import type {
  DroneState, MissionState, MissionEvent, MissionPhase, AutonomyInfo,
  Survivor, SystemHealth, SubsystemHealth, OccupancyGrid, TrajectoryPoint,
  CameraFrame, SimulationConfig,
} from '@/types';
import { apiUrl } from '@/services/api';

// ─── Defaults ─────────────────────────────────────────────────

const defaultSimDrone = (): DroneState => ({
  id: 'NIDAR-SIM',
  mode: 'IDLE',
  position: { x: 0, y: 0, z: 0 },
  velocity: { x: 0, y: 0, z: 0 },
  attitude: { roll: 0, pitch: 0, yaw: 0 },
  battery: { voltage: 16.8, current: 0, percentage: 100 },
  connectionStatus: 'DISCONNECTED',
  timestamp: Date.now(),
});

const defaultDisconnectedDrone = (): DroneState => ({
  id: 'NIDAR-01',
  mode: '--',
  position: { x: null, y: null, z: null },
  velocity: { x: null, y: null, z: null },
  attitude: { roll: null, pitch: null, yaw: null },
  battery: { voltage: null, current: null, percentage: null },
  connectionStatus: 'DISCONNECTED',
  timestamp: 0,
});

const defaultHealth = (): SystemHealth => ({
  px4:      { name: 'PX4',       status: 'OFFLINE', lastUpdate: Date.now() },
  ros2:     { name: 'ROS 2',     status: 'OFFLINE', lastUpdate: Date.now() },
  fastlio2: { name: 'FAST-LIO2', status: 'OFFLINE', lastUpdate: Date.now() },
  lidar:    { name: 'LiDAR',     status: 'OFFLINE', lastUpdate: Date.now() },
  imu:      { name: 'IMU',       status: 'OFFLINE', lastUpdate: Date.now() },
  camera:   { name: 'Camera',    status: 'OFFLINE', lastUpdate: Date.now() },
  yolo:     { name: 'YOLO',      status: 'OFFLINE', lastUpdate: Date.now() },
  planner:  { name: 'Planner',   status: 'OFFLINE', lastUpdate: Date.now() },
});

const MISSION_PHASES: MissionPhase[] = [
  { state: 'INIT',              status: 'PENDING' },
  { state: 'TAKEOFF',          status: 'PENDING' },
  { state: 'LOCALIZATION',     status: 'PENDING' },
  { state: 'EXPLORE',          status: 'PENDING' },
  { state: 'SURVIVOR_DETECTED',status: 'PENDING' },
  { state: 'CONTINUE_EXPLORE', status: 'PENDING' },
  { state: 'RETURN',           status: 'PENDING' },
  { state: 'LAND',             status: 'PENDING' },
];

// ─── Simulation Store ─────────────────────────────────────────

/** Competition grid survivors are reported in (arena_grid.yaml): cell (0, 0) = "A1". */
export interface ArenaGrid {
  originX: number;
  originY: number;
  cellSize: number;
  cellsX: number;
  cellsY: number;
}

/** Arena geometry served by the backend (/api/simulation/arena), arena `world` frame, metres. */
export interface ArenaInfo {
  name: string;
  bounds: { x_min: number; x_max: number; y_min: number; y_max: number };
  entry: { x: number; y: number };
  launch_pad: { x: number; y: number };
  grid: ArenaGrid;
}

interface SimState {
  drone: DroneState;
  missionState: MissionState;
  missionPhases: MissionPhase[];
  missionTimer: number;
  autonomy: AutonomyInfo;
  survivors: Survivor[];
  health: SystemHealth;
  camera: CameraFrame;
  simConfig: SimulationConfig;
  events: MissionEvent[];
  map: OccupancyGrid | null;
  trajectory: TrajectoryPoint[];
  arena: ArenaInfo | null;
  setDrone: (d: Partial<DroneState>) => void;
  setMissionState: (s: MissionState) => void;
  setHealth: (h: Partial<SystemHealth>) => void;
  setSubsystemHealth: (k: keyof SystemHealth, h: Partial<SubsystemHealth>) => void;
  setCamera: (c: Partial<CameraFrame>) => void;
  setSimConfig: (c: Partial<SimulationConfig>) => void;
  addSurvivor: (s: Survivor) => void;
  addEvent: (e: MissionEvent) => void;
  setMissionTimer: (t: number) => void;
  setAutonomy: (a: Partial<AutonomyInfo>) => void;
  advanceMissionPhase: () => void;
  setMissionPhases: (p: MissionPhase[]) => void;
  setSurvivors: (s: Survivor[]) => void;
  setMap: (m: OccupancyGrid | null) => void;
  addTrajectoryPoint: (p: TrajectoryPoint) => void;
  clearTrajectory: () => void;
  setArena: (a: ArenaInfo | null) => void;
}

export const useSimulationStore = create<SimState>()((set, get) => ({
  drone: defaultSimDrone(),
  missionState: 'IDLE',
  missionPhases: MISSION_PHASES.map(p => ({ ...p })),
  missionTimer: 0,
  autonomy: {
    state: 'IDLE', planner: 'FUEL', localization: 'FAST-LIO2',
    currentTarget: '—', distanceToTarget: 0, reason: 'Awaiting simulation start',
  },
  survivors: [],
  health: defaultHealth(),
  camera: { dataUrl: null, width: 640, height: 480, fps: 0, connected: false, boundingBoxes: [], timestamp: Date.now() },
  simConfig: { scenario: 'scenario_01', speed: 1, state: 'STOPPED', px4SitlConnected: false, ros2Connected: false, gazeboConnected: false },
  events: [],
  map: null,
  trajectory: [],
  arena: null,

  setDrone: (d) => set(s => ({ drone: { ...s.drone, ...d } })),
  setMissionState: (missionState) => set({ missionState }),
  setHealth: (h) => set(s => ({ health: { ...s.health, ...h } })),
  setSubsystemHealth: (k, h) => set(s => ({ health: { ...s.health, [k]: { ...s.health[k], ...h, lastUpdate: Date.now() } } })),
  setCamera: (c) => set(s => ({ camera: { ...s.camera, ...c } })),
  setSimConfig: (c) => set(s => ({ simConfig: { ...s.simConfig, ...c } })),
  addSurvivor: (survivor) => set(s => ({ survivors: [...s.survivors.filter(x => x.id !== survivor.id), survivor] })),
  addEvent: (e) => set(s => ({ events: [e, ...s.events].slice(0, 200) })),
  setMissionTimer: (missionTimer) => set({ missionTimer }),
  setAutonomy: (a) => set(s => ({ autonomy: { ...s.autonomy, ...a } })),
  advanceMissionPhase: () => {
    const phases = [...get().missionPhases];
    const activeIdx = phases.findIndex(p => p.status === 'ACTIVE');
    if (activeIdx >= 0) {
      phases[activeIdx] = { ...phases[activeIdx], status: 'COMPLETE' };
      if (activeIdx + 1 < phases.length)
        phases[activeIdx + 1] = { ...phases[activeIdx + 1], status: 'ACTIVE' };
    } else {
      const pendingIdx = phases.findIndex(p => p.status === 'PENDING');
      if (pendingIdx >= 0)
        phases[pendingIdx] = { ...phases[pendingIdx], status: 'ACTIVE' };
    }
    set({ missionPhases: phases });
  },
  setMissionPhases: (missionPhases) => set({ missionPhases }),
  setSurvivors: (survivors) => set({ survivors }),
  setMap: (map) => set({ map }),
  addTrajectoryPoint: (p) => set(s => ({ trajectory: [...s.trajectory, p].slice(-3000) })),
  clearTrajectory: () => set({ trajectory: [] }),
  setArena: (arena) => set({ arena }),
}));

// ─── Hardware Store ────────────────────────────────────────────

export interface DroneConnectionConfig {
  connection_type: 'serial' | 'udp' | 'tcp' | 'simulator';
  serial_port?: string;
  baud_rate?: number;
  host?: string;
  udp_port?: number;
  tcp_port?: number;
  drone_name?: string;
  sys_id?: number;
  home_lat?: number;
  home_lon?: number;
  force_connect?: boolean;
}

/** Operator commands the Hardware page sends to the real drone (POST /api/hardware/command). */
export type DroneCommand = 'takeoff' | 'land' | 'rtl';

export interface DroneCommandResult {
  ok: boolean;
  detail: string;
}

export interface HWState {
  hwConnectionState: 'DISCONNECTED' | 'CONNECTING' | 'CONNECTED' | 'DEGRADED' | 'ERROR';
  connectionError: string | null;
  drone: DroneState;
  missionState: MissionState;
  missionPhases: MissionPhase[];
  missionTimer: number;
  autonomy: AutonomyInfo;
  health: SystemHealth;
  camera: CameraFrame;
  map: OccupancyGrid | null;
  survivors: Survivor[];
  trajectory: TrajectoryPoint[];
  events: MissionEvent[];
  emergencyAborted: boolean;
  arena: ArenaInfo | null;
  /** Command in flight to the drone (buttons show it), or null. */
  commandPending: DroneCommand | null;
  setDrone: (d: Partial<DroneState>) => void;
  setMissionState: (s: MissionState) => void;
  setHealth: (h: Partial<SystemHealth>) => void;
  setSubsystemHealth: (k: keyof SystemHealth, h: Partial<SubsystemHealth>) => void;
  setCamera: (c: Partial<CameraFrame>) => void;
  setMap: (m: OccupancyGrid | null) => void;
  addTrajectoryPoint: (p: TrajectoryPoint) => void;
  clearTrajectory: () => void;
  addSurvivor: (s: Survivor) => void;
  setSurvivors: (s: Survivor[]) => void;
  addEvent: (e: MissionEvent) => void;
  setMissionTimer: (t: number) => void;
  setMissionPhases: (p: MissionPhase[]) => void;
  setAutonomy: (a: Partial<AutonomyInfo>) => void;
  setArena: (a: ArenaInfo | null) => void;
  setEmergencyAborted: (v: boolean) => void;
  triggerEmergencyAbort: () => Promise<void>;
  sendCommand: (cmd: DroneCommand) => Promise<DroneCommandResult>;
  advanceMissionPhase: () => void;
  connectDrone: (config?: DroneConnectionConfig) => Promise<boolean>;
  disconnectDrone: () => void;
  setConnectionState: (state: HWState['hwConnectionState'], error?: string | null) => void;
  isConnectModalOpen: boolean;
  setConnectModalOpen: (open: boolean) => void;
}

export const useHardwareStore = create<HWState>()((set, get) => ({
  isConnectModalOpen: false,
  setConnectModalOpen: (open: boolean) => set({ isConnectModalOpen: open }),
  hwConnectionState: 'DISCONNECTED',
  connectionError: null,
  drone: defaultDisconnectedDrone(),
  missionState: 'IDLE',
  missionPhases: MISSION_PHASES.map(p => ({ ...p })),
  missionTimer: 0,
  autonomy: {
    state: 'IDLE', planner: '--', localization: '--',
    currentTarget: '--', distanceToTarget: 0, reason: 'DRONE DISCONNECTED — Awaiting hardware connection',
  },
  survivors: [],
  health: defaultHealth(),
  camera: { dataUrl: null, width: 1280, height: 720, fps: 0, connected: false, boundingBoxes: [], timestamp: 0 },
  map: null,
  trajectory: [],
  events: [],
  emergencyAborted: false,
  arena: null,
  commandPending: null,

  setDrone: (d) => set(s => ({ drone: { ...s.drone, ...d } })),
  setMissionState: (missionState) => set({ missionState }),
  setHealth: (h) => set(s => ({ health: { ...s.health, ...h } })),
  setSubsystemHealth: (k, h) => set(s => ({ health: { ...s.health, [k]: { ...s.health[k], ...h, lastUpdate: Date.now() } } })),
  setCamera: (c) => set(s => ({ camera: { ...s.camera, ...c } })),
  setMap: (map) => set({ map }),
  addTrajectoryPoint: (p) => set(s => ({ trajectory: [...s.trajectory, p].slice(-3000) })),
  clearTrajectory: () => set({ trajectory: [] }),
  addSurvivor: (survivor) => set(s => ({ survivors: [...s.survivors.filter(x => x.id !== survivor.id), survivor] })),
  setSurvivors: (survivors) => set({ survivors }),
  addEvent: (e) => set(s => ({ events: [e, ...s.events].slice(0, 200) })),
  setMissionTimer: (missionTimer) => set({ missionTimer }),
  setMissionPhases: (missionPhases) => set({ missionPhases }),
  setAutonomy: (a) => set(s => ({ autonomy: { ...s.autonomy, ...a } })),
  setArena: (arena) => set({ arena }),
  setEmergencyAborted: (emergencyAborted) => set({ emergencyAborted }),
  // EMERGENCY ABORT: PX4 AUTO.LAND on every link to the drone (backend: /api/hardware/abort).
  // The backend puts the abort and its outcome on the event timeline.
  triggerEmergencyAbort: async () => {
    set({ emergencyAborted: true, missionState: 'ABORT' });
    try {
      const res = await fetch(apiUrl('/api/hardware/abort'), { method: 'POST' });
      if (!res.ok) {
        const body = await res.json().catch(() => ({}));
        throw new Error(body.detail || `HTTP ${res.status}`);
      }
    } catch (err: any) {
      get().addEvent({
        id: `ui-abort-${Date.now()}`,
        timestamp: Date.now(),
        message: `ABORT NOT DELIVERED: ${err.message || 'GCS backend unreachable'} — USE THE RC TRANSMITTER`,
        level: 'ERROR',
      });
    }
  },
  sendCommand: async (cmd) => {
    set({ commandPending: cmd });
    try {
      const res = await fetch(apiUrl('/api/hardware/command'), {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ command: cmd }),
      });
      const body = await res.json().catch(() => ({}));
      return { ok: res.ok && !!body.ok, detail: body.detail || `HTTP ${res.status}` };
    } catch (err: any) {
      const detail = err.message || 'GCS backend unreachable';
      get().addEvent({
        id: `ui-cmd-${Date.now()}`, timestamp: Date.now(), level: 'ERROR',
        message: `${cmd.toUpperCase()} NOT SENT: ${detail}`,
      });
      return { ok: false, detail };
    } finally {
      set({ commandPending: null });
    }
  },
  advanceMissionPhase: () => {
    const phases = [...get().missionPhases];
    const activeIdx = phases.findIndex(p => p.status === 'ACTIVE');
    if (activeIdx >= 0) {
      phases[activeIdx] = { ...phases[activeIdx], status: 'COMPLETE' };
      if (activeIdx + 1 < phases.length)
        phases[activeIdx + 1] = { ...phases[activeIdx + 1], status: 'ACTIVE' };
    } else {
      const pendingIdx = phases.findIndex(p => p.status === 'PENDING');
      if (pendingIdx >= 0)
        phases[pendingIdx] = { ...phases[pendingIdx], status: 'ACTIVE' };
    }
    set({ missionPhases: phases });
  },

  setConnectionState: (hwConnectionState, connectionError = null) => {
    set({
      hwConnectionState,
      connectionError,
      drone: {
        ...get().drone,
        connectionStatus: hwConnectionState,
      },
    });
  },

  connectDrone: async (config?: DroneConnectionConfig) => {
    const { addEvent, setConnectionState } = get();
    setConnectionState('CONNECTING', null);
    addEvent({
      id: `evt-${Date.now()}`,
      timestamp: Date.now(),
      message: `INITIATING CONNECTION [${config?.connection_type?.toUpperCase() || 'HARDWARE'}]...`,
      level: 'INFO',
    });

    try {
      const res = await fetch(apiUrl('/api/hardware/connect'), {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(config || {}),
      });

      if (!res.ok) {
        const errData = await res.json().catch(() => ({}));
        throw new Error(errData.detail || errData.error || `HTTP ${res.status}: Hardware connection refused`);
      }

      const data = await res.json();
      if (data.connected) {
        setConnectionState('CONNECTED', null);
        set({ emergencyAborted: false });
        if (config?.drone_name) {
          set(state => ({
            drone: {
              ...state.drone,
              id: config.drone_name!,
              connectionStatus: 'CONNECTED',
            },
          }));
        }
        addEvent({
          id: `evt-${Date.now()}`,
          timestamp: Date.now(),
          message: data.message || 'REAL HARDWARE CONNECTED: TELEMETRY STREAM VERIFIED.',
          level: 'SUCCESS',
        });
        return true;
      } else {
        const errMsg = data.error || 'Connection failed: No flight controller heartbeat detected.';
        setConnectionState('ERROR', errMsg);
        addEvent({
          id: `evt-${Date.now()}`,
          timestamp: Date.now(),
          message: `HARDWARE CONNECTION FAILED: ${errMsg}`,
          level: 'ERROR',
        });
        return false;
      }
    } catch (err: any) {
      const errMsg = err.message || 'Connection failed: Real hardware backend unreachable.';
      setConnectionState('ERROR', errMsg);
      addEvent({
        id: `evt-${Date.now()}`,
        timestamp: Date.now(),
        message: `HARDWARE CONNECTION FAILED: ${errMsg}`,
        level: 'ERROR',
      });
      return false;
    }
  },

  disconnectDrone: () => {
    // Close the backend's links too (the drone itself keeps flying its onboard mission).
    fetch(apiUrl('/api/hardware/disconnect'), { method: 'POST' }).catch(() => { /* offline */ });
    set({
      hwConnectionState: 'DISCONNECTED',
      connectionError: null,
      drone: defaultDisconnectedDrone(),
      health: defaultHealth(),
      map: null,
      trajectory: [],
      survivors: [],
      camera: { dataUrl: null, width: 1280, height: 720, fps: 0, connected: false, boundingBoxes: [], timestamp: 0 },
      autonomy: {
        state: 'IDLE', planner: '--', localization: '--',
        currentTarget: '--', distanceToTarget: 0, reason: 'DRONE DISCONNECTED',
      },
    });
    get().addEvent({
      id: `evt-${Date.now()}`,
      timestamp: Date.now(),
      message: 'DRONE DISCONNECTED BY OPERATOR.',
      level: 'WARN',
    });
  },
}));
