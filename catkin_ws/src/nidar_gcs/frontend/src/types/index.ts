// ============================================================
// NIDAR AirMouse GCS — Domain Types
// ============================================================

// ─── Enumerations ────────────────────────────────────────────

export type OperatingMode = 'simulation' | 'hardware';

export type MissionState =
  | 'INIT'
  | 'TAKEOFF'
  | 'LOCALIZATION'
  | 'EXPLORE'
  | 'SURVIVOR_DETECTED'
  | 'CONTINUE_EXPLORE'
  | 'RETURN'
  | 'LAND'
  | 'COMPLETE'
  | 'ABORT'
  | 'FAILSAFE'
  | 'IDLE';

export type HardwareConnectionState = 'DISCONNECTED' | 'CONNECTING' | 'CONNECTED' | 'DEGRADED' | 'ERROR';
export type ConnectionStatus = HardwareConnectionState;

export type SubsystemStatus = 'ONLINE' | 'OFFLINE' | 'DEGRADED' | 'UNKNOWN' | 'READY' | 'TRACKING' | 'RUNNING' | 'CONNECTED';

export type CellType = 'UNKNOWN' | 'FREE' | 'OCCUPIED' | 'EXPLORED' | 'FRONTIER';

export type SimulationState = 'STOPPED' | 'STARTING' | 'RUNNING' | 'PAUSED' | 'RESETTING' | 'ERROR';

export type SurvivorStatus = 'DETECTED' | 'CONFIRMED' | 'LOCALIZED' | 'REPORTED';

// ─── Core Drone State ────────────────────────────────────────

export interface Vec3 {
  x: number | null;
  y: number | null;
  z: number | null;
}

export interface Attitude {
  roll: number | null;
  pitch: number | null;
  yaw: number | null;
}

export interface Battery {
  voltage: number | null;
  current: number | null;
  percentage: number | null;
}

export interface DroneState {
  id: string;
  mode: string; // PX4 flight mode e.g. OFFBOARD, POSCTL, or '--'
  position: Vec3;
  velocity: Vec3;
  attitude: Attitude;
  battery: Battery;
  connectionStatus: HardwareConnectionState;
  timestamp: number;
}

// ─── Map ─────────────────────────────────────────────────────

export interface OccupancyGridCell {
  x: number;
  y: number;
  type: CellType;
}

export interface OccupancyGridMeta {
  width: number;       // cells
  height: number;      // cells
  resolution: number;  // meters per cell
  originX: number;     // meters
  originY: number;     // meters
}

export interface OccupancyGrid {
  meta: OccupancyGridMeta;
  data: Int8Array | number[];  // -1=unknown, 0=free, 100=occupied
  timestamp: number;
}

export interface TrajectoryPoint {
  x: number;
  y: number;
  timestamp: number;
}

// ─── Survivors ───────────────────────────────────────────────

export interface Survivor {
  id: number;
  gridLabel: string;   // e.g. "B4"
  position: Vec3;
  confidence: number;  // 0–100
  status: SurvivorStatus;
  detectedAt: number;  // timestamp
}

// ─── Mission ─────────────────────────────────────────────────

export interface MissionEvent {
  id: string;
  timestamp: number;
  message: string;
  level: 'INFO' | 'WARN' | 'ERROR' | 'SUCCESS';
}

export interface MissionPhase {
  state: MissionState;
  status: 'PENDING' | 'ACTIVE' | 'COMPLETE' | 'FAILED';
}

export interface AutonomyInfo {
  state: MissionState;
  planner: string;
  localization: string;
  currentTarget: string;
  distanceToTarget: number;
  reason: string;
}

// ─── System Health ────────────────────────────────────────────

export interface SubsystemHealth {
  name: string;
  status: SubsystemStatus;
  detail?: string;
  lastUpdate: number;
}

export interface SystemHealth {
  px4: SubsystemHealth;
  ros2: SubsystemHealth;
  fastlio2: SubsystemHealth;
  lidar: SubsystemHealth;
  imu: SubsystemHealth;
  camera: SubsystemHealth;
  yolo: SubsystemHealth;
  planner: SubsystemHealth;
}

// ─── Camera ──────────────────────────────────────────────────

export interface BoundingBox {
  x: number;
  y: number;
  width: number;
  height: number;
  confidence: number;
  survivorId: number;
  gridLabel?: string;
}

export interface CameraFrame {
  dataUrl: string | null;      // base64 image or null
  width: number;
  height: number;
  fps: number;
  connected: boolean;
  boundingBoxes: BoundingBox[];
  timestamp: number;
}

// ─── Simulation-specific ──────────────────────────────────────

export interface SimulationConfig {
  scenario: string;
  speed: number;            // 0.5x, 1x, 2x
  state: SimulationState;
  px4SitlConnected: boolean;
  ros2Connected: boolean;
  gazeboConnected: boolean;
}

export interface SimulationScenario {
  id: string;
  name: string;
  description: string;
  survivorCount: number;
}

// ─── WebSocket Messages ────────────────────────────────────────

export interface WsMessage<T = unknown> {
  type: string;
  payload: T;
  timestamp: number;
}
