// ============================================================
// NIDAR AirMouse — Domain Types & Interfaces
// ============================================================

export type AirMouseConnectionState =
  | 'DISCONNECTED'
  | 'CONNECTING'
  | 'CONNECTED'
  | 'RECONNECTING'
  | 'ERROR'
  | 'STALE';

export type AirMouseGesture =
  | 'IDLE'
  | 'MOVE'
  | 'LEFT_CLICK'
  | 'RIGHT_CLICK'
  | 'DOUBLE_CLICK'
  | 'SCROLL_UP'
  | 'SCROLL_DOWN'
  | 'DRAG'
  | 'RELEASE';

export interface AirMouseSettings {
  sensitivity: number;        // 1 - 20 (default: 8)
  smoothing: number;          // 0.05 - 0.95 (default: 0.35)
  deadzone: number;           // 0.001 - 0.05 (default: 0.005 radians)
  scrollSensitivity: number;  // 1 - 10 (default: 4)
  gesturesEnabled: boolean;   // default: true
  invertX: boolean;           // default: false
  invertY: boolean;           // default: false
}

export interface RawSensorData {
  // Position
  x: number | null;
  y: number | null;
  z: number | null;
  // Attitude / Orientation (radians)
  roll: number | null;
  pitch: number | null;
  yaw: number | null;
  // Velocity
  vx: number | null;
  vy: number | null;
  vz: number | null;
  // Battery
  voltage: number | null;
  current: number | null;
  batteryPercentage: number | null;
  // Status
  timestamp: number;
}

export interface AirMouseDiagnostics {
  connectionState: AirMouseConnectionState;
  connectionError: string | null;
  packetsReceived: number;
  packetsPerSecond: number;
  latencyMs: number;
  lastPacketTimestamp: number;
  currentGesture: AirMouseGesture;
  cursorX: number;
  cursorY: number;
  isDragging: boolean;
  leftButtonDown: boolean;
  rightButtonDown: boolean;
}

export interface AirMouseBackendPayload {
  type: string;
  mode?: string;
  payload?: {
    drone?: {
      id?: string;
      mode?: string;
      position?: { x?: number | null; y?: number | null; z?: number | null };
      velocity?: { x?: number | null; y?: number | null; z?: number | null };
      attitude?: { roll?: number | null; pitch?: number | null; yaw?: number | null };
      battery?: { voltage?: number | null; current?: number | null; percentage?: number | null };
      connectionStatus?: string;
      timestamp?: number;
    };
    health?: Record<string, { name?: string; status?: string }>;
    status?: string;
  };
  timestamp?: number;
}
