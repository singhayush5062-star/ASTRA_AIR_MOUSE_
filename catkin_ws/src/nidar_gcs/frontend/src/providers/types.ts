import type {
  DroneState,
  OccupancyGrid,
  CameraFrame,
  SystemHealth,
  MissionState,
  Survivor,
  MissionEvent,
  HardwareConnectionState,
} from '@/types';

export interface DroneProvider {
  getState(): DroneState;
  onStateChange(callback: (state: DroneState) => void): () => void;
}

export interface TelemetryProvider {
  subscribeTelemetry(callback: (telemetry: Partial<DroneState>) => void): () => void;
}

export interface MapProvider {
  getMap(): OccupancyGrid | null;
  subscribeMap(callback: (grid: OccupancyGrid) => void): () => void;
}

export interface CameraProvider {
  getFrame(): CameraFrame | null;
  subscribeFrames(callback: (frame: CameraFrame) => void): () => void;
}

export interface MissionProvider {
  getMissionState(): MissionState;
  subscribeEvents(callback: (event: MissionEvent) => void): () => void;
  triggerAbort(): Promise<void>;
}

export interface SurvivorProvider {
  getSurvivors(): Survivor[];
  subscribeSurvivors(callback: (survivor: Survivor) => void): () => void;
}

export interface SystemHealthProvider {
  getHealth(): SystemHealth;
  subscribeHealth(callback: (health: SystemHealth) => void): () => void;
}

export interface IHardwareProvider {
  connectionStatus: HardwareConnectionState;
  connect(): Promise<{ success: boolean; error?: string }>;
  disconnect(): Promise<void>;
}
