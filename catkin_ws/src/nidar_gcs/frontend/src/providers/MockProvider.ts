import type {
  DroneState,
  OccupancyGrid,
  CameraFrame,
  SystemHealth,
  MissionState,
  Survivor,
  MissionEvent,
} from '@/types';
import type {
  DroneProvider,
  TelemetryProvider,
  MapProvider,
  CameraProvider,
  MissionProvider,
  SurvivorProvider,
  SystemHealthProvider,
} from './types';

/**
 * MockProvider — For UI design & component development only.
 * Explicitly stamps every item with [MOCK / DEVELOPMENT] branding.
 * NEVER used as REAL or HARDWARE data.
 */
export class MockProvider
  implements
    DroneProvider,
    TelemetryProvider,
    MapProvider,
    CameraProvider,
    MissionProvider,
    SurvivorProvider,
    SystemHealthProvider
{
  public readonly isMock = true;
  public readonly providerTag = 'DEVELOPMENT / MOCK';

  getState(): DroneState {
    return {
      id: 'DEV-MOCK-01',
      mode: 'DEV_MOCK_OFFBOARD',
      position: { x: 1.23, y: -0.45, z: 1.2 },
      velocity: { x: 0.1, y: -0.1, z: 0.0 },
      attitude: { roll: 0.01, pitch: -0.02, yaw: 0.8 },
      battery: { voltage: 16.2, current: 6.5, percentage: 88 },
      connectionStatus: 'CONNECTED',
      timestamp: Date.now(),
    };
  }

  onStateChange(callback: (state: DroneState) => void): () => void {
    const id = setInterval(() => callback(this.getState()), 100);
    return () => clearInterval(id);
  }

  subscribeTelemetry(callback: (telemetry: Partial<DroneState>) => void): () => void {
    const id = setInterval(() => callback(this.getState()), 100);
    return () => clearInterval(id);
  }

  getMap(): OccupancyGrid | null {
    return null;
  }

  subscribeMap(_callback: (grid: OccupancyGrid) => void): () => void {
    return () => {};
  }

  getFrame(): CameraFrame | null {
    return {
      dataUrl: null,
      width: 640,
      height: 480,
      fps: 0,
      connected: false,
      boundingBoxes: [],
      timestamp: Date.now(),
    };
  }

  subscribeFrames(_callback: (frame: CameraFrame) => void): () => void {
    return () => {};
  }

  getMissionState(): MissionState {
    return 'IDLE';
  }

  subscribeEvents(callback: (event: MissionEvent) => void): () => void {
    callback({
      id: `dev-mock-${Date.now()}`,
      timestamp: Date.now(),
      message: '[MOCK / DEV] DEVELOPMENT PROVIDER ATTACHED',
      level: 'WARN',
    });
    return () => {};
  }

  async triggerAbort(): Promise<void> {}

  getSurvivors(): Survivor[] {
    return [];
  }

  subscribeSurvivors(_callback: (survivor: Survivor) => void): () => void {
    return () => {};
  }

  getHealth(): SystemHealth {
    const mk = (name: string) => ({
      name: `[MOCK] ${name}`,
      status: 'OFFLINE' as const,
      lastUpdate: Date.now(),
    });
    return {
      px4: mk('PX4'),
      ros2: mk('ROS 2'),
      fastlio2: mk('FAST-LIO2'),
      lidar: mk('LiDAR'),
      imu: mk('IMU'),
      camera: mk('Camera'),
      yolo: mk('YOLO'),
      planner: mk('Planner'),
    };
  }

  subscribeHealth(callback: (health: SystemHealth) => void): () => void {
    callback(this.getHealth());
    return () => {};
  }
}
