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
import type {
  DroneProvider,
  TelemetryProvider,
  MapProvider,
  CameraProvider,
  MissionProvider,
  SurvivorProvider,
  SystemHealthProvider,
  IHardwareProvider,
} from './types';
import { useHardwareStore } from '@/store';

export class HardwareProvider
  implements
    DroneProvider,
    TelemetryProvider,
    MapProvider,
    CameraProvider,
    MissionProvider,
    SurvivorProvider,
    SystemHealthProvider,
    IHardwareProvider
{
  private baseUrl: string;
  private ws: WebSocket | null = null;

  constructor(baseUrl: string = 'http://localhost:8000') {
    this.baseUrl = baseUrl;
  }

  get connectionStatus(): HardwareConnectionState {
    return useHardwareStore.getState().hwConnectionState;
  }

  async connect(): Promise<{ success: boolean; error?: string }> {
    const store = useHardwareStore.getState();
    await store.connectDrone();
    const updatedStatus = useHardwareStore.getState().hwConnectionState;
    if (updatedStatus === 'CONNECTED') {
      this.initWebSocket();
      return { success: true };
    }
    return {
      success: false,
      error: useHardwareStore.getState().connectionError || 'Hardware connection failed',
    };
  }

  async disconnect(): Promise<void> {
    if (this.ws) {
      this.ws.close();
      this.ws = null;
    }
    useHardwareStore.getState().disconnectDrone();
  }

  private initWebSocket() {
    try {
      const wsUrl = this.baseUrl.replace(/^http/, 'ws') + '/api/ws/hardware';
      this.ws = new WebSocket(wsUrl);
      this.ws.onmessage = (event) => {
        try {
          const msg = JSON.parse(event.data);
          if (msg.type === 'telemetry' && msg.payload?.drone) {
            useHardwareStore.getState().setDrone(msg.payload.drone);
          }
          if (msg.type === 'health' && msg.payload) {
            useHardwareStore.getState().setHealth(msg.payload);
          }
        } catch {
          // ignore malformed frame
        }
      };
      this.ws.onerror = () => {
        if (useHardwareStore.getState().hwConnectionState === 'CONNECTED') {
          useHardwareStore.getState().setConnectionState('DEGRADED', 'WebSocket link degraded');
        }
      };
      this.ws.onclose = () => {
        if (useHardwareStore.getState().hwConnectionState === 'CONNECTED') {
          useHardwareStore.getState().setConnectionState('DISCONNECTED', 'Hardware connection dropped');
        }
      };
    } catch (e) {
      // WS initialization error
    }
  }

  getState(): DroneState {
    return useHardwareStore.getState().drone;
  }

  onStateChange(callback: (state: DroneState) => void): () => void {
    return useHardwareStore.subscribe((s) => callback(s.drone));
  }

  subscribeTelemetry(callback: (telemetry: Partial<DroneState>) => void): () => void {
    return useHardwareStore.subscribe((s) => callback(s.drone));
  }

  getMap(): OccupancyGrid | null {
    return useHardwareStore.getState().map;
  }

  subscribeMap(callback: (grid: OccupancyGrid) => void): () => void {
    return useHardwareStore.subscribe((s) => {
      if (s.map) callback(s.map);
    });
  }

  getFrame(): CameraFrame | null {
    return useHardwareStore.getState().camera;
  }

  subscribeFrames(callback: (frame: CameraFrame) => void): () => void {
    return useHardwareStore.subscribe((s) => callback(s.camera));
  }

  getMissionState(): MissionState {
    return useHardwareStore.getState().missionState;
  }

  subscribeEvents(callback: (event: MissionEvent) => void): () => void {
    return useHardwareStore.subscribe((s) => {
      if (s.events.length > 0) callback(s.events[0]);
    });
  }

  async triggerAbort(): Promise<void> {
    useHardwareStore.getState().triggerEmergencyAbort();
    try {
      await fetch(`${this.baseUrl}/api/mission/abort`, { method: 'POST' });
    } catch {
      // Offline fallback already handled in store
    }
  }

  getSurvivors(): Survivor[] {
    return useHardwareStore.getState().survivors;
  }

  subscribeSurvivors(callback: (survivor: Survivor) => void): () => void {
    return useHardwareStore.subscribe((s) => {
      if (s.survivors.length > 0) {
        callback(s.survivors[s.survivors.length - 1]);
      }
    });
  }

  getHealth(): SystemHealth {
    return useHardwareStore.getState().health;
  }

  subscribeHealth(callback: (health: SystemHealth) => void): () => void {
    return useHardwareStore.subscribe((s) => callback(s.health));
  }
}
