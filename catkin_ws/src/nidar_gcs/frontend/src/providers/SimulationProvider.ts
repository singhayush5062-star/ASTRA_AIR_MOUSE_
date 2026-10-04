import type {
  DroneState,
  CameraFrame,
  SystemHealth,
  MissionState,
  Survivor,
  MissionEvent,
  SimulationConfig,
} from '@/types';
import type {
  DroneProvider,
  TelemetryProvider,
  CameraProvider,
  MissionProvider,
  SurvivorProvider,
  SystemHealthProvider,
} from './types';
import { useSimulationStore } from '@/store';
import { API_BASE } from '@/services/api';

export class SimulationProvider
  implements
    DroneProvider,
    TelemetryProvider,
    CameraProvider,
    MissionProvider,
    SurvivorProvider,
    SystemHealthProvider
{
  private baseUrl: string;

  constructor(baseUrl: string = API_BASE) {
    this.baseUrl = baseUrl;
  }

  getState(): DroneState {
    return useSimulationStore.getState().drone;
  }

  onStateChange(callback: (state: DroneState) => void): () => void {
    return useSimulationStore.subscribe((s) => callback(s.drone));
  }

  subscribeTelemetry(callback: (telemetry: Partial<DroneState>) => void): () => void {
    return useSimulationStore.subscribe((s) => callback(s.drone));
  }

  getFrame(): CameraFrame | null {
    return useSimulationStore.getState().camera;
  }

  subscribeFrames(callback: (frame: CameraFrame) => void): () => void {
    return useSimulationStore.subscribe((s) => callback(s.camera));
  }

  getMissionState(): MissionState {
    return useSimulationStore.getState().missionState;
  }

  subscribeEvents(callback: (event: MissionEvent) => void): () => void {
    return useSimulationStore.subscribe((s) => {
      if (s.events.length > 0) callback(s.events[0]);
    });
  }

  /** Operator emergency abort: the backend commands PX4 AUTO.LAND (land in place now). */
  async triggerAbort(): Promise<void> {
    const store = useSimulationStore.getState();
    store.setMissionState('ABORT');
    try {
      const res = await fetch(`${this.baseUrl}/api/mission/abort`, { method: 'POST' });
      if (!res.ok) throw new Error(`HTTP ${res.status}`);
    } catch {
      store.addEvent({
        id: `ui-${Date.now()}`, timestamp: Date.now(), level: 'ERROR',
        message: 'ABORT NOT DELIVERED — GCS BACKEND UNREACHABLE',
      });
    }
  }

  getSurvivors(): Survivor[] {
    return useSimulationStore.getState().survivors;
  }

  subscribeSurvivors(callback: (survivor: Survivor) => void): () => void {
    return useSimulationStore.subscribe((s) => {
      if (s.survivors.length > 0) {
        callback(s.survivors[s.survivors.length - 1]);
      }
    });
  }

  getHealth(): SystemHealth {
    return useSimulationStore.getState().health;
  }

  subscribeHealth(callback: (health: SystemHealth) => void): () => void {
    return useSimulationStore.subscribe((s) => callback(s.health));
  }

  /**
   * Start / pause / reset the real simulation through the GCS backend. The backend owns the
   * simulation state: the store shows the transitional state (STARTING, RESETTING) until the
   * next telemetry message reports what actually happened.
   */
  async sendSimControl(action: 'start' | 'pause' | 'reset', extra?: Partial<SimulationConfig>): Promise<void> {
    const store = useSimulationStore.getState();
    const previous = store.simConfig.state;
    if (action === 'start') {
      store.setSimConfig({ state: previous === 'PAUSED' ? 'RUNNING' : 'STARTING', ...extra });
    } else if (action === 'reset') {
      store.setSimConfig({ state: 'RESETTING', ...extra });
    } else if (extra) {
      store.setSimConfig(extra);
    }

    try {
      const res = await fetch(`${this.baseUrl}/api/simulation/control`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ action, ...extra }),
      });
      const data = await res.json().catch(() => null);
      if (data && data.sim_state) store.setSimConfig({ state: data.sim_state });
    } catch {
      store.setSimConfig({ state: previous });
      store.addEvent({
        id: `ui-${Date.now()}`, timestamp: Date.now(), level: 'ERROR',
        message: `GCS BACKEND UNREACHABLE — ${action.toUpperCase()} NOT SENT`,
      });
    }
  }
}
