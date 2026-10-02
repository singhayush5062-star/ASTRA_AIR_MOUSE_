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

  constructor(baseUrl: string = 'http://localhost:8000') {
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

  async triggerAbort(): Promise<void> {
    useSimulationStore.getState().setMissionState('ABORT');
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

  async sendSimControl(action: 'start' | 'pause' | 'reset', extra?: Partial<SimulationConfig>): Promise<void> {
    const store = useSimulationStore.getState();
    if (action === 'start') {
      store.setSimConfig({ state: 'RUNNING', ...extra });
    } else if (action === 'pause') {
      store.setSimConfig({ state: 'PAUSED', ...extra });
    } else if (action === 'reset') {
      store.setSimConfig({ state: 'STOPPED', ...extra });
    }

    try {
      await fetch(`${this.baseUrl}/api/simulation/control`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ action, ...extra }),
      });
    } catch {
      // Backend offline fallback handled locally
    }
  }
}
