import { useState, useMemo } from 'react';
import { useNavigate } from 'react-router-dom';
import {
  ChevronLeft, Play, Pause, RotateCcw, FastForward, Layers, Monitor,
} from 'lucide-react';
import { useSimulationStore } from '@/store';
import { useSimulationMockProvider } from '@/hooks/useMockProviders';
import { Sim3DView } from '@/components/simulation/Sim3DView';
import { CameraView } from '@/components/camera/CameraView';
import {
  SystemHealthRow, TelemetryValue, PanelHeader,
  MissionStateTracker, EventTimeline, BatteryBar, TopBarBadge,
} from '@/components/shared';
import { ResizeDivider } from '@/components/shared/ResizeDivider';
import { useResizableLayout } from '@/hooks/useResizableLayout';
import type { DroneState } from '@/types';

// ─── Simulation Controls ──────────────────────────────────────

const SCENARIOS = ['scenario_01', 'scenario_02', 'scenario_03', 'scenario_04'];

function SimControls() {
  const scenario    = useSimulationStore(s => s.simConfig.scenario);
  const simState    = useSimulationStore(s => s.simConfig.state);
  const setSimConfig = useSimulationStore(s => s.setSimConfig);
  const addEvent    = useSimulationStore(s => s.addEvent);

  const [speed, setSpeed] = useState(1);

  const handleStart = () => {
    setSimConfig({ state: 'RUNNING' });
    addEvent({ id: `e-${Date.now()}`, timestamp: Date.now(), message: 'SIMULATION STARTED', level: 'SUCCESS' });
  };
  const handlePause = () => {
    setSimConfig({ state: 'PAUSED' });
    addEvent({ id: `e-${Date.now()}`, timestamp: Date.now(), message: 'SIMULATION PAUSED', level: 'WARN' });
  };
  const handleReset = () => {
    setSimConfig({ state: 'STOPPED' });
    addEvent({ id: `e-${Date.now()}`, timestamp: Date.now(), message: 'SIMULATION RESET', level: 'WARN' });
  };
  const cycleSpeed = () => {
    const speeds = [0.5, 1, 2, 4];
    const idx = speeds.indexOf(speed);
    const next = speeds[(idx + 1) % speeds.length];
    setSpeed(next);
  };

  const btnBase = 'flex items-center gap-1.5 px-3 py-1.5 text-[10px] font-mono tracking-wider uppercase border transition-all duration-150';

  return (
    <div className="flex items-center" style={{ borderTop: '1px solid #27272A', background: '#111113' }}>
      <div className="flex items-center border-r" style={{ borderColor: '#27272A' }}>
        <div className="flex items-center gap-2 px-3 py-1.5 border-r" style={{ borderColor: '#27272A' }}>
          <Layers size={10} color="#A1A1AA" />
          <span className="text-[9px] font-mono text-disabled uppercase tracking-wider">SCENARIO</span>
          <select
            value={scenario}
            onChange={e => setSimConfig({ scenario: e.target.value })}
            className="text-[10px] font-mono border px-2 py-0.5 outline-none"
            style={{ background: '#18181B', color: '#00F0FF', borderColor: '#27272A', minWidth: 100 }}
          >
            {SCENARIOS.map(s => <option key={s} value={s}>{s.toUpperCase()}</option>)}
          </select>
        </div>
        <button
          onClick={cycleSpeed}
          className={`${btnBase} text-muted hover:text-text`}
          style={{ borderColor: 'transparent', background: 'transparent' }}
        >
          <FastForward size={10} />
          SPEED {speed}x
        </button>
      </div>

      <div className="flex items-center flex-1 justify-center">
        <button
          onClick={handleStart}
          disabled={simState === 'RUNNING'}
          className={`${btnBase} ${simState === 'RUNNING' ? 'opacity-30 cursor-not-allowed' : 'hover:text-green'}`}
          style={{ color: '#00FF41', borderColor: '#27272A', background: 'transparent' }}
        >
          <Play size={10} />START
        </button>
        <button
          onClick={handlePause}
          disabled={simState !== 'RUNNING'}
          className={`${btnBase} ${simState !== 'RUNNING' ? 'opacity-30 cursor-not-allowed' : 'hover:text-amber'}`}
          style={{ color: '#FFB000', borderColor: '#27272A', background: 'transparent', borderLeft: 'none' }}
        >
          <Pause size={10} />PAUSE
        </button>
        <button
          onClick={handleReset}
          className={`${btnBase} hover:text-red`}
          style={{ color: '#FF003C', borderColor: '#27272A', background: 'transparent', borderLeft: 'none' }}
        >
          <RotateCcw size={9} />RESET
        </button>
      </div>

      <div className="flex items-center gap-2 px-3 py-1.5 border-l text-[9px] font-mono" style={{ borderColor: '#27272A' }}>
        <span className={`status-dot ${simState === 'RUNNING' ? 'active' : simState === 'PAUSED' ? 'degraded' : 'idle'}`} />
        <span className="tracking-wider text-muted uppercase">{simState}</span>
      </div>
    </div>
  );
}

// ─── Telemetry panel ──────────────────────────────────────────

function SimTelemetry() {
  // Use granular primitive selectors to avoid object reference churn
  const px = useSimulationStore(s => s.drone.position.x);
  const py = useSimulationStore(s => s.drone.position.y);
  const pz = useSimulationStore(s => s.drone.position.z);
  const vx = useSimulationStore(s => s.drone.velocity.x);
  const vy = useSimulationStore(s => s.drone.velocity.y);
  const vz = useSimulationStore(s => s.drone.velocity.z);
  const roll  = useSimulationStore(s => s.drone.attitude.roll);
  const pitch = useSimulationStore(s => s.drone.attitude.pitch);
  const yaw   = useSimulationStore(s => s.drone.attitude.yaw);
  const batPct = useSimulationStore(s => s.drone.battery.percentage);
  const batV   = useSimulationStore(s => s.drone.battery.voltage);
  const batA   = useSimulationStore(s => s.drone.battery.current);
  const mode   = useSimulationStore(s => s.drone.mode);

  const missionState  = useSimulationStore(s => s.missionState);
  const missionPhases = useSimulationStore(s => s.missionPhases);
  const missionTimer  = useSimulationStore(s => s.missionTimer);
  const health        = useSimulationStore(s => s.health);
  const autonomy      = useSimulationStore(s => s.autonomy);
  const scenario      = useSimulationStore(s => s.simConfig.scenario);

  const mins = Math.floor(missionTimer / 60).toString().padStart(2, '0');
  const secs = (missionTimer % 60).toString().padStart(2, '0');

  return (
    <div className="flex flex-col h-full overflow-y-auto" style={{ scrollbarWidth: 'thin' }}>
      {/* Mission */}
      <div style={{ borderBottom: '1px solid #27272A' }}>
        <PanelHeader title="MISSION" accent="#FFB000" />
        <div className="px-3 py-2">
          <TelemetryValue label="STATE"   value={missionState} color="#FFB000" />
          <TelemetryValue label="ELAPSED" value={`${mins}:${secs}`} color="#00F0FF" />
        </div>
        <div className="px-3 pb-3">
          <MissionStateTracker phases={missionPhases} current={missionState} />
        </div>
      </div>

      {/* Position */}
      <div style={{ borderBottom: '1px solid #27272A' }}>
        <PanelHeader title="POSITION  (LOCAL XYZ)" accent="#00F0FF" />
        <div className="px-3 py-2">
          <TelemetryValue label="X" value={px !== null ? px.toFixed(3) : '--'} unit={px !== null ? 'm' : undefined} color="#00F0FF" />
          <TelemetryValue label="Y" value={py !== null ? py.toFixed(3) : '--'} unit={py !== null ? 'm' : undefined} color="#00F0FF" />
          <TelemetryValue label="Z" value={pz !== null ? pz.toFixed(3) : '--'} unit={pz !== null ? 'm' : undefined} color="#00F0FF" />
        </div>
      </div>

      {/* Attitude */}
      <div style={{ borderBottom: '1px solid #27272A' }}>
        <PanelHeader title="ATTITUDE" />
        <div className="px-3 py-2">
          <TelemetryValue label="ROLL"  value={roll !== null ? (roll  * 180 / Math.PI).toFixed(2) : '--'} unit={roll !== null ? '°' : undefined} />
          <TelemetryValue label="PITCH" value={pitch !== null ? (pitch * 180 / Math.PI).toFixed(2) : '--'} unit={pitch !== null ? '°' : undefined} />
          <TelemetryValue label="YAW"   value={yaw !== null ? (yaw   * 180 / Math.PI).toFixed(2) : '--'} unit={yaw !== null ? '°' : undefined} />
        </div>
      </div>

      {/* Velocity */}
      <div style={{ borderBottom: '1px solid #27272A' }}>
        <PanelHeader title="VELOCITY" />
        <div className="px-3 py-2">
          <TelemetryValue label="VX" value={vx !== null ? vx.toFixed(2) : '--'} unit={vx !== null ? 'm/s' : undefined} />
          <TelemetryValue label="VY" value={vy !== null ? vy.toFixed(2) : '--'} unit={vy !== null ? 'm/s' : undefined} />
          <TelemetryValue label="VZ" value={vz !== null ? vz.toFixed(2) : '--'} unit={vz !== null ? 'm/s' : undefined} />
        </div>
      </div>

      {/* Battery */}
      <div style={{ borderBottom: '1px solid #27272A' }}>
        <div className="px-3 py-2">
          <BatteryBar percentage={batPct} voltage={batV} current={batA} />
        </div>
      </div>

      {/* PX4 */}
      <div style={{ borderBottom: '1px solid #27272A' }}>
        <PanelHeader title="PX4 SITL" />
        <div className="px-3 py-2">
          <TelemetryValue label="MODE"     value={mode} color="#00F0FF" />
          <TelemetryValue label="SCENARIO" value={scenario.toUpperCase()} />
        </div>
      </div>

      {/* Autonomy */}
      <div style={{ borderBottom: '1px solid #27272A' }}>
        <PanelHeader title="AUTONOMY" accent="#FFB000" />
        <div className="px-3 py-2">
          <TelemetryValue label="PLANNER" value={autonomy.planner}                 color="#A1A1AA" />
          <TelemetryValue label="LOCALIZ" value={autonomy.localization}            color="#A1A1AA" />
          <TelemetryValue label="TARGET"  value={autonomy.currentTarget}           color="#FFB000" />
          <TelemetryValue label="DIST"    value={`${autonomy.distanceToTarget.toFixed(1)} m`} />
        </div>
        <div className="px-3 pb-2">
          <p className="text-[9px] font-mono text-disabled leading-snug">{autonomy.reason}</p>
        </div>
      </div>

      {/* System Health */}
      <div>
        <PanelHeader title="SYSTEM HEALTH" />
        <div className="px-3 py-1">
          {Object.values(health).map(sub => (
            <SystemHealthRow key={sub.name} subsystem={sub} />
          ))}
        </div>
      </div>
    </div>
  );
}

// ─── Top Bar ──────────────────────────────────────────────────

interface SimTopBarProps {
  onResetLayout?: () => void;
}

function SimTopBar({ onResetLayout }: SimTopBarProps) {
  const navigate = useNavigate();
  const gazebo    = useSimulationStore(s => s.simConfig.gazeboConnected);
  const px4Sitl   = useSimulationStore(s => s.simConfig.px4SitlConnected);
  const ros2      = useSimulationStore(s => s.simConfig.ros2Connected);
  const simState  = useSimulationStore(s => s.simConfig.state);
  const mState    = useSimulationStore(s => s.missionState);
  const timer     = useSimulationStore(s => s.missionTimer);
  const batPct    = useSimulationStore(s => s.drone.battery.percentage);

  const mins = Math.floor(timer / 60).toString().padStart(2, '0');
  const secs = (timer % 60).toString().padStart(2, '0');

  return (
    <div className="flex items-center h-10 flex-shrink-0"
      style={{ background: '#111113', borderBottom: '1px solid #27272A' }}>
      <button
        onClick={() => navigate('/')}
        className="flex items-center gap-1.5 px-3 h-full border-r text-muted hover:text-text text-[10px] font-mono tracking-wider uppercase"
        style={{ borderColor: '#27272A' }}
      >
        <ChevronLeft size={12} />BACK
      </button>
      <div className="flex items-center gap-2 px-4">
        <Monitor size={12} color="#00F0FF" />
        <span className="text-[11px] font-mono tracking-[0.15em] uppercase" style={{ color: '#00F0FF' }}>
          NIDAR SIMULATION
        </span>
      </div>
      <div className="flex items-center flex-1">
        <TopBarBadge label="GAZEBO"   value={gazebo  ? 'CONNECTED' : 'OFFLINE'} color={gazebo  ? '#00FF41' : '#FF003C'} dot dotStatus={gazebo  ? 'online' : 'offline'} />
        <TopBarBadge label="PX4 SITL" value={px4Sitl ? 'CONNECTED' : 'OFFLINE'} color={px4Sitl ? '#00FF41' : '#FF003C'} dot dotStatus={px4Sitl ? 'online' : 'offline'} />
        <TopBarBadge label="ROS 2"    value={ros2    ? 'CONNECTED' : 'OFFLINE'} color={ros2    ? '#00FF41' : '#FF003C'} dot dotStatus={ros2    ? 'online' : 'offline'} />
        <TopBarBadge label="MISSION"  value={mState}
          color={mState === 'EXPLORE' || mState === 'SURVIVOR_DETECTED' ? '#FFB000' : mState === 'COMPLETE' ? '#00FF41' : mState === 'ABORT' ? '#FF003C' : '#A1A1AA'}
          dot dotStatus={mState === 'IDLE' ? 'idle' : 'active'} />
        <TopBarBadge label="ELAPSED"  value={`${mins}:${secs}`} color="#00F0FF" />
        <TopBarBadge label="BATTERY"  value={batPct !== null ? `${batPct.toFixed(0)}%` : '--'}
          color={batPct === null ? '#A1A1AA' : batPct > 50 ? '#00FF41' : batPct > 20 ? '#FF5500' : '#FF003C'} />

        {onResetLayout && (
          <button
            onClick={onResetLayout}
            title="Reset panel sizes to defaults"
            className="flex items-center gap-1.5 px-2.5 py-1 text-[9px] font-mono tracking-wider uppercase border border-zinc-700 hover:border-zinc-500 text-zinc-400 hover:text-zinc-200 hover:bg-zinc-800/40 transition-colors ml-auto mr-3"
          >
            <RotateCcw size={10} />
            RESET LAYOUT
          </button>
        )}
      </div>
      <div className={`flex items-center gap-2 px-4 h-full border-l ${!onResetLayout ? 'ml-auto' : ''}`} style={{ borderColor: '#27272A' }}>
        <span className={`status-dot ${simState === 'RUNNING' ? 'active' : 'idle'}`} />
        <span className="text-[9px] font-mono tracking-widest text-muted uppercase">SIM {simState}</span>
      </div>
    </div>
  );
}

// ─── Stable drone snapshot for canvas (read-only, not reactive) ─

function useDroneSnapshot(): DroneState {
  // Use separate primitive selectors for the canvas view — these are batched
  const x     = useSimulationStore(s => s.drone.position.x);
  const y     = useSimulationStore(s => s.drone.position.y);
  const z     = useSimulationStore(s => s.drone.position.z);
  const vx    = useSimulationStore(s => s.drone.velocity.x);
  const vy    = useSimulationStore(s => s.drone.velocity.y);
  const vz    = useSimulationStore(s => s.drone.velocity.z);
  const roll  = useSimulationStore(s => s.drone.attitude.roll);
  const pitch = useSimulationStore(s => s.drone.attitude.pitch);
  const yaw   = useSimulationStore(s => s.drone.attitude.yaw);
  const batV  = useSimulationStore(s => s.drone.battery.voltage);
  const batA  = useSimulationStore(s => s.drone.battery.current);
  const batP  = useSimulationStore(s => s.drone.battery.percentage);
  const mode  = useSimulationStore(s => s.drone.mode);
  const cs    = useSimulationStore(s => s.drone.connectionStatus);

  return useMemo(() => ({
    id: 'NIDAR-01', mode, connectionStatus: cs, timestamp: Date.now(),
    position: { x, y, z },
    velocity: { x: vx, y: vy, z: vz },
    attitude: { roll, pitch, yaw },
    battery:  { voltage: batV, current: batA, percentage: batP },
  }), [x, y, z, vx, vy, vz, roll, pitch, yaw, batV, batA, batP, mode, cs]);
}

// ─── Simulation Dashboard ─────────────────────────────────────

export default function SimulationDashboard() {
  useSimulationMockProvider();

  const drone   = useDroneSnapshot();
  const events  = useSimulationStore(s => s.events);

  const trajX = useSimulationStore(s => s.drone.position.x);
  const trajY = useSimulationStore(s => s.drone.position.y);
  const trajectory = useMemo(() => (trajX !== null && trajY !== null ? [{ x: trajX, y: trajY }] : []), [trajX, trajY]);

  // VS Code style resizable layout for Simulation
  const {
    sidebarWidth,
    bottomHeight,
    timelineWidth,
    handleResizeSidebar,
    handleResizeBottom,
    handleResizeTimeline,
    handleSaveSidebar,
    handleSaveBottom,
    handleSaveTimeline,
    resetLayout,
    resetSidebar,
    resetBottom,
    resetTimeline,
  } = useResizableLayout({
    storageKeyPrefix: 'nidar_sim',
    defaultSidebarWidth: 260,
    minSidebarWidth: 180,
    maxSidebarWidth: 600,
    defaultBottomHeight: 200,
    minBottomHeight: 140,
    maxBottomHeight: 480,
    defaultTimelineWidth: 340,
    minTimelineWidth: 180,
    maxTimelineWidth: 550,
  });

  return (
    <div className="flex flex-col h-screen overflow-hidden"
      style={{ background: '#09090B', fontFamily: 'IBM Plex Sans, sans-serif' }}>
      <SimTopBar onResetLayout={resetLayout} />
      <div className="flex flex-1 overflow-hidden select-none" style={{ minHeight: 0 }}>
        {/* LEFT: 3D view + camera */}
        <div className="flex flex-col flex-1 overflow-hidden" style={{ minWidth: 0 }}>
          {/* 3D Simulation View */}
          <div className="flex-1 relative overflow-hidden"
            style={{ border: '1px solid #27272A', borderTop: 'none', borderLeft: 'none', minHeight: '120px' }}>
            <div
              className="absolute top-0 left-0 z-10 px-3 py-1.5 flex items-center gap-2"
              style={{ background: 'rgba(17,17,19,0.92)', borderBottom: '1px solid #27272A', borderRight: '1px solid #27272A' }}
            >
              <Layers size={9} color="#00F0FF" />
              <span className="text-[9px] font-mono tracking-[0.18em] uppercase text-muted">
                3D SIMULATION VIEW  —  GAZEBO + PX4 SITL  —  SCENARIO_01
              </span>
            </div>
            <Sim3DView drone={drone} trajectory={trajectory} className="w-full h-full" />
          </div>

          {/* Draggable Horizontal Divider */}
          <ResizeDivider
            direction="horizontal"
            onResize={handleResizeBottom}
            onResizeEnd={handleSaveBottom}
            onReset={resetBottom}
            accentColor="#00F0FF"
            title="Drag vertically to resize 3D View / Camera (Double-click to reset)"
          />

          {/* Camera + Events */}
          <div className="flex flex-shrink-0 overflow-hidden" style={{ height: `${bottomHeight}px` }}>
            <div className="flex-1 flex flex-col overflow-hidden" style={{ minWidth: '160px' }}>
              <PanelHeader title="SIMULATED CAMERA — YOLO OVERLAY" accent="#00F0FF" />
              <div className="flex-1 overflow-hidden">
                <CameraView mode="simulation" className="w-full h-full" />
              </div>
            </div>

            {/* Draggable Vertical Divider between Camera and Event Timeline */}
            <ResizeDivider
              direction="vertical"
              onResize={handleResizeTimeline}
              onResizeEnd={handleSaveTimeline}
              onReset={resetTimeline}
              accentColor="#00F0FF"
              title="Drag horizontally to resize Camera / Timeline (Double-click to reset)"
            />

            <div className="flex flex-col overflow-hidden flex-shrink-0" style={{ width: `${timelineWidth}px` }}>
              <PanelHeader title="EVENT TIMELINE" />
              <div className="flex-1 overflow-hidden px-1 py-1">
                <EventTimeline events={events} maxHeight="100%" />
              </div>
            </div>
          </div>
        </div>

        {/* Draggable Vertical Divider */}
        <ResizeDivider
          direction="vertical"
          onResize={handleResizeSidebar}
          onResizeEnd={handleSaveSidebar}
          onReset={resetSidebar}
          accentColor="#00F0FF"
          title="Drag horizontally to resize Right Telemetry (Double-click to reset)"
        />

        {/* RIGHT: Telemetry */}
        <div className="flex-shrink-0 overflow-hidden flex flex-col"
          style={{ width: `${sidebarWidth}px`, background: '#111113' }}>
          <SimTelemetry />
        </div>
      </div>

      <SimControls />
    </div>
  );
}
