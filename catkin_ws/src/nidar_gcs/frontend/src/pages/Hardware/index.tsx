import { useState, useRef } from 'react';
import { useNavigate } from 'react-router-dom';
import {
  ChevronLeft, Radio, Cpu, Activity, ShieldAlert, Wifi,
  AlertOctagon, Loader2, PowerOff, MousePointer, RotateCcw,
} from 'lucide-react';
import { useHardwareStore } from '@/store';
import { OccupancyGridMap } from '@/components/map/OccupancyGridMap';
import { CameraView, type CameraStatus } from '@/components/camera/CameraView';
import {
  SystemHealthRow, TelemetryValue, PanelHeader,
  MissionStateTracker, EventTimeline, BatteryBar, TopBarBadge,
} from '@/components/shared';
import { ResizeDivider } from '@/components/shared/ResizeDivider';
import { useResizableLayout } from '@/hooks/useResizableLayout';
import { ConnectDroneModal } from '@/components/hardware/ConnectDroneModal';
import { useAirMouse } from '@/hooks/useAirMouse';
import { AirMouseOverlay } from '@/components/airmouse/AirMouseOverlay';
import { AirMouseDashboard } from '@/components/airmouse/AirMouseDashboard';

// ─── Hardware Actions & Emergency Abort ────────────────────────

function HardwareControls() {
  const hwState             = useHardwareStore(s => s.hwConnectionState);
  const connError           = useHardwareStore(s => s.connectionError);
  const disconnectDrone     = useHardwareStore(s => s.disconnectDrone);
  const triggerAbort        = useHardwareStore(s => s.triggerEmergencyAbort);
  const aborted             = useHardwareStore(s => s.emergencyAborted);
  const setConnectModalOpen = useHardwareStore(s => s.setConnectModalOpen);

  const [armed, setArmed] = useState(false);
  const [countdown, setCountdown] = useState(0);
  const timerRef = useRef<ReturnType<typeof setTimeout> | undefined>(undefined);

  const handleAbortPress = () => {
    if (aborted) return;
    if (!armed) {
      setArmed(true);
      setCountdown(3);
      const interval = setInterval(() => {
        setCountdown(c => {
          if (c <= 1) { clearInterval(interval); setArmed(false); return 0; }
          return c - 1;
        });
      }, 1000);
      timerRef.current = setTimeout(() => setArmed(false), 3500);
      return;
    }
    clearTimeout(timerRef.current);
    setArmed(false);
    triggerAbort();
  };

  // If drone is disconnected or failed: show Connect button
  if (hwState === 'DISCONNECTED' || hwState === 'ERROR') {
    return (
      <div className="flex flex-col gap-2">
        {hwState === 'ERROR' && connError && (
          <div
            className="p-2 border text-[9px] font-mono leading-tight cursor-pointer hover:opacity-90"
            style={{ background: '#1c050a', borderColor: '#FF003C', color: '#FF003C' }}
            onClick={() => setConnectModalOpen(true)}
          >
            <div className="flex items-center gap-1 font-bold mb-1">
              <AlertOctagon size={11} /> CONNECTION FAILED
            </div>
            <p className="text-muted">{connError}</p>
          </div>
        )}

        <button
          onClick={() => setConnectModalOpen(true)}
          className="w-full flex items-center justify-center gap-2 py-3 text-xs font-mono font-bold tracking-[0.15em] uppercase transition-all duration-200 group"
          style={{
            background: 'linear-gradient(135deg, rgba(255,176,0,0.15), rgba(255,176,0,0.25))',
            border: '1px solid #FFB000',
            color: '#FFB000',
            boxShadow: '0 0 16px rgba(255,176,0,0.25)',
          }}
        >
          <Wifi size={14} className="group-hover:scale-110 transition-transform" />
          {hwState === 'ERROR' ? 'RETRY DRONE CONNECTION' : 'CONNECT DRONE'}
        </button>
      </div>
    );
  }

  // If connecting: show loading state
  if (hwState === 'CONNECTING') {
    return (
      <div
        className="w-full flex items-center justify-center gap-2.5 py-3 text-xs font-mono font-bold tracking-[0.15em] uppercase"
        style={{ background: '#18181B', border: '1px solid #FFB000', color: '#FFB000' }}
      >
        <Loader2 size={14} className="animate-spin" />
        CONNECTING TO DRONE...
      </div>
    );
  }

  // If connected: show Abort and Disconnect controls
  return (
    <div className="flex flex-col gap-2">
      {aborted ? (
        <div
          className="flex items-center justify-center gap-2 py-2 text-[11px] font-mono font-bold tracking-widest uppercase animate-pulse"
          style={{ background: '#1a0008', border: '2px solid #FF003C', color: '#FF003C' }}
        >
          <ShieldAlert size={14} />ABORT ACTIVE
        </div>
      ) : (
        <button
          onClick={handleAbortPress}
          className="w-full flex items-center justify-center gap-2 py-2 text-[11px] font-mono font-bold tracking-widest uppercase transition-all duration-150"
          style={armed ? {
            background: '#FF003C', border: '2px solid #FF003C', color: '#09090B',
            boxShadow: '0 0 20px rgba(255,0,60,0.6)',
          } : {
            background: '#110007', border: '2px solid #FF003C', color: '#FF003C',
          }}
        >
          <ShieldAlert size={14} />
          {armed ? `CONFIRM ABORT (${countdown})` : 'EMERGENCY ABORT'}
        </button>
      )}

      <button
        onClick={() => disconnectDrone()}
        className="w-full flex items-center justify-center gap-1.5 py-1.5 text-[9px] font-mono tracking-wider uppercase border text-muted hover:text-red hover:border-red transition-colors"
        style={{ borderColor: '#27272A', background: 'transparent' }}
      >
        <PowerOff size={11} />
        DISCONNECT
      </button>
    </div>
  );
}

// ─── Survivor Panel ────────────────────────────────────────────

function SurvivorPanel() {
  const hwState   = useHardwareStore(s => s.hwConnectionState);
  const survivors = useHardwareStore(s => s.survivors);
  const isConnected = hwState === 'CONNECTED';

  return (
    <div className="flex flex-col h-full overflow-hidden">
      <PanelHeader
        title="SURVIVORS"
        accent={isConnected ? '#00FF41' : '#A1A1AA'}
        right={
          <div className="flex items-center gap-1 text-[10px] font-mono" style={{ color: isConnected ? '#00FF41' : '#52525B' }}>
            <span className="font-bold">{String(survivors.length).padStart(2, '0')}</span>
            <span className="text-muted"> / 06</span>
          </div>
        }
      />
      <div className="flex-1 overflow-y-auto" style={{ scrollbarWidth: 'thin' }}>
        {!isConnected ? (
          <div className="flex flex-col items-center justify-center h-28 text-center p-4">
            <span className="text-[10px] font-mono tracking-widest text-disabled uppercase">
              NO DATA
            </span>
            <span className="text-[8px] font-mono text-disabled uppercase mt-1">
              HARDWARE DISCONNECTED
            </span>
          </div>
        ) : survivors.length === 0 ? (
          <div className="flex items-center justify-center h-20 text-[10px] font-mono text-disabled uppercase tracking-wider">
            NO SURVIVORS DETECTED
          </div>
        ) : (
          survivors.map(s => {
            const confColor = s.confidence >= 90 ? '#00FF41' : s.confidence >= 70 ? '#FFB000' : '#FF5500';
            return (
              <div key={s.id} className="px-3 py-2 border-b" style={{ borderColor: '#1c1c1e' }}>
                <div className="flex items-center justify-between mb-1">
                  <span className="text-[10px] font-mono font-bold tracking-wider" style={{ color: '#00FF41' }}>
                    SURVIVOR #{String(s.id).padStart(2, '0')}
                  </span>
                  <span className="text-[9px] font-mono px-1.5 py-0.5"
                    style={{ background: 'rgba(0,255,65,0.1)', color: '#00FF41', border: '1px solid rgba(0,255,65,0.3)' }}>
                    {s.status}
                  </span>
                </div>
                <div className="flex gap-4">
                  <div>
                    <div className="text-[8px] font-mono text-disabled uppercase tracking-wider">GRID</div>
                    <div className="text-[11px] font-mono font-bold" style={{ color: '#00F0FF' }}>{s.gridLabel}</div>
                  </div>
                  <div>
                    <div className="text-[8px] font-mono text-disabled uppercase tracking-wider">CONF</div>
                    <div className="text-[11px] font-mono font-bold" style={{ color: confColor }}>
                      {s.confidence.toFixed(1)}%
                    </div>
                  </div>
                </div>
                <div className="mt-1 text-[8px] font-mono text-disabled">
                  X:{s.position.x !== null ? s.position.x.toFixed(2) : '--'} Y:{s.position.y !== null ? s.position.y.toFixed(2) : '--'} Z:{s.position.z !== null ? s.position.z.toFixed(2) : '--'}
                </div>
                <div className="mt-1.5 h-1 w-full rounded-sm overflow-hidden" style={{ background: '#27272A' }}>
                  <div className="h-full" style={{ width: `${s.confidence}%`, background: confColor }} />
                </div>
              </div>
            );
          })
        )}
      </div>
    </div>
  );
}

// ─── Real Telemetry Panel ─────────────────────────────────────

function HardwareTelemetryPanel() {
  const hwState = useHardwareStore(s => s.hwConnectionState);
  const isConnected = hwState === 'CONNECTED';

  const px = useHardwareStore(s => s.drone.position.x);
  const py = useHardwareStore(s => s.drone.position.y);
  const pz = useHardwareStore(s => s.drone.position.z);
  const vx = useHardwareStore(s => s.drone.velocity.x);
  const vy = useHardwareStore(s => s.drone.velocity.y);
  const vz = useHardwareStore(s => s.drone.velocity.z);
  const roll  = useHardwareStore(s => s.drone.attitude.roll);
  const pitch = useHardwareStore(s => s.drone.attitude.pitch);
  const yaw   = useHardwareStore(s => s.drone.attitude.yaw);
  const batPct = useHardwareStore(s => s.drone.battery.percentage);
  const batV   = useHardwareStore(s => s.drone.battery.voltage);
  const batA   = useHardwareStore(s => s.drone.battery.current);
  const mState  = useHardwareStore(s => s.missionState);
  const phases  = useHardwareStore(s => s.missionPhases);
  const timer   = useHardwareStore(s => s.missionTimer);
  const health  = useHardwareStore(s => s.health);
  const autonomy = useHardwareStore(s => s.autonomy);

  const mins = Math.floor(timer / 60).toString().padStart(2, '0');
  const secs = (timer % 60).toString().padStart(2, '0');

  // Formatters with strict disconnected fallback
  const fmtPos = (v: number | null) => (isConnected && v !== null ? (v >= 0 ? `+${v.toFixed(2)}` : v.toFixed(2)) : '--');
  const fmtAtt = (v: number | null) => (isConnected && v !== null ? (v * 180 / Math.PI).toFixed(1) : '--');
  const fmtVel = (v: number | null) => (isConnected && v !== null ? v.toFixed(2) : '--');

  return (
    <div className="flex flex-col h-full overflow-y-auto" style={{ scrollbarWidth: 'thin' }}>
      {/* Mission State */}
      <div style={{ borderBottom: '1px solid #27272A' }}>
        <PanelHeader title="MISSION" accent="#FFB000" />
        <div className="px-3 py-2">
          <TelemetryValue label="STATE"   value={isConnected ? mState : '--'} color="#FFB000" />
          <TelemetryValue label="ELAPSED" value={isConnected ? `${mins}:${secs}` : '--:--'} color="#00F0FF" />
        </div>
        <div className="px-3 pb-3">
          <MissionStateTracker phases={phases} current={isConnected ? mState : 'IDLE'} />
        </div>
      </div>

      {/* Position (Local XYZ) */}
      <div style={{ borderBottom: '1px solid #27272A' }}>
        <PanelHeader title="POSITION  (LOCAL XYZ)" accent="#00F0FF" />
        <div className="px-3 py-2">
          <TelemetryValue label="X" value={fmtPos(px)} unit={isConnected && px !== null ? 'm' : undefined} color="#00F0FF" />
          <TelemetryValue label="Y" value={fmtPos(py)} unit={isConnected && py !== null ? 'm' : undefined} color="#00F0FF" />
          <TelemetryValue label="Z" value={fmtPos(pz)} unit={isConnected && pz !== null ? 'm' : undefined} color="#00F0FF" />
        </div>
      </div>

      {/* Attitude */}
      <div style={{ borderBottom: '1px solid #27272A' }}>
        <PanelHeader title="ATTITUDE" />
        <div className="px-3 py-2">
          <TelemetryValue label="ROLL"  value={fmtAtt(roll)}  unit={isConnected && roll !== null ? '°' : undefined} />
          <TelemetryValue label="PITCH" value={fmtAtt(pitch)} unit={isConnected && pitch !== null ? '°' : undefined} />
          <TelemetryValue label="YAW"   value={fmtAtt(yaw)}   unit={isConnected && yaw !== null ? '°' : undefined} />
        </div>
      </div>

      {/* Velocity */}
      <div style={{ borderBottom: '1px solid #27272A' }}>
        <PanelHeader title="VELOCITY" />
        <div className="px-3 py-2">
          <TelemetryValue label="VX" value={fmtVel(vx)} unit={isConnected && vx !== null ? 'm/s' : undefined} />
          <TelemetryValue label="VY" value={fmtVel(vy)} unit={isConnected && vy !== null ? 'm/s' : undefined} />
          <TelemetryValue label="VZ" value={fmtVel(vz)} unit={isConnected && vz !== null ? 'm/s' : undefined} />
        </div>
      </div>

      {/* Battery */}
      <div style={{ borderBottom: '1px solid #27272A' }}>
        <div className="px-3 py-2">
          <BatteryBar
            percentage={isConnected ? batPct : null}
            voltage={isConnected ? batV : null}
            current={isConnected ? batA : null}
          />
        </div>
      </div>

      {/* Autonomy */}
      <div style={{ borderBottom: '1px solid #27272A' }}>
        <PanelHeader title="AUTONOMY" accent="#FFB000" />
        <div className="px-3 py-2">
          <TelemetryValue label="PLANNER" value={isConnected ? autonomy.planner : '--'} color="#A1A1AA" />
          <TelemetryValue label="LOCALIZ" value={isConnected ? autonomy.localization : '--'} color="#A1A1AA" />
          <TelemetryValue label="TARGET"  value={isConnected ? autonomy.currentTarget : '--'} color="#FFB000" />
          <TelemetryValue label="DIST"    value={isConnected ? `${autonomy.distanceToTarget.toFixed(1)} m` : '--'} />
        </div>
        <div className="px-3 pb-2">
          <p className="text-[9px] font-mono text-disabled leading-snug">
            {isConnected ? autonomy.reason : 'Awaiting real drone connection'}
          </p>
        </div>
      </div>

      {/* Subsystem Health */}
      <div>
        <PanelHeader title="SYSTEM HEALTH" />
        <div className="px-3 py-1">
          {Object.values(health).map(sub => (
            <SystemHealthRow key={sub.name} subsystem={isConnected ? sub : { ...sub, status: 'OFFLINE' }} />
          ))}
        </div>
      </div>
    </div>
  );
}

interface HardwareTopBarProps {
  airMouseActive: boolean;
  onToggleAirMouse: () => void;
  onResetLayout?: () => void;
}

function HardwareTopBar({ airMouseActive, onToggleAirMouse, onResetLayout }: HardwareTopBarProps) {
  const navigate = useNavigate();
  const hwState = useHardwareStore(s => s.hwConnectionState);
  const isConnected = hwState === 'CONNECTED';

  const mode   = useHardwareStore(s => s.drone.mode);
  const mState = useHardwareStore(s => s.missionState);
  const timer  = useHardwareStore(s => s.missionTimer);
  const batPct = useHardwareStore(s => s.drone.battery.percentage);

  const mins = Math.floor(timer / 60).toString().padStart(2, '0');
  const secs = (timer % 60).toString().padStart(2, '0');

  // Status Badge Label and Color
  const connLabel =
    hwState === 'CONNECTED'   ? 'DRONE CONNECTED' :
    hwState === 'CONNECTING'  ? 'CONNECTING...' :
    hwState === 'DEGRADED'    ? 'LINK DEGRADED' :
    hwState === 'ERROR'       ? 'CONNECTION FAILED' :
    'DRONE DISCONNECTED';

  const connColor =
    hwState === 'CONNECTED'  ? '#00FF41' :
    hwState === 'CONNECTING' ? '#FFB000' :
    hwState === 'DEGRADED'   ? '#FF5500' :
    '#FF003C';

  const connDot =
    hwState === 'CONNECTED'  ? 'online' :
    hwState === 'CONNECTING' ? 'active' :
    hwState === 'DEGRADED'   ? 'degraded' :
    'offline';

  const setConnectModalOpen = useHardwareStore(s => s.setConnectModalOpen);

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
        <Radio size={12} color="#FFB000" />
        <span className="text-[11px] font-mono tracking-[0.15em] uppercase font-semibold" style={{ color: '#FFB000' }}>
          NIDAR AIR MOUSE
        </span>
        <span className="text-[9px] font-mono px-1.5 py-0.2 rounded" style={{ background: '#27272A', color: '#A1A1AA' }}>
          HARDWARE
        </span>
      </div>

      <div className="flex items-center flex-1">
        <TopBarBadge label="DRONE" value={connLabel} color={connColor} dot dotStatus={connDot} />
        <TopBarBadge label="MODE"  value={isConnected ? mode : '--'} color={isConnected ? '#00F0FF' : '#52525B'} />
        <TopBarBadge label="MISSION" value={isConnected ? mState : '--'}
          color={!isConnected ? '#52525B' :
                 mState === 'EXPLORE' || mState === 'SURVIVOR_DETECTED' ? '#FFB000' :
                 mState === 'COMPLETE' ? '#00FF41' : mState === 'ABORT' ? '#FF003C' : '#A1A1AA'}
          dot dotStatus={!isConnected ? 'offline' : mState === 'IDLE' ? 'idle' : mState === 'ABORT' ? 'offline' : 'active'} />
        <TopBarBadge label="ELAPSED" value={isConnected ? `${mins}:${secs}` : '--:--'} color={isConnected ? '#00F0FF' : '#52525B'} />
        <TopBarBadge label="BATTERY" value={isConnected && batPct !== null ? `${batPct.toFixed(0)}%` : '--'}
          color={!isConnected || batPct === null ? '#52525B' : batPct > 50 ? '#00FF41' : batPct > 20 ? '#FF5500' : '#FF003C'} />

        <div className="flex items-center gap-1.5 px-3 py-1.5 border-l" style={{ borderColor: '#27272A' }}>
          <Activity size={9} color={isConnected ? '#FF5500' : '#52525B'} />
          <span className="text-[9px] font-mono tracking-widest uppercase" style={{ color: isConnected ? '#FF5500' : '#52525B' }}>
            GPS DENIED
          </span>
        </div>

        {/* AirMouse Master Switch in TopBar */}
        <div className="flex items-center pl-3 border-l border-zinc-800">
          <button
            onClick={onToggleAirMouse}
            className={`flex items-center gap-1.5 px-2.5 py-1 text-[9px] font-mono font-bold tracking-wider uppercase border transition-colors ${
              airMouseActive
                ? 'border-cyan-400 bg-cyan-950/40 text-cyan-300 shadow-[0_0_10px_rgba(34,211,238,0.3)]'
                : 'border-zinc-700 text-zinc-400 hover:text-zinc-200'
            }`}
          >
            <MousePointer size={10} className={airMouseActive ? 'animate-pulse' : ''} />
            AIRMOUSE: {airMouseActive ? 'ACTIVE' : 'STANDBY'}
          </button>
        </div>

        {onResetLayout && (
          <button
            onClick={onResetLayout}
            title="Reset panel sizes to defaults"
            className="flex items-center gap-1.5 px-2.5 py-1 text-[9px] font-mono tracking-wider uppercase border border-zinc-700 hover:border-zinc-500 text-zinc-400 hover:text-zinc-200 hover:bg-zinc-800/40 transition-colors ml-auto mr-2"
          >
            <RotateCcw size={10} />
            RESET LAYOUT
          </button>
        )}

        <button
          onClick={() => setConnectModalOpen(true)}
          className={`flex items-center gap-1.5 px-2.5 py-1 text-[9px] font-mono tracking-wider uppercase border border-[#FFB000]/60 text-[#FFB000] hover:bg-[#FFB000]/15 transition-colors mr-3 ${!onResetLayout ? 'ml-auto' : ''}`}
        >
          <Wifi size={10} />
          {isConnected ? 'CONNECTION SETTINGS' : 'CONNECT NEW DRONE'}
        </button>
      </div>
    </div>
  );
}

// ─── Hardware Dashboard ────────────────────────────────────────

export default function HardwareDashboard() {
  const hwState     = useHardwareStore(s => s.hwConnectionState);
  const isConnectModalOpen = useHardwareStore(s => s.isConnectModalOpen);
  const setConnectModalOpen = useHardwareStore(s => s.setConnectModalOpen);

  const map         = useHardwareStore(s => s.map);
  const trajectory  = useHardwareStore(s => s.trajectory);
  const survivors   = useHardwareStore(s => s.survivors);
  const events      = useHardwareStore(s => s.events);
  const droneYaw    = useHardwareStore(s => s.drone.attitude.yaw);
  const droneX      = useHardwareStore(s => s.drone.position.x);
  const droneY      = useHardwareStore(s => s.drone.position.y);

  // AirMouse integration
  const airMouse = useAirMouse(false);
  const [rightPanelTab, setRightPanelTab] = useState<'airmouse' | 'telemetry'>('airmouse');

  // VS Code style resizable layout
  const {
    sidebarWidth,
    bottomHeight,
    timelineWidth,
    survivorsWidth,
    handleResizeSidebar,
    handleResizeBottom,
    handleResizeTimeline,
    handleResizeSurvivors,
    handleSaveSidebar,
    handleSaveBottom,
    handleSaveTimeline,
    handleSaveSurvivors,
    resetLayout,
    resetSidebar,
    resetBottom,
    resetTimeline,
    resetSurvivors,
  } = useResizableLayout({
    storageKeyPrefix: 'nidar_hw',
    defaultSidebarWidth: 490,
    minSidebarWidth: 320,
    maxSidebarWidth: 850,
    defaultBottomHeight: 210,
    minBottomHeight: 140,
    maxBottomHeight: 520,
    defaultTimelineWidth: 320,
    minTimelineWidth: 180,
    maxTimelineWidth: 550,
    defaultSurvivorsWidth: 220,
    minSurvivorsWidth: 160,
    maxSurvivorsWidth: 450,
  });

  const cameraStatus: CameraStatus =
    hwState === 'CONNECTED'  ? 'LIVE' :
    hwState === 'CONNECTING' ? 'CONNECTING' :
    hwState === 'ERROR'      ? 'ERROR' :
    'OFFLINE';

  return (
    <div className="flex flex-col h-screen overflow-hidden relative"
      style={{ background: '#09090B', fontFamily: 'IBM Plex Sans, sans-serif' }}>
      <HardwareTopBar
        airMouseActive={airMouse.isActive}
        onToggleAirMouse={() => (airMouse.isActive ? airMouse.stopAirMouse() : airMouse.startAirMouse())}
        onResetLayout={resetLayout}
      />

      <div className="flex flex-1 overflow-hidden select-none" style={{ minHeight: 0 }}>
        {/* LEFT: Map + Camera + Events */}
        <div className="flex flex-col flex-1 overflow-hidden" style={{ minWidth: 0 }}>
          {/* 2D Local Map Area */}
          <div className="flex-1 relative overflow-hidden"
            style={{ border: '1px solid #27272A', borderTop: 'none', borderLeft: 'none', minHeight: '120px' }}>
            <div className="absolute top-0 left-0 z-10 px-3 py-1.5 flex items-center gap-2"
              style={{ background: 'rgba(17,17,19,0.92)', borderBottom: '1px solid #27272A', borderRight: '1px solid #27272A' }}>
              <Cpu size={9} color="#FFB000" />
              <span className="text-[9px] font-mono tracking-[0.18em] uppercase text-muted">
                LOCAL 2D OCCUPANCY GRID — FAST-LIO2 — GPS-DENIED
              </span>
            </div>

            <OccupancyGridMap
              map={map}
              droneX={droneX}
              droneY={droneY}
              droneYaw={droneYaw}
              trajectory={trajectory}
              survivors={survivors}
              className="w-full h-full"
            />
          </div>

          {/* Draggable Horizontal Divider between Map and Camera */}
          <ResizeDivider
            direction="horizontal"
            onResize={handleResizeBottom}
            onResizeEnd={handleSaveBottom}
            onReset={resetBottom}
            accentColor="#FFB000"
            title="Drag vertically to resize Map / Camera (Double-click to reset)"
          />

          {/* Camera + Events */}
          <div className="flex flex-shrink-0 overflow-hidden"
            style={{ height: `${bottomHeight}px` }}>
            <div className="flex-1 flex flex-col overflow-hidden" style={{ minWidth: '160px' }}>
              <PanelHeader title="LIVE CAMERA — REAL STREAM" accent="#00F0FF" />
              <div className="flex-1 overflow-hidden">
                <CameraView mode="hardware" status={cameraStatus} className="w-full h-full" />
              </div>
            </div>

            {/* Draggable Vertical Divider between Live Camera and Event Timeline */}
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

        {/* Draggable Vertical Divider between Main Content and Right Sidebar */}
        <ResizeDivider
          direction="vertical"
          onResize={handleResizeSidebar}
          onResizeEnd={handleSaveSidebar}
          onReset={resetSidebar}
          accentColor="#FFB000"
          title="Drag horizontally to resize Right Sidebar (Double-click to reset)"
        />

        {/* RIGHT SIDEBAR: Survivors + AirMouse / Telemetry */}
        <div
          className="flex-shrink-0 flex overflow-hidden"
          style={{ width: `${sidebarWidth}px`, background: '#111113' }}
        >
          {/* CENTER-RIGHT: Survivors + Hardware Controls */}
          <div className="flex-shrink-0 flex flex-col overflow-hidden"
            style={{ width: `${survivorsWidth}px`, background: '#111113' }}>
            <div className="flex-1 overflow-hidden">
              <SurvivorPanel />
            </div>
            <div className="flex-shrink-0 p-3" style={{ borderTop: '1px solid #27272A' }}>
              <HardwareControls />
            </div>
          </div>

          {/* Draggable Vertical Divider between Survivors and AirMouse/Telemetry */}
          <ResizeDivider
            direction="vertical"
            onResize={handleResizeSurvivors}
            onResizeEnd={handleSaveSurvivors}
            onReset={resetSurvivors}
            accentColor="#FFB000"
            title="Drag horizontally to resize Survivors / AirMouse (Double-click to reset)"
          />

          {/* FAR RIGHT: AirMouse or Mission Telemetry */}
          <div className="flex-1 min-w-0 overflow-hidden flex flex-col"
            style={{ background: '#111113' }}>
            {/* Tab Switcher */}
            <div className="flex border-b border-zinc-800 bg-[#141518] text-[10px] font-mono font-bold tracking-wider uppercase flex-shrink-0">
              <button
                onClick={() => setRightPanelTab('airmouse')}
                className={`flex-1 py-2 flex items-center justify-center gap-1.5 border-b-2 transition-colors ${
                  rightPanelTab === 'airmouse'
                    ? 'border-[#FFB000] text-[#FFB000] bg-[#FFB000]/10'
                    : 'border-transparent text-zinc-400 hover:text-zinc-200'
                }`}
              >
                <MousePointer size={11} />
                AIRMOUSE
              </button>
              <button
                onClick={() => setRightPanelTab('telemetry')}
                className={`flex-1 py-2 flex items-center justify-center gap-1.5 border-b-2 transition-colors ${
                  rightPanelTab === 'telemetry'
                    ? 'border-[#FFB000] text-[#FFB000] bg-[#FFB000]/10'
                    : 'border-transparent text-zinc-400 hover:text-zinc-200'
                }`}
              >
                <Activity size={11} />
                TELEMETRY
              </button>
            </div>

            <div className="flex-1 overflow-hidden">
              {rightPanelTab === 'airmouse' ? (
                <AirMouseDashboard
                  isActive={airMouse.isActive}
                  onToggleActive={() => (airMouse.isActive ? airMouse.stopAirMouse() : airMouse.startAirMouse())}
                  connectionState={airMouse.connectionState}
                  connectionError={airMouse.connectionError}
                  settings={airMouse.settings}
                  onUpdateSettings={airMouse.updateSettings}
                  onCenterCursor={airMouse.centerCursor}
                  diagnostics={airMouse.diagnostics}
                  rawSensor={airMouse.rawSensor}
                />
              ) : (
                <HardwareTelemetryPanel />
              )}
            </div>
          </div>
        </div>
      </div>

      {/* Zero-lag Visual Virtual Cursor Overlay */}
      <AirMouseOverlay
        isActive={airMouse.isActive}
        subscribeCursor={airMouse.subscribeCursor}
      />

      <ConnectDroneModal
        isOpen={isConnectModalOpen}
        onClose={() => setConnectModalOpen(false)}
      />
    </div>
  );
}
