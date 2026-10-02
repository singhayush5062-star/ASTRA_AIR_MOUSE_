import { useState } from 'react';
import {
  MousePointer, Power, Sliders, Activity, Compass,
  ChevronDown, ChevronUp, Terminal, AlertTriangle,
} from 'lucide-react';
import type {
  AirMouseConnectionState,
  AirMouseSettings,
  AirMouseDiagnostics,
  RawSensorData,
} from '@/types/airMouse';

interface AirMouseDashboardProps {
  isActive: boolean;
  onToggleActive: () => void;
  connectionState: AirMouseConnectionState;
  connectionError: string | null;
  settings: AirMouseSettings;
  onUpdateSettings: (newSettings: Partial<AirMouseSettings>) => void;
  onCenterCursor: () => void;
  diagnostics: AirMouseDiagnostics;
  rawSensor: RawSensorData;
}

export function AirMouseDashboard({
  isActive,
  onToggleActive,
  connectionState,
  connectionError,
  settings,
  onUpdateSettings,
  onCenterCursor,
  diagnostics,
  rawSensor,
}: AirMouseDashboardProps) {
  const [showSettings, setShowSettings] = useState(false);
  const [debugMode, setDebugMode] = useState(false);

  // Status Indicator details
  const getStatusBadge = () => {
    switch (connectionState) {
      case 'CONNECTED':
        return {
          label: 'CONNECTED',
          dotColor: 'bg-emerald-400',
          textColor: 'text-emerald-400',
          borderColor: 'border-emerald-500/40',
        };
      case 'CONNECTING':
      case 'RECONNECTING':
        return {
          label: 'RECONNECTING',
          dotColor: 'bg-amber-400 animate-pulse',
          textColor: 'text-amber-400',
          borderColor: 'border-amber-500/40',
        };
      case 'STALE':
        return {
          label: 'STALE LINK',
          dotColor: 'bg-orange-500 animate-pulse',
          textColor: 'text-orange-400',
          borderColor: 'border-orange-500/40',
        };
      case 'ERROR':
        return {
          label: 'LINK ERROR',
          dotColor: 'bg-red-500',
          textColor: 'text-red-400',
          borderColor: 'border-red-500/40',
        };
      case 'DISCONNECTED':
      default:
        return {
          label: 'DISCONNECTED',
          dotColor: 'bg-zinc-500',
          textColor: 'text-zinc-400',
          borderColor: 'border-zinc-700',
        };
    }
  };

  const status = getStatusBadge();

  // Helper formatting for angles
  const fmtRadToDeg = (rad: number | null) =>
    rad !== null && !isNaN(rad) ? `${(rad * (180 / Math.PI)).toFixed(1)}°` : '--';
  const fmtNum = (num: number | null, unit = '') =>
    num !== null && !isNaN(num) ? `${num >= 0 ? `+${num.toFixed(2)}` : num.toFixed(2)}${unit}` : '--';

  return (
    <div className="flex flex-col h-full bg-[#111114] border-l border-zinc-800 text-[#E4E4E7] font-mono select-none overflow-y-auto">
      {/* Header Bar */}
      <div className="p-3.5 border-b border-zinc-800 flex items-center justify-between bg-[#15161A]">
        <div className="flex items-center gap-2">
          <MousePointer size={14} className="text-[#FFB000]" />
          <span className="text-xs font-black tracking-[0.18em] uppercase text-white">
            AIRMOUSE SYSTEM
          </span>
        </div>

        {/* Live Status Pill */}
        <div
          className={`flex items-center gap-1.5 px-2 py-0.5 border text-[10px] font-bold uppercase tracking-wider ${status.borderColor} ${status.textColor} bg-black/40`}
        >
          <span className={`w-2 h-2 rounded-full ${status.dotColor}`} />
          <span>{status.label}</span>
        </div>
      </div>

      <div className="p-3.5 space-y-4">
        {connectionError && (
          <div className="p-2 border border-red-500/40 bg-red-950/30 text-red-300 text-[10px] flex items-center gap-1.5">
            <AlertTriangle size={12} className="text-red-400 shrink-0" />
            <span>{connectionError}</span>
          </div>
        )}

        {/* START / STOP AIR MOUSE Master Switch */}
        <div className="space-y-1.5">
          <button
            onClick={onToggleActive}
            className={`w-full py-2.5 flex items-center justify-center gap-2 text-xs font-mono font-bold tracking-[0.2em] uppercase transition-all duration-200 border ${
              isActive
                ? 'bg-gradient-to-r from-red-600/30 to-red-500/20 border-red-500 text-red-400 shadow-[0_0_15px_rgba(239,68,68,0.25)] hover:bg-red-600/40'
                : 'bg-gradient-to-r from-[#FFB000]/20 to-[#FF8C00]/30 border-[#FFB000] text-[#FFB000] shadow-[0_0_15px_rgba(255,176,0,0.2)] hover:bg-[#FFB000]/30'
            }`}
          >
            <Power size={14} />
            {isActive ? 'STOP AIR MOUSE' : 'START AIR MOUSE'}
          </button>

          <div className="flex items-center justify-between text-[9px] text-zinc-500 px-1">
            <span>Mode: {isActive ? 'ACTIVE (SENSOR STREAMING)' : 'STANDBY'}</span>
            <button
              onClick={onCenterCursor}
              className="text-[#FFB000] hover:underline flex items-center gap-1"
            >
              <Compass size={10} /> Center Cursor
            </button>
          </div>
        </div>

        {/* Live Gesture & Motion State */}
        <div className="p-3 border border-zinc-800 bg-[#16171C]">
          <div className="text-[10px] font-bold text-zinc-400 tracking-wider uppercase mb-2 flex items-center justify-between">
            <span>GESTURE RECOGNITION</span>
            <span className="text-cyan-400 font-bold">{diagnostics.currentGesture}</span>
          </div>

          <div className="grid grid-cols-3 gap-1.5 text-center text-[10px]">
            <div
              className={`p-1.5 border transition-colors ${
                diagnostics.currentGesture === 'MOVE'
                  ? 'border-[#FFB000] bg-[#FFB000]/15 text-[#FFB000]'
                  : 'border-zinc-800 bg-[#1D1E24] text-zinc-400'
              }`}
            >
              MOVE
            </div>
            <div
              className={`p-1.5 border transition-colors ${
                diagnostics.currentGesture === 'LEFT_CLICK'
                  ? 'border-cyan-400 bg-cyan-400/20 text-cyan-300 shadow-[0_0_8px_rgba(34,211,238,0.4)]'
                  : 'border-zinc-800 bg-[#1D1E24] text-zinc-400'
              }`}
            >
              L-CLICK
            </div>
            <div
              className={`p-1.5 border transition-colors ${
                diagnostics.currentGesture.includes('SCROLL')
                  ? 'border-amber-400 bg-amber-400/20 text-amber-300 shadow-[0_0_8px_rgba(251,191,36,0.4)]'
                  : 'border-zinc-800 bg-[#1D1E24] text-zinc-400'
              }`}
            >
              SCROLL
            </div>
          </div>
        </div>

        {/* Real-Time Sensor Readout */}
        <div className="p-3 border border-zinc-800 bg-[#16171C] space-y-2.5">
          <div className="text-[10px] font-bold text-zinc-400 tracking-wider uppercase flex items-center justify-between">
            <span>LIVE IMU & SENSOR DATA</span>
            <Activity size={12} className="text-[#FFB000]" />
          </div>

          {/* Orientation: Roll, Pitch, Yaw */}
          <div className="grid grid-cols-3 gap-2">
            <div className="p-2 border border-zinc-800/80 bg-[#1A1B20]">
              <div className="text-[9px] text-zinc-500">PITCH (Y)</div>
              <div className="text-xs font-bold text-cyan-400">{fmtRadToDeg(rawSensor.pitch)}</div>
            </div>
            <div className="p-2 border border-zinc-800/80 bg-[#1A1B20]">
              <div className="text-[9px] text-zinc-500">YAW (X)</div>
              <div className="text-xs font-bold text-cyan-400">{fmtRadToDeg(rawSensor.yaw)}</div>
            </div>
            <div className="p-2 border border-zinc-800/80 bg-[#1A1B20]">
              <div className="text-[9px] text-zinc-500">ROLL</div>
              <div className="text-xs font-bold text-cyan-400">{fmtRadToDeg(rawSensor.roll)}</div>
            </div>
          </div>

          {/* Linear Velocity: Vx, Vy, Vz */}
          <div className="grid grid-cols-3 gap-2">
            <div className="p-2 border border-zinc-800/80 bg-[#1A1B20]">
              <div className="text-[9px] text-zinc-500">VX</div>
              <div className="text-xs font-bold text-[#FFB000]">{fmtNum(rawSensor.vx, ' m/s')}</div>
            </div>
            <div className="p-2 border border-zinc-800/80 bg-[#1A1B20]">
              <div className="text-[9px] text-zinc-500">VY</div>
              <div className="text-xs font-bold text-[#FFB000]">{fmtNum(rawSensor.vy, ' m/s')}</div>
            </div>
            <div className="p-2 border border-zinc-800/80 bg-[#1A1B20]">
              <div className="text-[9px] text-zinc-500">VZ</div>
              <div className="text-xs font-bold text-[#FFB000]">{fmtNum(rawSensor.vz, ' m/s')}</div>
            </div>
          </div>
        </div>

        {/* Network & Performance Metrics */}
        <div className="p-3 border border-zinc-800 bg-[#16171C]">
          <div className="text-[10px] font-bold text-zinc-400 tracking-wider uppercase mb-2">
            STREAM TELEMETRY STATS
          </div>

          <div className="grid grid-cols-2 gap-2 text-[10px]">
            <div className="flex justify-between border-b border-zinc-800/60 pb-1">
              <span className="text-zinc-500">Packets Recv:</span>
              <span className="text-white font-bold">{diagnostics.packetsReceived}</span>
            </div>
            <div className="flex justify-between border-b border-zinc-800/60 pb-1">
              <span className="text-zinc-500">Rate:</span>
              <span className="text-emerald-400 font-bold">{diagnostics.packetsPerSecond} pkt/s</span>
            </div>
            <div className="flex justify-between border-b border-zinc-800/60 pb-1">
              <span className="text-zinc-500">Latency:</span>
              <span className="text-cyan-400 font-bold">{diagnostics.latencyMs} ms</span>
            </div>
            <div className="flex justify-between border-b border-zinc-800/60 pb-1">
              <span className="text-zinc-500">Cursor (X,Y):</span>
              <span className="text-zinc-300 font-bold">
                {diagnostics.cursorX}, {diagnostics.cursorY}
              </span>
            </div>
          </div>
        </div>

        {/* Settings Collapsible */}
        <div className="border border-zinc-800 bg-[#16171C]">
          <button
            onClick={() => setShowSettings(!showSettings)}
            className="w-full p-2.5 flex items-center justify-between text-[10px] font-bold text-zinc-300 uppercase tracking-wider hover:bg-zinc-800/50 transition-colors"
          >
            <div className="flex items-center gap-1.5">
              <Sliders size={12} className="text-[#FFB000]" />
              <span>AIRMOUSE TUNING</span>
            </div>
            {showSettings ? <ChevronUp size={14} /> : <ChevronDown size={14} />}
          </button>

          {showSettings && (
            <div className="p-3 border-t border-zinc-800 space-y-3 text-[10px] animate-in fade-in duration-150">
              {/* Sensitivity */}
              <div>
                <div className="flex justify-between mb-1">
                  <span className="text-zinc-400">Sensitivity</span>
                  <span className="text-[#FFB000] font-bold">{settings.sensitivity}x</span>
                </div>
                <input
                  type="range"
                  min="1"
                  max="20"
                  step="0.5"
                  value={settings.sensitivity}
                  onChange={e => onUpdateSettings({ sensitivity: parseFloat(e.target.value) })}
                  className="w-full accent-[#FFB000] cursor-pointer"
                />
              </div>

              {/* Smoothing */}
              <div>
                <div className="flex justify-between mb-1">
                  <span className="text-zinc-400">Jitter Smoothing</span>
                  <span className="text-[#FFB000] font-bold">{Math.round(settings.smoothing * 100)}%</span>
                </div>
                <input
                  type="range"
                  min="0.05"
                  max="0.95"
                  step="0.05"
                  value={settings.smoothing}
                  onChange={e => onUpdateSettings({ smoothing: parseFloat(e.target.value) })}
                  className="w-full accent-[#FFB000] cursor-pointer"
                />
              </div>

              {/* Deadzone */}
              <div>
                <div className="flex justify-between mb-1">
                  <span className="text-zinc-400">Deadzone</span>
                  <span className="text-[#FFB000] font-bold">{(settings.deadzone * 1000).toFixed(1)} mrad</span>
                </div>
                <input
                  type="range"
                  min="0.001"
                  max="0.03"
                  step="0.001"
                  value={settings.deadzone}
                  onChange={e => onUpdateSettings({ deadzone: parseFloat(e.target.value) })}
                  className="w-full accent-[#FFB000] cursor-pointer"
                />
              </div>

              {/* Inversion Toggles */}
              <div className="flex items-center justify-between pt-1">
                <label className="flex items-center gap-1.5 cursor-pointer text-zinc-300">
                  <input
                    type="checkbox"
                    checked={settings.invertX}
                    onChange={e => onUpdateSettings({ invertX: e.target.checked })}
                    className="accent-[#FFB000]"
                  />
                  <span>Invert X</span>
                </label>

                <label className="flex items-center gap-1.5 cursor-pointer text-zinc-300">
                  <input
                    type="checkbox"
                    checked={settings.invertY}
                    onChange={e => onUpdateSettings({ invertY: e.target.checked })}
                    className="accent-[#FFB000]"
                  />
                  <span>Invert Y</span>
                </label>

                <label className="flex items-center gap-1.5 cursor-pointer text-zinc-300">
                  <input
                    type="checkbox"
                    checked={settings.gesturesEnabled}
                    onChange={e => onUpdateSettings({ gesturesEnabled: e.target.checked })}
                    className="accent-[#FFB000]"
                  />
                  <span>Gestures</span>
                </label>
              </div>
            </div>
          )}
        </div>

        {/* Debug Mode Toggle */}
        <div className="pt-1">
          <button
            onClick={() => setDebugMode(!debugMode)}
            className="flex items-center gap-1.5 text-[9px] text-zinc-500 hover:text-zinc-300 tracking-wider uppercase"
          >
            <Terminal size={11} />
            <span>{debugMode ? 'Hide Debug Packet' : 'Inspect Raw Packet'}</span>
          </button>

          {debugMode && (
            <pre className="mt-2 p-2 bg-black/90 border border-zinc-800 text-[9px] text-emerald-400 font-mono overflow-x-auto max-h-32">
              {JSON.stringify(rawSensor, null, 2)}
            </pre>
          )}
        </div>
      </div>
    </div>
  );
}
