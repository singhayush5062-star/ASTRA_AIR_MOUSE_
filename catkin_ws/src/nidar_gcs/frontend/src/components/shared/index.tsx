import type { SubsystemHealth, SubsystemStatus } from '@/types';

// ─── Status color map ─────────────────────────────────────────

export function statusColor(status: SubsystemStatus): string {
  switch (status) {
    case 'ONLINE':
    case 'CONNECTED':
    case 'TRACKING':
    case 'RUNNING':
    case 'READY':
      return '#00FF41';
    case 'DEGRADED':
      return '#FF5500';
    case 'OFFLINE':
    case 'UNKNOWN':
      return '#FF003C';
    default:
      return '#A1A1AA';
  }
}

export function statusDotClass(status: SubsystemStatus): string {
  switch (status) {
    case 'ONLINE':
    case 'CONNECTED':
    case 'TRACKING':
    case 'RUNNING':
    case 'READY':
      return 'online';
    case 'DEGRADED':
      return 'degraded';
    case 'OFFLINE':
    case 'UNKNOWN':
      return 'offline';
    default:
      return 'idle';
  }
}

// ─── SystemHealthRow ──────────────────────────────────────────

interface SystemHealthRowProps {
  subsystem: SubsystemHealth;
}

export function SystemHealthRow({ subsystem }: SystemHealthRowProps) {
  const color = statusColor(subsystem.status);
  return (
    <div className="flex items-center justify-between py-1.5 border-b last:border-b-0" style={{ borderColor: '#27272A' }}>
      <span className="text-[10px] font-mono tracking-widest uppercase text-muted">{subsystem.name}</span>
      <div className="flex items-center gap-2">
        <span className={`status-dot ${statusDotClass(subsystem.status)}`} />
        <span className="text-[10px] font-mono tracking-wider" style={{ color }}>
          {subsystem.status}
        </span>
      </div>
    </div>
  );
}

// ─── TelemetryValue ────────────────────────────────────────────

interface TelemetryValueProps {
  label: string;
  value: string | number;
  unit?: string;
  color?: string;
  highlighted?: boolean;
}

export function TelemetryValue({ label, value, unit, color = '#E4E4E7', highlighted }: TelemetryValueProps) {
  return (
    <div
      className="flex items-center justify-between py-1"
      style={highlighted ? { background: 'rgba(0,240,255,0.04)', padding: '3px 6px', margin: '0 -6px' } : undefined}
    >
      <span className="text-[10px] font-mono tracking-widest text-muted uppercase">{label}</span>
      <span className="text-[11px] font-mono font-medium" style={{ color }}>
        {value}{unit && <span className="text-muted ml-1 text-[9px]">{unit}</span>}
      </span>
    </div>
  );
}

// ─── PanelHeader ──────────────────────────────────────────────

interface PanelHeaderProps {
  title: string;
  accent?: string;
  right?: React.ReactNode;
}

export function PanelHeader({ title, accent, right }: PanelHeaderProps) {
  return (
    <div
      className="flex items-center justify-between px-3 py-2 border-b"
      style={{ borderColor: '#27272A', background: '#111113' }}
    >
      <span
        className="text-[9px] font-mono tracking-[0.2em] uppercase font-semibold"
        style={{ color: accent || '#A1A1AA' }}
      >
        {title}
      </span>
      {right}
    </div>
  );
}

// ─── MissionStateTracker ──────────────────────────────────────

import type { MissionPhase, MissionState } from '@/types';

interface MissionStateTrackerProps {
  phases: MissionPhase[];
  current: MissionState;
}

const STATE_LABELS: Record<string, string> = {
  INIT: 'INIT',
  TAKEOFF: 'TAKEOFF',
  LOCALIZATION: 'LOCALIZE',
  EXPLORE: 'EXPLORE',
  SURVIVOR_DETECTED: 'SURVIVOR',
  CONTINUE_EXPLORE: 'CONTINUE',
  RETURN: 'RETURN',
  LAND: 'LAND',
};

export function MissionStateTracker({ phases, current: _current }: MissionStateTrackerProps) {
  return (
    <div className="flex flex-col gap-0.5">
      {phases.map((phase) => {
        const isActive = phase.status === 'ACTIVE';
        const isDone = phase.status === 'COMPLETE';
        const isFailed = phase.status === 'FAILED';
        const color = isDone ? '#00FF41' : isActive ? '#FFB000' : isFailed ? '#FF003C' : '#52525B';
        const symbol = isDone ? '✓' : isActive ? '●' : isFailed ? '✗' : '○';

        return (
          <div key={phase.state} className="flex items-center gap-2 py-0.5">
            <span
              className="text-[10px] font-mono w-3 text-center"
              style={{ color, ...(isActive ? { textShadow: `0 0 6px ${color}` } : {}) }}
            >
              {symbol}
            </span>
            <span
              className="text-[10px] font-mono tracking-wider uppercase"
              style={{ color: isActive ? '#FFB000' : isDone ? '#A1A1AA' : '#52525B' }}
            >
              {STATE_LABELS[phase.state] || phase.state}
            </span>
          </div>
        );
      })}
    </div>
  );
}

// ─── EventTimeline ─────────────────────────────────────────────

import type { MissionEvent } from '@/types';

interface EventTimelineProps {
  events: MissionEvent[];
  maxHeight?: string;
}

const levelColor = (level: MissionEvent['level']) => {
  switch (level) {
    case 'SUCCESS': return '#00FF41';
    case 'WARN':    return '#FF5500';
    case 'ERROR':   return '#FF003C';
    default:        return '#A1A1AA';
  }
};

export function EventTimeline({ events, maxHeight = '200px' }: EventTimelineProps) {
  return (
    <div
      className="overflow-y-auto"
      style={{ maxHeight, scrollbarWidth: 'thin' }}
    >
      {events.length === 0 ? (
        <div className="text-[10px] font-mono text-disabled text-center py-4 tracking-wider">
          AWAITING EVENTS
        </div>
      ) : (
        <div className="flex flex-col">
          {events.map(evt => {
            const d = new Date(evt.timestamp);
            const ts = `${String(d.getHours()).padStart(2,'0')}:${String(d.getMinutes()).padStart(2,'0')}:${String(d.getSeconds()).padStart(2,'0')}`;
            return (
              <div key={evt.id} className="flex gap-3 py-1.5 border-b last:border-b-0" style={{ borderColor: '#1c1c1e' }}>
                <span className="text-[9px] font-mono text-disabled flex-shrink-0 pt-0.5">{ts}</span>
                <span className="text-[10px] font-mono leading-snug" style={{ color: levelColor(evt.level) }}>
                  {evt.message}
                </span>
              </div>
            );
          })}
        </div>
      )}
    </div>
  );
}

// ─── BatteryBar ───────────────────────────────────────────────

interface BatteryBarProps {
  percentage: number | null;
  voltage: number | null;
  current?: number | null;
}

export function BatteryBar({ percentage, voltage, current }: BatteryBarProps) {
  const isAvailable = percentage !== null && !isNaN(percentage);
  const color = !isAvailable ? '#52525B' : percentage > 50 ? '#00FF41' : percentage > 20 ? '#FF5500' : '#FF003C';
  const pctStr = isAvailable ? `${percentage.toFixed(0)}%` : '--';
  const voltStr = voltage !== null && !isNaN(voltage) ? `${voltage.toFixed(1)} V` : '-- V';
  const currStr = current !== null && current !== undefined && !isNaN(current) ? `${current.toFixed(1)} A` : '-- A';

  return (
    <div className="flex flex-col gap-1">
      <div className="flex items-center justify-between">
        <span className="text-[10px] font-mono tracking-widest text-muted uppercase">BATTERY</span>
        <span className="text-[11px] font-mono font-bold" style={{ color }}>
          {pctStr}
        </span>
      </div>
      <div className="h-1.5 w-full rounded-sm overflow-hidden" style={{ background: '#27272A' }}>
        <div
          className="h-full transition-all duration-1000"
          style={{ width: `${isAvailable ? Math.max(0, Math.min(100, percentage)) : 0}%`, background: color }}
        />
      </div>
      <div className="flex gap-3">
        <span className="text-[9px] font-mono text-disabled">{voltStr}</span>
        <span className="text-[9px] font-mono text-disabled">{currStr}</span>
      </div>
    </div>
  );
}

// ─── TopBarBadge ──────────────────────────────────────────────

interface TopBarBadgeProps {
  label: string;
  value: string;
  color?: string;
  dot?: boolean;
  dotStatus?: 'online' | 'offline' | 'active' | 'degraded' | 'idle';
}

export function TopBarBadge({ label, value, color, dot, dotStatus }: TopBarBadgeProps) {
  return (
    <div
      className="flex items-center gap-2 px-3 py-1.5 border-l"
      style={{ borderColor: '#27272A' }}
    >
      {dot && <span className={`status-dot ${dotStatus || 'idle'}`} />}
      <div className="flex flex-col">
        <span className="text-[8px] font-mono tracking-[0.2em] text-disabled uppercase">{label}</span>
        <span
          className="text-[10px] font-mono tracking-wider font-medium uppercase"
          style={{ color: color || '#E4E4E7' }}
        >
          {value}
        </span>
      </div>
    </div>
  );
}
