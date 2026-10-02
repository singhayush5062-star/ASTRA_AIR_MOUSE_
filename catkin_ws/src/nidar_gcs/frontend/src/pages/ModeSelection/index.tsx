import { useNavigate } from 'react-router-dom';
import { Monitor, Radio, Cpu, Wifi, ChevronRight, Activity } from 'lucide-react';

// ─── Status badge ─────────────────────────────────────────────

function StatusBadge({ ok, label }: { ok: boolean; label: string }) {
  return (
    <div className={`flex items-center gap-1.5 text-[10px] font-mono tracking-wider uppercase ${ok ? 'text-green' : 'text-muted'}`}>
      <span className={`status-dot ${ok ? 'online' : 'idle'}`} />
      {label}
    </div>
  );
}

// ─── Environment Card ─────────────────────────────────────────

interface EnvCardProps {
  title: string;
  subtitle: string;
  description: string;
  tags: string[];
  statusItems: { ok: boolean; label: string }[];
  buttonLabel: string;
  buttonColor: 'cyan' | 'amber';
  icon: React.ReactNode;
  route: string;
  accentColor: string;
  borderColor: string;
}

function EnvironmentCard({
  title, subtitle, description, tags, statusItems,
  buttonLabel, buttonColor, icon, route, accentColor, borderColor,
}: EnvCardProps) {
  const navigate = useNavigate();
  const isCyan = buttonColor === 'cyan';

  return (
    <div
      className="relative flex flex-col h-full cursor-pointer group"
      style={{ border: `1px solid ${borderColor}`, background: '#111113' }}
      onClick={() => navigate(route)}
    >
      {/* Corner accent */}
      <div
        className="absolute top-0 left-0 w-8 h-8"
        style={{
          borderTop: `2px solid ${accentColor}`,
          borderLeft: `2px solid ${accentColor}`,
          opacity: 0.7,
        }}
      />
      <div
        className="absolute bottom-0 right-0 w-8 h-8"
        style={{
          borderBottom: `2px solid ${accentColor}`,
          borderRight: `2px solid ${accentColor}`,
          opacity: 0.7,
        }}
      />

      {/* Hover glow overlay */}
      <div
        className="absolute inset-0 opacity-0 group-hover:opacity-100 transition-opacity duration-300 pointer-events-none"
        style={{ background: `radial-gradient(ellipse at 50% 0%, ${accentColor}08 0%, transparent 70%)` }}
      />

      {/* Content */}
      <div className="flex flex-col flex-1 p-8 gap-6">
        {/* Header */}
        <div className="flex items-start justify-between">
          <div
            className="p-3 rounded"
            style={{ background: `${accentColor}15`, border: `1px solid ${accentColor}30` }}
          >
            {icon}
          </div>
          <div className="flex flex-col gap-1.5 items-end">
            {statusItems.map((s, i) => <StatusBadge key={i} ok={s.ok} label={s.label} />)}
          </div>
        </div>

        {/* Title */}
        <div>
          <p className="text-[10px] font-mono tracking-[0.2em] uppercase mb-1" style={{ color: accentColor }}>
            {subtitle}
          </p>
          <h2 className="text-3xl font-bold tracking-tight text-text uppercase" style={{ fontFamily: 'IBM Plex Sans, sans-serif' }}>
            {title}
          </h2>
          <p className="text-sm text-muted mt-2 leading-relaxed" style={{ fontFamily: 'IBM Plex Sans, sans-serif' }}>
            {description}
          </p>
        </div>

        {/* Tags */}
        <div className="flex flex-wrap gap-2">
          {tags.map(tag => (
            <span
              key={tag}
              className="text-[9px] font-mono tracking-wider uppercase px-2 py-0.5"
              style={{
                border: `1px solid ${accentColor}30`,
                color: accentColor,
                background: `${accentColor}10`,
              }}
            >
              {tag}
            </span>
          ))}
        </div>

        {/* Divider */}
        <div className="border-t" style={{ borderColor: '#27272A' }} />

        {/* Feature list */}
        <div className="flex flex-col gap-2 flex-1">
          {[
            isCyan ? '3D Gazebo environment rendering' : 'Local 2D OccupancyGrid map',
            isCyan ? 'PX4 SITL flight controller' : 'FAST-LIO2 local odometry',
            isCyan ? 'Simulated LiDAR & camera' : 'Real LiDAR + camera feed',
            isCyan ? 'Scenario control & replay' : 'Emergency abort available',
          ].map(f => (
            <div key={f} className="flex items-center gap-2 text-xs text-muted">
              <span className="w-1 h-1 rounded-full flex-shrink-0" style={{ background: accentColor }} />
              {f}
            </div>
          ))}
        </div>

        {/* Button */}
        <button
          className="w-full flex items-center justify-center gap-3 py-4 text-sm font-semibold tracking-[0.12em] uppercase transition-all duration-200 group-hover:opacity-90"
          style={{
            background: isCyan
              ? 'linear-gradient(135deg, #00F0FF18, #00F0FF28)'
              : 'linear-gradient(135deg, #FFB00018, #FFB00028)',
            border: `1px solid ${accentColor}60`,
            color: accentColor,
            fontFamily: 'IBM Plex Sans, sans-serif',
          }}
          onClick={(e) => { e.stopPropagation(); navigate(route); }}
        >
          {buttonLabel}
          <ChevronRight size={14} strokeWidth={2.5} />
        </button>
      </div>
    </div>
  );
}

// ─── Mode Selection Page ──────────────────────────────────────

export default function ModeSelection() {
  return (
    <div
      className="min-h-screen flex flex-col items-center justify-center px-6 py-12"
      style={{ background: '#09090B', fontFamily: 'IBM Plex Sans, sans-serif' }}
    >
      {/* Background grid */}
      <div
        className="absolute inset-0 pointer-events-none"
        style={{
          backgroundImage: `
            linear-gradient(rgba(39,39,42,0.4) 1px, transparent 1px),
            linear-gradient(90deg, rgba(39,39,42,0.4) 1px, transparent 1px)
          `,
          backgroundSize: '40px 40px',
        }}
      />

      {/* Center gradient glow */}
      <div
        className="absolute inset-0 pointer-events-none"
        style={{
          background: 'radial-gradient(ellipse 80% 60% at 50% 50%, rgba(0,240,255,0.03) 0%, transparent 70%)',
        }}
      />

      <div className="relative z-10 w-full max-w-5xl flex flex-col items-center gap-12">

        {/* ── Header ── */}
        <div className="flex flex-col items-center gap-4 text-center">
          <div className="flex items-center gap-3">
            <div className="w-8 h-px" style={{ background: 'linear-gradient(90deg, transparent, #00F0FF)' }} />
            <span className="text-[9px] font-mono tracking-[0.3em] uppercase text-muted">
              NIDAR AIRMOUSE GCS v2.0
            </span>
            <div className="w-8 h-px" style={{ background: 'linear-gradient(90deg, #00F0FF, transparent)' }} />
          </div>

          <h1
            className="text-5xl font-bold tracking-tight uppercase"
            style={{
              background: 'linear-gradient(135deg, #E4E4E7 30%, #A1A1AA 100%)',
              WebkitBackgroundClip: 'text',
              WebkitTextFillColor: 'transparent',
              letterSpacing: '-0.02em',
            }}
          >
            NIDAR AIR MOUSE
          </h1>

          <p
            className="text-sm font-mono tracking-[0.2em] uppercase"
            style={{ color: '#00F0FF', textShadow: '0 0 12px rgba(0,240,255,0.4)' }}
          >
            AUTONOMOUS INDOOR SEARCH & RESCUE — 2026
          </p>

          <p className="text-xs text-muted tracking-wider mt-2">
            SELECT OPERATING ENVIRONMENT
          </p>
        </div>

        {/* ── Environment cards ── */}
        <div className="w-full grid grid-cols-2 gap-6" style={{ height: '520px' }}>
          <EnvironmentCard
            title="SIMULATION"
            subtitle="Operating Environment"
            description="Connect to Gazebo + PX4 SITL autonomous indoor simulation. Full robotics stack with simulated LiDAR, camera, and survivor detection."
            tags={['GAZEBO', 'PX4 SITL', 'ROS 2', 'FAST-LIO2', 'YOLO']}
            statusItems={[
              { ok: true, label: 'SIMULATOR READY' },
              { ok: true, label: 'PX4 SITL ONLINE' },
            ]}
            buttonLabel="LAUNCH SIMULATION"
            buttonColor="cyan"
            icon={<Monitor size={24} color="#00F0FF" />}
            route="/simulation"
            accentColor="#00F0FF"
            borderColor="#27272A"
          />

          <EnvironmentCard
            title="REAL HARDWARE"
            subtitle="Operating Environment"
            description="Connect to the physical NIDAR AirMouse drone. GPS-denied local navigation with FAST-LIO2 SLAM, real LiDAR, and live camera stream."
            tags={['PX4', 'ROS 2', 'FAST-LIO2', 'LIDAR', 'CAMERA', 'NO GPS']}
            statusItems={[
              { ok: false, label: 'DRONE DISCONNECTED' },
              { ok: false, label: 'ROS 2 OFFLINE' },
            ]}
            buttonLabel="CONNECT DRONE"
            buttonColor="amber"
            icon={<Radio size={24} color="#FFB000" />}
            route="/hardware"
            accentColor="#FFB000"
            borderColor="#27272A"
          />
        </div>

        {/* ── Footer info bar ── */}
        <div
          className="w-full flex items-center justify-between px-6 py-3"
          style={{ border: '1px solid #27272A', background: '#111113' }}
        >
          <div className="flex items-center gap-6">
            <div className="flex items-center gap-2 text-[10px] font-mono text-muted uppercase tracking-wider">
              <Activity size={10} />
              GPS-DENIED NAVIGATION
            </div>
            <div className="flex items-center gap-2 text-[10px] font-mono text-muted uppercase tracking-wider">
              <Cpu size={10} />
              NVIDIA JETSON ORIN
            </div>
            <div className="flex items-center gap-2 text-[10px] font-mono text-muted uppercase tracking-wider">
              <Wifi size={10} />
              LOCAL 5 GHz Wi-Fi
            </div>
          </div>
          <div className="text-[10px] font-mono text-disabled tracking-wider">
            NIDAR 2026 COMPETITION SYSTEM
          </div>
        </div>
      </div>
    </div>
  );
}
