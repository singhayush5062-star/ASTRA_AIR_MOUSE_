import { useEffect, useState } from 'react';
import {
  VideoOff, Loader2, RefreshCw,
  Sliders, Cpu, Radio, ShieldCheck,
} from 'lucide-react';
import type { CameraFrame } from '@/types';

// ─── Interfaces ───────────────────────────────────────────────

export type CameraStatus = 'OFFLINE' | 'CONNECTING' | 'LIVE' | 'ERROR';

interface FCCameraDiagnostics {
  camera_source: string;
  fc_host: string;
  stream_url: string;
  camera_status: 'CONNECTED' | 'OFFLINE' | 'CONNECTING' | 'RECONNECTING';
  receiving_frames: boolean;
  frame_fps: number;
  model_status: string;
  model_name: string;
  model_fps: number;
  inference_latency_ms: number;
  last_frame_timestamp: number;
  detected_persons: number;
}

interface CameraViewProps {
  mode: 'simulation' | 'hardware';
  status?: CameraStatus;
  cameraFrame?: CameraFrame;
  className?: string;
}

// ─── Component ────────────────────────────────────────────────

export function CameraView({
  cameraFrame,
  className = '',
}: CameraViewProps) {
  const [diagnostics, setDiagnostics] = useState<FCCameraDiagnostics>({
    camera_source: 'FC / Jetson',
    fc_host: '192.168.1.100',
    stream_url: 'rtsp://192.168.1.100:8554/live',
    camera_status: 'OFFLINE',
    receiving_frames: false,
    frame_fps: 0,
    model_status: 'RUNNING ON LAPTOP',
    model_name: 'YOLO26s-Person',
    model_fps: 0,
    inference_latency_ms: 0,
    last_frame_timestamp: 0,
    detected_persons: 0,
  });

  const [showConfig, setShowConfig] = useState(false);
  const [showDebug, setShowDebug] = useState(false);
  const [inputUrl, setInputUrl] = useState('rtsp://192.168.1.100:8554/live');
  const [isUpdating, setIsUpdating] = useState(false);
  const [reconnecting, setReconnecting] = useState(false);

  // Poll camera status every 1.5 seconds from backend
  useEffect(() => {
    let isMounted = true;

    const fetchStatus = async () => {
      try {
        const res = await fetch('http://localhost:8000/api/camera/status');
        if (res.ok) {
          const data = await res.json();
          if (isMounted) {
            setDiagnostics(data);
            if (!showConfig) {
              setInputUrl(data.stream_url || 'rtsp://192.168.1.100:8554/live');
            }
          }
        }
      } catch (err) {
        // Backend not reachable
      }
    };

    fetchStatus();
    const interval = setInterval(fetchStatus, 1500);
    return () => {
      isMounted = false;
      clearInterval(interval);
    };
  }, [showConfig]);

  // Handle stream URL config save
  const handleSaveConfig = async () => {
    setIsUpdating(true);
    try {
      const res = await fetch('http://localhost:8000/api/camera/config', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ stream_url: inputUrl }),
      });
      if (res.ok) {
        const data = await res.json();
        setDiagnostics(data);
        setShowConfig(false);
      }
    } catch (err) {
      console.error('Failed to update FC stream config:', err);
    } finally {
      setIsUpdating(false);
    }
  };

  // Trigger manual reconnect
  const handleReconnect = async () => {
    setReconnecting(true);
    try {
      await fetch('http://localhost:8000/api/camera/control', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ action: 'reconnect', stream_url: inputUrl }),
      });
    } catch (err) {
      console.error('Reconnect failed:', err);
    } finally {
      setTimeout(() => setReconnecting(false), 1000);
    }
  };

  const isConnected = diagnostics.camera_status === 'CONNECTED' && diagnostics.receiving_frames;

  // ── FC Camera Offline / Disconnected State ──
  if (!isConnected) {
    return (
      <div
        className={`relative flex flex-col items-center justify-center h-full w-full select-none overflow-hidden font-mono ${className}`}
        style={{
          background: '#09090B',
          backgroundImage: `
            linear-gradient(rgba(39,39,42,0.3) 1px, transparent 1px),
            linear-gradient(90deg, rgba(39,39,42,0.3) 1px, transparent 1px)
          `,
          backgroundSize: '24px 24px',
        }}
      >
        {/* Offline Center Banner */}
        <div className="flex flex-col items-center gap-3 p-6 text-center z-10 max-w-md">
          <div
            className="p-3 rounded-full flex items-center justify-center"
            style={{ background: 'rgba(255,0,60,0.08)', border: '1px solid rgba(255,0,60,0.25)' }}
          >
            {diagnostics.camera_status === 'CONNECTING' || reconnecting ? (
              <Loader2 size={24} className="animate-spin text-amber-400" />
            ) : (
              <VideoOff size={24} color="#FF003C" />
            )}
          </div>

          <div className="flex flex-col gap-1">
            <span className="text-[13px] tracking-[0.2em] font-black uppercase text-[#E4E4E7]">
              FC CAMERA OFFLINE
            </span>
            <span className="text-[10px] tracking-wider uppercase text-amber-400">
              {diagnostics.camera_status === 'RECONNECTING' || reconnecting
                ? 'ATTEMPTING RECONNECTION TO FC STREAM...'
                : 'WAITING FOR FC / JETSON CAMERA LINK'}
            </span>
          </div>

          <div className="p-2.5 w-full bg-[#111114] border border-zinc-800 text-[10px] text-zinc-400 text-left space-y-1">
            <div className="flex justify-between">
              <span className="text-zinc-500">CAMERA SOURCE:</span>
              <span className="text-white font-bold">FC / Jetson (Drone)</span>
            </div>
            <div className="flex justify-between">
              <span className="text-zinc-500">INFERENCE MODEL:</span>
              <span className="text-cyan-400 font-bold">YOLO26s (Running on Laptop)</span>
            </div>
            <div className="flex justify-between items-center overflow-hidden">
              <span className="text-zinc-500">STREAM URL:</span>
              <span className="text-amber-300 font-mono text-[9px] truncate max-w-[200px]" title={diagnostics.stream_url}>
                {diagnostics.stream_url}
              </span>
            </div>
          </div>

          {/* Action Buttons */}
          <div className="flex items-center gap-2 pt-1">
            <button
              onClick={handleReconnect}
              disabled={reconnecting}
              className="px-3 py-1.5 bg-[#FF003C]/20 hover:bg-[#FF003C]/30 border border-[#FF003C] text-red-300 hover:text-white text-[10px] font-bold tracking-wider uppercase flex items-center gap-1.5 transition-all"
            >
              <RefreshCw size={11} className={reconnecting ? 'animate-spin' : ''} />
              RECONNECT
            </button>

            <button
              onClick={() => setShowConfig(!showConfig)}
              className="px-3 py-1.5 bg-zinc-800 hover:bg-zinc-700 border border-zinc-600 text-zinc-200 text-[10px] font-bold tracking-wider uppercase flex items-center gap-1.5 transition-all"
            >
              <Sliders size={11} />
              CONFIGURE STREAM
            </button>

            <button
              onClick={() => setShowDebug(!showDebug)}
              className="px-2 py-1.5 bg-zinc-900 border border-zinc-800 text-zinc-400 hover:text-zinc-200 text-[10px] uppercase transition-all"
            >
              DEBUG
            </button>
          </div>
        </div>

        {/* Stream Configuration Drawer */}
        {showConfig && (
          <div className="absolute inset-x-4 top-10 bg-[#16171C] border border-amber-500/50 p-3 shadow-2xl z-20 max-w-md mx-auto">
            <div className="flex items-center justify-between pb-2 border-b border-zinc-800 text-[10px] font-bold text-amber-400">
              <span>FC / JETSON CAMERA STREAM CONFIG</span>
              <button onClick={() => setShowConfig(false)} className="text-zinc-400 hover:text-white">✕</button>
            </div>
            <div className="mt-2 space-y-2 text-[10px]">
              <div>
                <label className="text-zinc-400 block text-[9px] mb-0.5">STREAM URL (RTSP / HTTP / UDP):</label>
                <input
                  type="text"
                  value={inputUrl}
                  onChange={(e) => setInputUrl(e.target.value)}
                  className="w-full px-2 py-1 bg-black border border-zinc-700 text-white font-mono text-[10px] focus:border-amber-400 outline-none"
                  placeholder="rtsp://192.168.1.100:8554/live"
                />
              </div>

              {/* Quick Presets */}
              <div className="flex gap-1.5 text-[8px] text-zinc-500 pt-1">
                <span>PRESETS:</span>
                <button
                  onClick={() => setInputUrl('rtsp://192.168.1.100:8554/live')}
                  className="text-cyan-400 hover:underline"
                >
                  RTSP 8554
                </button>
                <span>|</span>
                <button
                  onClick={() => setInputUrl('http://192.168.1.100:8080/stream')}
                  className="text-cyan-400 hover:underline"
                >
                  HTTP 8080
                </button>
                <span>|</span>
                <button
                  onClick={() => setInputUrl('udp://@:5600')}
                  className="text-cyan-400 hover:underline"
                >
                  UDP 5600
                </button>
              </div>

              <div className="flex justify-end gap-2 pt-2">
                <button
                  onClick={() => setShowConfig(false)}
                  className="px-2.5 py-1 bg-zinc-800 text-zinc-300 text-[9px] uppercase hover:bg-zinc-700"
                >
                  Cancel
                </button>
                <button
                  onClick={handleSaveConfig}
                  disabled={isUpdating}
                  className="px-3 py-1 bg-amber-500 hover:bg-amber-400 text-black font-bold text-[9px] uppercase"
                >
                  {isUpdating ? 'Saving...' : 'Apply & Reconnect'}
                </button>
              </div>
            </div>
          </div>
        )}

        {/* Section 17 Debug Telemetry Modal */}
        {showDebug && (
          <div className="absolute inset-x-4 bottom-4 bg-[#111114] border border-cyan-500/50 p-3 shadow-2xl z-20 max-w-lg mx-auto text-[9px]">
            <div className="flex items-center justify-between pb-1.5 border-b border-zinc-800 font-bold text-cyan-400">
              <span className="flex items-center gap-1"><Cpu size={12} /> FC CAMERA & LAPTOP MODEL DEBUG</span>
              <button onClick={() => setShowDebug(false)} className="text-zinc-400 hover:text-white">✕</button>
            </div>
            <div className="grid grid-cols-2 gap-x-3 gap-y-1.5 mt-2 text-zinc-300">
              <div><span className="text-zinc-500">Camera Source:</span> <span className="text-white font-bold">{diagnostics.camera_source}</span></div>
              <div><span className="text-zinc-500">FC IP / Host:</span> <span className="text-white font-bold">{diagnostics.fc_host}</span></div>
              <div className="col-span-2 truncate"><span className="text-zinc-500">Stream URL:</span> <span className="text-amber-300 font-mono">{diagnostics.stream_url}</span></div>
              <div><span className="text-zinc-500">Camera Status:</span> <span className="text-red-400 font-bold">{diagnostics.camera_status}</span></div>
              <div><span className="text-zinc-500">Receiving Frames:</span> <span className="text-red-400 font-bold">{diagnostics.receiving_frames ? 'YES' : 'NO'}</span></div>
              <div><span className="text-zinc-500">Frame FPS:</span> <span className="text-white font-bold">{diagnostics.frame_fps}</span></div>
              <div><span className="text-zinc-500">Model Status:</span> <span className="text-cyan-400 font-bold">{diagnostics.model_status}</span></div>
              <div><span className="text-zinc-500">Model FPS:</span> <span className="text-white font-bold">{diagnostics.model_fps}</span></div>
              <div><span className="text-zinc-500">Inference Latency:</span> <span className="text-white font-bold">{diagnostics.inference_latency_ms} ms</span></div>
              <div><span className="text-zinc-500">Persons Detected:</span> <span className="text-emerald-400 font-bold">{diagnostics.detected_persons}</span></div>
            </div>
          </div>
        )}

        {/* Corner Status Pill */}
        <div
          className="absolute top-2 right-2 flex items-center gap-1.5 px-2 py-0.5 bg-black/80 border border-red-500/40 text-[9px]"
        >
          <span className="w-1.5 h-1.5 rounded-full bg-red-500 animate-pulse" />
          <span className="text-red-400 uppercase font-bold">FC CAMERA {diagnostics.camera_status}</span>
        </div>
      </div>
    );
  }

  // ── FC Camera Live Video Stream (Connected) ──
  return (
    <div className={`relative overflow-hidden w-full h-full select-none font-mono ${className}`} style={{ background: '#0a0a0c' }}>
      <img
        src={cameraFrame?.dataUrl || 'http://localhost:8000/api/camera/stream'}
        alt="FC / Jetson Camera Feed"
        className="w-full h-full object-contain bg-black/90"
      />

      {/* Top Left Status Badges */}
      <div className="absolute top-2 left-2 flex items-center gap-1.5 z-10">
        <div
          className="flex items-center gap-1.5 px-2 py-0.5"
          style={{ background: 'rgba(0,0,0,0.85)', border: '1px solid #00FF41' }}
        >
          <span className="w-2 h-2 rounded-full bg-emerald-400 animate-pulse" />
          <span className="text-[9px] tracking-wider uppercase font-semibold text-emerald-400 flex items-center gap-1">
            <Radio size={10} /> FC CAMERA: LIVE
          </span>
        </div>

        <div
          className="flex items-center gap-1.5 px-2 py-0.5"
          style={{ background: 'rgba(0,0,0,0.85)', border: '1px solid #00F0FF' }}
        >
          <Cpu size={10} className="text-cyan-400" />
          <span className="text-[9px] tracking-wider uppercase font-semibold text-cyan-400">
            INFERENCE: LAPTOP (YOLO26s)
          </span>
        </div>

        <button
          onClick={() => setShowDebug(!showDebug)}
          className="px-2 py-0.5 bg-black/80 hover:bg-zinc-800 border border-zinc-700 text-zinc-300 hover:text-white text-[9px] tracking-wider uppercase transition-colors"
        >
          DEBUG
        </button>
      </div>

      {/* Top Right Live Telemetry */}
      <div
        className="absolute top-2 right-2 flex items-center gap-2 px-2 py-0.5 bg-black/80 border border-zinc-700 text-[9px]"
      >
        <span className="text-zinc-400">FPS: <strong className="text-white">{diagnostics.frame_fps}</strong></span>
        <span className="text-zinc-600">|</span>
        <span className="text-zinc-400">LATENCY: <strong className="text-cyan-300">{diagnostics.inference_latency_ms}ms</strong></span>
        <span className="text-zinc-600">|</span>
        <span className="text-emerald-400 font-bold flex items-center gap-1">
          <ShieldCheck size={11} /> {diagnostics.detected_persons} SURVIVORS
        </span>
      </div>

      {/* Section 17 Debug Telemetry Modal */}
      {showDebug && (
        <div className="absolute inset-x-4 bottom-4 bg-[#111114]/95 backdrop-blur-md border border-cyan-500/50 p-3 shadow-2xl z-20 max-w-lg mx-auto text-[9px]">
          <div className="flex items-center justify-between pb-1.5 border-b border-zinc-800 font-bold text-cyan-400">
            <span className="flex items-center gap-1"><Cpu size={12} /> FC CAMERA & LAPTOP MODEL DEBUG</span>
            <button onClick={() => setShowDebug(false)} className="text-zinc-400 hover:text-white">✕</button>
          </div>
          <div className="grid grid-cols-2 gap-x-3 gap-y-1.5 mt-2 text-zinc-300">
            <div><span className="text-zinc-500">Camera Source:</span> <span className="text-white font-bold">{diagnostics.camera_source}</span></div>
            <div><span className="text-zinc-500">FC IP / Host:</span> <span className="text-white font-bold">{diagnostics.fc_host}</span></div>
            <div className="col-span-2 truncate"><span className="text-zinc-500">Stream URL:</span> <span className="text-amber-300 font-mono">{diagnostics.stream_url}</span></div>
            <div><span className="text-zinc-500">Camera Status:</span> <span className="text-emerald-400 font-bold">{diagnostics.camera_status}</span></div>
            <div><span className="text-zinc-500">Receiving Frames:</span> <span className="text-emerald-400 font-bold">YES</span></div>
            <div><span className="text-zinc-500">Frame FPS:</span> <span className="text-white font-bold">{diagnostics.frame_fps}</span></div>
            <div><span className="text-zinc-500">Model Status:</span> <span className="text-cyan-400 font-bold">{diagnostics.model_status}</span></div>
            <div><span className="text-zinc-500">Model FPS:</span> <span className="text-white font-bold">{diagnostics.model_fps}</span></div>
            <div><span className="text-zinc-500">Inference Latency:</span> <span className="text-white font-bold">{diagnostics.inference_latency_ms} ms</span></div>
            <div><span className="text-zinc-500">Persons Detected:</span> <span className="text-emerald-400 font-bold">{diagnostics.detected_persons}</span></div>
          </div>
        </div>
      )}
    </div>
  );
}
