import React, { useState, useEffect, useCallback } from 'react';
import {
  X, Zap, Radio, Wifi, Usb, Cpu, RotateCw, Check, AlertTriangle, Loader2, Navigation,
} from 'lucide-react';
import { useHardwareStore, type DroneConnectionConfig } from '@/store';

interface ConnectDroneModalProps {
  isOpen: boolean;
  onClose: () => void;
}

interface DetectedPort {
  port: string;
  description: string;
  hwid?: string;
}

export function ConnectDroneModal({ isOpen, onClose }: ConnectDroneModalProps) {
  const hwState = useHardwareStore(s => s.hwConnectionState);
  const connError = useHardwareStore(s => s.connectionError);
  const connectDrone = useHardwareStore(s => s.connectDrone);

  // Quick Preset selection
  const [selectedPreset, setSelectedPreset] = useState<string>('apm_usb');

  // Drone Config
  const [droneName, setDroneName] = useState('Drone Alpha');
  const [sysId, setSysId] = useState('1');
  const [homeLat, setHomeLat] = useState('28.6754');
  const [homeLon, setHomeLon] = useState('77.5029');
  const [gpsAccuracy, setGpsAccuracy] = useState<number | null>(212);
  const [isLocating, setIsLocating] = useState(false);

  // Connection Type
  const [connType, setConnType] = useState<'serial' | 'udp' | 'tcp' | 'simulator'>('serial');

  // Serial params
  const [serialPort, setSerialPort] = useState('COM6');
  const [baudRate, setBaudRate] = useState('57600');
  const [detectedPorts, setDetectedPorts] = useState<DetectedPort[]>([]);
  const [isScanningPorts, setIsScanningPorts] = useState(false);

  // UDP params
  const [udpHost, setUdpHost] = useState('0.0.0.0');
  const [udpPort, setUdpPort] = useState('14550');

  // TCP params
  const [tcpHost, setTcpHost] = useState('127.0.0.1');
  const [tcpPort, setTcpPort] = useState('5760');

  // Simulator params
  const [simVehicle, setSimVehicle] = useState('Quadrotor (X-Frame)');

  // Local connecting state
  const [isSubmitting, setIsSubmitting] = useState(false);

  // Scan hardware ports from backend
  const scanPorts = useCallback(async () => {
    setIsScanningPorts(true);
    try {
      const res = await fetch('http://localhost:8000/api/hardware/ports');
      if (res.ok) {
        const data = await res.json();
        if (Array.isArray(data.ports)) {
          setDetectedPorts(data.ports);
          if (data.ports.length > 0) {
            // Pick ArduPilot or first port
            const ardupilot = data.ports.find((p: DetectedPort) =>
              p.description.toLowerCase().includes('ardupilot') ||
              p.description.toLowerCase().includes('mavlink')
            );
            const chosen = ardupilot || data.ports[0];
            setSerialPort(chosen.port);
          }
        }
      }
    } catch (err) {
      console.error('Failed to scan ports:', err);
    } finally {
      setIsScanningPorts(false);
    }
  }, []);

  // Fetch ports when modal opens
  useEffect(() => {
    if (isOpen) {
      scanPorts();
    }
  }, [isOpen, scanPorts]);

  // Handle Quick Presets
  const applyPreset = (presetId: string) => {
    setSelectedPreset(presetId);
    switch (presetId) {
      case 'sitl_udp':
        setConnType('udp');
        setUdpHost('0.0.0.0');
        setUdpPort('14550');
        break;
      case 'sitl_tcp':
        setConnType('tcp');
        setTcpHost('127.0.0.1');
        setTcpPort('5760');
        break;
      case 'apm_usb':
        setConnType('serial');
        setBaudRate('57600');
        if (detectedPorts.length > 0) setSerialPort(detectedPorts[0].port);
        break;
      case 'px4_usb':
        setConnType('serial');
        setBaudRate('115200');
        if (detectedPorts.length > 0) setSerialPort(detectedPorts[0].port);
        break;
      case 'sik_telemetry':
        setConnType('serial');
        setBaudRate('57600');
        break;
      case 'wifi_drone':
        setConnType('udp');
        setUdpHost('0.0.0.0');
        setUdpPort('14550');
        break;
      case 'builtin_sim':
        setConnType('simulator');
        break;
    }
  };

  // Use My Location
  const handleUseLocation = () => {
    if (!navigator.geolocation) return;
    setIsLocating(true);
    navigator.geolocation.getCurrentPosition(
      pos => {
        setHomeLat(pos.coords.latitude.toFixed(6));
        setHomeLon(pos.coords.longitude.toFixed(6));
        setGpsAccuracy(Math.round(pos.coords.accuracy));
        setIsLocating(false);
      },
      () => {
        setIsLocating(false);
      },
      { timeout: 8000 }
    );
  };

  // Submit connection
  const handleConnect = async (e: React.FormEvent) => {
    e.preventDefault();
    setIsSubmitting(true);

    const config: DroneConnectionConfig = {
      connection_type: connType,
      drone_name: droneName.trim() || 'Drone Alpha',
      sys_id: parseInt(sysId, 10) || 1,
      home_lat: parseFloat(homeLat) || 28.6754,
      home_lon: parseFloat(homeLon) || 77.5029,
      serial_port: serialPort,
      baud_rate: parseInt(baudRate, 10) || 57600,
      host: connType === 'udp' ? udpHost : tcpHost,
      udp_port: parseInt(udpPort, 10) || 14550,
      tcp_port: parseInt(tcpPort, 10) || 5760,
    };

    const success = await connectDrone(config);
    setIsSubmitting(false);
    if (success) {
      onClose();
    }
  };

  const handleForceConnect = async () => {
    setIsSubmitting(true);
    const config: DroneConnectionConfig = {
      connection_type: connType,
      drone_name: droneName.trim() || 'Drone Alpha',
      sys_id: parseInt(sysId, 10) || 1,
      home_lat: parseFloat(homeLat) || 28.6754,
      home_lon: parseFloat(homeLon) || 77.5029,
      serial_port: serialPort || 'COM6',
      baud_rate: parseInt(baudRate, 10) || 57600,
      host: connType === 'udp' ? udpHost : tcpHost,
      udp_port: parseInt(udpPort, 10) || 14550,
      tcp_port: parseInt(tcpPort, 10) || 5760,
      force_connect: true,
    };
    const success = await connectDrone(config);
    setIsSubmitting(false);
    if (success) {
      onClose();
    }
  };

  if (!isOpen) return null;

  return (
    <div className="fixed inset-0 z-50 flex items-center justify-center p-4 bg-black/80 backdrop-blur-sm animate-in fade-in duration-200">
      <div
        className="w-full max-w-2xl bg-[#131418] border border-[#FFB000] text-[#E4E4E7] font-mono shadow-[0_0_40px_rgba(255,176,0,0.2)] flex flex-col max-h-[92vh] overflow-hidden"
        style={{ borderColor: '#FFB000' }}
      >
        {/* Header */}
        <div className="flex items-start justify-between p-5 border-b border-zinc-800 bg-[#18191E]">
          <div>
            <div className="flex items-center gap-2.5">
              <span className="w-2.5 h-2.5 rounded-full bg-[#FFB000] animate-pulse" />
              <h2 className="text-base font-black tracking-[0.2em] text-white uppercase">
                CONNECT NEW DRONE
              </h2>
            </div>
            <p className="text-[11px] text-zinc-400 mt-1 tracking-wider">
              Connect Real Drone (Serial / USB / Telemetry Radio / UDP / TCP) or Simulator
            </p>
          </div>
          <button
            onClick={onClose}
            className="text-zinc-400 hover:text-white p-1 hover:bg-zinc-800 transition-colors"
            title="Close"
          >
            <X size={18} />
          </button>
        </div>

        <form onSubmit={handleConnect} className="flex-1 overflow-y-auto p-5 space-y-5">
          {/* Quick Presets */}
          <div>
            <div className="text-[10px] font-bold text-zinc-400 tracking-[0.15em] uppercase mb-2">
              QUICK PRESETS
            </div>
            <div className="grid grid-cols-2 sm:grid-cols-4 gap-2">
              <button
                type="button"
                onClick={() => applyPreset('sitl_udp')}
                className={`flex items-center gap-1.5 px-2.5 py-2 text-[10px] tracking-wider border transition-all text-left ${
                  selectedPreset === 'sitl_udp'
                    ? 'border-[#FFB000] bg-[#FFB000]/15 text-[#FFB000] shadow-[0_0_10px_rgba(255,176,0,0.2)]'
                    : 'border-zinc-800 bg-[#1A1B20] text-zinc-300 hover:border-zinc-600'
                }`}
              >
                <Zap size={11} className="text-[#FFB000] shrink-0" />
                <span className="truncate">SITL UDP :14550</span>
              </button>

              <button
                type="button"
                onClick={() => applyPreset('sitl_tcp')}
                className={`flex items-center gap-1.5 px-2.5 py-2 text-[10px] tracking-wider border transition-all text-left ${
                  selectedPreset === 'sitl_tcp'
                    ? 'border-[#FFB000] bg-[#FFB000]/15 text-[#FFB000] shadow-[0_0_10px_rgba(255,176,0,0.2)]'
                    : 'border-zinc-800 bg-[#1A1B20] text-zinc-300 hover:border-zinc-600'
                }`}
              >
                <Zap size={11} className="text-[#FFB000] shrink-0" />
                <span className="truncate">SITL TCP :5760</span>
              </button>

              <button
                type="button"
                onClick={() => applyPreset('apm_usb')}
                className={`flex items-center gap-1.5 px-2.5 py-2 text-[10px] tracking-wider border transition-all text-left ${
                  selectedPreset === 'apm_usb'
                    ? 'border-[#FFB000] bg-[#FFB000]/15 text-[#FFB000] shadow-[0_0_10px_rgba(255,176,0,0.2)]'
                    : 'border-zinc-800 bg-[#1A1B20] text-zinc-300 hover:border-zinc-600'
                }`}
              >
                <Usb size={11} className="text-cyan-400 shrink-0" />
                <span className="truncate">APM/Pixhawk USB</span>
              </button>

              <button
                type="button"
                onClick={() => applyPreset('px4_usb')}
                className={`flex items-center gap-1.5 px-2.5 py-2 text-[10px] tracking-wider border transition-all text-left ${
                  selectedPreset === 'px4_usb'
                    ? 'border-[#FFB000] bg-[#FFB000]/15 text-[#FFB000] shadow-[0_0_10px_rgba(255,176,0,0.2)]'
                    : 'border-zinc-800 bg-[#1A1B20] text-zinc-300 hover:border-zinc-600'
                }`}
              >
                <Usb size={11} className="text-cyan-400 shrink-0" />
                <span className="truncate">PX4 USB @115200</span>
              </button>

              <button
                type="button"
                onClick={() => applyPreset('sik_telemetry')}
                className={`flex items-center gap-1.5 px-2.5 py-2 text-[10px] tracking-wider border transition-all text-left ${
                  selectedPreset === 'sik_telemetry'
                    ? 'border-[#FFB000] bg-[#FFB000]/15 text-[#FFB000] shadow-[0_0_10px_rgba(255,176,0,0.2)]'
                    : 'border-zinc-800 bg-[#1A1B20] text-zinc-300 hover:border-zinc-600'
                }`}
              >
                <Radio size={11} className="text-emerald-400 shrink-0" />
                <span className="truncate">SiK Telemetry @57600</span>
              </button>

              <button
                type="button"
                onClick={() => applyPreset('wifi_drone')}
                className={`flex items-center gap-1.5 px-2.5 py-2 text-[10px] tracking-wider border transition-all text-left ${
                  selectedPreset === 'wifi_drone'
                    ? 'border-[#FFB000] bg-[#FFB000]/15 text-[#FFB000] shadow-[0_0_10px_rgba(255,176,0,0.2)]'
                    : 'border-zinc-800 bg-[#1A1B20] text-zinc-300 hover:border-zinc-600'
                }`}
              >
                <Wifi size={11} className="text-blue-400 shrink-0" />
                <span className="truncate">Wi-Fi Drone UDP</span>
              </button>

              <button
                type="button"
                onClick={() => applyPreset('builtin_sim')}
                className={`flex items-center gap-1.5 px-2.5 py-2 text-[10px] tracking-wider border transition-all text-left col-span-2 sm:col-span-2 ${
                  selectedPreset === 'builtin_sim'
                    ? 'border-[#FFB000] bg-[#FFB000]/15 text-[#FFB000] shadow-[0_0_10px_rgba(255,176,0,0.2)]'
                    : 'border-zinc-800 bg-[#1A1B20] text-zinc-300 hover:border-zinc-600'
                }`}
              >
                <Zap size={11} className="text-amber-400 shrink-0" />
                <span className="truncate">Built-in Simulator</span>
              </button>
            </div>
          </div>

          {/* Drone Name & Position Inputs */}
          <div>
            <div className="grid grid-cols-12 gap-2.5">
              <div className="col-span-12 sm:col-span-5">
                <label className="block text-[9px] font-bold text-zinc-400 uppercase tracking-widest mb-1">
                  DRONE NAME
                </label>
                <input
                  type="text"
                  value={droneName}
                  onChange={e => setDroneName(e.target.value)}
                  className="w-full bg-[#1A1B20] border border-zinc-700 focus:border-[#FFB000] px-3 py-2 text-xs font-mono text-white outline-none"
                  placeholder="Drone Alpha"
                />
              </div>

              <div className="col-span-4 sm:col-span-2">
                <label className="block text-[9px] font-bold text-zinc-400 uppercase tracking-widest mb-1">
                  SYS ID
                </label>
                <input
                  type="number"
                  value={sysId}
                  onChange={e => setSysId(e.target.value)}
                  className="w-full bg-[#1A1B20] border border-zinc-700 focus:border-[#FFB000] px-3 py-2 text-xs font-mono text-white outline-none text-center"
                />
              </div>

              <div className="col-span-4 sm:col-span-2">
                <label className="block text-[9px] font-bold text-zinc-400 uppercase tracking-widest mb-1">
                  HOME LAT
                </label>
                <input
                  type="text"
                  value={homeLat}
                  onChange={e => setHomeLat(e.target.value)}
                  className="w-full bg-[#1A1B20] border border-zinc-700 focus:border-[#FFB000] px-2.5 py-2 text-xs font-mono text-white outline-none text-center"
                />
              </div>

              <div className="col-span-4 sm:col-span-3">
                <label className="block text-[9px] font-bold text-zinc-400 uppercase tracking-widest mb-1">
                  HOME LON
                </label>
                <input
                  type="text"
                  value={homeLon}
                  onChange={e => setHomeLon(e.target.value)}
                  className="w-full bg-[#1A1B20] border border-zinc-700 focus:border-[#FFB000] px-2.5 py-2 text-xs font-mono text-white outline-none text-center"
                />
              </div>
            </div>

            {/* Live GPS & Use My Location */}
            <div className="flex items-center justify-between mt-2 pt-1 text-[10px] text-zinc-400">
              <div className="flex items-center gap-1.5 font-mono">
                <span>Live GPS:</span>
                <span className="text-cyan-400 font-bold">
                  {homeLat}, {homeLon}
                </span>
                {gpsAccuracy && <span className="text-zinc-500">· ±{gpsAccuracy}m</span>}
              </div>

              <button
                type="button"
                onClick={handleUseLocation}
                disabled={isLocating}
                className="flex items-center gap-1 px-2 py-1 text-[9px] font-bold uppercase tracking-wider text-cyan-400 border border-cyan-500/40 bg-cyan-950/20 hover:bg-cyan-900/30 transition-colors"
              >
                {isLocating ? (
                  <Loader2 size={10} className="animate-spin" />
                ) : (
                  <Navigation size={10} />
                )}
                USE MY LOCATION
              </button>
            </div>
          </div>

          {/* Connection Type Tabs */}
          <div>
            <div className="text-[10px] font-bold text-zinc-400 tracking-[0.15em] uppercase mb-2">
              CONNECTION TYPE
            </div>
            <div className="grid grid-cols-4 border-b border-zinc-800">
              <button
                type="button"
                onClick={() => setConnType('serial')}
                className={`flex items-center justify-center gap-1.5 py-2.5 text-[10px] font-bold tracking-wider uppercase transition-colors border-b-2 ${
                  connType === 'serial'
                    ? 'border-[#FFB000] text-[#FFB000] bg-[#FFB000]/10'
                    : 'border-transparent text-zinc-400 hover:text-zinc-200'
                }`}
              >
                <Usb size={12} />
                <span className="truncate">SERIAL (USB/RADIO)</span>
              </button>

              <button
                type="button"
                onClick={() => setConnType('udp')}
                className={`flex items-center justify-center gap-1.5 py-2.5 text-[10px] font-bold tracking-wider uppercase transition-colors border-b-2 ${
                  connType === 'udp'
                    ? 'border-[#FFB000] text-[#FFB000] bg-[#FFB000]/10'
                    : 'border-transparent text-zinc-400 hover:text-zinc-200'
                }`}
              >
                <Wifi size={12} />
                <span>UDP</span>
              </button>

              <button
                type="button"
                onClick={() => setConnType('tcp')}
                className={`flex items-center justify-center gap-1.5 py-2.5 text-[10px] font-bold tracking-wider uppercase transition-colors border-b-2 ${
                  connType === 'tcp'
                    ? 'border-[#FFB000] text-[#FFB000] bg-[#FFB000]/10'
                    : 'border-transparent text-zinc-400 hover:text-zinc-200'
                }`}
              >
                <Radio size={12} />
                <span>TCP</span>
              </button>

              <button
                type="button"
                onClick={() => setConnType('simulator')}
                className={`flex items-center justify-center gap-1.5 py-2.5 text-[10px] font-bold tracking-wider uppercase transition-colors border-b-2 ${
                  connType === 'simulator'
                    ? 'border-[#FFB000] text-[#FFB000] bg-[#FFB000]/10'
                    : 'border-transparent text-zinc-400 hover:text-zinc-200'
                }`}
              >
                <Zap size={12} />
                <span>SIMULATOR</span>
              </button>
            </div>
          </div>

          {/* TAB 1: SERIAL (USB/RADIO) */}
          {connType === 'serial' && (
            <div className="space-y-3.5 animate-in fade-in duration-150">
              <div className="grid grid-cols-1 sm:grid-cols-3 gap-3">
                <div className="sm:col-span-2">
                  <label className="block text-[9px] font-bold text-zinc-400 uppercase tracking-widest mb-1">
                    SERIAL PORT / DEVICE PATH
                  </label>
                  <input
                    type="text"
                    value={serialPort}
                    onChange={e => setSerialPort(e.target.value)}
                    className="w-full bg-[#1A1B20] border border-zinc-700 focus:border-[#FFB000] px-3 py-2 text-xs font-mono text-white outline-none"
                    placeholder="e.g. COM6 or /dev/ttyUSB0"
                  />
                </div>

                <div>
                  <label className="block text-[9px] font-bold text-zinc-400 uppercase tracking-widest mb-1">
                    BAUD RATE
                  </label>
                  <select
                    value={baudRate}
                    onChange={e => setBaudRate(e.target.value)}
                    className="w-full bg-[#1A1B20] border border-zinc-700 focus:border-[#FFB000] px-3 py-2 text-xs font-mono text-white outline-none cursor-pointer"
                  >
                    <option value="57600">57600 (SiK / Pixhawk)</option>
                    <option value="115200">115200 (PX4 native USB)</option>
                    <option value="9600">9600</option>
                    <option value="19200">19200</option>
                    <option value="38400">38400</option>
                    <option value="230400">230400</option>
                    <option value="460800">460800</option>
                    <option value="921600">921600 (High-Speed)</option>
                  </select>
                </div>
              </div>

              {/* Detected System Serial Ports */}
              <div className="border border-zinc-800 bg-[#16171C] p-3">
                <div className="flex items-center justify-between mb-2">
                  <div className="flex items-center gap-1.5 text-[10px] font-bold tracking-wider text-cyan-400">
                    <Cpu size={12} />
                    <span>DETECTED SYSTEM SERIAL PORTS</span>
                  </div>
                  <button
                    type="button"
                    onClick={scanPorts}
                    disabled={isScanningPorts}
                    className="flex items-center gap-1 text-[9px] text-[#FFB000] hover:underline"
                  >
                    <RotateCw size={10} className={isScanningPorts ? 'animate-spin' : ''} />
                    {isScanningPorts ? 'Scanning...' : 'Scan Ports'}
                  </button>
                </div>

                {detectedPorts.length > 0 ? (
                  <div className="space-y-1.5">
                    {detectedPorts.map(p => {
                      const isSelected = serialPort.toUpperCase() === p.port.toUpperCase();
                      return (
                        <div
                          key={p.port}
                          onClick={() => {
                            setSerialPort(p.port);
                            if (p.description.toLowerCase().includes('px4')) {
                              setBaudRate('115200');
                            } else {
                              setBaudRate('57600');
                            }
                          }}
                          className={`flex items-center justify-between p-2 text-[11px] font-mono cursor-pointer border transition-all ${
                            isSelected
                              ? 'bg-[#FFB000]/15 border-[#FFB000] text-[#FFB000]'
                              : 'bg-[#1D1E24] border-zinc-800 text-zinc-300 hover:border-zinc-600'
                          }`}
                        >
                          <div className="flex items-center gap-2">
                            <span className="font-bold text-white px-1.5 py-0.5 bg-zinc-800 border border-zinc-700 text-[10px]">
                              {p.port}
                            </span>
                            <span className="truncate">{p.description}</span>
                          </div>
                          {isSelected ? (
                            <span className="flex items-center gap-1 text-[9px] font-bold uppercase tracking-wider text-[#FFB000]">
                              <Check size={11} /> SELECTED
                            </span>
                          ) : (
                            <span className="text-[9px] text-zinc-500 hover:text-zinc-300">
                              Click to Select
                            </span>
                          )}
                        </div>
                      );
                    })}
                  </div>
                ) : (
                  <p className="text-[10px] text-zinc-400 py-1 leading-relaxed">
                    No physical COM ports detected on host. Plug in USB cable or Telemetry Radio module and click Scan.
                  </p>
                )}

                <div className="mt-2 text-[9px] text-zinc-400 border-t border-zinc-800/80 pt-1.5">
                  Pixhawk/APM USB → 57600 · PX4 native USB → 115200 · SiK Telemetry Radio → 57600.
                </div>
              </div>
            </div>
          )}

          {/* TAB 2: UDP */}
          {connType === 'udp' && (
            <div className="space-y-3 animate-in fade-in duration-150">
              <div className="grid grid-cols-2 gap-3">
                <div>
                  <label className="block text-[9px] font-bold text-zinc-400 uppercase tracking-widest mb-1">
                    BIND HOST / IP
                  </label>
                  <input
                    type="text"
                    value={udpHost}
                    onChange={e => setUdpHost(e.target.value)}
                    className="w-full bg-[#1A1B20] border border-zinc-700 focus:border-[#FFB000] px-3 py-2 text-xs font-mono text-white outline-none"
                    placeholder="0.0.0.0"
                  />
                </div>
                <div>
                  <label className="block text-[9px] font-bold text-zinc-400 uppercase tracking-widest mb-1">
                    UDP PORT
                  </label>
                  <input
                    type="number"
                    value={udpPort}
                    onChange={e => setUdpPort(e.target.value)}
                    className="w-full bg-[#1A1B20] border border-zinc-700 focus:border-[#FFB000] px-3 py-2 text-xs font-mono text-white outline-none"
                    placeholder="14550"
                  />
                </div>
              </div>
              <div className="p-2.5 border border-zinc-800 bg-[#16171C] text-[10px] text-zinc-400 leading-relaxed">
                Listens for incoming MAVLink packets from QGroundControl, Mission Planner MAVLink mirror, companion computer (Jetson / RPi), or SITL.
              </div>
            </div>
          )}

          {/* TAB 3: TCP */}
          {connType === 'tcp' && (
            <div className="space-y-3 animate-in fade-in duration-150">
              <div className="grid grid-cols-2 gap-3">
                <div>
                  <label className="block text-[9px] font-bold text-zinc-400 uppercase tracking-widest mb-1">
                    TCP SERVER HOST
                  </label>
                  <input
                    type="text"
                    value={tcpHost}
                    onChange={e => setTcpHost(e.target.value)}
                    className="w-full bg-[#1A1B20] border border-zinc-700 focus:border-[#FFB000] px-3 py-2 text-xs font-mono text-white outline-none"
                    placeholder="127.0.0.1"
                  />
                </div>
                <div>
                  <label className="block text-[9px] font-bold text-zinc-400 uppercase tracking-widest mb-1">
                    TCP PORT
                  </label>
                  <input
                    type="number"
                    value={tcpPort}
                    onChange={e => setTcpPort(e.target.value)}
                    className="w-full bg-[#1A1B20] border border-zinc-700 focus:border-[#FFB000] px-3 py-2 text-xs font-mono text-white outline-none"
                    placeholder="5760"
                  />
                </div>
              </div>
              <div className="p-2.5 border border-zinc-800 bg-[#16171C] text-[10px] text-zinc-400 leading-relaxed">
                Connects as a TCP client to ArduPilot SITL (:5760), mavlink-router TCP endpoint, or remote network server.
              </div>
            </div>
          )}

          {/* TAB 4: SIMULATOR */}
          {connType === 'simulator' && (
            <div className="space-y-3 animate-in fade-in duration-150">
              <div>
                <label className="block text-[9px] font-bold text-zinc-400 uppercase tracking-widest mb-1">
                  VEHICLE AIRFRAME
                </label>
                <select
                  value={simVehicle}
                  onChange={e => setSimVehicle(e.target.value)}
                  className="w-full bg-[#1A1B20] border border-zinc-700 focus:border-[#FFB000] px-3 py-2 text-xs font-mono text-white outline-none"
                >
                  <option value="Quadrotor (X-Frame)">Quadrotor (X-Frame)</option>
                  <option value="Hexacopter">Hexacopter</option>
                  <option value="Fixed-Wing VTOL">Fixed-Wing VTOL</option>
                </select>
              </div>
              <div className="p-2.5 border border-zinc-800 bg-[#16171C] text-[10px] text-zinc-400 leading-relaxed">
                Connects to the GCS built-in simulation engine. Generates realistic IMU, LiDAR 2D Occupancy, and GPS-denied FAST-LIO2 telemetry.
              </div>
            </div>
          )}

          {/* Error notice */}
          {connError && (
            <div className="p-3 border border-red-600/80 bg-red-950/50 text-red-300 text-[10px] space-y-2">
              <div className="flex items-center gap-2">
                <AlertTriangle size={15} className="shrink-0 text-red-400" />
                <span className="font-bold">{connError}</span>
              </div>
              <p className="text-zinc-400 text-[9px] leading-relaxed">
                Windows detected a USB Device Descriptor error on this cable/port. You can re-plug the cable into a different port, or force connect in Bench Mode to test telemetry immediately.
              </p>
              <div className="pt-1 flex items-center justify-end">
                <button
                  type="button"
                  onClick={handleForceConnect}
                  disabled={isSubmitting}
                  className="flex items-center gap-1.5 px-3 py-1.5 bg-[#FFB000]/20 hover:bg-[#FFB000]/30 border border-[#FFB000] text-[#FFB000] text-[10px] font-bold uppercase tracking-wider transition-all"
                >
                  <Zap size={11} />
                  Force Connect ({serialPort || 'COM6'} Bench Mode)
                </button>
              </div>
            </div>
          )}

          {/* Action Buttons */}
          <div className="flex items-center gap-3 pt-2">
            <button
              type="button"
              onClick={onClose}
              className="flex-1 py-3 border border-zinc-700 bg-zinc-800/60 hover:bg-zinc-700 text-xs font-mono font-bold tracking-[0.15em] text-zinc-300 uppercase transition-all"
            >
              CANCEL
            </button>

            <button
              type="submit"
              disabled={isSubmitting || hwState === 'CONNECTING'}
              className="flex-[2] flex items-center justify-center gap-2 py-3 text-xs font-mono font-bold tracking-[0.2em] uppercase transition-all text-black disabled:opacity-50"
              style={{
                background: 'linear-gradient(135deg, #FFB000 0%, #FF8C00 100%)',
                boxShadow: '0 0 20px rgba(255,176,0,0.3)',
              }}
            >
              {isSubmitting || hwState === 'CONNECTING' ? (
                <>
                  <Loader2 size={15} className="animate-spin" />
                  CONNECTING DRONE...
                </>
              ) : (
                <>
                  <Wifi size={15} />
                  CONNECT DRONE
                </>
              )}
            </button>
          </div>
        </form>
      </div>
    </div>
  );
}
