import { useState, useEffect, useRef, useCallback } from 'react';
import type {
  AirMouseConnectionState,
  AirMouseGesture,
  AirMouseSettings,
  AirMouseDiagnostics,
  RawSensorData,
  AirMouseBackendPayload,
} from '@/types/airMouse';
import { SensorMotionProcessor, type CursorState } from '@/utils/sensorProcessing';

const DEFAULT_SETTINGS: AirMouseSettings = {
  sensitivity: 8,
  smoothing: 0.35,
  deadzone: 0.005,
  scrollSensitivity: 4,
  gesturesEnabled: true,
  invertX: false,
  invertY: false,
};

export function useAirMouse(autoStart = false) {
  const [isActive, setIsActive] = useState(autoStart);
  const [connectionState, setConnectionState] = useState<AirMouseConnectionState>('DISCONNECTED');
  const [connectionError, setConnectionError] = useState<string | null>(null);

  // Settings
  const [settings, setSettings] = useState<AirMouseSettings>(DEFAULT_SETTINGS);

  // Throttled UI Diagnostics state
  const [diagnostics, setDiagnostics] = useState<AirMouseDiagnostics>({
    connectionState: 'DISCONNECTED',
    connectionError: null,
    packetsReceived: 0,
    packetsPerSecond: 0,
    latencyMs: 0,
    lastPacketTimestamp: 0,
    currentGesture: 'IDLE',
    cursorX: typeof window !== 'undefined' ? window.innerWidth / 2 : 500,
    cursorY: typeof window !== 'undefined' ? window.innerHeight / 2 : 400,
    isDragging: false,
    leftButtonDown: false,
    rightButtonDown: false,
  });

  // Latest Raw Sensor Data for dashboard displays
  const [rawSensor, setRawSensor] = useState<RawSensorData>({
    x: 0,
    y: 0,
    z: 0,
    roll: 0,
    pitch: 0,
    yaw: 0,
    vx: 0,
    vy: 0,
    vz: 0,
    voltage: null,
    current: null,
    batteryPercentage: null,
    timestamp: 0,
  });

  // Refs for High-Frequency Sensor Processing (no React re-render lag)
  const processorRef = useRef<SensorMotionProcessor>(new SensorMotionProcessor());
  const wsRef = useRef<WebSocket | null>(null);
  const reconnectTimeoutRef = useRef<ReturnType<typeof setTimeout> | null>(null);
  const staleCheckTimerRef = useRef<ReturnType<typeof setInterval> | null>(null);

  // Performance metrics tracking refs
  const packetCountRef = useRef(0);
  const lastSecPacketsRef = useRef(0);
  const lastPacketTimeRef = useRef(Date.now());
  const lastUiUpdateRef = useRef(0);
  const settingsRef = useRef(settings);
  settingsRef.current = settings;

  const latestCursorRef = useRef<CursorState>({
    x: typeof window !== 'undefined' ? window.innerWidth / 2 : 500,
    y: typeof window !== 'undefined' ? window.innerHeight / 2 : 400,
    isDragging: false,
    leftButtonDown: false,
    rightButtonDown: false,
  });

  // Registered DOM cursor callback
  const cursorListenerRef = useRef<((cursor: CursorState, gesture: AirMouseGesture) => void) | null>(null);

  const subscribeCursor = useCallback((cb: (cursor: CursorState, gesture: AirMouseGesture) => void) => {
    cursorListenerRef.current = cb;
    return () => {
      if (cursorListenerRef.current === cb) {
        cursorListenerRef.current = null;
      }
    };
  }, []);

  // Update Settings
  const updateSettings = useCallback((newSettings: Partial<AirMouseSettings>) => {
    setSettings(prev => ({ ...prev, ...newSettings }));
  }, []);

  // Reset / Center Cursor
  const centerCursor = useCallback(() => {
    processorRef.current.centerCursor();
    const centered = processorRef.current.getCursorState();
    latestCursorRef.current = centered;
    if (cursorListenerRef.current) {
      cursorListenerRef.current(centered, 'IDLE');
    }
  }, []);

  // Handle incoming validated sensor payload
  const handleIncomingMessage = useCallback((rawData: unknown) => {
    if (!rawData || typeof rawData !== 'object') return;
    const packet = rawData as AirMouseBackendPayload;

    const now = Date.now();
    packetCountRef.current += 1;
    lastPacketTimeRef.current = now;

    // Calculate packet latency if timestamp present
    const packetTimestampSec = packet.timestamp || packet.payload?.drone?.timestamp || 0;
    const latencyMs = packetTimestampSec > 0 ? Math.max(0, Math.round(now - packetTimestampSec * 1000)) : 15;

    // Extract drone telemetry
    const drone = packet.payload?.drone;
    if (!drone) return;

    const sensor: RawSensorData = {
      x: drone.position?.x ?? null,
      y: drone.position?.y ?? null,
      z: drone.position?.z ?? null,
      roll: drone.attitude?.roll ?? null,
      pitch: drone.attitude?.pitch ?? null,
      yaw: drone.attitude?.yaw ?? null,
      vx: drone.velocity?.x ?? null,
      vy: drone.velocity?.y ?? null,
      vz: drone.velocity?.z ?? null,
      voltage: drone.battery?.voltage ?? null,
      current: drone.battery?.current ?? null,
      batteryPercentage: drone.battery?.percentage ?? null,
      timestamp: packetTimestampSec || now / 1000,
    };

    // 1. High-frequency Motion Processing
    const result = processorRef.current.processSensorPacket(sensor, settingsRef.current);
    latestCursorRef.current = result.cursor;

    // Notify cursor overlay directly via RAF/callback
    if (cursorListenerRef.current) {
      cursorListenerRef.current(result.cursor, result.gesture);
    }

    // 2. Dispatch Interactive Browser Actions (Scroll & Click)
    if (result.scrollDeltaY !== 0 && typeof window !== 'undefined') {
      window.scrollBy({ top: result.scrollDeltaY, behavior: 'auto' });
    }

    if (result.clickedLeft && typeof document !== 'undefined') {
      const targetElem = document.elementFromPoint(result.cursor.x, result.cursor.y);
      if (targetElem) {
        targetElem.dispatchEvent(
          new MouseEvent('click', {
            bubbles: true,
            cancelable: true,
            clientX: result.cursor.x,
            clientY: result.cursor.y,
          })
        );
      }
    }

    if (result.clickedRight && typeof document !== 'undefined') {
      const targetElem = document.elementFromPoint(result.cursor.x, result.cursor.y);
      if (targetElem) {
        targetElem.dispatchEvent(
          new MouseEvent('contextmenu', {
            bubbles: true,
            cancelable: true,
            clientX: result.cursor.x,
            clientY: result.cursor.y,
          })
        );
      }
    }

    // 3. Throttled UI State Update (~4 Hz max) to keep React silky smooth
    if (now - lastUiUpdateRef.current > 250) {
      lastUiUpdateRef.current = now;
      setRawSensor(sensor);
      setDiagnostics(prev => ({
        ...prev,
        packetsReceived: packetCountRef.current,
        latencyMs,
        lastPacketTimestamp: now,
        currentGesture: result.gesture,
        cursorX: Math.round(result.cursor.x),
        cursorY: Math.round(result.cursor.y),
        isDragging: result.cursor.isDragging,
        leftButtonDown: result.cursor.leftButtonDown,
        rightButtonDown: result.cursor.rightButtonDown,
      }));
    }
  }, []);

  // Connect WebSocket
  const connectWebSocket = useCallback(() => {
    if (wsRef.current && (wsRef.current.readyState === WebSocket.OPEN || wsRef.current.readyState === WebSocket.CONNECTING)) {
      return;
    }

    const wsUrl = (import.meta.env.VITE_WS_URL || 'ws://localhost:8000/api/ws') + '/hardware';
    setConnectionState('CONNECTING');
    setConnectionError(null);

    try {
      const ws = new WebSocket(wsUrl);
      wsRef.current = ws;

      ws.onopen = () => {
        setConnectionState('CONNECTED');
        setConnectionError(null);
        processorRef.current.resetBaseline();
      };

      ws.onmessage = (event) => {
        try {
          const parsed = JSON.parse(event.data);
          handleIncomingMessage(parsed);
        } catch (err) {
          console.warn('[AirMouse WS] Malformed packet received:', err);
        }
      };

      ws.onerror = () => {
        setConnectionState('ERROR');
        setConnectionError('WebSocket link error');
      };

      ws.onclose = () => {
        wsRef.current = null;
        if (isActive) {
          setConnectionState('RECONNECTING');
          reconnectTimeoutRef.current = setTimeout(() => {
            connectWebSocket();
          }, 1500);
        } else {
          setConnectionState('DISCONNECTED');
        }
      };
    } catch (err: any) {
      setConnectionState('ERROR');
      setConnectionError(err.message || 'Failed to initialize WebSocket');
    }
  }, [handleIncomingMessage, isActive]);

  // Clean disconnect
  const disconnectWebSocket = useCallback(() => {
    if (reconnectTimeoutRef.current) {
      clearTimeout(reconnectTimeoutRef.current);
      reconnectTimeoutRef.current = null;
    }
    if (wsRef.current) {
      wsRef.current.close();
      wsRef.current = null;
    }
    processorRef.current.centerCursor();
    setConnectionState('DISCONNECTED');
  }, []);

  // Lifecycle control: Start / Stop AirMouse
  const startAirMouse = useCallback(() => {
    setIsActive(true);
    connectWebSocket();
  }, [connectWebSocket]);

  const stopAirMouse = useCallback(() => {
    setIsActive(false);
    disconnectWebSocket();
  }, [disconnectWebSocket]);

  // Stale connection detector & 1 Hz Packets/sec counter
  useEffect(() => {
    if (!isActive) return;

    staleCheckTimerRef.current = setInterval(() => {
      const now = Date.now();
      const pps = packetCountRef.current - lastSecPacketsRef.current;
      lastSecPacketsRef.current = packetCountRef.current;

      setDiagnostics(prev => ({
        ...prev,
        packetsPerSecond: Math.max(0, pps),
        connectionState: (now - lastPacketTimeRef.current > 2500 && prev.connectionState === 'CONNECTED')
          ? 'STALE'
          : prev.connectionState,
      }));
    }, 1000);

    return () => {
      if (staleCheckTimerRef.current) {
        clearInterval(staleCheckTimerRef.current);
        staleCheckTimerRef.current = null;
      }
    };
  }, [isActive]);

  // Sync connection state to diagnostics
  useEffect(() => {
    setDiagnostics(prev => ({
      ...prev,
      connectionState,
      connectionError,
    }));
  }, [connectionState, connectionError]);

  // Auto-connect when active changes
  useEffect(() => {
    if (isActive) {
      connectWebSocket();
    } else {
      disconnectWebSocket();
    }
    return () => {
      disconnectWebSocket();
    };
  }, [isActive, connectWebSocket, disconnectWebSocket]);

  return {
    isActive,
    startAirMouse,
    stopAirMouse,
    connectionState,
    connectionError,
    settings,
    updateSettings,
    centerCursor,
    diagnostics,
    rawSensor,
    subscribeCursor,
  };
}
