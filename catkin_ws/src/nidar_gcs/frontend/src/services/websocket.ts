/**
 * NIDAR GCS WebSocket Service
 * ===========================
 * Manages WebSocket connections to the FastAPI backend.
 * Handles automatic reconnection with exponential backoff.
 * When backend is unavailable, the frontend uses mock providers.
 */

const BASE_WS = import.meta.env.VITE_WS_URL || 'ws://localhost:8000/api/ws';

type MessageHandler = (data: unknown) => void;

// ─── WebSocket Manager ───────────────────────────────────────

class ManagedWebSocket {
  private ws: WebSocket | null = null;
  private handlers: MessageHandler[] = [];
  private reconnectDelay = 500;
  private maxDelay = 30_000;
  private stopped = false;
  private readonly url: string;
  private readonly name: string;
  private reconnectTimer: ReturnType<typeof setTimeout> | null = null;

  constructor(url: string, name: string) {
    this.url = url;
    this.name = name;
  }

  connect(): void {
    if (this.stopped) return;
    try {
      this.ws = new WebSocket(this.url);
      this.ws.onopen = () => {
        console.debug(`[WS] ${this.name} connected`);
        this.reconnectDelay = 500;
      };
      this.ws.onmessage = (ev) => {
        try {
          const data = JSON.parse(ev.data);
          this.handlers.forEach(h => h(data));
        } catch {
          // Non-JSON frame — ignore
        }
      };
      this.ws.onclose = () => {
        if (!this.stopped) this.scheduleReconnect();
      };
      this.ws.onerror = () => {
        this.ws?.close();
      };
    } catch {
      this.scheduleReconnect();
    }
  }

  private scheduleReconnect(): void {
    if (this.stopped) return;
    console.debug(`[WS] ${this.name} reconnecting in ${this.reconnectDelay}ms`);
    this.reconnectTimer = setTimeout(() => {
      this.reconnectDelay = Math.min(this.reconnectDelay * 2, this.maxDelay);
      this.connect();
    }, this.reconnectDelay);
  }

  subscribe(handler: MessageHandler): () => void {
    this.handlers.push(handler);
    return () => {
      this.handlers = this.handlers.filter(h => h !== handler);
    };
  }

  disconnect(): void {
    this.stopped = true;
    if (this.reconnectTimer) clearTimeout(this.reconnectTimer);
    this.ws?.close();
    this.ws = null;
  }

  get isConnected(): boolean {
    return this.ws?.readyState === WebSocket.OPEN;
  }
}

// ─── Singleton channels ────────────────────────────────────

export const telemetryWS = new ManagedWebSocket(`${BASE_WS}/telemetry`, 'Telemetry');
export const mapWS       = new ManagedWebSocket(`${BASE_WS}/map`,       'Map');
export const eventsWS    = new ManagedWebSocket(`${BASE_WS}/events`,    'Events');
export const cameraWS    = new ManagedWebSocket(`${BASE_WS}/camera`,    'Camera');

/**
 * Connect all WebSocket channels to the backend.
 * Call once in App.tsx if backend integration is desired.
 */
export function connectAll(): void {
  telemetryWS.connect();
  mapWS.connect();
  eventsWS.connect();
  cameraWS.connect();
}

/**
 * Disconnect all channels.
 */
export function disconnectAll(): void {
  telemetryWS.disconnect();
  mapWS.disconnect();
  eventsWS.disconnect();
  cameraWS.disconnect();
}
