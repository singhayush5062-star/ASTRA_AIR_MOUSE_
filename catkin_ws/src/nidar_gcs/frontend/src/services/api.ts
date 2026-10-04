/**
 * Where the GCS backend is, for every REST call and WebSocket in the UI.
 *
 * When the UI is served by the backend itself (http://<host>:8000, scripts/start_gcs.sh) the
 * backend is the page's own origin -- so the dashboard also works when it is opened from another
 * computer (http://<gcs-laptop-ip>:8000). Under the Vite dev server (:5173) it is
 * localhost:8000. VITE_API_URL / VITE_WS_URL override both.
 */
const devServer = typeof window !== 'undefined' && window.location.port === '5173';

export const API_BASE: string = import.meta.env.VITE_API_URL
  || (devServer || typeof window === 'undefined' ? 'http://localhost:8000' : window.location.origin);

export const WS_BASE: string = import.meta.env.VITE_WS_URL
  || API_BASE.replace(/^http/, 'ws') + '/api/ws';

export const apiUrl = (path: string): string => `${API_BASE}${path}`;
