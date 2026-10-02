import { useEffect, useRef } from 'react';
import type { DroneState } from '@/types';

// ─── Maze geometry ─────────────────────────────────────────────
// Walls are defined as [x1, y1, x2, y2] in world-space meters (15×15 arena)

const WALLS: [number, number, number, number][] = [
  // Outer boundary
  [0, 0, 15, 0], [15, 0, 15, 15], [15, 15, 0, 15], [0, 15, 0, 0],
  // Internal corridors
  [3, 0, 3, 5],  [3, 5, 6, 5],   [6, 5, 6, 3],   [6, 3, 9, 3],
  [9, 3, 9, 7],  [9, 7, 12, 7],  [12, 7, 12, 4],
  [3, 8, 3, 12], [3, 12, 7, 12], [7, 12, 7, 9],   [7, 9, 11, 9],
  [11, 9, 11, 13],[11, 13, 8, 13],[5, 6, 5, 8],    [5, 8, 8, 8],
  [8, 8, 8, 6],  [8, 6, 6, 6],
  // Room dividers
  [1, 1, 3, 1],  [1, 1, 1, 4],   [1, 4, 3, 4],
  [12, 1, 14, 1],[12, 1, 12, 3], [12, 3, 14, 3],  [14, 1, 14, 3],
  [12, 11, 14, 11],[12,11,12,14],[12,14,14,14],    [14,11,14,14],
];

// Rooms defined as [x, y, w, h] filled areas
const ROOMS: [number, number, number, number][] = [
  [1, 1, 2, 3],
  [12, 1, 2, 2],
  [12, 11, 2, 3],
  [1, 10, 2, 4],
];

// Entry/exit marker
const ENTRY = { x: 7.5, y: 0.3 };

// ─── Isometric projection ─────────────────────────────────────

interface IsoCtx {
  canvas: HTMLCanvasElement;
  ctx: CanvasRenderingContext2D;
  cx: number;   // canvas center x
  cy: number;   // canvas center y
  scale: number;
  tileW: number;
  tileH: number;
}

function worldToIso(wx: number, wy: number, iz: IsoCtx) {
  const rx = wx - 7.5;
  const ry = wy - 7.5;
  const iso_x = (rx - ry) * iz.tileW * 0.5;
  const iso_y = (rx + ry) * iz.tileH * 0.25;
  return { x: iz.cx + iso_x, y: iz.cy + iso_y };
}

function isoTile(ctx: CanvasRenderingContext2D, iz: IsoCtx,
  wx: number, wy: number, size: number,
  topColor: string, leftColor: string, rightColor: string, height = 0.2) {

  const corners = [
    worldToIso(wx,        wy,        iz),
    worldToIso(wx + size, wy,        iz),
    worldToIso(wx + size, wy + size, iz),
    worldToIso(wx,        wy + size, iz),
  ];
  const top_h = height * iz.tileH * 0.5;

  // Top face
  ctx.beginPath();
  corners.forEach((c, i) => i === 0 ? ctx.moveTo(c.x, c.y - top_h) : ctx.lineTo(c.x, c.y - top_h));
  ctx.closePath();
  ctx.fillStyle = topColor;
  ctx.fill();

  // Left face
  ctx.beginPath();
  ctx.moveTo(corners[3].x, corners[3].y - top_h);
  ctx.lineTo(corners[2].x, corners[2].y - top_h);
  ctx.lineTo(corners[2].x, corners[2].y);
  ctx.lineTo(corners[3].x, corners[3].y);
  ctx.closePath();
  ctx.fillStyle = leftColor;
  ctx.fill();

  // Right face
  ctx.beginPath();
  ctx.moveTo(corners[1].x, corners[1].y - top_h);
  ctx.lineTo(corners[2].x, corners[2].y - top_h);
  ctx.lineTo(corners[2].x, corners[2].y);
  ctx.lineTo(corners[1].x, corners[1].y);
  ctx.closePath();
  ctx.fillStyle = rightColor;
  ctx.fill();
}

// ─── Main component ───────────────────────────────────────────

interface Sim3DViewProps {
  drone: DroneState;
  trajectory?: { x: number; y: number }[];
  className?: string;
}

export function Sim3DView({ drone, trajectory = [], className = '' }: Sim3DViewProps) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const animRef = useRef<number>(0);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    const resize = () => {
      canvas.width = canvas.offsetWidth;
      canvas.height = canvas.offsetHeight;
    };
    resize();
    const ro = new ResizeObserver(resize);
    ro.observe(canvas);

    let frame = 0;

    const draw = () => {
      frame++;
      const W = canvas.width;
      const H = canvas.height;
      ctx.clearRect(0, 0, W, H);

      // Background
      ctx.fillStyle = '#09090B';
      ctx.fillRect(0, 0, W, H);

      const scale = Math.min(W, H) / 22;
      const iz: IsoCtx = {
        canvas, ctx,
        cx: W * 0.5,
        cy: H * 0.38,
        scale,
        tileW: scale * 2,
        tileH: scale,
      };

      // ── Floor grid ──
      for (let gx = 0; gx < 15; gx++) {
        for (let gy = 0; gy < 15; gy++) {
          const p0 = worldToIso(gx, gy, iz);
          const p1 = worldToIso(gx + 1, gy, iz);
          const p2 = worldToIso(gx + 1, gy + 1, iz);
          const p3 = worldToIso(gx, gy + 1, iz);
          ctx.beginPath();
          ctx.moveTo(p0.x, p0.y);
          ctx.lineTo(p1.x, p1.y);
          ctx.lineTo(p2.x, p2.y);
          ctx.lineTo(p3.x, p3.y);
          ctx.closePath();
          ctx.fillStyle = '#111113';
          ctx.fill();
          ctx.strokeStyle = '#1c1c1e';
          ctx.lineWidth = 0.5;
          ctx.stroke();
        }
      }

      // ── Rooms (floor highlights) ──
      ROOMS.forEach(([rx, ry, rw, rh]) => {
        for (let dx = 0; dx < rw; dx++) {
          for (let dy = 0; dy < rh; dy++) {
            const p0 = worldToIso(rx + dx, ry + dy, iz);
            const p1 = worldToIso(rx + dx + 1, ry + dy, iz);
            const p2 = worldToIso(rx + dx + 1, ry + dy + 1, iz);
            const p3 = worldToIso(rx + dx, ry + dy + 1, iz);
            ctx.beginPath();
            ctx.moveTo(p0.x, p0.y); ctx.lineTo(p1.x, p1.y);
            ctx.lineTo(p2.x, p2.y); ctx.lineTo(p3.x, p3.y);
            ctx.closePath();
            ctx.fillStyle = '#14141a';
            ctx.fill();
          }
        }
      });

      // ── Walls ──
      WALLS.forEach(([x1, y1, x2, y2]) => {
        const dx = x2 - x1;
        const dy = y2 - y1;
        if (Math.abs(dx) > Math.abs(dy)) {
          // Horizontal wall → draw as isometric block along X
          const len = Math.abs(dx);
          const startX = dx > 0 ? x1 : x2;
          const wY = y1;
          for (let i = 0; i < len; i++) {
            isoTile(ctx, iz, startX + i, wY - 0.25, 1,
              '#2a2a2f', '#1c1c22', '#232328', 3);
          }
        } else {
          // Vertical wall → draw along Y
          const len = Math.abs(dy);
          const startY = dy > 0 ? y1 : y2;
          const wX = x1;
          for (let i = 0; i < len; i++) {
            isoTile(ctx, iz, wX - 0.25, startY + i, 1,
              '#2a2a2f', '#1c1c22', '#232328', 3);
          }
        }
      });

      // ── Entry/Exit marker ──
      {
        const ep = worldToIso(ENTRY.x, ENTRY.y, iz);
        const pulse = 0.6 + 0.4 * Math.sin(frame * 0.05);
        ctx.beginPath();
        ctx.arc(ep.x, ep.y, 8 * pulse, 0, Math.PI * 2);
        ctx.fillStyle = `rgba(255,176,0,${0.15 * pulse})`;
        ctx.fill();
        ctx.beginPath();
        ctx.arc(ep.x, ep.y, 4, 0, Math.PI * 2);
        ctx.fillStyle = '#FFB000';
        ctx.fill();
        ctx.font = `bold ${scale * 0.4}px JetBrains Mono, monospace`;
        ctx.fillStyle = '#FFB000';
        ctx.textAlign = 'center';
        ctx.fillText('EXIT', ep.x, ep.y - 14);
      }

      // ── Trajectory ──
      if (trajectory.length > 1) {
        ctx.beginPath();
        trajectory.forEach((pt, i) => {
          const wx = pt.x + 7.5;
          const wy = pt.y + 7.5;
          const p = worldToIso(wx, wy, iz);
          if (i === 0) ctx.moveTo(p.x, p.y);
          else ctx.lineTo(p.x, p.y);
        });
        ctx.strokeStyle = 'rgba(0,240,255,0.35)';
        ctx.lineWidth = 1.5;
        ctx.setLineDash([3, 4]);
        ctx.stroke();
        ctx.setLineDash([]);
      }

      // ── Drone ──
      {
        const dwx = (drone.position.x ?? 0) + 7.5;
        const dwy = (drone.position.y ?? 0) + 7.5;
        const dz  = drone.position.z ?? 0;
        const dp = worldToIso(dwx, dwy, iz);
        const screenZ = dz * iz.tileH * 0.5;

        // Shadow
        ctx.beginPath();
        ctx.ellipse(dp.x, dp.y, 10, 5, 0, 0, Math.PI * 2);
        ctx.fillStyle = 'rgba(0,240,255,0.08)';
        ctx.fill();

        // Vertical line
        ctx.beginPath();
        ctx.moveTo(dp.x, dp.y);
        ctx.lineTo(dp.x, dp.y - screenZ);
        ctx.strokeStyle = 'rgba(0,240,255,0.25)';
        ctx.lineWidth = 1;
        ctx.stroke();

        // Drone body at altitude
        const dx = dp.x;
        const dy = dp.y - screenZ;

        // Orientation indicator from yaw
        const yaw = drone.attitude.yaw ?? 0;
        const armL = scale * 0.45;
        const armAngles = [yaw, yaw + Math.PI, yaw + Math.PI / 2, yaw - Math.PI / 2];

        // Arms
        armAngles.forEach(angle => {
          ctx.beginPath();
          ctx.moveTo(dx, dy);
          ctx.lineTo(dx + Math.cos(angle) * armL, dy + Math.sin(angle) * armL * 0.5);
          ctx.strokeStyle = '#00F0FF';
          ctx.lineWidth = 1.5;
          ctx.stroke();
        });

        // Rotors
        armAngles.forEach(angle => {
          const rx = dx + Math.cos(angle) * armL;
          const ry = dy + Math.sin(angle) * armL * 0.5;
          const rotorPhase = (frame * 0.3 + angle) % (Math.PI * 2);
          ctx.beginPath();
          ctx.ellipse(rx, ry, scale * 0.18 * Math.abs(Math.cos(rotorPhase)) + 1, scale * 0.08, 0, 0, Math.PI * 2);
          ctx.strokeStyle = 'rgba(0,240,255,0.6)';
          ctx.lineWidth = 1;
          ctx.stroke();
        });

        // Body center
        const bodyPulse = 0.8 + 0.2 * Math.sin(frame * 0.1);
        ctx.beginPath();
        ctx.arc(dx, dy, scale * 0.1 * bodyPulse, 0, Math.PI * 2);
        ctx.fillStyle = '#00F0FF';
        ctx.fill();

        // LiDAR scan ring
        const scanProgress = (frame * 0.04) % (Math.PI * 2);
        ctx.beginPath();
        ctx.arc(dx, dy, scale * 0.7, scanProgress, scanProgress + Math.PI * 0.6);
        ctx.strokeStyle = 'rgba(0,240,255,0.2)';
        ctx.lineWidth = 2;
        ctx.stroke();

        // Position label
        ctx.font = `${scale * 0.35}px JetBrains Mono, monospace`;
        ctx.fillStyle = '#00F0FF';
        ctx.textAlign = 'center';
        const posXStr = drone.position.x !== null ? drone.position.x.toFixed(1) : '--';
        const posYStr = drone.position.y !== null ? drone.position.y.toFixed(1) : '--';
        ctx.fillText(
          `X:${posXStr} Y:${posYStr}`,
          dx, dy - scale * 0.9
        );
      }

      // ── Coordinate axes ──
      {
        const orig = worldToIso(7.5, 7.5, iz);
        ctx.font = `${scale * 0.3}px JetBrains Mono, monospace`;

        // X axis (red)
        const xEnd = worldToIso(9.5, 7.5, iz);
        ctx.beginPath(); ctx.moveTo(orig.x, orig.y); ctx.lineTo(xEnd.x, xEnd.y);
        ctx.strokeStyle = '#FF003C'; ctx.lineWidth = 1.5; ctx.stroke();
        ctx.fillStyle = '#FF003C'; ctx.textAlign = 'center';
        ctx.fillText('+X', xEnd.x, xEnd.y - 4);

        // Y axis (green)
        const yEnd = worldToIso(7.5, 9.5, iz);
        ctx.beginPath(); ctx.moveTo(orig.x, orig.y); ctx.lineTo(yEnd.x, yEnd.y);
        ctx.strokeStyle = '#00FF41'; ctx.stroke();
        ctx.fillStyle = '#00FF41';
        ctx.fillText('+Y', yEnd.x + 4, yEnd.y);
      }

      // ── HUD overlay ──
      ctx.font = `${scale * 0.3}px JetBrains Mono, monospace`;
      ctx.fillStyle = '#52525B';
      ctx.textAlign = 'left';
      ctx.fillText('GAZEBO SIMULATION  |  SCENARIO_01  |  15m × 15m', 10, H - 10);
      ctx.textAlign = 'right';
      const posZStr = drone.position.z !== null ? `${drone.position.z.toFixed(2)} m` : '-- m';
      const yawStr = drone.attitude.yaw !== null ? `${(drone.attitude.yaw * 180 / Math.PI).toFixed(1)}°` : '--°';
      ctx.fillText(`Z: ${posZStr}  |  YAW: ${yawStr}`, W - 10, H - 10);

      animRef.current = requestAnimationFrame(draw);
    };

    animRef.current = requestAnimationFrame(draw);
    return () => {
      cancelAnimationFrame(animRef.current);
      ro.disconnect();
    };
  }, [drone, trajectory]);

  return (
    <canvas
      ref={canvasRef}
      className={`w-full h-full ${className}`}
      style={{ display: 'block' }}
    />
  );
}
