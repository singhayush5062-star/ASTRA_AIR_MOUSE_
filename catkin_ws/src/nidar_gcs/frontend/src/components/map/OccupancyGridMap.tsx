import { useEffect, useRef } from 'react';
import type { OccupancyGrid, TrajectoryPoint, Survivor } from '@/types';
import type { ArenaGrid } from '@/store';

// ─── Map renderer ─────────────────────────────────────────────

const CELL_COLORS = {
  unknown:    '#0d0d10',
  free:       '#161619',
  occupied:   '#3a3a42',
  explored:   '#1a1a22',
  frontier:   '#332800',
  drone:      '#00F0FF',
  trajectory: '#00F0FF',
  survivor:   '#00FF41',
  start:      '#FFB000',
};

// ─── Build a demo procedural map ─────────────────────────────

export function buildDemoMap(): OccupancyGrid {
  const W = 80, H = 80;
  const data = new Array(W * H).fill(-1); // all unknown initially

  // Carve free corridors
  const carve = (x0: number, y0: number, x1: number, y1: number) => {
    const dx = Math.sign(x1 - x0), dy = Math.sign(y1 - y0);
    let x = x0, y = y0;
    while (x !== x1 || y !== y1) {
      for (let ox = -1; ox <= 1; ox++)
        for (let oy = -1; oy <= 1; oy++) {
          const nx = x + ox, ny = y + oy;
          if (nx >= 0 && nx < W && ny >= 0 && ny < H)
            data[ny * W + nx] = 0;
        }
      if (x !== x1) x += dx;
      else y += dy;
    }
  };

  // Main corridors
  carve(40, 40, 40, 10); carve(40, 10, 60, 10); carve(60, 10, 60, 30);
  carve(40, 40, 20, 40); carve(20, 40, 20, 20); carve(20, 20, 50, 20);
  carve(40, 40, 40, 65); carve(40, 65, 60, 65); carve(60, 65, 60, 50);
  carve(40, 40, 15, 40); carve(15, 40, 15, 60); carve(15, 60, 35, 60);
  carve(50, 20, 50, 40); carve(30, 20, 30, 35);

  // Rooms
  const fillRoom = (cx: number, cy: number, rw: number, rh: number) => {
    for (let x = cx - rw; x <= cx + rw; x++)
      for (let y = cy - rh; y <= cy + rh; y++)
        if (x >= 0 && x < W && y >= 0 && y < H)
          data[y * W + x] = 0;
  };
  fillRoom(60, 10, 5, 5); fillRoom(20, 20, 5, 5);
  fillRoom(60, 65, 5, 5); fillRoom(15, 55, 5, 5);
  fillRoom(50, 32, 4, 4);

  // Mark explored region (near origin)
  for (let i = 0; i < W * H; i++) {
    if (data[i] === 0) {
      const x = i % W, y = Math.floor(i / W);
      const dist = Math.sqrt((x - 40) ** 2 + (y - 40) ** 2);
      if (dist < 18) data[i] = 50; // explored marker
    }
  }

  // Frontier cells (edges of explored)
  for (let i = 0; i < W * H; i++) {
    if (data[i] === 0) {
      const x = i % W, y = Math.floor(i / W);
      const dist = Math.sqrt((x - 40) ** 2 + (y - 40) ** 2);
      if (dist >= 16 && dist < 20) data[i] = 75; // frontier
    }
  }

  // Walls
  for (let i = 0; i < W * H; i++) {
    if (data[i] === -1) {
      const x = i % W, y = Math.floor(i / W);
      let hasNeighborFree = false;
      for (const [dx, dy] of [[-1,0],[1,0],[0,-1],[0,1]]) {
        const nx = x + dx, ny = y + dy;
        if (nx >= 0 && nx < W && ny >= 0 && ny < H && data[ny * W + nx] >= 0)
          hasNeighborFree = true;
      }
      if (hasNeighborFree) data[i] = 100; // wall
    }
  }

  return {
    meta: { width: W, height: H, resolution: 0.2, originX: -8, originY: -8 },
    data,
    timestamp: Date.now(),
  };
}

// ─── Component ────────────────────────────────────────────────

interface OccupancyGridMapProps {
  map: OccupancyGrid | null;
  droneX: number | null;
  droneY: number | null;
  droneYaw: number | null;
  trajectory: TrajectoryPoint[];
  survivors: Survivor[];
  /** Competition grid (A1..): drawn over the map, survivor grid boxes highlighted. */
  grid?: ArenaGrid | null;
  /** Entry/exit marker position (world frame) and label. */
  startPoint?: { x: number; y: number };
  startLabel?: string;
  /** Text under "NO DRONE DATA" while there is no map. */
  emptyHint?: string;
  className?: string;
}

// Survivor hotspot: radius of the heat area drawn around a tagged survivor, metres. The
// detector's 0.75 m association radius plus its measured localisation error (~0.5 m median).
const HOTSPOT_RADIUS_M = 1.0;

function cellLabel(gx: number, gy: number) {
  // Same convention as nidar_map2d/grid_visualizer.py: row letter from y (south to north),
  // column number from x (west to east).
  return `${String.fromCharCode(65 + gy)}${gx + 1}`;
}

export function OccupancyGridMap({
  map, droneX, droneY, droneYaw, trajectory, survivors, grid = null,
  startPoint = { x: 0, y: 0 }, startLabel = 'START',
  emptyHint = 'FAST-LIO2 local occupancy grid offline. Connect physical drone to begin GPS-denied 2D mapping.',
  className = '',
}: OccupancyGridMapProps) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const animRef = useRef<number>(0);
  // Static cells are rendered once per map update into an offscreen layer; only the pulsing
  // frontier cells are redrawn every frame (a 0.05 m arena map has ~118k cells).
  const layerRef = useRef<{ key: string; canvas: HTMLCanvasElement; frontier: number[] } | null>(null);

  // Hooks first, unconditionally: the "no map" early return below used to sit above this
  // effect, which breaks React's hook order the moment a map arrives.
  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas || !map) return;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    // Assigning canvas.width clears the canvas even when the value is unchanged, and this effect
    // re-runs on every telemetry update (10 Hz): only resize when the size really changed.
    const resize = () => {
      if (canvas.width !== canvas.offsetWidth) canvas.width = canvas.offsetWidth;
      if (canvas.height !== canvas.offsetHeight) canvas.height = canvas.offsetHeight;
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
      ctx.fillStyle = '#09090B';
      ctx.fillRect(0, 0, W, H);

      const { meta, data } = map;
      const mW = meta.width, mH = meta.height;

      // Fit map in canvas with padding
      const pad = 20;
      const scaleX = (W - pad * 2) / mW;
      const scaleY = (H - pad * 2) / mH;
      const scale = Math.min(scaleX, scaleY);
      const offX = (W - mW * scale) / 2;
      const offY = (H - mH * scale) / 2;

      // Map world → canvas. North (+Y) is up: grid row 0 is the map's southern edge.
      const toCanvas = (wx: number, wy: number) => ({
        cx: offX + (wx - meta.originX) / meta.resolution * scale,
        cy: offY + (mH - (wy - meta.originY) / meta.resolution) * scale,
      });
      const rowY = (y: number) => offY + (mH - 1 - y) * scale;

      // ── Draw cells ── (cached per map update and canvas size; frontier pulses below)
      const key = `${map.timestamp}|${W}|${H}`;
      let layer = layerRef.current;
      if (!layer || layer.key !== key) {
        const off = document.createElement('canvas');
        off.width = W;
        off.height = H;
        const octx = off.getContext('2d');
        const frontier: number[] = [];
        if (octx) {
          for (let y = 0; y < mH; y++) {
            for (let x = 0; x < mW; x++) {
              const val = data[y * mW + x];
              let color: string;
              if (val === -1) color = CELL_COLORS.unknown;
              else if (val === 100) color = CELL_COLORS.occupied;
              else if (val === 75) { frontier.push(y * mW + x); continue; }
              else if (val === 50) color = CELL_COLORS.explored;
              else color = CELL_COLORS.free;

              octx.fillStyle = color;
              octx.fillRect(
                offX + x * scale,
                rowY(y),
                Math.ceil(scale) + 0.5,
                Math.ceil(scale) + 0.5,
              );
            }
          }
        }
        layer = { key, canvas: off, frontier };
        layerRef.current = layer;
      }
      ctx.drawImage(layer.canvas, 0, 0);
      for (const i of layer.frontier) {
        const x = i % mW, y = Math.floor(i / mW);
        // Frontier — pulsing amber
        const pulse = 0.5 + 0.5 * Math.sin(frame * 0.08 + x * 0.3);
        ctx.fillStyle = `rgba(255,176,0,${0.15 + 0.1 * pulse})`;
        ctx.fillRect(offX + x * scale, rowY(y), Math.ceil(scale) + 0.5, Math.ceil(scale) + 0.5);
      }

      // ── Grid overlay (only if cells large enough) ──
      if (scale > 4) {
        ctx.strokeStyle = 'rgba(27,27,30,0.6)';
        ctx.lineWidth = 0.5;
        for (let x = 0; x <= mW; x += 5) {
          ctx.beginPath();
          ctx.moveTo(offX + x * scale, offY);
          ctx.lineTo(offX + x * scale, offY + mH * scale);
          ctx.stroke();
        }
        for (let y = 0; y <= mH; y += 5) {
          ctx.beginPath();
          ctx.moveTo(offX, offY + y * scale);
          ctx.lineTo(offX + mW * scale, offY + y * scale);
          ctx.stroke();
        }
      }

      // ── Competition grid (A1..) — survivor grid boxes highlighted ──
      if (grid) {
        const tagged = new Set<string>();
        survivors.forEach(s => {
          if (s.position.x === null || s.position.y === null) return;
          const gx = Math.floor((s.position.x - grid.originX) / grid.cellSize);
          const gy = Math.floor((s.position.y - grid.originY) / grid.cellSize);
          if (gx >= 0 && gy >= 0 && gx < grid.cellsX && gy < grid.cellsY) tagged.add(cellLabel(gx, gy));
        });
        for (let gy = 0; gy < grid.cellsY; gy++) {
          for (let gx = 0; gx < grid.cellsX; gx++) {
            const label = cellLabel(gx, gy);
            const nw = toCanvas(grid.originX + gx * grid.cellSize, grid.originY + (gy + 1) * grid.cellSize);
            const se = toCanvas(grid.originX + (gx + 1) * grid.cellSize, grid.originY + gy * grid.cellSize);
            const w = se.cx - nw.cx, h = se.cy - nw.cy;
            const isTagged = tagged.has(label);
            if (isTagged) {
              ctx.fillStyle = 'rgba(0,255,65,0.08)';
              ctx.fillRect(nw.cx, nw.cy, w, h);
              ctx.strokeStyle = 'rgba(0,255,65,0.75)';
              ctx.lineWidth = 1.5;
              ctx.setLineDash([4, 3]);
            } else {
              ctx.strokeStyle = 'rgba(82,82,91,0.5)';
              ctx.lineWidth = 0.75;
            }
            ctx.strokeRect(nw.cx, nw.cy, w, h);
            ctx.setLineDash([]);
            ctx.font = `${isTagged ? 'bold ' : ''}8px JetBrains Mono, monospace`;
            ctx.fillStyle = isTagged ? '#00FF41' : 'rgba(113,113,122,0.85)';
            ctx.textAlign = 'left';
            ctx.fillText(label, nw.cx + 3, nw.cy + 10);
          }
        }
      }

      // ── Trajectory ──
      if (trajectory.length > 1) {
        ctx.beginPath();
        trajectory.forEach((pt, i) => {
          const { cx, cy } = toCanvas(pt.x, pt.y);
          if (i === 0) ctx.moveTo(cx, cy);
          else ctx.lineTo(cx, cy);
        });
        ctx.strokeStyle = 'rgba(0,240,255,0.4)';
        ctx.lineWidth = 1.5;
        ctx.setLineDash([3, 4]);
        ctx.stroke();
        ctx.setLineDash([]);
      }

      // ── Survivors ── (hotspot area, then the original marker)
      survivors.forEach(surv => {
        if (surv.position.x === null || surv.position.y === null) return;
        const { cx, cy } = toCanvas(surv.position.x, surv.position.y);
        const pulse = 0.7 + 0.3 * Math.sin(frame * 0.06 + surv.id);
        const r = HOTSPOT_RADIUS_M / meta.resolution * scale;
        const heat = ctx.createRadialGradient(cx, cy, 0, cx, cy, r);
        heat.addColorStop(0, `rgba(255,0,60,${0.55 * pulse})`);
        heat.addColorStop(0.45, `rgba(255,85,0,${0.3 * pulse})`);
        heat.addColorStop(1, 'rgba(255,176,0,0)');
        ctx.beginPath();
        ctx.arc(cx, cy, r, 0, Math.PI * 2);
        ctx.fillStyle = heat;
        ctx.fill();
        ctx.beginPath();
        ctx.arc(cx, cy, 8 * pulse, 0, Math.PI * 2);
        ctx.fillStyle = `rgba(0,255,65,${0.12 * pulse})`;
        ctx.fill();
        ctx.beginPath();
        ctx.arc(cx, cy, 5, 0, Math.PI * 2);
        ctx.fillStyle = '#00FF41';
        ctx.fill();
        ctx.font = 'bold 8px JetBrains Mono, monospace';
        ctx.fillStyle = '#00FF41';
        ctx.textAlign = 'center';
        ctx.fillText(surv.gridLabel ? `S${surv.id} · ${surv.gridLabel}` : `S${surv.id}`, cx, cy - 10);
      });

      // ── Start/Exit marker ──
      {
        const { cx, cy } = toCanvas(startPoint.x, startPoint.y);
        const pulse = 0.6 + 0.4 * Math.sin(frame * 0.05);
        ctx.beginPath();
        ctx.arc(cx, cy, 10 * pulse, 0, Math.PI * 2);
        ctx.fillStyle = `rgba(255,176,0,${0.1 * pulse})`;
        ctx.fill();
        ctx.beginPath();
        ctx.arc(cx, cy, 5, 0, Math.PI * 2);
        ctx.fillStyle = '#FFB000';
        ctx.fill();
        ctx.font = '7px JetBrains Mono, monospace';
        ctx.fillStyle = '#FFB000';
        ctx.textAlign = 'center';
        ctx.fillText(startLabel, cx, cy - 12);
      }

      // ── Drone (Only drawn if telemetry coordinates exist) ──
      if (droneX !== null && droneY !== null) {
        // canvas y points down, world +Y points up: mirror the heading
        const yaw = -(droneYaw ?? 0);
        const { cx, cy } = toCanvas(droneX, droneY);
        const armL = Math.max(8, scale * 0.6);
        const angles = [yaw, yaw + Math.PI, yaw + Math.PI / 2, yaw - Math.PI / 2];

        // Drone shadow
        ctx.beginPath();
        ctx.arc(cx, cy, armL * 1.3, 0, Math.PI * 2);
        ctx.fillStyle = 'rgba(0,240,255,0.06)';
        ctx.fill();

        // Arms
        angles.forEach(a => {
          ctx.beginPath();
          ctx.moveTo(cx, cy);
          ctx.lineTo(cx + Math.cos(a) * armL, cy + Math.sin(a) * armL);
          ctx.strokeStyle = '#00F0FF';
          ctx.lineWidth = 2;
          ctx.stroke();
        });

        // Rotors
        angles.forEach(a => {
          const rx = cx + Math.cos(a) * armL;
          const ry = cy + Math.sin(a) * armL;
          ctx.beginPath();
          ctx.arc(rx, ry, armL * 0.35, 0, Math.PI * 2);
          ctx.strokeStyle = 'rgba(0,240,255,0.5)';
          ctx.lineWidth = 1.5;
          ctx.stroke();
        });

        // Body
        ctx.beginPath();
        ctx.arc(cx, cy, 4, 0, Math.PI * 2);
        ctx.fillStyle = '#00F0FF';
        ctx.shadowColor = '#00F0FF';
        ctx.shadowBlur = 8;
        ctx.fill();
        ctx.shadowBlur = 0;

        // Direction arrow
        ctx.beginPath();
        ctx.moveTo(cx + Math.cos(yaw) * 14, cy + Math.sin(yaw) * 14);
        ctx.lineTo(cx + Math.cos(yaw + 2.6) * 8, cy + Math.sin(yaw + 2.6) * 8);
        ctx.lineTo(cx + Math.cos(yaw - 2.6) * 8, cy + Math.sin(yaw - 2.6) * 8);
        ctx.closePath();
        ctx.fillStyle = '#00F0FF';
        ctx.fill();
      }

      // ── Scale bar ──
      {
        const barMeters = 2;
        const barPx = (barMeters / meta.resolution) * scale;
        const barX = offX + 10;
        const barY = offY + mH * scale + 8;
        ctx.beginPath();
        ctx.moveTo(barX, barY); ctx.lineTo(barX + barPx, barY);
        ctx.strokeStyle = '#A1A1AA'; ctx.lineWidth = 1.5; ctx.stroke();
        ctx.font = '8px JetBrains Mono, monospace';
        ctx.fillStyle = '#A1A1AA'; ctx.textAlign = 'left';
        ctx.fillText(`${barMeters}m`, barX + barPx + 4, barY + 4);
      }

      // ── Coordinate axes ──
      {
        const org = toCanvas(0, 0);
        const xEnd = toCanvas(1.5, 0);
        const yEnd = toCanvas(0, 1.5);
        ctx.lineWidth = 1.5;
        ctx.beginPath(); ctx.moveTo(org.cx, org.cy); ctx.lineTo(xEnd.cx, xEnd.cy);
        ctx.strokeStyle = '#FF003C'; ctx.stroke();
        ctx.beginPath(); ctx.moveTo(org.cx, org.cy); ctx.lineTo(yEnd.cx, yEnd.cy);
        ctx.strokeStyle = '#00FF41'; ctx.stroke();
        ctx.font = '7px JetBrains Mono, monospace';
        ctx.fillStyle = '#FF003C'; ctx.textAlign = 'left';
        ctx.fillText('+X', xEnd.cx + 2, xEnd.cy + 4);
        ctx.fillStyle = '#00FF41';
        ctx.fillText('+Y', yEnd.cx + 2, yEnd.cy + 4);
      }

      // ── Legend ──
      const legendItems = [
        { color: CELL_COLORS.occupied, label: 'WALL' },
        { color: CELL_COLORS.explored, label: 'EXPLORED' },
        { color: CELL_COLORS.free,     label: 'FREE' },
        { color: '#FFB000',            label: 'FRONTIER' },
        { color: CELL_COLORS.survivor, label: 'SURVIVOR' },
        { color: '#FF5500',            label: 'HOTSPOT' },
        { color: CELL_COLORS.drone,    label: 'DRONE' },
      ];
      const legX = W - 100;
      const legY = H - legendItems.length * 14 - 8;
      ctx.fillStyle = 'rgba(9,9,11,0.8)';
      ctx.fillRect(legX - 6, legY - 10, 100, legendItems.length * 14 + 14);
      legendItems.forEach((item, i) => {
        ctx.fillStyle = item.color;
        ctx.fillRect(legX, legY + i * 14, 8, 8);
        ctx.fillStyle = '#A1A1AA';
        ctx.font = '8px JetBrains Mono, monospace';
        ctx.textAlign = 'left';
        ctx.fillText(item.label, legX + 12, legY + i * 14 + 7);
      });

      animRef.current = requestAnimationFrame(draw);
    };

    animRef.current = requestAnimationFrame(draw);
    return () => { cancelAnimationFrame(animRef.current); ro.disconnect(); };
  }, [map, droneX, droneY, droneYaw, trajectory, survivors, grid, startPoint.x, startPoint.y, startLabel]);

  if (!map) {
    return (
      <div
        className={`relative flex flex-col items-center justify-center h-full w-full select-none ${className}`}
        style={{
          background: '#09090B',
          backgroundImage: `
            linear-gradient(rgba(39,39,42,0.3) 1px, transparent 1px),
            linear-gradient(90deg, rgba(39,39,42,0.3) 1px, transparent 1px)
          `,
          backgroundSize: '40px 40px',
        }}
      >
        <div className="flex flex-col items-center gap-2 text-center p-6 z-10">
          <span className="text-[13px] font-mono tracking-[0.25em] text-muted uppercase font-bold">
            LOCAL MAP
          </span>
          <span className="text-[11px] font-mono tracking-widest text-disabled uppercase" style={{ color: '#FF003C' }}>
            NO DRONE DATA
          </span>
          <p className="text-[9px] font-mono text-disabled max-w-sm mt-1 leading-relaxed">
            {emptyHint}
          </p>
        </div>
      </div>
    );
  }

  return (
    <canvas
      ref={canvasRef}
      className={`w-full h-full ${className}`}
      style={{ display: 'block' }}
    />
  );
}
