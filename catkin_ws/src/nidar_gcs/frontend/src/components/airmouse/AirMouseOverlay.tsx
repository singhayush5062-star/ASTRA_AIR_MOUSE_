import { useEffect, useRef, useState } from 'react';
import type { AirMouseGesture } from '@/types/airMouse';
import type { CursorState } from '@/utils/sensorProcessing';

interface AirMouseOverlayProps {
  isActive: boolean;
  subscribeCursor: (cb: (cursor: CursorState, gesture: AirMouseGesture) => void) => () => void;
}

export function AirMouseOverlay({ isActive, subscribeCursor }: AirMouseOverlayProps) {
  const cursorRef = useRef<HTMLDivElement | null>(null);
  const ringRef = useRef<HTMLDivElement | null>(null);
  const badgeRef = useRef<HTMLSpanElement | null>(null);

  const [activeGesture, setActiveGesture] = useState<AirMouseGesture>('IDLE');
  const [clickPulse, setClickPulse] = useState(false);

  useEffect(() => {
    if (!isActive) return;

    const unsubscribe = subscribeCursor((cursor, gesture) => {
      // Direct DOM update for zero-latency 60fps movement
      if (cursorRef.current) {
        cursorRef.current.style.transform = `translate3d(${cursor.x}px, ${cursor.y}px, 0)`;
      }

      // If gesture changed, update badge
      if (gesture !== activeGesture) {
        setActiveGesture(gesture);
        if (gesture === 'LEFT_CLICK' || gesture === 'RIGHT_CLICK' || gesture === 'DOUBLE_CLICK') {
          setClickPulse(true);
          setTimeout(() => setClickPulse(false), 250);
        }
      }
    });

    return () => {
      unsubscribe();
    };
  }, [isActive, activeGesture, subscribeCursor]);

  if (!isActive) return null;

  return (
    <div className="pointer-events-none fixed inset-0 z-[9999] overflow-hidden">
      {/* Reticle / AirMouse Cursor */}
      <div
        ref={cursorRef}
        className="absolute top-0 left-0 -ml-4 -mt-4 w-8 h-8 flex items-center justify-center transition-transform duration-[16ms] ease-out will-change-transform"
      >
        {/* Click ripple animation */}
        {clickPulse && (
          <div className="absolute inset-0 rounded-full border-2 border-cyan-400 animate-ping opacity-80" />
        )}

        {/* Outer Ring */}
        <div
          ref={ringRef}
          className={`w-6 h-6 rounded-full border-2 flex items-center justify-center transition-colors ${
            activeGesture === 'LEFT_CLICK' || activeGesture === 'DOUBLE_CLICK'
              ? 'border-cyan-400 bg-cyan-400/20'
              : activeGesture === 'RIGHT_CLICK'
              ? 'border-red-400 bg-red-400/20'
              : activeGesture.includes('SCROLL')
              ? 'border-amber-400 bg-amber-400/20'
              : 'border-[#FFB000] bg-[#FFB000]/10 shadow-[0_0_10px_rgba(255,176,0,0.5)]'
          }`}
        >
          {/* Center Point */}
          <div className="w-1.5 h-1.5 rounded-full bg-white shadow-sm" />
        </div>

        {/* Crosshair ticks */}
        <div className="absolute w-8 h-[1px] bg-[#FFB000]/50" />
        <div className="absolute h-8 w-[1px] bg-[#FFB000]/50" />

        {/* Gesture Badge tooltip */}
        {activeGesture !== 'IDLE' && activeGesture !== 'MOVE' && (
          <span
            ref={badgeRef}
            className="absolute left-7 top-1 px-1.5 py-0.5 rounded text-[9px] font-mono font-bold tracking-wider uppercase whitespace-nowrap bg-black/90 border border-zinc-700 text-cyan-300 shadow-md animate-in fade-in zoom-in-75"
          >
            {activeGesture}
          </span>
        )}
      </div>
    </div>
  );
}
