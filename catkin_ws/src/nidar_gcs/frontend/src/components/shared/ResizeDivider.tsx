import { useState, useRef, useCallback } from 'react';

interface ResizeDividerProps {
  direction: 'vertical' | 'horizontal';
  onResize: (delta: number) => void;
  onResizeEnd?: () => void;
  onReset?: () => void;
  accentColor?: string;
  className?: string;
  title?: string;
}

export function ResizeDivider({
  direction,
  onResize,
  onResizeEnd,
  onReset,
  accentColor = '#FFB000',
  className = '',
  title = 'Drag to resize, double-click to reset',
}: ResizeDividerProps) {
  const [isDragging, setIsDragging] = useState(false);
  const [isHovered, setIsHovered] = useState(false);

  const startCoordRef = useRef<number>(0);
  const rafIdRef = useRef<number | null>(null);

  const handlePointerDown = useCallback((e: React.PointerEvent<HTMLDivElement>) => {
    e.preventDefault();
    e.stopPropagation();

    // Capture pointer so fast movements never lose focus
    (e.currentTarget as HTMLElement).setPointerCapture(e.pointerId);

    startCoordRef.current = direction === 'vertical' ? e.clientX : e.clientY;
    setIsDragging(true);

    // Prevent text selection & lock cursor while dragging
    document.body.style.userSelect = 'none';
    document.body.style.cursor = direction === 'vertical' ? 'col-resize' : 'row-resize';
  }, [direction]);

  const handlePointerMove = useCallback((e: React.PointerEvent<HTMLDivElement>) => {
    if (!isDragging) return;

    const currentCoord = direction === 'vertical' ? e.clientX : e.clientY;
    const delta = currentCoord - startCoordRef.current;
    if (delta === 0) return;

    startCoordRef.current = currentCoord;

    if (rafIdRef.current !== null) {
      cancelAnimationFrame(rafIdRef.current);
    }

    rafIdRef.current = requestAnimationFrame(() => {
      onResize(delta);
    });
  }, [direction, isDragging, onResize]);

  const handlePointerUp = useCallback((e: React.PointerEvent<HTMLDivElement>) => {
    if (!isDragging) return;

    try {
      (e.currentTarget as HTMLElement).releasePointerCapture(e.pointerId);
    } catch {
      // Ignore if capture was already released
    }

    setIsDragging(false);
    document.body.style.userSelect = '';
    document.body.style.cursor = '';

    if (rafIdRef.current !== null) {
      cancelAnimationFrame(rafIdRef.current);
      rafIdRef.current = null;
    }

    if (onResizeEnd) {
      onResizeEnd();
    }
  }, [isDragging, onResizeEnd]);

  const handleDoubleClick = useCallback((e: React.MouseEvent) => {
    e.preventDefault();
    e.stopPropagation();
    if (onReset) {
      onReset();
    }
  }, [onReset]);

  const isVertical = direction === 'vertical';

  return (
    <div
      onPointerDown={handlePointerDown}
      onPointerMove={handlePointerMove}
      onPointerUp={handlePointerUp}
      onPointerCancel={handlePointerUp}
      onMouseEnter={() => setIsHovered(true)}
      onMouseLeave={() => setIsHovered(false)}
      onDoubleClick={handleDoubleClick}
      title={title}
      className={`relative select-none flex-shrink-0 transition-colors duration-150 ${
        isVertical ? 'cursor-col-resize w-1 h-full' : 'cursor-row-resize h-1 w-full'
      } ${className}`}
      style={{
        backgroundColor: isDragging
          ? accentColor
          : isHovered
          ? '#3F3F46'
          : '#27272A',
        boxShadow: isDragging ? `0 0 8px ${accentColor}80` : 'none',
        zIndex: isDragging ? 40 : 20,
      }}
    >
      {/* Expanded invisible hit area (8px total) for effortless mouse grabbing */}
      <div
        className={`absolute ${
          isVertical
            ? '-inset-x-1.5 inset-y-0 cursor-col-resize'
            : '-inset-y-1.5 inset-x-0 cursor-row-resize'
        }`}
      />
    </div>
  );
}
