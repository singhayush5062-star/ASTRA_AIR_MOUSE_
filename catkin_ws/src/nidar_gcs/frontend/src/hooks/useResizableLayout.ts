import { useState, useEffect, useCallback } from 'react';

interface UseResizableLayoutOptions {
  storageKeyPrefix?: string;
  defaultSidebarWidth?: number;
  minSidebarWidth?: number;
  maxSidebarWidth?: number;
  defaultBottomHeight?: number;
  minBottomHeight?: number;
  maxBottomHeight?: number;
  defaultTimelineWidth?: number;
  minTimelineWidth?: number;
  maxTimelineWidth?: number;
  defaultSurvivorsWidth?: number;
  minSurvivorsWidth?: number;
  maxSurvivorsWidth?: number;
}

export function useResizableLayout({
  storageKeyPrefix = 'nidar',
  defaultSidebarWidth = 490,
  minSidebarWidth = 260,
  maxSidebarWidth = 850,
  defaultBottomHeight = 210,
  minBottomHeight = 140,
  maxBottomHeight = 600,
  defaultTimelineWidth = 320,
  minTimelineWidth = 180,
  maxTimelineWidth = 550,
  defaultSurvivorsWidth = 220,
  minSurvivorsWidth = 160,
  maxSurvivorsWidth = 450,
}: UseResizableLayoutOptions = {}) {
  const sidebarStorageKey = `${storageKeyPrefix}_sidebar_width`;
  const bottomStorageKey = `${storageKeyPrefix}_bottom_panel_height`;
  const timelineStorageKey = `${storageKeyPrefix}_timeline_width`;
  const survivorsStorageKey = `${storageKeyPrefix}_survivors_width`;

  // Read saved sizes from localStorage with fallback to defaults
  const [sidebarWidth, setSidebarWidth] = useState<number>(() => {
    if (typeof window !== 'undefined') {
      const saved = localStorage.getItem(sidebarStorageKey);
      if (saved) {
        const val = parseInt(saved, 10);
        if (!isNaN(val) && val >= minSidebarWidth && val <= maxSidebarWidth) {
          return val;
        }
      }
    }
    return defaultSidebarWidth;
  });

  const [bottomHeight, setBottomHeight] = useState<number>(() => {
    if (typeof window !== 'undefined') {
      const saved = localStorage.getItem(bottomStorageKey);
      if (saved) {
        const val = parseInt(saved, 10);
        if (!isNaN(val) && val >= minBottomHeight && val <= maxBottomHeight) {
          return val;
        }
      }
    }
    return defaultBottomHeight;
  });

  const [timelineWidth, setTimelineWidth] = useState<number>(() => {
    if (typeof window !== 'undefined') {
      const saved = localStorage.getItem(timelineStorageKey);
      if (saved) {
        const val = parseInt(saved, 10);
        if (!isNaN(val) && val >= minTimelineWidth && val <= maxTimelineWidth) {
          return val;
        }
      }
    }
    return defaultTimelineWidth;
  });

  const [survivorsWidth, setSurvivorsWidth] = useState<number>(() => {
    if (typeof window !== 'undefined') {
      const saved = localStorage.getItem(survivorsStorageKey);
      if (saved) {
        const val = parseInt(saved, 10);
        if (!isNaN(val) && val >= minSurvivorsWidth && val <= maxSurvivorsWidth) {
          return val;
        }
      }
    }
    return defaultSurvivorsWidth;
  });

  // Dynamic bounds clamping based on current window dimensions
  const getClampedSidebar = useCallback((w: number) => {
    const maxAllowed = typeof window !== 'undefined'
      ? Math.min(maxSidebarWidth, Math.max(minSidebarWidth + 100, window.innerWidth - 320))
      : maxSidebarWidth;
    return Math.max(minSidebarWidth, Math.min(maxAllowed, w));
  }, [maxSidebarWidth, minSidebarWidth]);

  const getClampedBottom = useCallback((h: number) => {
    const maxAllowed = typeof window !== 'undefined'
      ? Math.min(maxBottomHeight, Math.max(minBottomHeight + 50, window.innerHeight - 200))
      : maxBottomHeight;
    return Math.max(minBottomHeight, Math.min(maxAllowed, h));
  }, [maxBottomHeight, minBottomHeight]);

  const getClampedTimeline = useCallback((tw: number) => {
    const maxAllowed = typeof window !== 'undefined'
      ? Math.min(maxTimelineWidth, Math.max(minTimelineWidth + 50, window.innerWidth - 600))
      : maxTimelineWidth;
    return Math.max(minTimelineWidth, Math.min(maxAllowed, tw));
  }, [maxTimelineWidth, minTimelineWidth]);

  const getClampedSurvivors = useCallback((sw: number, currentSidebar = sidebarWidth) => {
    const maxAllowed = Math.min(maxSurvivorsWidth, Math.max(minSurvivorsWidth + 40, currentSidebar - 160));
    return Math.max(minSurvivorsWidth, Math.min(maxAllowed, sw));
  }, [maxSurvivorsWidth, minSurvivorsWidth, sidebarWidth]);

  // Adjust on window resize to ensure no overflow
  useEffect(() => {
    const handleWindowResize = () => {
      setSidebarWidth(prev => {
        const nextSidebar = getClampedSidebar(prev);
        setSurvivorsWidth(currSw => getClampedSurvivors(currSw, nextSidebar));
        return nextSidebar;
      });
      setBottomHeight(prev => getClampedBottom(prev));
      setTimelineWidth(prev => getClampedTimeline(prev));
    };

    window.addEventListener('resize', handleWindowResize);
    return () => window.removeEventListener('resize', handleWindowResize);
  }, [getClampedBottom, getClampedSidebar, getClampedSurvivors, getClampedTimeline]);

  // Handlers for resizing
  // Dragging divider left (delta < 0) -> expands right panel
  // Dragging divider right (delta > 0) -> shrinks right panel
  const handleResizeSidebar = useCallback((delta: number) => {
    setSidebarWidth(prev => {
      const nextSidebar = getClampedSidebar(prev - delta);
      setSurvivorsWidth(currSw => getClampedSurvivors(currSw, nextSidebar));
      return nextSidebar;
    });
  }, [getClampedSidebar, getClampedSurvivors]);

  const handleResizeBottom = useCallback((delta: number) => {
    setBottomHeight(prev => getClampedBottom(prev - delta));
  }, [getClampedBottom]);

  const handleResizeTimeline = useCallback((delta: number) => {
    setTimelineWidth(prev => getClampedTimeline(prev - delta));
  }, [getClampedTimeline]);

  // Dragging divider right (delta > 0) -> expands survivors panel
  // Dragging divider left (delta < 0) -> narrows survivors panel
  const handleResizeSurvivors = useCallback((delta: number) => {
    setSurvivorsWidth(prev => getClampedSurvivors(prev + delta));
  }, [getClampedSurvivors]);

  // Save to localStorage on resize completion
  const handleSaveSidebar = useCallback(() => {
    try {
      localStorage.setItem(sidebarStorageKey, sidebarWidth.toString());
    } catch {
      // Ignore localStorage errors
    }
  }, [sidebarStorageKey, sidebarWidth]);

  const handleSaveBottom = useCallback(() => {
    try {
      localStorage.setItem(bottomStorageKey, bottomHeight.toString());
    } catch {
      // Ignore localStorage errors
    }
  }, [bottomHeight, bottomStorageKey]);

  const handleSaveTimeline = useCallback(() => {
    try {
      localStorage.setItem(timelineStorageKey, timelineWidth.toString());
    } catch {
      // Ignore localStorage errors
    }
  }, [timelineStorageKey, timelineWidth]);

  const handleSaveSurvivors = useCallback(() => {
    try {
      localStorage.setItem(survivorsStorageKey, survivorsWidth.toString());
    } catch {
      // Ignore localStorage errors
    }
  }, [survivorsStorageKey, survivorsWidth]);

  // Reset to default dimensions
  const resetLayout = useCallback(() => {
    setSidebarWidth(defaultSidebarWidth);
    setBottomHeight(defaultBottomHeight);
    setTimelineWidth(defaultTimelineWidth);
    setSurvivorsWidth(defaultSurvivorsWidth);
    try {
      localStorage.removeItem(sidebarStorageKey);
      localStorage.removeItem(bottomStorageKey);
      localStorage.removeItem(timelineStorageKey);
      localStorage.removeItem(survivorsStorageKey);
    } catch {
      // Ignore localStorage errors
    }
  }, [bottomStorageKey, defaultBottomHeight, defaultSidebarWidth, defaultSurvivorsWidth, defaultTimelineWidth, sidebarStorageKey, survivorsStorageKey, timelineStorageKey]);

  return {
    sidebarWidth,
    bottomHeight,
    timelineWidth,
    survivorsWidth,
    handleResizeSidebar,
    handleResizeBottom,
    handleResizeTimeline,
    handleResizeSurvivors,
    handleSaveSidebar,
    handleSaveBottom,
    handleSaveTimeline,
    handleSaveSurvivors,
    resetLayout,
    resetSidebar: () => setSidebarWidth(defaultSidebarWidth),
    resetBottom: () => setBottomHeight(defaultBottomHeight),
    resetTimeline: () => setTimelineWidth(defaultTimelineWidth),
    resetSurvivors: () => setSurvivorsWidth(defaultSurvivorsWidth),
  };
}
