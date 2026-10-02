import type { AirMouseGesture, AirMouseSettings, RawSensorData } from '@/types/airMouse';

export interface CursorState {
  x: number;
  y: number;
  isDragging: boolean;
  leftButtonDown: boolean;
  rightButtonDown: boolean;
}

export class SensorMotionProcessor {
  private prevYaw: number | null = null;
  private prevPitch: number | null = null;
  private prevRoll: number | null = null;
  private prevTimestamp = 0;

  // Smoothing states
  private smoothedDx = 0;
  private smoothedDy = 0;

  // Current screen cursor position
  private cursorX = typeof window !== 'undefined' ? window.innerWidth / 2 : 500;
  private cursorY = typeof window !== 'undefined' ? window.innerHeight / 2 : 400;

  // Gesture state tracking (edge-triggered)
  private currentGesture: AirMouseGesture = 'IDLE';
  private clickArmed = false;
  private clickCooldownUntil = 0;
  private lastClickTimestamp = 0;
  private isDragging = false;
  private leftDown = false;
  private rightDown = false;

  constructor() {
    this.centerCursor();
  }

  public centerCursor(): void {
    if (typeof window !== 'undefined') {
      this.cursorX = window.innerWidth / 2;
      this.cursorY = window.innerHeight / 2;
    }
    this.prevYaw = null;
    this.prevPitch = null;
    this.prevRoll = null;
    this.smoothedDx = 0;
    this.smoothedDy = 0;
    this.leftDown = false;
    this.rightDown = false;
    this.isDragging = false;
    this.currentGesture = 'IDLE';
  }

  public resetBaseline(): void {
    this.prevYaw = null;
    this.prevPitch = null;
    this.prevRoll = null;
    this.smoothedDx = 0;
    this.smoothedDy = 0;
  }

  /**
   * Process raw sensor packet and compute new cursor position, gestures, and mouse actions.
   */
  public processSensorPacket(
    sensor: RawSensorData,
    settings: AirMouseSettings
  ): {
    cursor: CursorState;
    gesture: AirMouseGesture;
    scrollDeltaY: number;
    clickedLeft: boolean;
    clickedRight: boolean;
    doubleClicked: boolean;
  } {
    const now = Date.now();
    let scrollDeltaY = 0;
    let clickedLeft = false;
    let clickedRight = false;
    let doubleClicked = false;

    // Guard: Validate sensor values
    const yaw = typeof sensor.yaw === 'number' && !isNaN(sensor.yaw) ? sensor.yaw : null;
    const pitch = typeof sensor.pitch === 'number' && !isNaN(sensor.pitch) ? sensor.pitch : null;
    const roll = typeof sensor.roll === 'number' && !isNaN(sensor.roll) ? sensor.roll : null;

    if (yaw === null || pitch === null) {
      return {
        cursor: this.getCursorState(),
        gesture: this.currentGesture,
        scrollDeltaY: 0,
        clickedLeft: false,
        clickedRight: false,
        doubleClicked: false,
      };
    }

    // Initialize baseline on first valid frame
    if (this.prevYaw === null || this.prevPitch === null) {
      this.prevYaw = yaw;
      this.prevPitch = pitch;
      this.prevRoll = roll ?? 0;
      this.prevTimestamp = now;
      return {
        cursor: this.getCursorState(),
        gesture: 'IDLE',
        scrollDeltaY: 0,
        clickedLeft: false,
        clickedRight: false,
        doubleClicked: false,
      };
    }

    // 1. Calculate Angular Deltas and dt
    const dt = this.prevTimestamp > 0 ? Math.min(0.1, (now - this.prevTimestamp) / 1000) : 0.02;
    let rawDeltaYaw = yaw - this.prevYaw;
    let rawDeltaPitch = pitch - this.prevPitch;
    const rawDeltaRoll = (roll ?? 0) - (this.prevRoll ?? 0);

    // Normalize wrap-around (-PI to +PI)
    if (rawDeltaYaw > Math.PI) rawDeltaYaw -= 2 * Math.PI;
    if (rawDeltaYaw < -Math.PI) rawDeltaYaw += 2 * Math.PI;
    if (rawDeltaPitch > Math.PI) rawDeltaPitch -= 2 * Math.PI;
    if (rawDeltaPitch < -Math.PI) rawDeltaPitch += 2 * Math.PI;

    this.prevYaw = yaw;
    this.prevPitch = pitch;
    if (roll !== null) this.prevRoll = roll;

    // 2. Dead-zone Filter
    const filteredYaw = this.applyDeadzone(rawDeltaYaw, settings.deadzone);
    const filteredPitch = this.applyDeadzone(rawDeltaPitch, settings.deadzone);

    // 3. Dynamic Velocity Scaling
    // If the sensor packet includes physical velocity (vx, vy), use it to modulate acceleration
    const speedMagnitude = Math.hypot(sensor.vx ?? 0, sensor.vy ?? 0);
    const accelMultiplier = Math.min(2.5, 1.0 + speedMagnitude * 0.5);

    // 4. Sensitivity & Inversion Mapping
    // Pitch (tilt up -> negative pitch in standard aviation -> cursor up)
    const sensFactor = settings.sensitivity * 120 * accelMultiplier * (dt / 0.02);
    let targetDx = filteredYaw * sensFactor * (settings.invertX ? -1 : 1);
    let targetDy = -filteredPitch * sensFactor * (settings.invertY ? -1 : 1);

    // 5. Exponential Smoothing (Jitter reduction)
    const alpha = Math.max(0.05, Math.min(0.95, 1.0 - settings.smoothing));
    this.smoothedDx += (targetDx - this.smoothedDx) * alpha;
    this.smoothedDy += (targetDy - this.smoothedDy) * alpha;

    // 6. Update Screen Cursor with Boundary Clamping
    if (typeof window !== 'undefined') {
      this.cursorX = Math.max(0, Math.min(window.innerWidth, this.cursorX + this.smoothedDx));
      this.cursorY = Math.max(0, Math.min(window.innerHeight, this.cursorY + this.smoothedDy));
    }

    // 7. Motion Gesture Classification
    const motionMag = Math.hypot(this.smoothedDx, this.smoothedDy);
    let detectedGesture: AirMouseGesture = motionMag > 1.2 ? 'MOVE' : 'IDLE';

    // 8. Gesture Recognition & State Machine (Edge-triggered)
    if (settings.gesturesEnabled && now > this.clickCooldownUntil) {
      // Click Gesture Detection:
      // Sharp downward pitch impulse (> 0.12 rad delta) followed by recovery
      if (rawDeltaPitch < -0.10 && !this.clickArmed) {
        this.clickArmed = true;
      } else if (this.clickArmed && rawDeltaPitch > 0.04) {
        // Pitch returned: trigger left click
        this.clickArmed = false;
        this.clickCooldownUntil = now + 350; // 350ms debounce
        clickedLeft = true;
        detectedGesture = 'LEFT_CLICK';

        // Check for double click
        if (now - this.lastClickTimestamp < 400) {
          doubleClicked = true;
          detectedGesture = 'DOUBLE_CLICK';
        }
        this.lastClickTimestamp = now;
      }

      // Right Click Gesture:
      // Rapid lateral roll tilt impulse (> 0.22 rad)
      if (Math.abs(rawDeltaRoll) > 0.22 && !this.clickArmed) {
        clickedRight = true;
        detectedGesture = 'RIGHT_CLICK';
        this.clickCooldownUntil = now + 400;
      }

      // Scroll Gesture:
      // Sustained roll tilt (> 0.4 rad or ~23 degrees) activates vertical scrolling
      if (roll !== null && Math.abs(roll) > 0.40) {
        const scrollDir = roll > 0 ? 1 : -1;
        scrollDeltaY = scrollDir * settings.scrollSensitivity * 15;
        detectedGesture = scrollDir > 0 ? 'SCROLL_DOWN' : 'SCROLL_UP';
      }
    }

    this.currentGesture = detectedGesture;
    this.prevTimestamp = now;

    return {
      cursor: this.getCursorState(),
      gesture: this.currentGesture,
      scrollDeltaY,
      clickedLeft,
      clickedRight,
      doubleClicked,
    };
  }

  private applyDeadzone(value: number, deadzone: number): number {
    const abs = Math.abs(value);
    if (abs < deadzone) return 0;
    return Math.sign(value) * ((abs - deadzone) / (1 - deadzone));
  }

  public getCursorState(): CursorState {
    return {
      x: this.cursorX,
      y: this.cursorY,
      isDragging: this.isDragging,
      leftButtonDown: this.leftDown,
      rightButtonDown: this.rightDown,
    };
  }
}
