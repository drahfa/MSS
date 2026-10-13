// HTML5 Gamepad API Pilot Controller with Dead Zones and Button Actions.

export class GamepadController {
  constructor(config = {}) {
    this.deadZone = config.deadZone ?? 0.08;
    this.invertHeave = config.invertHeave ?? false;
    this.invertSurge = config.invertSurge ?? false;
    this.isConnected = false;
    this.gamepadIndex = null;

    if (typeof window !== 'undefined') {
      window.addEventListener('gamepadconnected', e => {
        this.isConnected = true;
        this.gamepadIndex = e.gamepad.index;
      });
      window.addEventListener('gamepaddisconnected', e => {
        if (this.gamepadIndex === e.gamepad.index) {
          this.isConnected = false;
          this.gamepadIndex = null;
        }
      });
    }
  }

  applyDeadZone(value) {
    if (Math.abs(value) < this.deadZone) return 0;
    const sign = Math.sign(value);
    return sign * ((Math.abs(value) - this.deadZone) / (1.0 - this.deadZone));
  }

  poll() {
    if (typeof navigator === 'undefined' || !navigator.getGamepads) {
      return { active: false, surge: 0, sway: 0, heave: 0, yaw: 0, buttons: {} };
    }

    const gamepads = navigator.getGamepads();
    const gp = (this.gamepadIndex !== null && gamepads[this.gamepadIndex]) ||
               Array.from(gamepads).find(g => g && g.connected);

    if (!gp) {
      this.isConnected = false;
      return { active: false, surge: 0, sway: 0, heave: 0, yaw: 0, buttons: {} };
    }

    this.isConnected = true;
    this.gamepadIndex = gp.index;

    // Standard Gamepad Mapping:
    // Axis 0: Left Stick X (Sway)
    // Axis 1: Left Stick Y (Surge, inverted where -1 is forward)
    // Axis 2: Right Stick X (Yaw)
    // Axis 3: Right Stick Y (Heave optional)
    // Button 6: LT (Heave Down)
    // Button 7: RT (Heave Up)

    const rawSway = gp.axes[0] ?? 0;
    const rawSurge = -(gp.axes[1] ?? 0); // Stick up is negative
    const rawYaw = gp.axes[2] ?? 0;

    // Heave: Triggers RT (up) - LT (down), or Right Stick Y
    let rawHeave = 0;
    if (gp.buttons[7] && gp.buttons[6]) {
      const up = gp.buttons[7].value ?? (gp.buttons[7].pressed ? 1 : 0);
      const down = gp.buttons[6].value ?? (gp.buttons[6].pressed ? 1 : 0);
      rawHeave = down - up; // Down is positive Z in NED
    } else if (gp.axes[3] !== undefined) {
      rawHeave = gp.axes[3];
    }

    const surge = this.applyDeadZone(this.invertSurge ? -rawSurge : rawSurge);
    const sway = this.applyDeadZone(rawSway);
    const heave = this.applyDeadZone(this.invertHeave ? -rawHeave : rawHeave);
    const yaw = this.applyDeadZone(rawYaw);

    return {
      active: true,
      name: gp.id,
      surge,
      sway,
      heave,
      yaw,
      buttons: {
        a: gp.buttons[0]?.pressed ?? false, // Bottom button (Assist)
        b: gp.buttons[1]?.pressed ?? false, // Right button (Reset)
        x: gp.buttons[2]?.pressed ?? false, // Left button (Camera)
        y: gp.buttons[3]?.pressed ?? false, // Top button (Pause)
        lb: gp.buttons[4]?.pressed ?? false,
        rb: gp.buttons[5]?.pressed ?? false,
      },
    };
  }
}
