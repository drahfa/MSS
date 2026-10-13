// Actuator Failure Injection Framework for ROV Thruster Systems.
// Models disabled, degraded, and stuck thruster fault modes with allocator awareness.

export const FaultMode = {
  OK: 'ok',
  OFFLINE: 'offline', // Total loss of power (0% output)
  DEGRADED: 'degraded', // Biofouled / damaged blade (partial efficiency)
  STUCK: 'stuck', // ESC locked / hardover command
};

export class FailureInjector {
  constructor(thrusterCount = 8) {
    this.thrusterCount = thrusterCount;
    this.faults = Array.from({ length: thrusterCount }, () => ({
      mode: FaultMode.OK,
      efficiency: 1.0,
      stuckValue: 0.0,
    }));
  }

  resize(count) {
    this.thrusterCount = count;
    this.faults = Array.from({ length: count }, (_, i) => this.faults[i] || {
      mode: FaultMode.OK,
      efficiency: 1.0,
      stuckValue: 0.0,
    });
  }

  setFault(channelIndex, mode, options = {}) {
    if (channelIndex < 0 || channelIndex >= this.thrusterCount) return;
    this.faults[channelIndex] = {
      mode,
      efficiency: options.efficiency ?? (mode === FaultMode.DEGRADED ? 0.40 : 1.0),
      stuckValue: options.stuckValue ?? (mode === FaultMode.STUCK ? 0.70 : 0.0),
    };
  }

  clearFault(channelIndex) {
    if (channelIndex < 0 || channelIndex >= this.thrusterCount) return;
    this.faults[channelIndex] = {
      mode: FaultMode.OK,
      efficiency: 1.0,
      stuckValue: 0.0,
    };
  }

  clearAllFaults() {
    for (let i = 0; i < this.thrusterCount; i++) {
      this.clearFault(i);
    }
  }

  hasFault(channelIndex) {
    return this.faults[channelIndex]?.mode !== FaultMode.OK;
  }

  hasAnyFaults() {
    return this.faults.some(f => f.mode !== FaultMode.OK);
  }

  /**
   * Applies active fault modifications to raw pilot/autopilot thruster commands.
   * @param {number[]} commands Raw commanded values [-1, 1]
   * @returns {number[]} Modified actuator inputs
   */
  processCommands(commands) {
    return commands.map((cmd, i) => {
      const fault = this.faults[i];
      if (!fault) return cmd;

      switch (fault.mode) {
        case FaultMode.OFFLINE:
          return 0.0;
        case FaultMode.DEGRADED:
          return cmd * fault.efficiency;
        case FaultMode.STUCK:
          return fault.stuckValue;
        case FaultMode.OK:
        default:
          return cmd;
      }
    });
  }

  /**
   * Returns active health weighting vector for fault-tolerant allocation.
   * (0 for offline, fractional for degraded, 1 for healthy).
   */
  getAllocatorWeights() {
    return this.faults.map(f => {
      if (f.mode === FaultMode.OFFLINE) return 0.0;
      if (f.mode === FaultMode.DEGRADED) return f.efficiency;
      return 1.0;
    });
  }
}
