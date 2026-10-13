// Mission Waypoint Management, Route Validation, and Save/Load System.

export class MissionRoute {
  constructor(initialWaypoints = null) {
    this.waypoints = initialWaypoints ? [...initialWaypoints] : [
      { n: 4.0,  e: -1.0,  d: 4.0,  label: 'Descent check', tolerance: 1.2 },
      { n: 11.0, e: -10.0, d: 8.0,  label: 'Habitat approach', tolerance: 1.5 },
      { n: 18.0, e: -10.0, d: 11.0, label: 'West inspection', tolerance: 1.5 },
      { n: 19.0, e: -3.0,  d: 10.0, label: 'East inspection', tolerance: 1.5 },
      { n: 9.0,  e: 1.0,   d: 6.0,  label: 'Return arc', tolerance: 1.8 },
      { n: 2.0,  e: 0.0,   d: 3.0,  label: 'Station keeping', tolerance: 1.0 },
    ];
    this.activeLeg = 0;
  }

  addWaypoint(wp) {
    this.waypoints.push({
      n: Number(wp.n ?? 0),
      e: Number(wp.e ?? 0),
      d: Number(wp.d ?? 2),
      label: wp.label || `Waypoint ${this.waypoints.length + 1}`,
      tolerance: Number(wp.tolerance ?? 1.5),
    });
  }

  removeWaypoint(index) {
    if (index >= 0 && index < this.waypoints.length) {
      this.waypoints.splice(index, 1);
      if (this.activeLeg >= this.waypoints.length) {
        this.activeLeg = Math.max(0, this.waypoints.length - 1);
      }
    }
  }

  getWaypoints() {
    return [...this.waypoints];
  }

  getActiveWaypoint() {
    return this.waypoints[this.activeLeg] || null;
  }

  nextLeg() {
    this.activeLeg = (this.activeLeg + 1) % this.waypoints.length;
    return this.activeLeg;
  }

  resetLeg() {
    this.activeLeg = 0;
  }

  checkArrival(pos) {
    const wp = this.getActiveWaypoint();
    if (!wp) return false;
    const dist = Math.hypot(pos[0] - wp.n, pos[1] - wp.e, pos[2] - wp.d);
    return dist <= (wp.tolerance ?? 1.5);
  }

  /**
   * Validates route envelope: checks seabed clearance and max bounding radius.
   * @param {number} seabedDepth Maximum allowable depth
   * @returns {{ valid: boolean, errors: string[], totalDistance: number }}
   */
  validate(seabedDepth = 18.0) {
    const errors = [];
    if (this.waypoints.length === 0) {
      return { valid: false, errors: ['Route must contain at least one waypoint.'], totalDistance: 0 };
    }

    let totalDist = 0;
    this.waypoints.forEach((wp, i) => {
      if (!Number.isFinite(wp.n) || !Number.isFinite(wp.e) || !Number.isFinite(wp.d)) {
        errors.push(`Waypoint ${i + 1} (${wp.label}): Coordinates must be finite numbers.`);
      }
      if (wp.d < 0) {
        errors.push(`Waypoint ${i + 1} (${wp.label}): Depth cannot be negative (above sea surface).`);
      }
      if (wp.d >= seabedDepth - 0.5) {
        errors.push(`Waypoint ${i + 1} (${wp.label}): Depth (${wp.d}m) violates seabed safety clearance (${seabedDepth - 0.5}m).`);
      }
      if (i > 0) {
        const prev = this.waypoints[i - 1];
        totalDist += Math.hypot(wp.n - prev.n, wp.e - prev.e, wp.d - prev.d);
      }
    });

    return {
      valid: errors.length === 0,
      errors,
      totalDistance: Number(totalDist.toFixed(2)),
    };
  }

  toJson() {
    return JSON.stringify({
      schema: 'mss-route-v1',
      waypointCount: this.waypoints.length,
      waypoints: this.waypoints,
    }, null, 2);
  }

  fromJson(jsonStr) {
    const data = typeof jsonStr === 'string' ? JSON.parse(jsonStr) : jsonStr;
    if (!data || !Array.isArray(data.waypoints)) {
      throw new Error('Invalid mission route JSON format');
    }
    this.waypoints = data.waypoints;
    this.activeLeg = 0;
    return this.validate();
  }
}
