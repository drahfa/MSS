// Objective Pilot Scoring Engine and Flight Quality Metric (FQM) Calculator.
// Milestone 2.0 — Validated Training Platform.

export class ScoringEngine {
  constructor(scenario = null) {
    this.reset(scenario);
  }

  reset(scenario = null) {
    this.scenario = scenario;
    this.time = 0;
    this.waypointIndex = 0;
    this.isComplete = false;

    // Metrics accumulators
    this.sumSquareCte = 0;
    this.samplesCte = 0;
    this.maxCte = 0;

    this.thrusterEnergy = 0;
    this.thrusterChatter = 0;
    this.lastCommands = null;

    this.tetherExceedanceTime = 0;
    this.snagEvents = 0;

    this.contactEvents = 0;
    this.accumulatedImpactForce = 0;
    this.maxImpactSpeed = 0;

    this.infractions = [];
  }

  /**
   * Distance from 3D point P to line segment AB.
   */
  static distanceToSegment(p, a, b) {
    const ab = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    const ap = [p[0] - a[0], p[1] - a[1], p[2] - a[2]];
    const abLenSq = ab[0] * ab[0] + ab[1] * ab[1] + ab[2] * ab[2];

    if (abLenSq < 1e-8) {
      return Math.hypot(ap[0], ap[1], ap[2]);
    }

    const t = Math.max(0, Math.min(1, (ap[0] * ab[0] + ap[1] * ab[1] + ap[2] * ab[2]) / abLenSq));
    const closest = [a[0] + t * ab[0], a[1] + t * ab[1], a[2] + t * ab[2]];
    return Math.hypot(p[0] - closest[0], p[1] - closest[1], p[2] - closest[2]);
  }

  update(dt, pos, nu, thrusters, tetherState, contactState) {
    if (this.isComplete || !this.scenario) return;
    this.time += dt;

    const waypoints = this.scenario.waypoints;
    const currentWp = waypoints[this.waypointIndex];
    const prevWp = this.waypointIndex > 0 ? waypoints[this.waypointIndex - 1] : waypoints[0];

    // 1. Cross-Track Error (CTE)
    const p = [pos[0], pos[1], pos[2]];
    const a = [prevWp.n, prevWp.e, prevWp.d];
    const b = [currentWp.n, currentWp.e, currentWp.d];
    const cte = ScoringEngine.distanceToSegment(p, a, b);

    this.sumSquareCte += cte * cte;
    this.samplesCte += 1;
    if (cte > this.maxCte) this.maxCte = cte;

    if (cte > (this.scenario.corridorTolerance || 2.0)) {
      if (!this.infractions.includes('Corridor Excursion')) {
        this.infractions.push('Corridor Excursion');
      }
    }

    // 2. Thruster Energy & Chatter
    let instantaneousPower = 0;
    thrusters.forEach((u, i) => {
      instantaneousPower += Math.abs(u) * Math.abs(u) * Math.abs(u);
      if (this.lastCommands && this.lastCommands[i] !== undefined) {
        this.thrusterChatter += Math.abs(u - this.lastCommands[i]);
      }
    });
    this.thrusterEnergy += instantaneousPower * dt;
    this.lastCommands = [...thrusters];

    // 3. Tether Safety
    if (tetherState) {
      if (tetherState.advisory || tetherState.snag) {
        this.tetherExceedanceTime += dt;
      }
      if (tetherState.snag && !this.infractions.includes('Tether Snag')) {
        this.snagEvents += 1;
        this.infractions.push('Tether Snag');
      }
    }

    // 4. Contact & Impact
    if (contactState && contactState.contactActive) {
      this.contactEvents += 1;
      this.accumulatedImpactForce += (contactState.normalForce || 0) * dt;
      const speed = Math.hypot(nu[0], nu[1], nu[2]);
      if (speed > this.maxImpactSpeed) this.maxImpactSpeed = speed;

      const maxSafeSpeed = this.scenario.safetyEnvelope?.maxImpactSpeed || 0.25;
      if (speed > maxSafeSpeed && !this.infractions.includes('Hard Impact')) {
        this.infractions.push('Hard Impact');
      }
    }

    // 5. Waypoint Arrival Check
    const distToWp = Math.hypot(currentWp.n - pos[0], currentWp.e - pos[1], currentWp.d - pos[2]);
    const acceptanceRadius = 1.35;
    if (distToWp < acceptanceRadius) {
      if (this.waypointIndex < waypoints.length - 1) {
        this.waypointIndex += 1;
      } else {
        this.isComplete = true;
      }
    }
  }

  computeScore() {
    if (!this.scenario) return { score: 100, grade: 'A', subScores: {}, infractions: [] };

    // RMS Cross-Track Error
    const rmsCte = this.samplesCte > 0 ? Math.sqrt(this.sumSquareCte / this.samplesCte) : 0;
    const allowableCte = this.scenario.corridorTolerance || 2.0;
    const penCte = Math.min(100, (rmsCte / allowableCte) * 100);

    // Power & Chatter Penalty
    const expectedEnergy = (this.scenario.timeBudget || 60) * 1.5;
    const penPwr = Math.min(100, (this.thrusterEnergy / expectedEnergy) * 50 + (this.thrusterChatter / (this.time * 8 || 1)) * 10);

    // Tether Penalty
    const penTeth = Math.min(100, (this.tetherExceedanceTime / (this.time || 1)) * 60 + this.snagEvents * 35);

    // Contact & Impact Penalty
    const penColl = Math.min(100, this.contactEvents * 3 + (this.maxImpactSpeed > 0.25 ? 30 : 0));

    // Time Pace Penalty
    const timeBudget = this.scenario.timeBudget || 60;
    const timeExcess = Math.max(0, this.time - timeBudget);
    const penTime = Math.min(100, (timeExcess / timeBudget) * 60);

    // Multi-criteria weighted Flight Quality Metric (FQM)
    // Priority weights: CTE (35%), Collisions (25%), Tether (20%), Power (10%), Time (10%)
    const totalPenalty = (
      0.35 * penCte +
      0.25 * penColl +
      0.20 * penTeth +
      0.10 * penPwr +
      0.10 * penTime
    );

    const finalScore = Math.max(0, Math.min(100, Math.round((100 - totalPenalty) * 10) / 10));

    let grade = 'F';
    if (finalScore >= 90.0) grade = 'A';
    else if (finalScore >= 80.0) grade = 'B';
    else if (finalScore >= 70.0) grade = 'C';

    return {
      score: finalScore,
      grade,
      rmsCte: Number(rmsCte.toFixed(3)),
      maxCte: Number(this.maxCte.toFixed(3)),
      transitTime: Number(this.time.toFixed(1)),
      subScores: {
        tracking: Math.max(0, Math.round(100 - penCte)),
        contactIntegrity: Math.max(0, Math.round(100 - penColl)),
        tetherSafety: Math.max(0, Math.round(100 - penTeth)),
        powerEfficiency: Math.max(0, Math.round(100 - penPwr)),
        pace: Math.max(0, Math.round(100 - penTime)),
      },
      infractions: [...this.infractions],
      isComplete: this.isComplete,
      waypointProgress: `${this.waypointIndex + 1} / ${this.scenario.waypoints.length}`,
    };
  }
}
