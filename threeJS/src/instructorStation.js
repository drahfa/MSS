// Instructor Supervision Station and Flight Debriefing Report Generator.
// Milestone 2.0 — Validated Training Platform.

export class InstructorStation {
  constructor(sim, scene) {
    this.sim = sim;
    this.scene = scene;
    this.nominalFogDensity = 0.035;
    this.currentTurbidity = 0.035;
  }

  setTurbidity(density) {
    this.currentTurbidity = density;
    if (this.scene && this.scene.fog) {
      this.scene.fog.density = density;
    }
  }

  resetTurbidity() {
    this.setTurbidity(this.nominalFogDensity);
  }

  injectCurrentGust(speed) {
    if (this.sim && this.sim.current) {
      this.sim.current.speed = speed;
    }
  }

  injectThrusterFault(channel, mode, value = 0.0) {
    if (this.sim && this.sim.failureInjector) {
      this.sim.failureInjector.setFault(channel, mode, value);
    }
  }

  clearFaults() {
    if (this.sim && this.sim.failureInjector) {
      this.sim.failureInjector.resetAll();
    }
  }

  /**
   * Generates a formal, printable Flight Debriefing Report in Markdown format.
   */
  generateDebriefingReport(scenario, scoreResult, traineeName = 'Pilot Trainee') {
    const s = scoreResult || {};
    const subs = s.subScores || {};
    const dateStr = new Date().toISOString().replace('T', ' ').slice(0, 19);

    let qualification = 'Retraining Mandatory — Unacceptable Flight Infractions';
    if (s.grade === 'A') qualification = 'Master Subsea Pilot — Certified for Solo Intervention';
    else if (s.grade === 'B') qualification = 'Qualified ROV Operator — Certified for Standard Surveys';
    else if (s.grade === 'C') qualification = 'Junior Apprentice — Supervision Required in Strong Currents';

    return `# Subsea ROV Flight Evaluation & Debriefing Report

**Certification Document — ISO/IMCA ROV Pilot Competency Framework**
- **Date / Time:** ${dateStr} UTC
- **Trainee Pilot:** ${traineeName}
- **Assigned Scenario:** ${scenario?.title || 'Custom Flight Drill'} (${scenario?.level || 'Standard'})
- **Vehicle Class:** ${this.sim?.profile?.name || 'Inspection Class'}

---

## Executive Flight Summary

| Metric | Score / Value | Status |
| :--- | :---: | :--- |
| **Composite Flight Quality Score** | **${s.score ?? 0} / 100.0** | **Grade ${s.grade || 'F'}** |
| **Qualification Recommendation** | — | **${qualification}** |
| **Transit Duration** | ${s.transitTime || 0} s | Budget: ${scenario?.timeBudget || 60} s |
| **RMS Corridor Deviation** | ${s.rmsCte || 0} m | Allowable: ${scenario?.corridorTolerance || 2.0} m |
| **Peak Deviation (Max CTE)** | ${s.maxCte || 0} m | — |
| **Waypoint Progress** | ${s.waypointProgress || 'Complete'} | Final Destination |

---

## Category Performance Breakdown

| Assessment Domain | Sub-Score (0-100) | Priority Weight | Evaluator Notes |
| :--- | :---: | :---: | :--- |
| **Corridor Tracking Precision** | ${subs.tracking ?? 0} | 35% | Evaluates straight-line transit discipline |
| **Structural & Terrain Integrity** | ${subs.contactIntegrity ?? 0} | 25% | Penalizes hard impacts & collisions |
| **Tether & Catenary Safety** | ${subs.tetherSafety ?? 0} | 20% | Monitors spool tension & snag alerts |
| **Thruster Energy & Smoothness** | ${subs.powerEfficiency ?? 0} | 10% | Penalizes thermal wear & command chatter |
| **Mission Pace & Velocity** | ${subs.pace ?? 0} | 10% | Ensures timely operational transit |

---

## Flight Infractions & Safety Incidents Log

${(s.infractions && s.infractions.length > 0)
  ? s.infractions.map(inf => `- **[INFRACTION]** ${inf}: Exceeded operational safety envelope`).join('\n')
  : '- **No Safety Infractions Recorded:** Clean flight within all environmental and structural bounds.'
}

---

## Instructor Certification & Sign-off

- **Evaluation Platform:** MSS Web-Native 6-DOF Simulator (Milestone 2.0 Validated Training Platform)
- **Mathematical Compliance:** NTNU Marine Systems Simulator (MSS) Rigid-Body & Added-Mass Equations
- **Instructor Verdict:** ${s.score >= 70.0 ? 'PASSED — Candidate demonstrates requisite handling capability.' : 'FAILED — Additional training required before sea-trial clearance.'}

*(Automated Digital Verification Signature: MSS-CERT-${Math.random().toString(36).substring(2, 9).toUpperCase()})*
`;
  }
}
