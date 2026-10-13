// Declarative Training Scenario Definitions for Subsea ROV Flight Certification.
// Milestone 2.0 — Validated Training Platform.

export const TRAINING_SCENARIOS = {
  'pipeline-survey': {
    id: 'pipeline-survey',
    title: 'Subsea Pipeline Survey',
    level: 'Level 1 — Trainee',
    description: 'Precision straight-line pipeline tracking at 1.5 m altitude with cross-track tolerance under 1.5 m.',
    timeBudget: 60.0,
    current: { speed: 0.15, direction: 1.57 }, // 0.15 m/s eastward cross-current
    corridorTolerance: 1.5,
    waypoints: [
      { n: 2.0, e: 0.0, d: 3.0, label: 'Start Staging', heading: 0.0 },
      { n: 8.0, e: 0.2, d: 8.0, label: 'Descent to Pipeline', heading: 0.0 },
      { n: 16.0, e: 0.0, d: 14.0, label: 'Pipeline Intercept', heading: 0.0 },
      { n: 26.0, e: -0.2, d: 16.5, label: 'Midline Inspection', heading: 0.0 },
      { n: 36.0, e: 0.0, d: 16.5, label: 'Pipeline Terminal', heading: 0.0 },
    ],
    safetyEnvelope: {
      minAltitude: 0.5,
      maxTetherLength: 60.0,
      maxImpactSpeed: 0.25,
    },
  },

  'habitat-docking': {
    id: 'habitat-docking',
    title: 'Benthic Habitat Inspection & Docking',
    level: 'Level 2 — Operator',
    description: 'Circumnavigate underwater habitat viewport ring at 11 m depth and execute precision docking.',
    timeBudget: 90.0,
    current: { speed: 0.22, direction: 2.14 }, // Moderate oblique current
    corridorTolerance: 1.8,
    waypoints: [
      { n: 3.0, e: 0.0, d: 3.0, label: 'Staging Area', heading: -0.5 },
      { n: 10.0, e: -6.0, d: 8.0, label: 'Habitat Approach', heading: -0.7 },
      { n: 16.0, e: -10.0, d: 11.0, label: 'West Viewport Seam', heading: 0.2 },
      { n: 18.0, e: -4.0, d: 11.0, label: 'East Flange Inspection', heading: 1.5 },
      { n: 12.0, e: -2.0, d: 11.5, label: 'Docking Alignment Target', heading: 3.14 },
    ],
    safetyEnvelope: {
      minAltitude: 0.4,
      maxTetherLength: 75.0,
      maxImpactSpeed: 0.20,
    },
  },

  'high-shear-nav': {
    id: 'high-shear-nav',
    title: 'High-Shear Current Navigation',
    level: 'Level 3 — Specialist',
    description: 'Compensate for intense 0.55 m/s surface current shear decaying to 0.25 m/s benthic layer.',
    timeBudget: 80.0,
    current: { speed: 0.55, direction: 1.20 }, // High surface shear
    corridorTolerance: 2.2,
    waypoints: [
      { n: 0.0, e: 0.0, d: 2.0, label: 'Splash Zone Entry', heading: 0.0 },
      { n: 6.0, e: 2.0, d: 6.0, label: 'Upper Shear Transit', heading: -0.4 },
      { n: 12.0, e: 1.0, d: 10.0, label: 'Midwater Boundary Transition', heading: -0.6 },
      { n: 18.0, e: 0.0, d: 15.0, label: 'Benthic Quiet Zone', heading: 0.0 },
      { n: 22.0, e: -1.0, d: 17.0, label: 'Seabed Station Hold', heading: 0.0 },
    ],
    safetyEnvelope: {
      minAltitude: 0.3,
      maxTetherLength: 85.0,
      maxImpactSpeed: 0.25,
    },
  },

  'thruster-recovery': {
    id: 'thruster-recovery',
    title: 'Emergency Thruster Abort & Recovery',
    level: 'Level 3 — Specialist',
    description: 'Respond to actuator failure at depth and return to staging envelope on reduced authority.',
    timeBudget: 75.0,
    current: { speed: 0.25, direction: 2.80 },
    corridorTolerance: 2.5,
    waypoints: [
      { n: 2.0, e: 0.0, d: 3.0, label: 'Transit Departure', heading: 0.0 },
      { n: 14.0, e: -4.0, d: 9.0, label: 'Mid-Water Failure Point', heading: 0.0 },
      { n: 10.0, e: -2.0, d: 7.0, label: 'Emergency Abort Turn', heading: 3.14 },
      { n: 3.0, e: 0.0, d: 3.0, label: 'Safe Recovery Envelope', heading: 3.14 },
    ],
    faultInjection: {
      triggerTime: 18.0, // Injected 18 seconds into mission
      thruster: 0,       // Front port vector H1
      mode: 'OFFLINE',
    },
    safetyEnvelope: {
      minAltitude: 0.5,
      maxTetherLength: 70.0,
      maxImpactSpeed: 0.30,
    },
  },
};
