import { test } from 'node:test';
import assert from 'node:assert/strict';
import { TRAINING_SCENARIOS } from '../src/scenarios.js';
import { ScoringEngine } from '../src/scoringEngine.js';

test('Training scenarios define valid waypoint corridors and safety envelopes', () => {
  const scenarioKeys = Object.keys(TRAINING_SCENARIOS);
  assert.equal(scenarioKeys.length >= 4, true, 'At least 4 standardized training scenarios defined');

  scenarioKeys.forEach(key => {
    const s = TRAINING_SCENARIOS[key];
    assert.ok(s.id, 'Scenario must have an id');
    assert.ok(s.title, 'Scenario must have a title');
    assert.ok(s.level, 'Scenario must have a qualification level');
    assert.ok(s.timeBudget > 0, 'Scenario must have a positive time budget');
    assert.ok(s.current && s.current.speed >= 0, 'Scenario must define ocean current');
    assert.ok(Array.isArray(s.waypoints) && s.waypoints.length >= 3, 'Scenario must have at least 3 waypoints');
    assert.ok(s.safetyEnvelope.minAltitude > 0, 'Safety envelope must define min altitude');
    assert.ok(s.safetyEnvelope.maxTetherLength > 0, 'Safety envelope must define max tether length');
  });
});

test('distanceToSegment computes exact geometric orthogonal and endpoint distances', () => {
  const a = [0, 0, 0];
  const b = [10, 0, 0];

  // Point on segment
  assert.equal(ScoringEngine.distanceToSegment([5, 0, 0], a, b), 0);

  // Orthogonal point at offset 3
  assert.equal(ScoringEngine.distanceToSegment([5, 3, 0], a, b), 3);

  // Orthogonal point in 3D
  assert.equal(Math.round(ScoringEngine.distanceToSegment([5, 3, 4], a, b)), 5);

  // Before segment start (clamped to A)
  assert.equal(ScoringEngine.distanceToSegment([-4, 0, 0], a, b), 4);

  // Beyond segment end (clamped to B)
  assert.equal(ScoringEngine.distanceToSegment([13, 4, 0], a, b), 5);
});

test('ScoringEngine awards Grade A for near-perfect trajectory along corridor', () => {
  const scenario = TRAINING_SCENARIOS['pipeline-survey'];
  const engine = new ScoringEngine(scenario);

  const dt = 0.1;
  const steps = 100;
  // Follow waypoints closely
  for (let i = 0; i < steps; i++) {
    const fraction = i / steps;
    const n = 2.0 + fraction * 6.0; // Moving from WP0 to WP1
    const pos = [n, 0.05, 3.0 + fraction * 5.0];
    const nu = [0.8, 0, 0.5, 0, 0, 0];
    const thrusters = [0.2, 0.2, 0.2, 0.2, 0.2, 0.2];
    const tether = { advisory: false, snag: false };
    const contact = { contactActive: false };

    engine.update(dt, pos, nu, thrusters, tether, contact);
  }

  const result = engine.computeScore();
  assert.equal(result.grade, 'A', 'Smooth corridor tracking earns Grade A');
  assert.ok(result.score >= 90.0, `Score must be >= 90 (got ${result.score})`);
  assert.equal(result.infractions.length, 0, 'No infractions on clean run');
  assert.ok(result.rmsCte < 0.5, 'RMS CTE must be small');
});

test('ScoringEngine penalizes large cross-track excursions and assigns lower grade', () => {
  const scenario = TRAINING_SCENARIOS['pipeline-survey'];
  const engine = new ScoringEngine(scenario);

  const dt = 0.1;
  const steps = 100;
  // Large lateral offset (5 meters off corridor)
  for (let i = 0; i < steps; i++) {
    const pos = [4.0, 5.0, 5.0];
    const nu = [0.2, 0, 0, 0, 0, 0];
    const thrusters = [0.8, 0.8, 0.8, 0.8, 0.8, 0.8];
    const tether = { advisory: false, snag: false };
    const contact = { contactActive: false };

    engine.update(dt, pos, nu, thrusters, tether, contact);
  }

  const result = engine.computeScore();
  assert.ok(result.score < 80.0, `Score must drop below 80 for 5m excursion (got ${result.score})`);
  assert.ok(result.infractions.includes('Corridor Excursion'), 'Must record Corridor Excursion infraction');
});

test('ScoringEngine penalizes tether snags and high-velocity collisions heavily', () => {
  const scenario = TRAINING_SCENARIOS['habitat-docking'];
  const engine = new ScoringEngine(scenario);

  const dt = 0.1;
  for (let i = 0; i < 50; i++) {
    const pos = [5.0, 0, 5.0];
    const nu = [0.6, 0, 0.4, 0, 0, 0]; // High speed 0.72 m/s
    const thrusters = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5];
    const tether = { advisory: true, snag: true }; // Snag!
    const contact = { contactActive: true, normalForce: 450 }; // Collision!

    engine.update(dt, pos, nu, thrusters, tether, contact);
  }

  const result = engine.computeScore();
  assert.ok(result.infractions.includes('Tether Snag'), 'Must record Tether Snag');
  assert.ok(result.infractions.includes('Hard Impact'), 'Must record Hard Impact');
  assert.ok(result.score < 70.0, `Score must drop below 70 under snag and impact (got ${result.score})`);
  assert.equal(result.grade, 'F', 'Unsafe operations earn Grade F');
});

test('ScoringEngine progresses through waypoints when vehicle enters acceptance radius', () => {
  const scenario = TRAINING_SCENARIOS['pipeline-survey'];
  const engine = new ScoringEngine(scenario);

  assert.equal(engine.waypointIndex, 0);

  // Move directly to Waypoint 0 [2.0, 0.0, 3.0]
  engine.update(0.1, [2.0, 0.0, 3.0], [0, 0, 0], [0, 0, 0, 0, 0, 0], null, null);
  assert.equal(engine.waypointIndex, 1, 'Must advance to waypoint 1');

  // Move directly to Waypoint 1 [8.0, 0.2, 8.0]
  engine.update(0.1, [8.0, 0.2, 8.0], [0, 0, 0], [0, 0, 0, 0, 0, 0], null, null);
  assert.equal(engine.waypointIndex, 2, 'Must advance to waypoint 2');
});
