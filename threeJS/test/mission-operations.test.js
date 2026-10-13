import { test } from 'node:test';
import assert from 'node:assert/strict';
import { RovDynamics, ROV_PROFILES } from '../src/physics.js';
import { TelemetryRecorder } from '../src/telemetryRecorder.js';
import { SensorSuite } from '../src/sensors.js';
import { TetherModel } from '../src/tether.js';
import { ContactModel } from '../src/contact.js';
import { FailureInjector, FaultMode } from '../src/failureInjection.js';
import { MissionRoute } from '../src/missionRoute.js';
import { Rquat } from '../src/mss/library.js';

test('Telemetry recording, CSV/JSON export, and deterministic replay achieve exact state repeatability', () => {
  const sim = new RovDynamics('inspection');
  const recorder = new TelemetryRecorder();

  recorder.startRecording();

  // Run 100 simulation steps with deterministic pilot inputs
  const inputSequence = [];
  for (let step = 0; step < 100; step++) {
    const pilotInput = {
      surge: 0.6 * Math.sin(step * 0.1),
      sway: 0.4 * Math.cos(step * 0.08),
      heave: 0.3 * Math.sin(step * 0.05),
      yaw: 0.5 * Math.sin(step * 0.12),
    };
    inputSequence.push(pilotInput);

    sim.setPilot(pilotInput, true, { depth: 2.5, heading: 0.2 });
    recorder.recordFrame(sim.time, sim.state, sim.thrusters, pilotInput, true);
    sim.step(0.01);
  }

  const recordedCount = recorder.stopRecording();
  assert.equal(recordedCount, 100);

  // Test JSON and CSV export
  const jsonExport = recorder.exportJson();
  assert.ok(jsonExport.includes('mss-telemetry-v1'));
  const parsedJson = JSON.parse(jsonExport);
  assert.equal(parsedJson.frames.length, 100);

  const csvExport = recorder.exportCsv();
  assert.ok(csvExport.startsWith('time,u,v,w'));
  const csvLines = csvExport.trim().split('\n');
  assert.equal(csvLines.length, 101); // 1 header + 100 data rows

  // Replay from identical initial conditions
  const replaySim = new RovDynamics('inspection');
  const replayer = new TelemetryRecorder();
  replayer.loadFrames(parsedJson.frames);
  assert.ok(replayer.startReplay());

  let step = 0;
  while (replayer.isReplaying) {
    const frame = replayer.getNextReplayFrame();
    if (!frame) break;

    // Verify replay state matches recorded state within floating point precision
    for (let i = 0; i < 9; i++) {
      const err = Math.abs(replaySim.state[i] - frame.state[i]);
      assert.ok(err < 1e-4, `State [${i}] diverged at step ${step}: err=${err}`);
    }

    replaySim.setPilot(frame.pilotInput, frame.assist, { depth: 2.5, heading: 0.2 });
    replaySim.step(0.01);
    step++;
  }
  assert.equal(step, 100);
});

test('Sensor suite measures DVL bottom lock, depth, compass, and IMU specific force accurately', () => {
  const sensors = new SensorSuite({ enabled: true });
  const R = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]; // Identity orientation

  // Vehicle at 5m depth (altitude = 18 - 5 = 13m)
  const state = [0.8, -0.2, 0.1, 0.02, -0.01, 0.05, 0, 0, 5.0, 1, 0, 0, 0];
  const nuDot = [0.1, 0, 0, 0, 0, 0];

  const readings = sensors.sample(state, nuDot, R, 0.01);

  // DVL should have bottom lock at 13m altitude
  assert.ok(readings.dvl.bottomLock);
  assert.ok(Math.abs(readings.dvl.u - 0.8) < 0.05);
  assert.ok(Math.abs(readings.dvl.v - (-0.2)) < 0.05);

  // Hydrostatic depth transducer
  assert.ok(Math.abs(readings.depth - 5.0) < 0.1);

  // Acoustic altimeter
  assert.ok(Math.abs(readings.altimeter - 13.0) < 0.15);

  // Compass with magnetic declination
  const expectedHdg = 0 + (1.2 * Math.PI / 180);
  assert.ok(Math.abs(readings.compass - expectedHdg) < 0.05);

  // Accelerometer specific force at rest has gravity component ~ 9.81 m/s^2 Down
  assert.ok(Math.abs(readings.imu.accel[2] - (-9.81)) < 0.3);

  // Test DVL bottom lock loss when altitude exceeds maximum (e.g. altitude = 35m)
  const deepState = [0.8, 0, 0, 0, 0, 0, 0, 0, -17.0, 1, 0, 0, 0]; // 35m above seabed
  const deepReadings = sensors.sample(deepState, nuDot, R, 0.01);
  assert.equal(deepReadings.dvl.bottomLock, false);
  assert.equal(deepReadings.dvl.u, null);
});

test('Tether model calculates catenary payout, hydrodynamic drag, and triggers snag alerts', () => {
  const tether = new TetherModel({ maxLength: 100.0, warningThresholdRatio: 0.85 });
  const R = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];

  // 1. Nominal operating envelope (distance = 40m)
  const posNominal = [30.0, 20.0, 15.0];
  const nuNominal = [1.2, 0, 0];
  const resNominal = tether.compute(posNominal, nuNominal, R);

  assert.ok(resNominal.payoutLength > 35.0 && resNominal.payoutLength < 45.0);
  assert.equal(resNominal.isNearLimit, false);
  assert.equal(resNominal.isSnagged, false);
  assert.ok(resNominal.dragForceBody[0] < 0, 'Tether line drag must oppose forward motion');
  assert.equal(resNominal.tension, 0);

  // 2. Near limit envelope (distance = 80m -> payout ~ 86.4m > 85m)
  const posNear = [70.0, 35.0, 20.0];
  const resNear = tether.compute(posNear, [0, 0, 0], R);
  assert.ok(resNear.isNearLimit);
  assert.equal(resNear.isSnagged, false);
  assert.ok(resNear.warning.includes('ADVISORY'));

  // 3. Snagged envelope (distance = 96m -> payout ~ 103.7m > 100m)
  const posSnag = [85.0, 40.0, 25.0];
  const resSnag = tether.compute(posSnag, [0, 0, 0], R);
  assert.ok(resSnag.isSnagged);
  assert.ok(resSnag.tension > 100.0, 'Restraining cable tension should be non-zero when snagged');
  assert.ok(resSnag.warning.includes('SNAGGED'));
});

test('Contact model exerts normal restoring and Coulomb friction forces upon seabed contact', () => {
  const contact = new ContactModel({ seabedDepth: 18.0, hullRadius: 0.5 });
  const R = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];

  // Midwater: no contact
  const resMidwater = contact.evaluate([0, 0, 10.0], [1, 0, 0], R);
  assert.equal(resMidwater.isSeabedContact, false);
  assert.equal(resMidwater.penetrationDepth, 0);
  assert.equal(resMidwater.contactForceBody[2], 0);

  // Contact: lowest point = 17.8 + 0.5 = 18.3m (penetration = 0.3m)
  const resContact = contact.evaluate([0, 0, 17.8], [0.5, 0, 0.2], R);
  assert.ok(resContact.isSeabedContact);
  assert.ok(resContact.penetrationDepth > 0.29);
  // Normal force must push UP (negative Down in NED)
  assert.ok(resContact.contactForceNed[2] < -1000.0);
  // Tangential Coulomb friction must oppose positive surge velocity
  assert.ok(resContact.contactForceNed[0] < 0);
});

test('Failure injector processes offline, degraded, and stuck faults with allocator awareness', () => {
  const injector = new FailureInjector(4);
  const rawCommands = [0.8, 0.8, 0.8, 0.8];

  // Nominal: all pass unchanged
  const nominal = injector.processCommands(rawCommands);
  assert.deepEqual(nominal, rawCommands);
  assert.equal(injector.hasAnyFaults(), false);

  // Inject OFFLINE on channel 0
  injector.setFault(0, FaultMode.OFFLINE);
  assert.ok(injector.hasFault(0));

  // Inject DEGRADED on channel 1 (40% efficiency)
  injector.setFault(1, FaultMode.DEGRADED, { efficiency: 0.40 });

  // Inject STUCK on channel 2 (-0.75 hardover)
  injector.setFault(2, FaultMode.STUCK, { stuckValue: -0.75 });

  const processed = injector.processCommands(rawCommands);
  assert.equal(processed[0], 0.0, 'Channel 0 offline should output 0');
  assert.ok(Math.abs(processed[1] - 0.32) < 1e-4, 'Channel 1 degraded should scale by 0.4');
  assert.equal(processed[2], -0.75, 'Channel 2 stuck should output stuckValue');
  assert.equal(processed[3], 0.8, 'Channel 3 healthy should remain unchanged');

  // Verify allocator health weights
  const weights = injector.getAllocatorWeights();
  assert.equal(weights[0], 0.0);
  assert.equal(weights[1], 0.40);
  assert.equal(weights[2], 1.0);
  assert.equal(weights[3], 1.0);

  // Clear faults
  injector.clearAllFaults();
  assert.equal(injector.hasAnyFaults(), false);
});

test('Mission route validates safety boundaries, waypoint arrivals, and JSON serialization', () => {
  const route = new MissionRoute();
  const val = route.validate(18.0);
  assert.ok(val.valid);
  assert.ok(val.totalDistance > 40.0);

  // Verify waypoint arrival detection
  const activeWp = route.getActiveWaypoint();
  assert.ok(activeWp);
  // Vehicle located directly at waypoint
  assert.ok(route.checkArrival([activeWp.n, activeWp.e, activeWp.d]));
  // Vehicle far from waypoint
  assert.equal(route.checkArrival([activeWp.n + 10, activeWp.e, activeWp.d]), false);

  // Add invalid waypoint violating seabed safety clearance
  route.addWaypoint({ n: 5, e: 5, d: 19.0, label: 'Unsafe Sub-Seabed' });
  const valInvalid = route.validate(18.0);
  assert.equal(valInvalid.valid, false);
  assert.ok(valInvalid.errors[0].includes('seabed safety clearance'));

  // Test JSON round-trip
  route.removeWaypoint(route.getWaypoints().length - 1);
  const jsonStr = route.toJson();
  const newRoute = new MissionRoute();
  const res = newRoute.fromJson(jsonStr);
  assert.ok(res.valid);
  assert.equal(newRoute.getWaypoints().length, 6);
});
