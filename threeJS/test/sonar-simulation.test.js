import { test } from 'node:test';
import assert from 'node:assert/strict';
import { ForwardLookingSonar } from '../src/forwardLookingSonar.js';

test('ForwardLookingSonar initializes 96 beams and 128 range bins correctly', () => {
  const sonar = new ForwardLookingSonar({
    beamCount: 96,
    binCount: 128,
  });

  assert.equal(sonar.beamCount, 96);
  assert.equal(sonar.binCount, 128);
  assert.equal(sonar.beamAngles.length, 96);

  // Check symmetrical FOV [-pi/3, +pi/3]
  assert.ok(Math.abs(sonar.beamAngles[0] - (-Math.PI / 3)) < 0.05);
  assert.ok(Math.abs(sonar.beamAngles[95] - (Math.PI / 3)) < 0.05);

  // Center beams should be around 0
  const centerBeam = sonar.beamAngles[48];
  assert.ok(Math.abs(centerBeam) < 0.03);
});

test('ForwardLookingSonar switches between Navigation and Inspection modes', () => {
  const sonar = new ForwardLookingSonar();
  assert.equal(sonar.rMax, 40.0);
  assert.equal(sonar.frequencyKhz, 450);

  sonar.setMode('inspection');
  assert.equal(sonar.rMax, 20.0);
  assert.equal(sonar.frequencyKhz, 900);
  assert.equal(sonar.alphaDb, 0.285);

  sonar.setMode('navigation');
  assert.equal(sonar.rMax, 40.0);
  assert.equal(sonar.frequencyKhz, 450);
  assert.equal(sonar.alphaDb, 0.088);
});

test('ForwardLookingSonar detects target and creates acoustic peak at target range', () => {
  const sonar = new ForwardLookingSonar({ beamCount: 32, binCount: 64, rMax: 30.0 });
  const rovPos = [0, 0, 5]; // At (0,0,5)
  const rovEuler = [0, 0, 0]; // Facing North (0 rad)

  // Place obstacle 12m directly ahead along North
  const obstacles = [
    { pos: [12.0, 0.0, 5.0], radius: 1.5, strength: 0.9 },
  ];

  sonar.scan(rovPos, rovEuler, obstacles);

  // Center beam (around index 16) should see the obstacle at ~10.5 to 12m
  const centerBeam = 16;
  const deltaR = (sonar.rMax - sonar.rMin) / sonar.binCount;

  let maxInt = 0;
  let maxRange = 0;
  for (let k = 0; k < sonar.binCount; k++) {
    const val = sonar.intensities[centerBeam * sonar.binCount + k];
    if (val > maxInt) {
      maxInt = val;
      maxRange = sonar.rMin + k * deltaR;
    }
  }

  assert.ok(maxInt > 0.3, 'Acoustic peak must be strong for obstacle');
  assert.ok(Math.abs(maxRange - 11.0) < 2.0, `Detected range ${maxRange} should match obstacle distance ~11-12m`);
});

test('ForwardLookingSonar casts acoustic shadow behind solid obstacle', () => {
  const sonar = new ForwardLookingSonar({ beamCount: 32, binCount: 64, rMax: 30.0 });
  const rovPos = [0, 0, 5];
  const rovEuler = [0, 0, 0];

  const obstacles = [
    { pos: [10.0, 0.0, 5.0], radius: 2.0, strength: 0.9 },
  ];

  sonar.scan(rovPos, rovEuler, obstacles);

  const centerBeam = 16;
  const deltaR = (sonar.rMax - sonar.rMin) / sonar.binCount;

  // Behind the obstacle (e.g. range 18m to 25m), intensity must be zero (acoustic shadow)
  for (let k = 0; k < sonar.binCount; k++) {
    const r = sonar.rMin + k * deltaR;
    if (r > 15.0 && r < 25.0) {
      const val = sonar.intensities[centerBeam * sonar.binCount + k];
      assert.equal(val, 0.0, `Range ${r}m behind obstacle must be in acoustic shadow (zero intensity)`);
    }
  }
});

test('ForwardLookingSonar renderDisplay runs cleanly on 2D context', () => {
  const sonar = new ForwardLookingSonar({ beamCount: 16, binCount: 32 });
  sonar.scan([0, 0, 2], [0, 0, 0]);

  // Mock HTML5 canvas
  const mockCanvas = {
    width: 240,
    height: 160,
    getContext: () => ({
      fillRect: () => {},
      beginPath: () => {},
      arc: () => {},
      stroke: () => {},
      moveTo: () => {},
      lineTo: () => {},
      fillText: () => {},
      fillStyle: '',
      strokeStyle: '',
      lineWidth: 1,
      font: '',
    }),
  };

  assert.doesNotThrow(() => {
    sonar.renderDisplay(mockCanvas, 'amber');
    sonar.renderDisplay(mockCanvas, 'cyan');
  });
});
