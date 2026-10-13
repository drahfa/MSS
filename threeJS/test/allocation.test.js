import { test } from 'node:test';
import assert from 'node:assert/strict';
import { ROV_PROFILES, makeAllocator, thrusterWrench } from '../src/physics.js';

test('Pure-axis allocation satisfies saturation and produces matching forces', () => {
  const axes = [
    { name: 'Surge', demand: [1, 0, 0, 0, 0, 0], axisIdx: 0 },
    { name: 'Sway', demand: [0, 1, 0, 0, 0, 0], axisIdx: 1 },
    { name: 'Heave', demand: [0, 0, 1, 0, 0, 0], axisIdx: 2 },
    { name: 'Roll', demand: [0, 0, 0, 1, 0, 0], axisIdx: 3 },
    { name: 'Pitch', demand: [0, 0, 0, 0, 1, 0], axisIdx: 4 },
    { name: 'Yaw', demand: [0, 0, 0, 0, 0, 1], axisIdx: 5 },
  ];

  for (const [key, profile] of Object.entries(ROV_PROFILES)) {
    const allocate = makeAllocator(profile);

    for (const { name, demand, axisIdx } of axes) {
      const commands = allocate(demand);

      // 1. All output commands must strictly be within [-1, 1]
      for (let i = 0; i < commands.length; i++) {
        assert.ok(
          commands[i] >= -1 - 1e-9 && commands[i] <= 1 + 1e-9,
          `${key} ${name}: command[${i}] = ${commands[i]} is out of [-1, 1] bounds`
        );
        assert.ok(!Number.isNaN(commands[i]) && Number.isFinite(commands[i]), `${key} ${name}: NaN or non-finite command`);
      }

      // 2. Resulting wrench must align with demand if degree of freedom is supported
      const wrench = thrusterWrench(commands, profile);

      // Check whether profile has physical authority in this axis
      const hasAuthority = profile.thrusters.some(t => {
        if (axisIdx === 0 && t.f[0] !== 0) return true;
        if (axisIdx === 1 && t.f[1] !== 0) return true;
        if (axisIdx === 2 && t.f[2] !== 0) return true;
        if (axisIdx >= 3) {
          // Check moment components
          const fx = t.f[0], fy = t.f[1], fz = t.f[2];
          const rx = t.r[0], ry = t.r[1], rz = t.r[2];
          const mx = ry * fz - rz * fy;
          const my = rz * fx - rx * fz;
          const mz = rx * fy - ry * fx;
          if (axisIdx === 3 && Math.abs(mx) > 1e-4) return true;
          if (axisIdx === 4 && Math.abs(my) > 1e-4) return true;
          if (axisIdx === 5 && Math.abs(mz) > 1e-4) return true;
        }
        return false;
      });

      if (hasAuthority) {
        assert.ok(
          wrench[axisIdx] > 1e-4,
          `${key} ${name}: expected positive wrench along axis ${axisIdx}, got ${wrench[axisIdx]}`
        );
      } else {
        // If unsupported (e.g. sway or pitch on Hobbyist), generated force along this axis must be near zero
        assert.ok(
          Math.abs(wrench[axisIdx]) < 1e-4,
          `${key} ${name}: expected zero wrench on unsupported axis ${axisIdx}, got ${wrench[axisIdx]}`
        );
      }
    }
  }
});
