import { test } from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { ROV_PROFILES } from '../src/physics.js';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const modelsDir = path.resolve(__dirname, '../public/models');

test('Every ROV GLB has matching rotor count and contiguous channel_index metadata', () => {
  for (const [key, profile] of Object.entries(ROV_PROFILES)) {
    const glbPath = path.join(modelsDir, `rov-${key}.glb`);
    assert.ok(fs.existsSync(glbPath), `Model file missing: ${glbPath}`);

    const buffer = fs.readFileSync(glbPath);
    // Parse glTF header (magic: 0x46546C67, version: 2, length, jsonLen, jsonType)
    const magic = buffer.readUInt32LE(0);
    assert.equal(magic, 0x46546C67, `${key}: Invalid GLB magic header`);

    const jsonLen = buffer.readUInt32LE(12);
    const gltf = JSON.parse(buffer.slice(20, 20 + jsonLen).toString('utf8'));

    const rotorNodes = gltf.nodes.filter(n => n.name && n.name.startsWith('rotor_'));

    // 1. Rotor count in 3D model must match profile thrusters count exactly
    assert.equal(
      rotorNodes.length,
      profile.thrusters.length,
      `${key}: Model has ${rotorNodes.length} rotors but profile defines ${profile.thrusters.length}`
    );

    // 2. Rotor channel indices must be contiguous from 0 to N-1
    const channels = rotorNodes
      .map(n => n.extras?.channel_index)
      .sort((a, b) => a - b);

    for (let i = 0; i < profile.thrusters.length; i++) {
      assert.equal(
        channels[i],
        i,
        `${key}: Expected rotor channel_index ${i}, found ${channels[i]}`
      );
    }
  }
});
