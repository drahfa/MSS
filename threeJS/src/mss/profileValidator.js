// Real-time parameter schema validator and exporter for 6-DOF ROV dynamic profiles.
// Validates against public/schemas/rov-profile.schema.json rules.

/**
 * Validates a candidate ROV profile object in real time.
 * @param {any} profile Candidate profile object
 * @returns {{ valid: boolean, errors: string[], summary?: object }}
 */
export function validateProfile(profile) {
  const errors = [];

  if (!profile || typeof profile !== 'object' || Array.isArray(profile)) {
    return { valid: false, errors: ['Profile root must be a valid JSON object.'] };
  }

  // Required top-level properties
  const requiredFields = [
    'name', 'role', 'mass', 'inertia', 'addedMass',
    'linearDamping', 'quadraticDamping', 'leverScale', 'cg', 'thrusters'
  ];

  for (const field of requiredFields) {
    if (profile[field] === undefined || profile[field] === null) {
      errors.push(`Missing required field: "${field}".`);
    }
  }

  // Name and Role
  if (profile.name !== undefined && (typeof profile.name !== 'string' || profile.name.trim() === '')) {
    errors.push('Field "name" must be a non-empty string.');
  }
  if (profile.role !== undefined && (typeof profile.role !== 'string' || profile.role.trim() === '')) {
    errors.push('Field "role" must be a non-empty string.');
  }

  // Mass
  if (profile.mass !== undefined) {
    if (typeof profile.mass !== 'number' || !Number.isFinite(profile.mass) || profile.mass <= 0) {
      errors.push(`Field "mass" must be a positive number (> 0), got: ${profile.mass}.`);
    }
  }

  // Inertia [Ix, Iy, Iz]
  if (profile.inertia !== undefined) {
    if (!Array.isArray(profile.inertia) || profile.inertia.length !== 3) {
      errors.push('Field "inertia" must be an array of exactly 3 principal moments [Ix, Iy, Iz].');
    } else {
      profile.inertia.forEach((val, i) => {
        if (typeof val !== 'number' || !Number.isFinite(val) || val <= 0) {
          errors.push(`Field "inertia[${i}]" must be a positive number (> 0), got: ${val}.`);
        }
      });
    }
  }

  // Added Mass [6 elements or 6x6 matrix]
  if (profile.addedMassMatrix !== undefined) {
    if (!Array.isArray(profile.addedMassMatrix) || profile.addedMassMatrix.length !== 6 ||
        !profile.addedMassMatrix.every(row => Array.isArray(row) && row.length === 6 && row.every(Number.isFinite))) {
      errors.push('Field "addedMassMatrix" must be a 6x6 numerical matrix.');
    }
  } else if (profile.addedMass !== undefined) {
    if (!Array.isArray(profile.addedMass) || profile.addedMass.length !== 6 ||
        !profile.addedMass.every(v => typeof v === 'number' && Number.isFinite(v))) {
      errors.push('Field "addedMass" must be an array of 6 numerical coefficients.');
    }
  }

  // Linear Damping [6 non-negative elements or 6x6 matrix]
  if (profile.linearDampingMatrix !== undefined) {
    if (!Array.isArray(profile.linearDampingMatrix) || profile.linearDampingMatrix.length !== 6 ||
        !profile.linearDampingMatrix.every(row => Array.isArray(row) && row.length === 6 && row.every(v => typeof v === 'number' && Number.isFinite(v)))) {
      errors.push('Field "linearDampingMatrix" must be a 6x6 numerical matrix.');
    }
  } else if (profile.linearDamping !== undefined) {
    if (!Array.isArray(profile.linearDamping) || profile.linearDamping.length !== 6 ||
        !profile.linearDamping.every(v => typeof v === 'number' && Number.isFinite(v) && v >= 0)) {
      errors.push('Field "linearDamping" must be an array of 6 non-negative numbers.');
    }
  }

  // Quadratic Damping [6 non-negative elements]
  if (profile.quadraticDamping !== undefined) {
    if (!Array.isArray(profile.quadraticDamping) || profile.quadraticDamping.length !== 6 ||
        !profile.quadraticDamping.every(v => typeof v === 'number' && Number.isFinite(v) && v >= 0)) {
      errors.push('Field "quadraticDamping" must be an array of 6 non-negative numbers.');
    }
  }

  // Lever scale
  if (profile.leverScale !== undefined) {
    if (typeof profile.leverScale !== 'number' || !Number.isFinite(profile.leverScale) || profile.leverScale <= 0) {
      errors.push('Field "leverScale" must be a positive number (> 0).');
    }
  }

  // Center of Gravity (cg)
  if (profile.cg !== undefined) {
    if (typeof profile.cg !== 'number' || !Number.isFinite(profile.cg)) {
      errors.push('Field "cg" must be a finite number.');
    }
  }

  // Buoyancy ratio
  if (profile.buoyancyRatio !== undefined) {
    if (typeof profile.buoyancyRatio !== 'number' || !Number.isFinite(profile.buoyancyRatio) || profile.buoyancyRatio <= 0) {
      errors.push('Field "buoyancyRatio" must be a positive number (> 0).');
    }
  }

  // Thrusters array
  if (profile.thrusters !== undefined) {
    if (!Array.isArray(profile.thrusters) || profile.thrusters.length === 0) {
      errors.push('Field "thrusters" must be a non-empty array of thruster units.');
    } else {
      profile.thrusters.forEach((t, i) => {
        if (!t || typeof t !== 'object') {
          errors.push(`Thruster [${i}] must be an object.`);
          return;
        }
        if (!t.id || typeof t.id !== 'string') {
          errors.push(`Thruster [${i}]: "id" must be a non-empty string.`);
        }
        if (!t.label || typeof t.label !== 'string') {
          errors.push(`Thruster [${i}]: "label" must be a string.`);
        }
        if (!Array.isArray(t.r) || t.r.length !== 3 || !t.r.every(Number.isFinite)) {
          errors.push(`Thruster [${i}]: "r" position must be an array of 3 numbers.`);
        }
        if (!Array.isArray(t.f) || t.f.length !== 3 || !t.f.every(Number.isFinite)) {
          errors.push(`Thruster [${i}]: "f" direction must be an array of 3 numbers.`);
        } else {
          const normF = Math.hypot(t.f[0], t.f[1], t.f[2]);
          if (normF < 1e-6) {
            errors.push(`Thruster [${i}]: thrust orientation vector "f" cannot be zero.`);
          }
        }
        if (typeof t.force !== 'number' || !Number.isFinite(t.force) || t.force <= 0) {
          errors.push(`Thruster [${i}]: "force" must be a positive number (> 0).`);
        }
        if (t.reverseEfficiency !== undefined && (typeof t.reverseEfficiency !== 'number' || t.reverseEfficiency < 0.1 || t.reverseEfficiency > 1.0)) {
          errors.push(`Thruster [${i}]: "reverseEfficiency" must be between 0.1 and 1.0.`);
        }
        if (t.deadband !== undefined && (typeof t.deadband !== 'number' || t.deadband < 0 || t.deadband > 0.3)) {
          errors.push(`Thruster [${i}]: "deadband" must be between 0 and 0.3.`);
        }
      });
    }
  }

  if (errors.length > 0) {
    return { valid: false, errors };
  }

  // Synthesize diagnostic summary
  let totalFwdThrust = 0;
  let totalVertThrust = 0;
  profile.thrusters.forEach(t => {
    if (Math.abs(t.f[0]) > 0.1) totalFwdThrust += t.force * Math.abs(t.f[0]);
    if (Math.abs(t.f[2]) > 0.1) totalVertThrust += t.force * Math.abs(t.f[2]);
  });

  return {
    valid: true,
    errors: [],
    summary: {
      name: profile.name,
      role: profile.role,
      mass: profile.mass,
      thrusterCount: profile.thrusters.length,
      forwardBollardEst: totalFwdThrust.toFixed(1),
      verticalBollardEst: totalVertThrust.toFixed(1),
      buoyancyRatio: profile.buoyancyRatio ?? 1.006,
      cgOffset: profile.cg,
    }
  };
}

/**
 * Formats a clean JSON string conforming to the schema.
 */
export function formatProfileJson(profile) {
  const cleanProfile = {
    $schema: '../schemas/rov-profile.schema.json',
    name: profile.name,
    role: profile.role,
    mass: profile.mass,
    inertia: [...profile.inertia],
    addedMass: [...profile.addedMass],
    ...(profile.addedMassMatrix ? { addedMassMatrix: profile.addedMassMatrix } : {}),
    linearDamping: [...profile.linearDamping],
    ...(profile.linearDampingMatrix ? { linearDampingMatrix: profile.linearDampingMatrix } : {}),
    quadraticDamping: [...profile.quadraticDamping],
    leverScale: profile.leverScale ?? 1.0,
    cg: profile.cg ?? 0.05,
    cb: profile.cb ?? [0, 0, 0],
    buoyancyRatio: profile.buoyancyRatio ?? 1.006,
    cameraDistance: profile.cameraDistance ?? 6.0,
    actuatorTau: profile.actuatorTau ?? 0.15,
    thrusters: profile.thrusters.map(t => ({
      id: t.id,
      label: t.label,
      r: [...t.r],
      f: [...t.f],
      force: t.force,
      reverseEfficiency: t.reverseEfficiency ?? 0.75,
      deadband: t.deadband ?? 0.04,
      kAdvance: t.kAdvance ?? 0.30,
      vMax: t.vMax ?? 2.0,
    })),
  };
  return JSON.stringify(cleanProfile, null, 2);
}
