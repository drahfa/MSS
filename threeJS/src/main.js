import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import createHabitat from './assets/underwater-research-habitat.mjs';
import { RovDynamics, ROV_PROFILES, clamp } from './physics.js';
import { validateProfile, formatProfileJson } from './mss/profileValidator.js';
import { GamepadController } from './gamepad.js';
import { TelemetryRecorder } from './telemetryRecorder.js';
import { TRAINING_SCENARIOS } from './scenarios.js';
import { ScoringEngine } from './scoringEngine.js';
import { InstructorStation } from './instructorStation.js';
import { getLanguage, setLanguage, t } from './i18n.js';
import { ManipulatorView } from './manipulatorView.js';
import { MultiRovView } from './multiRovView.js';
import { ForwardLookingSonar } from './forwardLookingSonar.js';
import './style.css';

const $ = id => document.getElementById(id);
const sim = new RovDynamics();
const keys = new Set();
let running = true, assist = true, showTrail = true, cameraMode = 0, demoMode = false;
let targetDepth = 2, targetHeading = sim.getEuler()[2], accumulator = 0, last = performance.now();
let demoLeg = 0, demoLegStarted = 0, assistBeforeDemo = true, cameraBeforeDemo = 0;
const depthHistory = [];
const demoRoute = [
  { n: 4,  e: -1,  d: 4,  label: 'Descent check' },
  { n: 11, e: -10, d: 8,  label: 'Habitat approach' },
  { n: 18, e: -10, d: 11, label: 'West inspection' },
  { n: 19, e: -3,  d: 10, label: 'East inspection' },
  { n: 9,  e: 1,   d: 6,  label: 'Return arc' },
  { n: 2,  e: 0,   d: 3,  label: 'Station keeping' },
];

const sceneHost = $('scene');
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x052330);
scene.fog = new THREE.FogExp2(0x052330, 0.035);
const camera = new THREE.PerspectiveCamera(52, 1, 0.05, 180);
camera.position.set(5, 3.2, 6);
const renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: 'high-performance' });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
renderer.outputColorSpace = THREE.SRGBColorSpace;
sceneHost.appendChild(renderer.domElement);

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.enablePan = false;
controls.minDistance = 2.2;
controls.maxDistance = 18;
controls.maxPolarAngle = Math.PI * 0.82;
const cameraLookTarget = new THREE.Vector3();
// Blender authors the shaft on local Z. Its glTF Y-up conversion maps the
// exported blade-plane normal to the rotor node's local Y axis.
const rotorShaftAxis = new THREE.Vector3(0, 1, 0);
const cameraNames = ['Third person', 'Top down', 'FPV'];
const modelLoader = new GLTFLoader();
const modelCache = new Map();
let vehicleKey = 'inspection';

scene.add(new THREE.HemisphereLight(0x70d6da, 0x07151b, 1.5));
const keyLight = new THREE.DirectionalLight(0xb5fbff, 2.8);
keyLight.position.set(-4, 7, 3);
keyLight.castShadow = true;
keyLight.shadow.mapSize.set(1024, 1024);
scene.add(keyLight);

function enableShadows(object) {
  object.traverse(child => { if (child.isMesh) { child.castShadow = true; child.receiveShadow = true; } });
  return object;
}

// Polyfork assets are authored Y-up in metres and face +Z. The child offset
// puts the submarine's hydrodynamic origin near its geometric centre; the
// PI yaw maps its +Z nose to the simulator's North (-Z) at heading zero.
const rov = new THREE.Group();
rov.rotation.order = 'YXZ';
let rovModel = null;

const rovLights = [];
[-.34, .34].forEach(x => {
  const spot = new THREE.SpotLight(0xbaf8ff, 28, 22, .24, .7, 1.3);
  spot.position.set(x, .12, -1.1);
  spot.target.position.set(x, -.25, -10);
  rov.add(spot, spot.target);
  rovLights.push({ spot, target: spot.target, side: Math.sign(x) });
});
scene.add(rov);

const manipulatorView = new ManipulatorView(rov);
const multiRovView = new MultiRovView(scene);
const sonar = new ForwardLookingSonar();
let armActive = false;
let fleetActive = false;
let armGrip = 0.5;
let sonarPalette = 'amber';
manipulatorView.setVisible(false);

function drawSonar() {
  const sonarCanvas = $('sonarCanvas');
  if (sonarCanvas) {
    console.log('DRAW_SONAR_CALLED', sonarCanvas.width, sonarCanvas.height);
    sonar.scan(sim.state.slice(6, 9), sim.getEuler());
    sonar.renderDisplay(sonarCanvas, sonarPalette);
  }
}

$('sonarModeBtn')?.addEventListener('click', () => {
  if (sonar.frequencyKhz === 450) {
    sonar.setMode('inspection');
    $('sonarModeBtn').textContent = '900 kHz · 20m';
    $('sonarModeBtn').style.color = 'var(--cyan)';
    sonarPalette = 'cyan';
  } else {
    sonar.setMode('navigation');
    $('sonarModeBtn').textContent = '450 kHz · 40m';
    $('sonarModeBtn').style.color = 'var(--amber)';
    sonarPalette = 'amber';
  }
  drawSonar();
});

async function loadRovClass(key) {
  const profile = ROV_PROFILES[key];
  $('rovClass').disabled = true;
  $('statusText').textContent = `Loading ${profile.name.toLowerCase()} vehicle`;
  try {
    let source = modelCache.get(key);
    if (!source) {
      const base = import.meta.env.BASE_URL.replace(/\/$/, '');
      source = (await modelLoader.loadAsync(`${base}/models/rov-${key}.glb`)).scene;
      modelCache.set(key, source);
    }
    const nextModel = enableShadows(source.clone(true));
    nextModel.rotation.y = Math.PI;
    nextModel.userData.rotors = [];
    nextModel.traverse(child => {
      if (child.name.startsWith('rotor_')) nextModel.userData.rotors.push(child);
    });
    nextModel.userData.rotors.sort((a, b) => Number(a.userData.channel_index) - Number(b.userData.channel_index));
    if (rovModel) rov.remove(rovModel);
    rovModel = nextModel;
    rov.add(rovModel);
    vehicleKey = key;
    sim.configure(key);
    renderThrusterChannels(profile);
    const lightScale = profile.leverScale;
    rovLights.forEach(({spot,target,side}) => {
      spot.position.set(side * .34 * lightScale, .12 * lightScale, -.72 * lightScale);
      target.position.set(side * .25 * lightScale, -.18 * lightScale, -8 * lightScale);
      spot.distance = Math.max(14, profile.cameraDistance * 2.2);
    });
    manipulatorView.updateProfile(profile);
    $('vehicleSpec').textContent = `${profile.mass.toLocaleString()} kg · ${profile.role} · ${profile.thrusters.length} thrusters`;
    $('vehicleStateLabel').textContent = `${profile.name} vehicle state`;
    reset();
  } catch (error) {
    console.error(error);
    $('statusText').textContent = `Could not load ${profile.name.toLowerCase()} model`;
  } finally {
    $('rovClass').disabled = false;
  }
}

const floorGeo = new THREE.PlaneGeometry(120, 120, 90, 90);
const pos = floorGeo.attributes.position;
for (let i = 0; i < pos.count; i++) {
  const x = pos.getX(i), y = pos.getY(i);
  pos.setZ(i, .42 * Math.sin(x * .13) * Math.cos(y * .17) + .17 * Math.sin((x + y) * .5));
}
floorGeo.computeVertexNormals();
const floor = new THREE.Mesh(floorGeo, new THREE.MeshStandardMaterial({ color: 0x173c3d, roughness: .98, metalness: 0 }));
floor.rotation.x = -Math.PI / 2; floor.position.y = -18; floor.receiveShadow = true; scene.add(floor);

const grid = new THREE.GridHelper(80, 80, 0x34777a, 0x17454d);
grid.position.y = -17.94; grid.material.opacity = .24; grid.material.transparent = true; scene.add(grid);

const habitat = enableShadows(createHabitat());
habitat.position.set(-7, -17.92, -15);
habitat.rotation.y = -.34;
scene.add(habitat);

const particles = new THREE.BufferGeometry();
const particleData = new Float32Array(900 * 3);
for (let i = 0; i < particleData.length; i += 3) { particleData[i] = (Math.random()-.5)*40; particleData[i+1]=(Math.random()-.5)*28; particleData[i+2]=(Math.random()-.5)*40; }
particles.setAttribute('position', new THREE.BufferAttribute(particleData, 3));
const motes = new THREE.Points(particles, new THREE.PointsMaterial({ color: 0x8eced0, size: .025, transparent: true, opacity: .48 })); scene.add(motes);

const trailPoints = Array.from({length: 240}, () => new THREE.Vector3());
const trailGeo = new THREE.BufferGeometry().setFromPoints(trailPoints);
const trail = new THREE.Line(trailGeo, new THREE.LineBasicMaterial({ color: 0x4de0d1, transparent: true, opacity: .78 }));
scene.add(trail);
let lastTrailTime = 0;

const markerMat = new THREE.MeshBasicMaterial({ color: 0xf0b451, wireframe: true, transparent: true, opacity: .68 });
[[6,-5,-7],[-7,-7,-13],[3,8,-16]].forEach(([x,z,y]) => { const marker = new THREE.Mesh(new THREE.TorusGeometry(.36,.025,8,32), markerMat); marker.rotation.x=Math.PI/2; marker.position.set(x,y,z); scene.add(marker); });

function renderThrusterChannels(profile) {
  const panel = $('thrusters');
  panel.replaceChildren();
  profile.thrusters.forEach((thruster, index) => {
    const channel = document.createElement('div');
    channel.innerHTML = `<span><strong>${thruster.id}</strong>${thruster.label}</span><i><b id="thruster${index + 1}"></b></i><small>±${thruster.force.toLocaleString()} N</small>`;
    panel.appendChild(channel);
  });
  panel.setAttribute('aria-label', `${profile.thrusters.length} thruster outputs for ${profile.name}`);
}

const gamepad = new GamepadController();
const telemetry = new TelemetryRecorder();
let activeFaultIndex = 0;
const scenarioMarkerGroup = new THREE.Group();
scene.add(scenarioMarkerGroup);

function updateScenarioMarkers(waypoints) {
  scenarioMarkerGroup.clear();
  if (!waypoints) return;
  waypoints.forEach(wp => {
    const ring = new THREE.Mesh(new THREE.TorusGeometry(0.5, 0.035, 8, 32), markerMat);
    ring.rotation.x = Math.PI / 2;
    // NED to Three.js: East=X, Depth=-Y, North=-Z
    ring.position.set(wp.e, -wp.d, -wp.n);
    scenarioMarkerGroup.add(ring);
  });
}

const scoringEngine = new ScoringEngine();
const scenarioKeys = [null, 'pipeline-survey', 'habitat-docking', 'high-shear-nav', 'thruster-recovery'];
let currentScenarioIdx = 0;

function selectScenarioByIndex(idx) {
  currentScenarioIdx = idx % scenarioKeys.length;
  const key = scenarioKeys[currentScenarioIdx];
  if (!key) {
    scoringEngine.reset(null);
    $('scoringSection').style.display = 'none';
    $('scenarioBtn').textContent = 'Scenario: Free';
    updateScenarioMarkers(null);
  } else {
    const sc = TRAINING_SCENARIOS[key];
    scoringEngine.reset(sc);
    $('scoringSection').style.display = 'block';
    $('scenarioTitle').textContent = sc.title;
    $('scenarioBtn').textContent = `Scenario: ${sc.title.split(' ')[0]}`;
    sim.current.speed = sc.current.speed;
    sim.current.direction = sc.current.direction;
    $('currentOutput').textContent = `${sc.current.speed.toFixed(2)} m/s`;
    $('current').value = sc.current.speed;
    updateScenarioMarkers(sc.waypoints);
    reset();
    const startWp = sc.waypoints[0];
    sim.state[6] = startWp.n;
    sim.state[7] = startWp.e;
    sim.state[8] = startWp.d;
    targetDepth = startWp.d;
  }
}

const faultModes = [
  { label: 'None', mode: 'OK', thruster: 0, val: 1.0 },
  { label: 'T1 50% Degraded', mode: 'DEGRADED', thruster: 0, val: 0.5 },
  { label: 'T1 Stuck 100%', mode: 'STUCK', thruster: 0, val: 1.0 },
  { label: 'T1 Offline', mode: 'OFFLINE', thruster: 0, val: 0.0 },
];

function axis(positive, negative) { return (keys.has(positive) ? 1 : 0) - (keys.has(negative) ? 1 : 0); }
function readPilot() {
  const kb = { surge: axis('KeyW','KeyS'), sway: axis('KeyD','KeyA'), heave: axis('KeyF','KeyR'), yaw: axis('KeyE','KeyQ') };
  const gp = gamepad.poll();
  if (gp.active) {
    return {
      surge: Math.abs(gp.surge) > 0.05 ? gp.surge : kb.surge,
      sway: Math.abs(gp.sway) > 0.05 ? gp.sway : kb.sway,
      heave: Math.abs(gp.heave) > 0.05 ? gp.heave : kb.heave,
      yaw: Math.abs(gp.yaw) > 0.05 ? gp.yaw : kb.yaw,
    };
  }
  return kb;
}

function syncStatus() {
  const leg = demoRoute[demoLeg];
  $('statusText').textContent = !running ? (demoMode ? 'Demo paused' : 'Simulation paused') : demoMode ? `Demo route · ${leg.label}` : 'Simulation live';
  $('pauseButton').childNodes[0].nodeValue = running ? 'Pause ' : 'Resume ';
  document.body.classList.toggle('paused', !running);
}
function updateModeIndicators() {
  ['stationIndicator','headingIndicator','depthIndicator'].forEach(id => $(id).classList.toggle('active', assist));
  $('demoIndicator').classList.toggle('active', demoMode);
}
function togglePause() { running = !running; syncStatus(); }
function reset() { sim.reset(); targetDepth=2; targetHeading=sim.getEuler()[2]; demoLeg=0; demoLegStarted=sim.time; depthHistory.length=0; lastTrailTime=0; trailPoints.forEach(p=>p.set(0,0,0)); trailGeo.setFromPoints(trailPoints); syncStatus(); }
function syncCameraButton() {
  $('cameraButton').textContent = `${cameraNames[cameraMode]} `;
  const shortcut = document.createElement('kbd');
  shortcut.textContent = 'C';
  $('cameraButton').appendChild(shortcut);
  $('cameraButton').setAttribute('aria-label', `${cameraNames[cameraMode]} view; change camera`);
}
function changeCamera() { cameraMode=(cameraMode+1)%3; syncCameraButton(); }

function toggleDemo() {
  demoMode = !demoMode;
  keys.clear();
  if (demoMode) {
    assistBeforeDemo = assist;
    cameraBeforeDemo = cameraMode;
    assist = true;
    cameraMode = 0;
    syncCameraButton();
    running = true;
    demoLeg = 0;
    demoLegStarted = sim.time;
    $('controlTitle').textContent = 'Autonomous demo';
    $('controlDescription').textContent = 'The pilot follows a six-leg habitat inspection route.';
  } else {
    assist = assistBeforeDemo;
    cameraMode = cameraBeforeDemo;
    syncCameraButton();
    targetDepth = sim.state[8];
    targetHeading = sim.getEuler()[2];
    $('controlTitle').textContent = 'Keyboard control';
    $('controlDescription').textContent = 'Commands blend through a first-order actuator model.';
  }
  $('assistButton').classList.toggle('active', assist);
  $('assistButton').setAttribute('aria-pressed', assist);
  $('demoButton').setAttribute('aria-pressed', demoMode);
  $('demoButton').textContent = demoMode ? 'Exit demo ' : 'Demo ';
  const shortcut = document.createElement('kbd'); shortcut.textContent = '~'; $('demoButton').appendChild(shortcut);
  document.body.classList.toggle('demo', demoMode);
  updateModeIndicators();
  syncStatus();
}

function toggleArm() {
  armActive = !armActive;
  sim.useManipulator = armActive;
  manipulatorView.setVisible(armActive);
  const ind = $('armIndicator');
  if (ind) {
    ind.classList.toggle('active', armActive);
    const span = ind.querySelector('span');
    if (span) span.textContent = armActive ? 'ARM READY' : 'Arm Stow';
  }
}

function toggleFleet() {
  fleetActive = !fleetActive;
  multiRovView.setFollowerEnabled(fleetActive);
  const ind = $('dualRovIndicator');
  if (ind) {
    ind.classList.toggle('active', fleetActive);
    const span = ind.querySelector('span');
    if (span) span.textContent = fleetActive ? 'FLEET DUAL' : 'Fleet Solo';
  }
}

window.addEventListener('keydown', event => {
  if (event.target.tagName === 'TEXTAREA' || event.target.tagName === 'INPUT') return;
  if (['Space','KeyW','KeyA','KeyS','KeyD','KeyR','KeyF','KeyQ','KeyE','Digit1','Digit2','Digit3'].includes(event.code)) event.preventDefault();
  if (!event.repeat && event.code === 'Space') togglePause();
  if (!event.repeat && event.code === 'KeyX') reset();
  if (!event.repeat && event.code === 'KeyC') changeCamera();
  if (!event.repeat && event.code === 'KeyH') $('assistButton').click();
  if (!event.repeat && event.code === 'Backquote') toggleDemo();
  if (!event.repeat && event.code === 'KeyM') toggleArm();
  if (!event.repeat && event.code === 'KeyT') toggleFleet();
  if (!event.repeat && event.code === 'KeyG') armGrip = armGrip > 0.5 ? 0.0 : 1.0;
  keys.add(event.code);
});
window.addEventListener('keyup', event => {
  if (event.target.tagName === 'TEXTAREA' || event.target.tagName === 'INPUT') return;
  keys.delete(event.code);
});
window.addEventListener('blur', () => keys.clear());
$('pauseButton').addEventListener('click', togglePause);
$('resetButton').addEventListener('click', reset);
$('demoButton').addEventListener('click', toggleDemo);
$('cameraButton').addEventListener('click', changeCamera);
$('trailButton').addEventListener('click', () => { showTrail=!showTrail; trail.visible=showTrail; $('trailButton').textContent=`Trail ${showTrail?'on':'off'}`; $('trailButton').setAttribute('aria-pressed', showTrail); });
$('assistButton').addEventListener('click', () => { if (demoMode) return; assist=!assist; $('assistButton').classList.toggle('active',assist); $('assistButton').setAttribute('aria-pressed',assist); targetDepth=sim.state[8]; targetHeading=sim.state[11]; updateModeIndicators(); });
$('current').addEventListener('input', event => { sim.current.speed=Number(event.target.value); $('currentOutput').textContent=`${sim.current.speed.toFixed(2)} m/s`; });
$('rovClass').addEventListener('change', event => loadRovClass(event.target.value));
$('armIndicator')?.addEventListener('click', toggleArm);
$('dualRovIndicator')?.addEventListener('click', toggleFleet);

$('recordButton')?.addEventListener('click', () => {
  if (!telemetry.isRecording) {
    telemetry.startRecording();
    $('recordButton').textContent = 'Stop & Save CSV';
    $('recordButton').setAttribute('aria-pressed', 'true');
    $('recordButton').style.borderColor = 'var(--coral)';
    $('recordButton').style.color = 'var(--coral)';
  } else {
    telemetry.stopRecording();
    $('recordButton').textContent = 'Rec Telemetry';
    $('recordButton').setAttribute('aria-pressed', 'false');
    $('recordButton').style.borderColor = '';
    $('recordButton').style.color = '';

    const csv = telemetry.exportCsv();
    const blob = new Blob([csv], { type: 'text/csv' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `mss-telemetry-${Math.round(sim.time)}s.csv`;
    a.click();
    URL.revokeObjectURL(url);
  }
});

$('faultButton')?.addEventListener('click', () => {
  activeFaultIndex = (activeFaultIndex + 1) % faultModes.length;
  const f = faultModes[activeFaultIndex];
  if (f.mode === 'OK') {
    sim.failureInjector.resetAll();
    $('faultButton').textContent = 'Fault: None';
    $('faultButton').style.borderColor = '';
    $('faultButton').style.color = '';
  } else {
    sim.failureInjector.setFault(f.thruster, f.mode, f.val);
    $('faultButton').textContent = `Fault: ${f.label}`;
    $('faultButton').style.borderColor = 'var(--coral)';
    $('faultButton').style.color = 'var(--coral)';
  }
});

$('scenarioBtn')?.addEventListener('click', () => {
  selectScenarioByIndex(currentScenarioIdx + 1);
});

// Profile Parameter Import/Export & Real-Time Validation
const profileModal = $('profileModal');
const profileJsonText = $('profileJsonText');
const validationBadge = $('validationBadge');
const validationErrors = $('validationErrors');
const vehicleSummary = $('vehicleSummary');
const applyProfileBtn = $('applyProfileBtn');
let validatedCandidate = null;

function runLiveValidation() {
  const text = profileJsonText.value.trim();
  validationErrors.innerHTML = '';
  vehicleSummary.innerHTML = '';
  validatedCandidate = null;

  if (!text) {
    validationBadge.className = 'val-badge val-idle';
    validationBadge.textContent = 'Waiting for input';
    applyProfileBtn.disabled = true;
    return;
  }

  let parsed;
  try {
    parsed = JSON.parse(text);
  } catch (err) {
    validationBadge.className = 'val-badge val-invalid';
    validationBadge.textContent = 'Invalid JSON Syntax';
    const li = document.createElement('li');
    li.textContent = err.message;
    validationErrors.appendChild(li);
    applyProfileBtn.disabled = true;
    return;
  }

  const result = validateProfile(parsed);
  if (!result.valid) {
    validationBadge.className = 'val-badge val-invalid';
    validationBadge.textContent = `Schema Invalid (${result.errors.length} error${result.errors.length > 1 ? 's' : ''})`;
    result.errors.forEach(err => {
      const li = document.createElement('li');
      li.textContent = err;
      validationErrors.appendChild(li);
    });
    applyProfileBtn.disabled = true;
  } else {
    validationBadge.className = 'val-badge val-valid';
    validationBadge.textContent = 'Schema Valid ✓';
    applyProfileBtn.disabled = false;
    validatedCandidate = parsed;

    const s = result.summary;
    vehicleSummary.innerHTML = `
      <strong>${s.name}</strong> (${s.role})<br>
      <span>Mass: ${s.mass} kg · Buoyancy Ratio: ${s.buoyancyRatio}</span><br>
      <span>Thrusters: ${s.thrusterCount} units (Est. Fwd: ${s.forwardBollardEst} N, Vert: ${s.verticalBollardEst} N)</span><br>
      <span>Center of Gravity (cg): ${s.cgOffset} m</span>
    `;
  }
}

function openProfileModal() {
  profileJsonText.value = formatProfileJson(sim.profile);
  runLiveValidation();
  profileModal.showModal();
}

function closeProfileModal() {
  profileModal.close();
}

function exportProfile() {
  const text = profileJsonText.value || formatProfileJson(sim.profile);
  const blob = new Blob([text], { type: 'application/json' });
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url;
  const name = (validatedCandidate?.name || sim.profile.name).toLowerCase().replace(/[^a-z0-9]+/g, '-');
  a.download = `${name}-profile.json`;
  a.click();
  URL.revokeObjectURL(url);
}

function importProfileFile(e) {
  const file = e.target.files?.[0];
  if (!file) return;
  const reader = new FileReader();
  reader.onload = evt => {
    profileJsonText.value = evt.target.result;
    runLiveValidation();
  };
  reader.readAsText(file);
  e.target.value = '';
}

function reloadProfilePreset() {
  profileJsonText.value = formatProfileJson(ROV_PROFILES[vehicleKey]);
  runLiveValidation();
}

function applyProfileToSimulator() {
  if (!validatedCandidate) return;
  const p = validatedCandidate;
  sim.configure(vehicleKey, p);
  renderThrusterChannels(p);
  $('vehicleSpec').textContent = `${p.mass.toLocaleString()} kg · ${p.role} · ${p.thrusters.length} thrusters (Custom Profile)`;
  $('vehicleStateLabel').textContent = `${p.name} vehicle state`;
  $('statusText').textContent = `Applied custom profile: ${p.name}`;
  reset();
  closeProfileModal();
}

$('profileButton').addEventListener('click', openProfileModal);
$('closeProfileModal').addEventListener('click', closeProfileModal);
$('cancelProfileBtn').addEventListener('click', closeProfileModal);
$('profileJsonText').addEventListener('input', runLiveValidation);
$('exportProfileBtn').addEventListener('click', exportProfile);
$('importProfileFile').addEventListener('change', importProfileFile);
$('reloadProfilePresetBtn').addEventListener('click', reloadProfilePreset);
$('applyProfileBtn').addEventListener('click', applyProfileToSimulator);

// Instructor Station & Flight Debriefing Setup
const instructor = new InstructorStation(sim, scene);
const instructorModal = $('instructorModal');
const debriefModal = $('debriefModal');

$('instructorBtn')?.addEventListener('click', () => instructorModal?.showModal());
$('closeInstructorModal')?.addEventListener('click', () => instructorModal?.close());
$('doneInstructorBtn')?.addEventListener('click', () => instructorModal?.close());

$('turbiditySlider')?.addEventListener('input', e => {
  const val = Number(e.target.value);
  instructor.setTurbidity(val);
  $('turbidityValue').textContent = `${val.toFixed(3)} m⁻¹ (${val > 0.1 ? 'Turbid Silt Plume' : 'Clear Water'})`;
});

$('gustBtn')?.addEventListener('click', () => {
  instructor.injectCurrentGust(sim.current.speed + 0.3);
  $('currentOutput').textContent = `${sim.current.speed.toFixed(2)} m/s`;
  $('current').value = sim.current.speed;
});

$('failT1Degraded')?.addEventListener('click', () => instructor.injectThrusterFault(0, 'DEGRADED', 0.5));
$('failT1Offline')?.addEventListener('click', () => instructor.injectThrusterFault(0, 'OFFLINE', 0.0));
$('failT1Stuck')?.addEventListener('click', () => instructor.injectThrusterFault(0, 'STUCK', 1.0));
$('clearAllFaultsBtn')?.addEventListener('click', () => instructor.clearFaults());

function openDebrief() {
  const sc = TRAINING_SCENARIOS[scenarioKeys[currentScenarioIdx]] || { title: 'Free Flight Training', level: 'General', timeBudget: 60 };
  const scoreData = scoringEngine.computeScore();
  const report = instructor.generateDebriefingReport(sc, scoreData, 'ROV Pilot');
  $('debriefReportText').value = report;
  debriefModal?.showModal();
}

$('debriefBtn')?.addEventListener('click', openDebrief);
$('closeDebriefModal')?.addEventListener('click', () => debriefModal?.close());
$('doneDebriefBtn')?.addEventListener('click', () => debriefModal?.close());

$('downloadDebriefMdBtn')?.addEventListener('click', () => {
  const text = $('debriefReportText').value;
  const blob = new Blob([text], { type: 'text/markdown' });
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url;
  a.download = `flight-debrief-${Math.round(sim.time)}s.md`;
  a.click();
  URL.revokeObjectURL(url);
});

$('downloadDebriefJsonBtn')?.addEventListener('click', () => {
  const scoreData = scoringEngine.computeScore();
  const blob = new Blob([JSON.stringify(scoreData, null, 2)], { type: 'application/json' });
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url;
  a.download = `flight-metrics-${Math.round(sim.time)}s.json`;
  a.click();
  URL.revokeObjectURL(url);
});

// Localization Toggle
$('langBtn')?.addEventListener('click', () => {
  const nextLang = getLanguage() === 'en' ? 'no' : 'en';
  setLanguage(nextLang);
  $('langBtn').textContent = nextLang === 'en' ? 'NO/EN' : 'EN/NO';
  $('statusText').textContent = t(running ? 'resume' : 'pause');
});

function resize() { const {clientWidth:w,clientHeight:h}=sceneHost; renderer.setSize(w,h,false); camera.aspect=w/h; camera.updateProjectionMatrix(); }
new ResizeObserver(resize).observe(sceneHost);

function updateSimulation(frameDt) {
  let pilot = readPilot();
  if (demoMode) {
    const waypoint = demoRoute[demoLeg];
    const dn = waypoint.n - sim.state[6], de = waypoint.e - sim.state[7];
    const horizontalDistance = Math.hypot(dn, de);
    const course = Math.atan2(de, dn);
    const relativeBearing = Math.atan2(Math.sin(course - sim.state[11]), Math.cos(course - sim.state[11]));
    const demand = clamp(horizontalDistance * .14, .18, .72);
    pilot = {
      surge: clamp(Math.cos(relativeBearing) * demand, -.55, .72),
      sway: clamp(Math.sin(relativeBearing) * demand, -.62, .62),
      heave: 0,
      yaw: 0,
    };
    targetDepth = waypoint.d;
    targetHeading = course;
    if ((horizontalDistance < 1.35 && Math.abs(waypoint.d - sim.state[8]) < .75) || sim.time - demoLegStarted > 28) {
      demoLeg = (demoLeg + 1) % demoRoute.length;
      demoLegStarted = sim.time;
      syncStatus();
    }
  }
  if (assist) {
    if (pilot.heave) targetDepth = clamp(targetDepth + pilot.heave * frameDt * .65, .3, 30);
    if (pilot.yaw) targetHeading = sim.getEuler()[2];
  }
  sim.setPilot(pilot, assist, { depth: targetDepth, heading: targetHeading });
  accumulator += Math.min(frameDt, .05);
  while (accumulator >= .01) {
    sim.step(.01);
    if (telemetry.isRecording) {
      telemetry.recordFrame(sim.time, sim.state, sim.thrusters, pilot, assist);
    }
    if (scenarioKeys[currentScenarioIdx]) {
      const sc = TRAINING_SCENARIOS[scenarioKeys[currentScenarioIdx]];
      scoringEngine.update(.01, sim.state.slice(6, 9), sim.state.slice(0, 6), sim.thrusters, sim.tetherState, sim.contactState);
      if (sc.faultInjection && sim.time >= sc.faultInjection.triggerTime) {
        sim.failureInjector.setFault(sc.faultInjection.thruster, sc.faultInjection.mode, 0.0);
      }
    }
    accumulator -= .01;
  }
}

function updateVisuals(dt) {
  const [n,e,d] = sim.state.slice(6, 9);
  const [phi,theta,psi] = sim.getEuler();
  // MSS NED/FSD -> Three.js: East=X, Up=Y, North=-Z.
  rov.position.set(e, -d, -n);
  rov.rotation.set(theta, -psi, -phi);
  const follow = new THREE.Vector3(e,-d,-n);
  if (cameraMode === 0) {
    const forward = new THREE.Vector3(
      Math.sin(psi) * Math.cos(theta),
      Math.sin(theta),
      -Math.cos(psi) * Math.cos(theta),
    ).normalize();
    const flatForward = new THREE.Vector3(forward.x, 0, forward.z).normalize();
    const chaseDistance = sim.profile.cameraDistance;
    const desiredPosition = follow.clone().addScaledVector(flatForward, -chaseDistance).add(new THREE.Vector3(0, Math.max(2.1, chaseDistance * .4), 0));
    const desiredLook = follow.clone().addScaledVector(forward, 2.25).add(new THREE.Vector3(0, .2, 0));
    camera.up.set(0, 1, 0);
    camera.position.lerp(desiredPosition, 1-Math.exp(-dt*3.2));
    cameraLookTarget.lerp(desiredLook, 1-Math.exp(-dt*4.4));
    camera.lookAt(cameraLookTarget);
    controls.enabled=false;
  } else if (cameraMode === 1) {
    camera.up.set(0, 0, -1);
    camera.position.lerp(follow.clone().add(new THREE.Vector3(0,10,.01)), 1-Math.exp(-dt*3));
    camera.lookAt(follow);
    controls.enabled=false;
  } else {
    camera.up.set(0, 1, 0);
    const fpvScale = sim.profile.leverScale;
    const offset = new THREE.Vector3(.15*fpvScale,.18*fpvScale,-.48*fpvScale).applyAxisAngle(new THREE.Vector3(0,1,0),-psi); camera.position.copy(follow.clone().add(offset));
    camera.lookAt(follow.clone().add(new THREE.Vector3(Math.sin(psi)*8,theta*5,-Math.cos(psi)*8))); controls.enabled=false;
  }
  if (controls.enabled) controls.update();
  rovModel?.userData.tick?.(sim.time);
  // Compose the spin on the exported node's local shaft axis so horizontal
  // and vertical thrusters retain their independently authored orientation.
  if (running) {
    rovModel?.userData.rotors?.forEach((rotor, index) => {
      const channel = Number.isInteger(rotor.userData.channel_index) ? rotor.userData.channel_index : index;
      const command = sim.thrusters[channel] ?? 0;
      rotor.rotateOnAxis(rotorShaftAxis, dt * 28 * command);
    });
    // In Three.js: East=+X, North=-Z. Direction betaVc is clockwise from North.
    const currentEast = Math.sin(sim.current.direction) * sim.current.speed;
    const currentNorth = Math.cos(sim.current.direction) * sim.current.speed;
    motes.position.x += currentEast * dt * .08;
    motes.position.z -= currentNorth * dt * .08;
  }
  habitat.userData.tick?.(sim.time);
  if (sim.time-lastTrailTime>.12) { lastTrailTime=sim.time; trailPoints.shift(); trailPoints.push(follow.clone()); trailGeo.setFromPoints(trailPoints); }

  // Animate UVMS manipulator arm if active
  if (armActive) {
    if (keys.has('Digit1')) sim.manipulator.q[0] = clamp(sim.manipulator.q[0] + (keys.has('ShiftLeft') ? -0.02 : 0.02), -1.5, 1.5);
    if (keys.has('Digit2')) sim.manipulator.q[1] = clamp(sim.manipulator.q[1] + (keys.has('ShiftLeft') ? -0.02 : 0.02), -0.8, 1.2);
    if (keys.has('Digit3')) sim.manipulator.q[2] = clamp(sim.manipulator.q[2] + (keys.has('ShiftLeft') ? -0.02 : 0.02), -1.5, 1.5);
    manipulatorView.updateJoints(sim.manipulator.q, armGrip);
  }

  // Update dynamic 3D umbilical cables and dual-tether anti-entanglement visualizer
  lastMultiRes = multiRovView.update(dt, sim.state.slice(6, 9), sim.lumpedTether);
}

let lastMultiRes = null;

function drawDepthChart() {
  const canvas=$('depthChart'), ctx=canvas.getContext('2d'), w=canvas.width,h=canvas.height;
  ctx.clearRect(0,0,w,h); ctx.strokeStyle='rgba(144,196,198,.16)'; ctx.lineWidth=1;
  for(let i=1;i<4;i++){ctx.beginPath();ctx.moveTo(0,i*h/4);ctx.lineTo(w,i*h/4);ctx.stroke();}
  if(depthHistory.length<2)return;
  const values=depthHistory.map(x=>x.depth), min=Math.min(...values,targetDepth)-.5,max=Math.max(...values,targetDepth)+.5, span=Math.max(1,max-min);
  const y=v=>h-(v-min)/span*h;
  ctx.setLineDash([7,7]);ctx.strokeStyle='#e8ae55';ctx.beginPath();ctx.moveTo(0,y(targetDepth));ctx.lineTo(w,y(targetDepth));ctx.stroke();ctx.setLineDash([]);
  ctx.strokeStyle='#52d4cc';ctx.lineWidth=3;ctx.beginPath();depthHistory.forEach((x,i)=>{const px=i/(depthHistory.length-1)*w,py=y(x.depth);i?ctx.lineTo(px,py):ctx.moveTo(px,py);});ctx.stroke();
}

let uiClock=0;
function updateUI(dt) {
  uiClock+=dt; if(uiClock<.08)return; uiClock=0;
  const [u,v,w] = sim.state.slice(0, 3);
  const d = sim.state[8];
  const [phi,theta,psi] = sim.getEuler();
  const degrees=x=>x*180/Math.PI, hdg=(degrees(psi)+360)%360;
  $('depth').textContent=d.toFixed(2); $('depthReadout').textContent=`${d.toFixed(1)} m`; $('depthTarget').textContent=`${targetDepth.toFixed(2)} m`;
  $('depthBar').style.width=`${clamp(d/30*100,1,100)}%`; $('roll').textContent=`${degrees(phi).toFixed(1)}°`; $('pitch').textContent=`${degrees(theta).toFixed(1)}°`;
  $('heading').textContent=`${String(Math.round(hdg)).padStart(3,'0')}°`; $('headingReadout').textContent=`HDG ${String(Math.round(hdg)).padStart(3,'0')}°`;
  $('compassHeading').textContent=`${String(Math.round(hdg)).padStart(3,'0')}°`; $('headingDial').style.setProperty('--heading',`${-hdg}deg`);
  $('speed').textContent=`${Math.hypot(u,v,w).toFixed(2)} m/s`; $('surge').textContent=`${u.toFixed(2)} m/s`; $('sway').textContent=`${v.toFixed(2)} m/s`; $('heave').textContent=`${w.toFixed(2)} m/s`; $('altitude').textContent=`${Math.max(0,18-d).toFixed(2)} m`; $('currentValue').textContent=`${(sim.localCurrentSpeed ?? sim.current.speed).toFixed(2)} m/s`;
  if (sim.tetherState) {
    const tp = $('tetherPayout');
    if (tp) tp.textContent = `${sim.tetherState.payout.toFixed(1)} m`;
    const isSnag = sim.tetherState.snag || sim.tetherState.advisory;
    const tInd = $('tetherIndicator');
    if (tInd) {
      tInd.classList.toggle('active', !isSnag);
      const span = tInd.querySelector('span');
      if (span) span.textContent = sim.tetherState.snag ? 'SNAG ALERT' : (sim.tetherState.advisory ? 'TETHER 88%' : 'Tether OK');
    }
  }
  if (armActive) {
    const fk = sim.manipulator.computeForwardKinematics();
    $('armReach').textContent = `${fk.eePos[0].toFixed(2)} m`;
  } else {
    $('armReach').textContent = 'STOW';
  }
  if (fleetActive && lastMultiRes?.clearance) {
    const clr = lastMultiRes.clearance;
    const clrEl = $('dualTetherClearance');
    clrEl.textContent = `${clr.minDistance.toFixed(1)} m`;
    clrEl.style.color = clr.criticalAlert ? 'var(--coral)' : (clr.advisory ? 'var(--amber)' : 'var(--cyan)');
  } else {
    $('dualTetherClearance').textContent = 'SOLO';
    $('dualTetherClearance').style.color = '';
  }
  if (sim.sensorReadings?.dvl) {
    const dvlEl = $('dvlLock');
    if (dvlEl) {
      dvlEl.textContent = sim.sensorReadings.dvl.bottomLock ? 'LOCK' : 'LOST';
      dvlEl.style.color = sim.sensorReadings.dvl.bottomLock ? 'var(--cyan)' : 'var(--coral)';
    }
  }
  $('gamepadIndicator')?.classList.toggle('active', gamepad.isConnected);
  $('attitudeDial').style.setProperty('--roll',`${-degrees(phi)}deg`); $('attitudeDial').style.setProperty('--pitch',`${clamp(degrees(theta)*.8,-24,24)}px`);
  const minutes=Math.floor(sim.time/60),seconds=(sim.time%60).toFixed(1).padStart(4,'0'); $('elapsed').textContent=`${String(minutes).padStart(2,'0')}:${seconds}`;
  let load=0; sim.thrusters.forEach((x,i)=>{ $('thruster'+(i+1)).style.setProperty('--load',`${Math.abs(x)*100}%`); $('thruster'+(i+1)).classList.toggle('reverse',x<0); load+=x*x; });
  if (scenarioKeys[currentScenarioIdx]) {
    const scoreData = scoringEngine.computeScore();
    $('liveScore').textContent = scoreData.score.toFixed(1);
    $('liveGrade').textContent = `Grade ${scoreData.grade}`;
    $('scoreInfractions').textContent = scoreData.infractions.length ? scoreData.infractions.join(', ') : 'None';
    $('scoreInfractions').style.color = scoreData.infractions.length ? 'var(--coral)' : '#6e9298';
  }
  depthHistory.push({time:sim.time,depth:d}); while(depthHistory.length>375)depthHistory.shift(); drawDepthChart();
  drawSonar();
}

function animate(now) {
  requestAnimationFrame(animate); const dt=Math.min((now-last)/1000,.05); last=now;
  if(running) updateSimulation(dt); updateVisuals(dt); updateUI(dt); renderer.render(scene,camera);
}
resize(); requestAnimationFrame(animate);
updateModeIndicators();
loadRovClass(vehicleKey);
updateUI(0.1);
drawSonar();
