import assert from 'node:assert/strict';
import { mkdir, readFile, writeFile } from 'node:fs/promises';
import path from 'node:path';
import loadMujoco from '@mujoco/mujoco';
import { createDualSimulation } from '../src/dual-simulation.js';
import { COOP_PHASES } from '../src/cup-spoon-cooperation.js';

const option = (name) => process.argv.find(arg => arg.startsWith(`--${name}=`))?.split('=').slice(1).join('=');
const selectedSeeds = option('seeds')?.split(',').map(Number);
if (selectedSeeds) assert(selectedSeeds.every(seed => Number.isInteger(seed) && seed >= 0 && seed <= 0xffffffff), '--seeds must be uint32 values');
const count = selectedSeeds?.length ?? Number(option('cases') || 12);
assert(Number.isInteger(count) && count > 0 && count <= 1000, '--cases must be between 1 and 1000');
const reportPath = path.resolve(option('report') || '.reports/cooperation-benchmark.json');
const root = new URL('../../rebotarm_mujoco_rs/models/', import.meta.url);
const xml = await readFile(new URL('rs_dual_tabletop_scene.xml', root), 'utf8');
const mujoco = await loadMujoco(), vfs = new mujoco.MjVFS();
for (const file of new Set([...xml.matchAll(/file="([^"]+)"/g)].map(match => match[1]))) {
  vfs.addBuffer(`meshes/${file}`, await readFile(new URL(`meshes/${file}`, root)));
}
const model = mujoco.MjModel.from_xml_string(xml, vfs), data = new mujoco.MjData(model);
const simulation = createDualSimulation(mujoco, model, data);
const initialQpos = Array.from(data.qpos), initialRestPositions = Array.from(data.mocap_pos), initialRestRotations = Array.from(data.mocap_quat);
const body = name => mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY.value, name);
const cup = body('coop_cup'), spoon = body('coop_spoon');
const objectBodies = new Set([cup, spoon]);
const friction = Array.from(model.geom_friction);
const seeds = selectedSeeds || [1, 2, 3, 4, 7, 17, 42, 73, 1234, 2026, 65535, 0xffffffff];
const results = [];
function finish() {
  for (let frame = 0; frame < 4000 && simulation.state().running; frame++) simulation.step([16, 33, 67, 120][frame % 4]);
  const state = simulation.state().cooperation;
  if (state.complete) simulation.step(1000);
  return simulation.state().cooperation;
}
try {
  // The same seed must restore both objects and their physical fixture exactly.
  assert.throws(() => simulation.setCooperationLayout(NaN), RangeError);
  assert.deepEqual(Array.from(data.qpos), initialQpos, 'Reject invalid input without mutating the scene');
  simulation.setCooperationLayout(17);
  const seededQpos = Array.from(data.qpos), seededRests = Array.from(data.mocap_pos);
  simulation.setCooperationLayout(17);
  assert.deepEqual(Array.from(data.qpos), seededQpos, 'Seeded physics reset must be reproducible');
  assert.deepEqual(Array.from(data.mocap_pos), seededRests, 'Fixtures must not accumulate transforms');
  assert.notDeepEqual(seededRests, initialRestPositions, 'Move the spoon fixture with the spoon');
  simulation.setCooperationLayout(42);
  const supportedAt = simulation.state().cooperation.spoonPosition;
  simulation.step(6000);
  const supportedNow = simulation.state().cooperation.spoonPosition;
  assert(Math.hypot(...supportedNow.map((value, axis) => value - supportedAt[axis])) < .001,
    'The relocated spoon must remain on the physical fixture while both arms are idle');
  simulation.reset();
  assert.deepEqual(Array.from(data.qpos), initialQpos);
  assert.deepEqual(Array.from(data.mocap_pos), initialRestPositions);
  assert.deepEqual(Array.from(data.mocap_quat), initialRestRotations);

  for (let index = 0; index <= count; index++) {
    const seed = index === 0 ? null : seeds[index - 1] ?? (Math.imul(index, 2654435761) >>> 0);
    const frictionScale = index === 0 ? 1 : [0.9, 1, 1.1][index % 3];
    model.geom_friction.set(friction);
    for (let geom = 0; geom < model.ngeom; geom++) if (objectBodies.has(model.geom_bodyid[geom])) {
      for (let axis = 0; axis < 3; axis++) model.geom_friction[geom * 3 + axis] *= frictionScale;
    }
    const began = performance.now();
    const started = simulation.startCooperation({ seed });
    if (started && index === 1) {
      assert.equal(simulation.setCooperationLayout(123), false, 'Cannot move objects during a task');
      simulation.setPaused(true);
      const time = data.time, positions = Array.from(data.qpos);
      simulation.step(250);
      assert.equal(data.time, time); assert.deepEqual(Array.from(data.qpos), positions);
      simulation.setPaused(false);
    }
    const state = finish();
    const visited = state.history.map(entry => entry.phase);
    const success = started && state.complete && !state.error && state.bowl.inside && state.cupUpright > .95 && !state.cupHeld && !state.spoonHeld;
    results.push({ seed, frictionScale, success, failure: state.failure,
      seconds: state.metrics.elapsed, metrics: state.metrics, bowl: state.bowl,
      milliseconds: Math.round(performance.now() - began), phases: visited });
    if (success) {
      assert.deepEqual(visited, COOP_PHASES.map(phase => phase[0]));
      assert(state.graspContact.bilateral, 'Both spoon fingers must have verified contact');
      assert(state.metrics.graspWaitSeconds >= .65, 'Wait for measured closure before lifting');
      assert(state.maximumCupHeight > .90, 'Actually lift the cup');
    }
    console.log(`${seed === null ? 'default' : `seed ${seed}`} / friction ${frictionScale}: ${success ? 'PASS' : `FAIL ${state.failure?.phase}: ${state.failure?.detail || 'timeout/unstable'}`} (${state.metrics.elapsed.toFixed(2)} sim s)`);
  }

  // Failures must be observable and stop the task instead of attaching remotely.
  model.geom_friction.set(friction);
  simulation.setCooperationLayout(null);
  const cupJoint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT.value, 'coop_cup_free');
  data.qpos[model.jnt_qposadr[cupJoint]] = 5;
  mujoco.mj_forward(model, data);
  assert.equal(simulation.startCooperation({ resetScene: false }), false);
  assert.equal(simulation.state().cooperation.failure.phase, 'planning');
  assert.equal(simulation.state().cooperation.failure.code, 'planning-failed');
  assert.equal(simulation.state().cooperation.cupHeld, false);
  assert.equal(simulation.state().cooperation.spoonHeld, false);

  assert(simulation.startCooperation({ seed: 17 }), 'Retry must clear the previous failure');
  while (simulation.state().cooperation.phase === 'approach') simulation.step(10);
  const spoonJoint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT.value, 'coop_spoon_free');
  data.qpos[model.jnt_qposadr[spoonJoint] + 1] += .025;
  mujoco.mj_forward(model, data);
  finish();
  assert.equal(simulation.state().cooperation.failure?.code, 'object-moved-before-grasp');
  assert.equal(simulation.state().cooperation.failure.phase, 'grasp');
  assert.equal(simulation.state().cooperation.spoonHeld, false);
  assert(simulation.startCooperation({ seed: 17 }));
  assert.equal(finish().complete, true, 'Recover by reinitializing the same layout');

  const passed = results.filter(result => result.success).length;
  const report = { version: 1, source: 'MuJoCo ground-truth poses; assisted grasp',
    range: { translationMetres: { x: .010, y: .008 }, yawRadians: .08, frictionScale: [.9, 1.1] },
    total: results.length, passed, successRate: passed / results.length, results };
  await mkdir(path.dirname(reportPath), { recursive: true });
  await writeFile(reportPath, JSON.stringify(report, null, 2) + '\n');
  console.log(`${passed}/${results.length} passed. Report: ${reportPath}`);
  assert.equal(passed, results.length, 'A randomized cooperation case failed; inspect the JSON report');
} finally { data.delete(); model.delete(); vfs.delete(); }
