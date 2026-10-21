import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import loadMujoco from '@mujoco/mujoco';
import { createDualSimulation, TABLETOP_TASKS } from '../src/dual-simulation.js';

const models = fileURLToPath(new URL('../../rebotarm_mujoco_rs/models/', import.meta.url));
const xml = await readFile(path.join(models, 'rs_dual_tabletop_scene.xml'), 'utf8');
const mujoco = await loadMujoco();
const vfs = new mujoco.MjVFS();
for (const file of new Set([...xml.matchAll(/file="([^"]+)"/g)].map((match) => match[1]))) {
  vfs.addBuffer(`meshes/${file}`, await readFile(path.join(models, 'meshes', file)));
}
const model = mujoco.MjModel.from_xml_string(xml, vfs);
const data = new mujoco.MjData(model);
const simulation = createDualSimulation(mujoco, model, data);
const startQpos = Array.from(data.qpos);
const bodyType = mujoco.mjtObj.mjOBJ_BODY.value;
const fingerIds = new Set(['left', 'right'].flatMap(side => ['left', 'right'].map(finger =>
  mujoco.mj_name2id(model, bodyType, `${side}_gripper_${finger}`))));
const objectIds = new Set(TABLETOP_TASKS.map(task => mujoco.mj_name2id(model, bodyType, task.id)));
// Check the actual compiled collision mesh, not the non-colliding visual geom.
for (const [name, minimum] of [['egg', [.036, .042, .048]], ['apple', [.045, .045, .038]], ['lime', [.041, .035, .042]]]) {
  const geomId = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM.value, `${name}_collision`);
  assert.equal(model.geom_type[geomId], mujoco.mjtGeom.mjGEOM_MESH.value);
  const meshId = model.geom_dataid[geomId];
  const min = [Infinity, Infinity, Infinity], max = [-Infinity, -Infinity, -Infinity];
  for (let v = 0; v < model.mesh_vertnum[meshId]; v++) {
    const offset = (model.mesh_vertadr[meshId] + v) * 3;
    for (let axis = 0; axis < 3; axis++) {
      let value = 0;
      for (let j = 0; j < 3; j++) value += data.geom_xmat[geomId*9+axis*3+j] * model.mesh_vert[offset+j];
      min[axis] = Math.min(min[axis], value); max[axis] = Math.max(max[axis], value);
    }
  }
  minimum.forEach((size, axis) => assert(max[axis]-min[axis] >= size-.0001, `${name}: collision surface is smaller than the visible prop on axis ${axis}`));
}
let maximumPenetration = 0;
function sampleFingerContacts() {
  const contacts = data.contact;
  try {
    for (let i = 0; i < Math.min(data.ncon, contacts.size()); i++) {
      const contact = contacts.get(i);
      try {
        const a = model.geom_bodyid[contact.geom1], b = model.geom_bodyid[contact.geom2];
        if ((fingerIds.has(a) && objectIds.has(b)) || (fingerIds.has(b) && objectIds.has(a))) {
          maximumPenetration = Math.max(maximumPenetration, -contact.dist);
        }
      } finally { contact.delete(); }
    }
  } finally { contacts.delete(); }
}
simulation.step(500);
simulation.start();
let last = '';
for (let step = 0; step < 12000 && simulation.state().running; step++) {
  simulation.step(10);
  sampleFingerContacts();
  const s = simulation.state();
  const stages = Object.values(s.arms).map((arm) => `${arm.selectedId}:${arm.stage}`).join(' / ');
  if (stages !== last) { console.log(data.time.toFixed(2), stages); last = stages; }
}
const final = simulation.state();
console.log(JSON.stringify(final));
assert.equal(final.error, '', `Physical grasp failed: ${final.error}`);
assert.equal(final.running, false, 'Task timed out');
assert.equal(final.completed.length, 4);
for (const task of TABLETOP_TASKS) {
  const id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY.value, task.id);
  const [x, y, z] = data.xpos.subarray(id * 3, id * 3 + 3);
  const targetY = (task.arm === 'left' ? -.34 : .34) + task.slot;
  assert.ok(Math.hypot(x - .40, y - targetY) < .025, `${task.id} missed container: ${x},${y},${z}`);
  assert.ok(z > .85 && z < .90, `${task.id} fell or remained airborne`);
}
simulation.reset();
assert.deepEqual(Array.from(data.qpos), startQpos, 'Reset must restore both arms and all objects');
simulation.start(['egg']);
simulation.togglePause();
const pausedTime = data.time;
simulation.step(100);
assert.equal(data.time, pausedTime);
simulation.togglePause();
simulation.step(10);
assert.ok(data.time > pausedTime);
assert.ok(Array.from(data.qpos).every(Number.isFinite));
// A selected item must also work from a fresh scene, with irregular render batches.
for (const task of TABLETOP_TASKS) {
  simulation.reset();
  simulation.step(300);
  assert.equal(simulation.start([task.id]), true);
  for (let frame = 0; frame < 4000 && simulation.state().running; frame++) {
    simulation.step([16, 33, 67, 120][frame % 4]);
    sampleFingerContacts();
  }
  assert.equal(simulation.state().error, '', `Single-item ${task.id} failed`);
  assert.deepEqual(simulation.state().completed, [task.id]);
  assert.equal(simulation.start([task.id]), false, 'Do not re-grasp an already stored object');
}
assert(maximumPenetration < .0015, `Finger penetration reached ${maximumPenetration*1000} mm`);
console.log(`dual tabletop: sorting, individual items, collision shapes, reset and pause passed; maximum finger penetration ${(maximumPenetration*1000).toFixed(3)} mm`);
data.delete(); model.delete(); vfs.delete();
