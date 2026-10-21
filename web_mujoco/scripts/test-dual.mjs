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
simulation.step(500);
simulation.start();
let last = '';
for (let step = 0; step < 12000 && simulation.state().running; step++) {
  simulation.step(10);
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
  }
  assert.equal(simulation.state().error, '', `Single-item ${task.id} failed`);
  assert.deepEqual(simulation.state().completed, [task.id]);
  assert.equal(simulation.start([task.id]), false, 'Do not re-grasp an already stored object');
}
console.log('dual tabletop: both physical sorting queues, individual items, containers, reset and pause passed');
data.delete(); model.delete(); vfs.delete();
