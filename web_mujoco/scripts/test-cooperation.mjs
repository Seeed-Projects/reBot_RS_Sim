import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import loadMujoco from '@mujoco/mujoco';
import { createDualSimulation } from '../src/dual-simulation.js';
import { createTabletopProp } from '../src/tabletop-props-view.js';

const cupVisual = createTabletopProp('cup_visual');
const handleVertices = cupVisual.getObjectByName('cup_handle').geometry.attributes.position;
for (let i = 0; i < handleVertices.count; i++) {
  assert.ok(Math.hypot(handleVertices.getX(i), handleVertices.getY(i)) >= .0324,
    'Handle must not protrude into the cup cavity');
}
cupVisual.traverse(object => { object.geometry?.dispose(); object.material?.dispose(); });

const models = fileURLToPath(new URL('../../rebotarm_mujoco_rs/models/', import.meta.url));
const xml = await readFile(path.join(models, 'rs_dual_tabletop_scene.xml'), 'utf8');
const mujoco = await loadMujoco();
const vfs = new mujoco.MjVFS();
for (const file of new Set([...xml.matchAll(/file="([^"]+)"/g)].map((match) => match[1]))) vfs.addBuffer(`meshes/${file}`, await readFile(path.join(models, 'meshes', file)));
const model = mujoco.MjModel.from_xml_string(xml, vfs);
const data = new mujoco.MjData(model);
const sim = createDualSimulation(mujoco, model, data);
console.log('Scene compiled, planning collaboration');
const began = sim.startCooperation();
console.log('Plan:', began ? 'ready' : sim.state().error);
assert.ok(began, sim.state().error);
let previous = '';
const visited = new Set();
let checkedCentralGrasp = false;
for (let frame = 0; frame < 7000 && sim.state().running; frame++) {
  sim.step([10, 20, 33, 67][frame % 4]);
  const s = sim.state().cooperation;
  visited.add(s.phase);
  if (s.phase === 'grasp' && s.spoonHeld) {
    const body = sim.arms.right.ik.bodyId;
    const point = Array.from(data.xpos.slice(body * 3, body * 3 + 3));
    assert.ok(Math.abs(point[0] - (.326 + .4314) / 2) < .004, 'Grasp the centre of the support span');
    assert.ok(Math.abs(point[1] - .085) < .004);
    assert.ok(data.xmat[body * 9 + 6] < -.995, 'Gripper must point vertically down at grasp');
    for (const name of ['joint7', 'joint_left', 'joint_right']) {
      assert.ok(Math.abs(data.qpos[sim.arms.right.joints.byName[name].qposadr] - .0034) < .001,
        'Close the measured spoon fingers before attaching or lifting');
    }
    assert.ok(point[2] < .887, 'Lower the fingertips past the spoon before closing');
    assert.ok(s.graspContact.bilateral, 'Both fingers must contact the spoon before attachment');
    assert.ok(s.graspContact.penetration < .0008, 'Do not close through the spoon');
    checkedCentralGrasp = true;
  }
  if (previous !== s.phase) { console.log(data.time.toFixed(2), s.phase); previous = s.phase; }
}
if (sim.state().error) {
  console.log('Failed state', JSON.stringify(sim.state().cooperation));
  for (const [side, arm] of Object.entries(sim.arms)) console.log(side, 'tcp', Array.from(data.xpos.slice(arm.ik.bodyId * 3, arm.ik.bodyId * 3 + 3)), 'q',Array.from(data.xquat.slice(arm.ik.bodyId*4,arm.ik.bodyId*4+4)), 'telemetry', arm.physics.telemetry());
}
assert.equal(sim.state().error, '');
assert.ok(sim.state().cooperation.complete);
assert.ok(visited.has('insert'));
assert.ok(checkedCentralGrasp, 'Must validate the actual central, vertical grasp');
assert.ok(sim.state().cooperation.maximumCupHeight > .90, 'Cup must actually be lifted');
assert.ok(sim.state().cooperation.bowl.inside);
assert.equal(sim.state().cooperation.cupHeld, false);
assert.equal(sim.state().cooperation.spoonHeld, false);
sim.step(1000);
assert.ok(sim.state().cooperation.bowl.inside, 'Spoon must remain in the cup after release');
sim.reset();
assert.equal(sim.state().cooperation.complete, false);
assert.equal(sim.state().cooperation.cupHeld, false);
console.log('Cup/spoon: synchronized grasp, lift, insertion, physical release, settling and reset passed');
data.delete(); model.delete(); vfs.delete();
