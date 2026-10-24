import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import loadMujoco from '@mujoco/mujoco';
import { createDualSimulation } from '../src/dual-simulation.js';
import { createTabletopProp } from '../src/tabletop-props-view.js';
import { Quaternion, Vector3 } from 'three';
import { FEEDING_LAYOUT } from '../src/feeding-layout.js';
import { COFFEE_LAYOUT } from '../src/coffee-view.js';

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
const sim = createDualSimulation(mujoco, model, data, {graspMode:'contact'});
assert.throws(() => sim.startCooperation({task:'coffee-feeding',seed:1}), /standard layout/);
console.log('Scene compiled, planning collaboration');
const began = sim.startCooperation({task:'coffee-feeding'});
console.log('Plan:', began ? 'ready' : sim.state().error);
assert.ok(began, sim.state().error);
let previous = '';
const visited = new Set();
let checkedCentralGrasp = false;
let tasteStartedAt = null, tasteDuration = 0;
let previousFill = 0;
const bodyType = mujoco.mjtObj.mjOBJ_BODY.value;
const spoolId = mujoco.mj_name2id(model, bodyType, 'spool');
const cupId = mujoco.mj_name2id(model, bodyType, 'coop_cup');
const leftHand = new Set(['left_gripper_end', 'left_gripper_coupler', 'left_gripper_left', 'left_gripper_right']
  .map(name => mujoco.mj_name2id(model, bodyType, name)));
leftHand.add(cupId);
const spoolStart = Array.from(data.xpos.subarray(spoolId*3, spoolId*3+3));
let maximumSpoolMovement = 0, spoolContacts = 0;
const spoonId = mujoco.mj_name2id(model, bodyType, 'coop_spoon');
const rightHand = new Set(['right_gripper_end', 'right_gripper_coupler', 'right_gripper_left', 'right_gripper_right'].map(name => mujoco.mj_name2id(model, bodyType, name)));
rightHand.add(spoonId);
const cupContactPhases = new Map();
for (let frame = 0; frame < 7000 && sim.state().running; frame++) {
  sim.step([10, 20, 33, 67][frame % 4]);
  const s = sim.state().cooperation;
  const spoolMovement = Math.hypot(...spoolStart.map((value, axis) => data.xpos[spoolId*3+axis]-value));
  if (spoolMovement > maximumSpoolMovement) {
    if (maximumSpoolMovement < .003 && spoolMovement >= .003) console.log('Spool disturbed during', s.phase, 'at', data.time.toFixed(3));
    maximumSpoolMovement = spoolMovement;
  }
  const contacts = data.contact;
  try {
    for (let i = 0; i < Math.min(data.ncon, contacts.size()); i++) {
      const contact = contacts.get(i);
      try {
        const a = model.geom_bodyid[contact.geom1], b = model.geom_bodyid[contact.geom2];
        if (['align', 'insert', 'stir', 'withdraw', 'return-spoon', 'reinsert'].includes(s.phase) && contact.dist < .001 &&
            ((a === cupId && rightHand.has(b)) || (b === cupId && rightHand.has(a)))) {
          const names = [contact.geom1, contact.geom2].map(geom => mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM.value, geom));
          const key = `${s.phase}:${names.join('/')}`;
          const old = cupContactPhases.get(key) || 0;
          cupContactPhases.set(key, Math.max(old, -contact.dist));
        }
        if (contact.dist < 0 && ((a === spoolId && leftHand.has(b)) || (b === spoolId && leftHand.has(a)))) {
          if (!spoolContacts) console.log('Cup/hand hit spool during', s.phase, 'at', data.time.toFixed(3));
          spoolContacts++;
        }
      } finally { contact.delete(); }
    }
  } finally { contacts.delete(); }
  visited.add(s.phase);
  assert.ok(s.coffeeFill >= previousFill, 'Coffee level must not disappear when the cup moves');
  previousFill = s.coffeeFill;
  if (s.phase === 'dispense') {
    assert.ok(s.dispensing && s.cupHeld);
    assert.ok(Math.hypot(s.cupPosition[0]-COFFEE_LAYOUT.spout[0], s.cupPosition[1]-COFFEE_LAYOUT.spout[1]) < .008,
      'Pour into the measured cup opening');
    assert.ok(COFFEE_LAYOUT.spout[2]-s.cupPosition[2] > .05, 'Spout must clear the cup rim');
  }
  if (s.phase === 'stir') {
    assert.ok(s.bowl.inside, 'Keep the spoon bowl inside the cup throughout stirring');
    assert.ok(s.cupHeld && s.spoonHeld, 'Keep both grasps active while stirring');
    assert.ok(s.cupUpright > .97, 'Keep the cup upright during stirring');
    assert.equal(s.coffeeFill, 1, 'Fill the cup before stirring');
    assert.ok(s.bowl.local[2] < .024, `Keep the spoon bowl at least 10 mm below the coffee surface: ${JSON.stringify(s.bowl.local)}`);
    assert.ok(s.bowl.local[2] > -.025, `Leave clearance above the cup bottom: ${JSON.stringify(s.bowl.local)}`);
  }
  if (s.phase === 'taste') {
    tasteStartedAt ??= sim.state().time;
    tasteDuration = sim.state().time - tasteStartedAt;
    const mouthDistance = new Vector3(...s.bowlPosition).distanceTo(new Vector3(...FEEDING_LAYOUT.mouth));
    assert.ok(mouthDistance < .018, `Spoon must reach the visible mouth, distance ${mouthDistance}`);
    assert.ok(s.spoonHeld, 'Hold the spoon throughout feeding');
    const spoonUp = new Vector3(0, 0, 1).applyQuaternion(new Quaternion(...s.spoonQuaternion));
    assert.ok(spoonUp.z > .98, 'Keep the bowl facing up at the mouth');
  }
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
  const contacts = data.contact;
  try {
    for (let i = 0; i < Math.min(data.ncon, contacts.size()); i++) {
      const contact = contacts.get(i);
      try {
        const names = [contact.geom1, contact.geom2].map(geom =>
          mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM.value, geom) ||
          mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY.value, model.geom_bodyid[geom]));
        if (names.some(name => name?.startsWith('coffee_'))) console.log('Machine contact', names, contact.dist);
      } finally { contact.delete(); }
    }
  } finally { contacts.delete(); }
  for (const [side, arm] of Object.entries(sim.arms)) console.log(side, 'tcp', Array.from(data.xpos.slice(arm.ik.bodyId * 3, arm.ik.bodyId * 3 + 3)), 'q',Array.from(data.xquat.slice(arm.ik.bodyId*4,arm.ik.bodyId*4+4)), 'telemetry', arm.physics.telemetry());
}
assert.equal(sim.state().error, '');
console.log('Insertion/stir cup contacts (mm)', [...cupContactPhases].map(([key, depth]) => [key, depth*1000]));
for (const [pair, penetration] of cupContactPhases) assert.ok(penetration < .0001, `Cup strike during ${pair}: ${penetration*1000} mm`);
assert.equal(spoolContacts, 0, 'Cup and left gripper must not hit the yarn spool');
assert.ok(maximumSpoolMovement < .003, `Coffee workflow moved the yarn spool ${maximumSpoolMovement*1000} mm`);
assert.ok(sim.state().cooperation.complete);
assert.ok(visited.has('insert'));
for (const phase of ['coffee-approach', 'dispense', 'coffee-clear', 'raise-spoon', 'stir', 'withdraw', 'feed-approach', 'feed', 'taste', 'feed-retreat', 'return-spoon', 'reinsert']) {
  assert.ok(visited.has(phase), `Missing feeding phase: ${phase}`);
}
assert.ok(sim.state().cooperation.stirringTravel > .03, 'Spoon must physically trace two stirring circles');
assert.ok(sim.state().cooperation.fed);
assert.equal(sim.state().cooperation.coffeeFill, 1, 'Coffee must remain in the cup after the demonstration');
assert.ok(sim.state().cooperation.minimumMouthDistance < .015);
assert.ok(tasteDuration >= 1, 'Pause at the mouth before retracting');
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
assert.equal(sim.state().cooperation.fed, false);
assert.equal(sim.state().cooperation.stirringTravel, 0);
assert.equal(sim.state().cooperation.coffeeFill, 0, 'Reset must empty the cup');
console.log(`Coffee: fill, deep stirring, feeding, return and reset passed; no cup/hand contacts with yarn, maximum spool movement ${(maximumSpoolMovement*1000).toFixed(3)} mm`);
assert.ok(sim.startCooperation({task:'spoon-in-cup'}), sim.state().error);
assert.equal(sim.state().cooperation.task, 'spoon-in-cup');
assert.equal(sim.state().cooperation.graspMode, 'contact-only', 'Switch back to the contact controller after the assisted demonstration');
sim.reset();
sim.dispose(); data.delete(); model.delete(); vfs.delete();
