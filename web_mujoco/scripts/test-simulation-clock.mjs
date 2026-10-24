import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import loadMujoco from '@mujoco/mujoco';
import { createSimulationClock, advanceSimulation } from '../src/simulation-clock.js';
import { createAgvController } from '../src/agv-control.js';
import { createPhysicsController } from '../src/pd-control.js';
import { bindJoints } from '../src/kinematics.js';
import { CAMPUS_LAYOUT } from '../src/campus-layout.js';

for (const fps of [5, 15, 30, 60, 120, 144]) {
  const clock = createSimulationClock(0.001);
  clock.reset(0);
  let steps = 0;
  for (let frame = 1; frame <= fps * 3; frame += 1) steps += clock.steps(frame * 1000 / fps);
  assert.equal(steps, 3000, `${fps} FPS must advance three seconds, not ${steps / 1000}`);
}
const clock = createSimulationClock(0.001);
for (const speed of [1, 2, 4]) for (const fps of [5, 30, 60, 144]) {
  const scaled = createSimulationClock(.001); scaled.reset(0);
  let steps = 0;
  for (let frame = 1; frame <= fps * 3; frame++) steps += scaled.steps(frame * 1000 / fps, { speed });
  assert.equal(steps, 3000 * speed, `Dual-arm ${speed}x speed must not depend on ${fps} FPS`);
  assert.equal(scaled.steps(10000, { speed, paused: true }), 0);
  assert.equal(scaled.steps(10010, { speed }), 10 * speed, 'Resume without replaying paused time');
}
assert.throws(() => clock.steps(0, { speed: Infinity }), RangeError);
clock.reset(0);
assert.equal(clock.steps(1), 1);
assert.equal(clock.steps(10001), 250, 'Long stalls must have a bounded catch-up');
assert.equal(clock.steps(10002), 1, 'Dropped time must not leave growing simulation debt');
clock.reset();
assert.equal(clock.steps(999999), 0, 'Resuming a hidden tab must not advance suspended time');
assert.equal(clock.steps(1000019), 20);

const mujoco = await loadMujoco();
const modelDir = new URL('../../rebotarm_mujoco_rs/models/', import.meta.url);
const xmlName = 'rs_agv_campus_scene.xml';
const xml = await readFile(new URL(xmlName, modelDir), 'utf8');
const vfs = new mujoco.MjVFS();
vfs.addBuffer(xmlName, new TextEncoder().encode(xml));
const files = [...new Set([...xml.matchAll(/\sfile="([^"]+\.(?:stl|obj|msh))"/gi)].map((match) => `meshes/${match[1]}`))];
for (const name of files) vfs.addBuffer(name, await readFile(new URL(name, modelDir)));
const model = mujoco.MjModel.from_xml_path(xmlName, vfs);
const data = new mujoco.MjData(model);
try {
  mujoco.mj_forward(model, data);
  const drive = createAgvController(mujoco, model, data, CAMPUS_LAYOUT);
  const physics = createPhysicsController(mujoco, model, data, bindJoints(mujoco, model), {
    beforeStep: () => drive.apply(), afterStep: () => drive.enforceHeldPose()
  });
  for (const fps of [5, 15, 30, 60, 120]) {
    physics.reset();
    drive.releasePlanarPose();
    drive.setPlanarPose(-2, 0.6, 0);
    drive.setCommand(0.65, 0);
    const timer = createSimulationClock(model.opt.timestep);
    timer.reset(0);
    let previousControlTime = data.time;
    for (let frame = 1; frame <= fps * 3; frame += 1) {
      advanceSimulation(physics, timer.steps(frame * 1000 / fps), () => {
        assert(data.time - previousControlTime <= 0.012001, 'Task-control cadence depends on the render frame rate');
        previousControlTime = data.time;
      });
    }
    const distance = drive.pose().x + 2;
    assert(Math.abs(data.time - 3) < 1e-6, `${fps} FPS: incorrect physics time`);
    assert(Math.abs(distance - 1.95) < 0.04, `${fps} FPS: 0.65 m/s travelled only ${distance} m in 3 wall seconds`);
    console.log(`${fps} FPS: 3 wall seconds -> ${data.time.toFixed(3)} simulation seconds, ${distance.toFixed(3)} m at 0.65 m/s`);
  }
  drive.stop();
  const stoppedAt = drive.pose();
  advanceSimulation(physics, 250, () => {});
  assert(Math.hypot(drive.pose().x - stoppedAt.x, drive.pose().y - stoppedAt.y) < 0.01, 'Stop must remain effective with multi-step catch-up');
  console.log('Timing ok: real AGV displacement, 5–144 FPS, task cadence, pause/resume, bounded stalls and stop');
} finally {
  data.delete(); model.delete(); vfs.delete();
}

// Standard-arm motion must take the same wall time on slow and fast displays.
const armVfs = new mujoco.MjVFS();
const armXml = await readFile(new URL('rs_arm.xml', modelDir), 'utf8');
armVfs.addBuffer('rs_arm.xml', new TextEncoder().encode(armXml));
const armScene = await readFile(new URL('rs_grasp_scene.xml', modelDir), 'utf8');
for (const file of new Set([...armXml.matchAll(/\sfile="([^"]+\.(?:stl|obj|msh))"/gi)].map(match => `meshes/${match[1]}`))) {
  armVfs.addBuffer(file, await readFile(new URL(file, modelDir)));
}
const armModel = mujoco.MjModel.from_xml_string(armScene, armVfs);
const armData = new mujoco.MjData(armModel);
try {
  const joints = bindJoints(mujoco, armModel);
  const armPhysics = createPhysicsController(mujoco, armModel, armData, joints);
  let referencePositions;
  for (const fps of [5, 30, 60, 144]) {
    armPhysics.reset();
    armPhysics.setTarget('joint1', 0.5);
    const timer = createSimulationClock(armModel.opt.timestep);
    timer.reset(0);
    let previousControlTime = 0;
    for (let frame = 1; frame <= fps * 3; frame += 1) {
      advanceSimulation(armPhysics, timer.steps(frame * 1000 / fps), () => {
        assert(armData.time - previousControlTime <= 0.012001, 'Arm control cadence must stay within 12 ms');
        previousControlTime = armData.time;
      });
    }
    assert(Math.abs(armData.time - 3) < 1e-6, `${fps} FPS: arm must run at 1x`);
    const positions = joints.visible.map(joint => armData.qpos[joint.qposadr]);
    assert(Math.abs(positions[0] - 0.5) < 0.02, 'Arm must reach the commanded joint angle');
    if (referencePositions) positions.forEach((value, index) => {
      assert(Math.abs(value - referencePositions[index]) < 1e-9, `${fps} FPS: arm motion changed with frame rate`);
    });
    referencePositions = positions;
    console.log(`${fps} FPS: standard arm advanced 3 simulation seconds in 3 wall seconds`);
  }
} finally {
  armData.delete(); armModel.delete(); armVfs.delete();
}
