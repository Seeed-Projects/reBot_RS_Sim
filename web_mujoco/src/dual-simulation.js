import { bindJoints, applyKinematicPose } from './kinematics.js';
import { createPhysicsController } from './pd-control.js';
import { createTcpIk } from './tcp-ik.js';
import { createGraspDemo } from './grasp-demo.js';
import { createCupSpoonCooperation } from './cup-spoon-cooperation.js';
import { createPosePlanner } from './pose-ik.js';
import { ARM_JOINTS } from './kinematics.js';
import { Quaternion, Vector3 } from 'three';
import { createCooperationLayout, validateLayoutSeed } from './cooperation-layout.js';
import { COOPERATIVE_TASKS } from './cooperative-actions.js';

export const TABLETOP_TASKS = Object.freeze([
  { id: 'egg', label: '鸡蛋', arm: 'left', destination: '托盘', color: '#ebd0a0', slot: -.05 },
  { id: 'spool', label: '线轴', arm: 'left', destination: '托盘', color: '#5086aa', slot: .05 },
  { id: 'apple', label: '苹果', arm: 'right', destination: '收纳篮', color: '#cf4435', slot: .05 },
  { id: 'lime', label: '青柠', arm: 'right', destination: '收纳篮', color: '#9bba3e', slot: -.05 }
]);

const READY = { joint1: 0, joint2: .55, joint3: .27, joint4: .32, joint5: .05, joint6: 0, joint7: .046 };

// The raised mounts require a downward wrist at the table. Solve each waypoint
// in scratch data, then move motor targets toward it at a bounded joint speed.
function tabletopIk(mujoco, model, data, joints, ik) {
  const orientation = new Quaternion().setFromAxisAngle(new Vector3(0, 1, 0), Math.PI / 2);
  let waypoint = null, solved = null;
  return { ...ik, servoStep(target, dt, angles) {
    if (!waypoint || Math.hypot(target.x-waypoint.x, target.y-waypoint.y, target.z-waypoint.z) > .001) {
      const planner = createPosePlanner(mujoco, model, data, joints, ik.bodyId);
      try { solved = planner.solve([target.x, target.y, target.z], orientation, angles); }
      finally { planner.dispose(); }
      waypoint = { ...target };
    }
    const delta = Math.max(...ARM_JOINTS.map(j => Math.abs(solved.angles[j.name] - angles[j.name])));
    const gain = Math.min(1, 2.0 * dt / Math.max(delta, 1e-9));
    const next = { ...angles };
    for (const joint of ARM_JOINTS) next[joint.name] += (solved.angles[joint.name] - angles[joint.name]) * gain;
    return { angles: next };
  } };
}

// Both controllers share one MjData. Apply both sets of motor torques before
// exactly one mj_step; stepping each controller separately would double time.
export function createDualSimulation(mujoco, model, data, {graspMode='assisted'}={}) {
  const arms = {};
  let running = false;
  let paused = false;
  let completed = new Set();
  let error = '';
  let activeTasks = new Set();
  let mode = 'sorting';
  let controlStep = 0;
  let layoutSeed = null, layout = null;
  const performanceMetrics={physicsSteps:0,physicsMs:0,maxBatchMs:0};
  const cooperationLayout = createCooperationLayout(mujoco, model, data);
  for (const [side, y] of [['left', -.34], ['right', .34]]) {
    const joints = bindJoints(mujoco, model, `${side}_`);
    const physics = createPhysicsController(mujoco, model, data, joints);
    const ik = createTcpIk(mujoco, model, data, joints);
    const tasks = TABLETOP_TASKS.filter((task) => task.arm === side);
    const demo = createGraspDemo({
      mujoco, model, data, joints, physics, ik: tabletopIk(mujoco, model, data, joints, ik),
      workspaceOffset: { x: 0, y, z: .74 },
      visionTargets: tasks.map((task) => ({ id: task.id, body: task.id })),
      storageZones: Object.fromEntries(tasks.map((task) => [task.id, { x: .40, y: task.slot, z: .128 }]))
    });
    arms[side] = { joints, physics, ik, demo, queue: [] };
  }
  const cooperation = createCupSpoonCooperation({ mujoco, model, data, arms, graspMode });
  const initialSolver={cone:model.opt.cone,impratio:model.opt.impratio};

  function state() {
    return {
      running, paused, error, completed: [...completed], time: data.time,
      activeTasks: [...activeTasks],
      mode, layout, performance:{...performanceMetrics}, cooperation: cooperation.state(),
      arms: Object.fromEntries(Object.entries(arms).map(([side, arm]) => [side, arm.demo.state()]))
    };
  }

  function reset({ seed = null } = {}) {
    validateLayoutSeed(seed);
    running = paused = false;
    controlStep = 0;
    model.opt.cone=initialSolver.cone;model.opt.impratio=initialSolver.impratio;
    completed = new Set();
    activeTasks = new Set();
    error = '';
    mujoco.mj_resetData(model, data);
    cooperation.reset();
    mode = 'sorting';
    for (const arm of Object.values(arms)) {
      arm.demo.reset();
      arm.queue = [];
      arm.physics.setTargets(READY);
      applyKinematicPose(mujoco, model, data, arm.joints, READY);
    }
    layoutSeed = seed;
    layout = cooperationLayout.apply(seed);
  }

  function start(ids = TABLETOP_TASKS.map((task) => task.id)) {
    if (running) return false;
    const pending = TABLETOP_TASKS.filter((task) => ids.includes(task.id) && !completed.has(task.id));
    if (!pending.length) return false;
    mode = 'sorting';
    activeTasks = new Set(pending.map((task) => task.id));
    error = '';
    running = true;
    paused = false;
    for (const [side, arm] of Object.entries(arms)) {
      arm.queue = pending.filter((task) => task.arm === side).map((task) => task.id);
      arm.demo.reset();
      if (arm.queue.length) arm.demo.start(arm.queue.shift());
    }
    return true;
  }

  function update() {
    if (!running || paused) return;
    if (mode === 'cooperation') {
      cooperation.update();
      const current = cooperation.state();
      running = current.running;
      error = current.error;
      return;
    }
    for (const arm of Object.values(arms)) {
      if (arm.demo.isRunning()) arm.demo.update();
      const current = arm.demo.state();
      if (current.stage === 'failed') {
        error = `${current.selectedId}: ${current.message}`;
        running = false;
        for (const other of Object.values(arms)) if (other !== arm) other.demo.cancel();
        return;
      }
      if (current.stage === 'complete') {
        completed.add(current.selectedId);
        if (arm.queue.length) arm.demo.start(arm.queue.shift());
      }
    }
    running = Object.values(arms).some((arm) => arm.demo.isRunning());
  }

  function step(count) {
    if (paused) return;
    const began=performance.now();
    for (let i = 0; i < count; i++) {
      if (controlStep++ % 10 === 0) update();
      arms.left.physics.applyCtrl();
      arms.right.physics.applyCtrl();
      mujoco.mj_step(model, data);
    }
    const ms=performance.now()-began;performanceMetrics.physicsSteps+=count;performanceMetrics.physicsMs+=ms;performanceMetrics.maxBatchMs=Math.max(performanceMetrics.maxBatchMs,ms);
  }

  reset();
  return { state, reset, start, step, arms, dispose:cooperation.dispose, setObservationProvider:cooperation.setObservationProvider,
    setCooperationLayout(seed) {
      validateLayoutSeed(seed);
      if (running) return false;
      reset({ seed }); step(250); mode = 'cooperation';
      return true;
    },
    startCooperation({ seed = layoutSeed, resetScene = true,task='spoon-in-cup' } = {}) {
      if (running) return false;
      if(!COOPERATIVE_TASKS[task])throw new RangeError('Unknown cooperative task');
      validateLayoutSeed(seed);
      if (resetScene) reset({ seed });
      // Settle the free objects onto their supports before planning the grasp.
      if(graspMode==='contact') {
        model.opt.cone=mujoco.mjtCone.mjCONE_ELLIPTIC.value;model.opt.impratio=10;
      }
      step(graspMode==='contact'?2500:250);
      mode = 'cooperation';
      running = cooperation.start({task});
      error = cooperation.state().error;
      return running;
    },
    togglePause() { if (running) paused = !paused; },
    setPaused(value) { if (running) paused = Boolean(value); }
  };
}
