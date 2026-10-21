import { Quaternion, Vector3 } from 'three';
import { ARM_JOINTS, readAngles } from './kinematics.js';
import { createPosePlanner } from './pose-ik.js';

export const COOP_PHASES = Object.freeze([
  ['approach', '双臂靠近杯柄与勺柄', .08],
  ['grasp', '左臂持杯，右臂取勺', .20],
  ['lift', '同时抬起，保持杯口向上', .32],
  ['align', '双臂在中央会合、对准杯口', .48],
  ['insert', '左臂持稳杯子，右臂缓慢插入', .66],
  ['release-spoon', '右臂松勺，确认勺头入杯', .76],
  ['clear-right', '右臂抬起避让', .82],
  ['lower-cup', '左臂把杯子放回桌面', .90],
  ['release-cup', '左臂松开杯柄', .95],
  ['retreat', '双臂退出工作区', .98]
]);

const CUP_POINT = [.29, -.138, .884];
const INSERTION_Q = new Quaternion().setFromAxisAngle(new Vector3(0, 1, 0), Math.PI / 6);
// Midpoint of the clear span between the two spoon rests, approached from above.
// Put the spoon 18 mm inside the fingers, rather than balancing on their tips.
const SPOON_POINT = [.3787, .085, .883];
const LEFT_Q = new Quaternion().setFromAxisAngle(new Vector3(0, 0, 1), Math.PI / 2).multiply(INSERTION_Q);
const RIGHT_Q = new Quaternion().setFromAxisAngle(new Vector3(0, 1, 0), Math.PI / 2);
const SPOON_X = .227;
const CLOSED_WIDTH = { left: .004, right: .0034 };
const clamp = (value, lo, hi) => Math.min(hi, Math.max(lo, value));

export function createCupSpoonCooperation({ mujoco, model, data, arms }) {
  const id = (type, name) => {
    const result = mujoco.mj_name2id(model, type, name);
    if (result < 0) throw new Error(`Missing cooperative scene element: ${name}`);
    return result;
  };
  const cup = id(mujoco.mjtObj.mjOBJ_BODY.value, 'coop_cup');
  const spoon = id(mujoco.mjtObj.mjOBJ_BODY.value, 'coop_spoon');
  const cupWeld = id(mujoco.mjtObj.mjOBJ_EQUALITY.value, 'cup_grasp');
  const spoonWeld = id(mujoco.mjtObj.mjOBJ_EQUALITY.value, 'spoon_grasp');
  const spoonFingers = new Set(['right_gripper_left', 'right_gripper_right'].map(name => id(mujoco.mjtObj.mjOBJ_BODY.value, name)));
  function spoonContact() {
    const touching = new Set();
    let penetration = 0;
    const contacts = data.contact;
    try {
      for (let i = 0; i < Math.min(data.ncon, contacts.size()); i++) {
        const c = contacts.get(i);
        try {
          const a = model.geom_bodyid[c.geom1], b = model.geom_bodyid[c.geom2];
          const finger = a === spoon ? b : b === spoon ? a : -1;
          if (spoonFingers.has(finger) && c.dist < .0003) {
            touching.add(finger); penetration = Math.max(penetration, -c.dist);
          }
        } finally { c.delete(); }
      }
    } finally { contacts.delete(); }
    return { bilateral: touching.size === 2, penetration };
  }
  // The current WASM bool-array getter is unbound. The supported state API
  // updates equality flags while preserving the four joint-coupling equalities.
  const active = Array(model.neq).fill(1);
  const activeSpec = mujoco.mjtState.mjSTATE_EQ_ACTIVE.value;
  function setActive(index, enabled) {
    active[index] = enabled ? 1 : 0;
    mujoco.mj_setState(model, data, active, activeSpec);
  }
  let running = false, phase = -1, started = 0, duration = 0;
  let complete = false, error = '', plans = [], from = {}, attached = false;
  let inserted = false, maximumCupHeight = 0;
  let closingAt = -1, attachedAt = -1;
  let graspContact = { bilateral: false, penetration: 0 };

  const position = (body) => new Vector3(...data.xpos.subarray(body * 3, body * 3 + 3));
  const quaternion = (body) => new Quaternion(data.xquat[body * 4 + 1], data.xquat[body * 4 + 2], data.xquat[body * 4 + 3], data.xquat[body * 4]);
  function bowlInCup() {
    const bowl = new Vector3(.094, 0, -.001).applyQuaternion(quaternion(spoon)).add(position(spoon));
    const local = bowl.sub(position(cup)).applyQuaternion(quaternion(cup).invert());
    return { inside: Math.hypot(local.x, local.y) < .030 && local.z > -.045 && local.z < .040, local: local.toArray() };
  }
  function state() {
    const current = COOP_PHASES[phase];
    return { running, complete, error, phase: current?.[0] || (complete ? 'complete' : 'idle'),
      label: current?.[1] || (complete ? '杯勺协作完成' : '准备协作'), progress: complete ? 1 : current?.[2] || 0,
      cupHeld: Boolean(active[cupWeld]), spoonHeld: Boolean(active[spoonWeld]),
      cupPosition: position(cup).toArray(), spoonPosition: position(spoon).toArray(),
      cupQuaternion: quaternion(cup).toArray(), spoonQuaternion: quaternion(spoon).toArray(),
      cupUpright: data.xmat[cup * 9 + 8], bowl: bowlInCup(), inserted, maximumCupHeight, graspContact };
  }
  function detach() { setActive(cupWeld, false); setActive(spoonWeld, false); }
  function reset() {
    detach(); running = complete = attached = inserted = false;
    phase = -1; error = ''; maximumCupHeight = 0; plans = [];
    graspContact = { bilateral: false, penetration: 0 };
  }
  function fail(message) { error = message; running = false; detach(); }
  function attach(weld, hand, object) {
    const handQ = quaternion(hand);
    const offset = position(object).sub(position(hand)).applyQuaternion(handQ.clone().invert());
    const relative = handQ.invert().multiply(quaternion(object)).normalize();
    // MuJoCo weld data: anchor(3), relative position(3), quaternion(wxyz), torque scale.
    model.eq_data.set([...offset.toArray(), relative.w, relative.x, relative.y, relative.z], weld * 11 + 3);
    setActive(weld, true);
  }
  function buildPlans() {
    const goals = [
      { left: [[.29, -.138, 1.015], LEFT_Q], right: [[.3787, .085, .944], RIGHT_Q] },
      { left: [CUP_POINT, LEFT_Q], right: [SPOON_POINT, RIGHT_Q] },
      { left: [[.29, -.138, .914], LEFT_Q], right: [[.3787, .085, .944], RIGHT_Q] },
      { left: [[.28, -.033, .914], LEFT_Q], right: [[SPOON_X, .035, .979], RIGHT_Q] },
      { left: [[.28, -.033, .914], LEFT_Q], right: [[SPOON_X, .035, .948], RIGHT_Q] },
      null,
      { left: [[.28, -.033, .914], LEFT_Q], right: [[SPOON_X, .15, 1.00], RIGHT_Q] },
      { left: [[.28, -.033, .884], LEFT_Q], right: [[SPOON_X, .15, 1.00], RIGHT_Q] },
      null,
      { left: [[.28, -.033, 1.035], LEFT_Q], right: [[SPOON_X, .15, 1.00], RIGHT_Q] }
    ];
    const result = Array.from({ length: goals.length }, () => ({}));
    for (const side of ['left', 'right']) {
      const arm = arms[side];
      const planner = createPosePlanner(mujoco, model, data, arm.joints, arm.ik.bodyId);
      let seed = readAngles(data, arm.joints);
      try {
        for (let index = 0; index < goals.length; index++) {
          if (goals[index]) {
            const [point, q] = goals[index][side];
            const solved = planner.solve(point, q, seed);
            if (solved.positionError > .006 || solved.rotationError > .07) {
              throw new Error(`${side}/${COOP_PHASES[index][0]} unreachable (${solved.positionError.toFixed(4)} m, ${solved.rotationError.toFixed(3)} rad)`);
            }
            seed = solved.angles;
          }
          result[index][side] = { ...seed };
        }
      } finally { planner.dispose(); }
    }
    return result;
  }
  function enter(index) {
    phase = index; started = data.time; attached = false;
    closingAt = attachedAt = -1;
    from = Object.fromEntries(Object.entries(arms).map(([side, arm]) => [side, readAngles(data, arm.joints)]));
    const delta = Math.max(...Object.keys(arms).flatMap((side) => ARM_JOINTS.map((joint) => Math.abs(plans[index][side][joint.name] - from[side][joint.name]))));
    duration = Math.max(index === 4 ? 2 : 1.1, delta / .65);
    if (index === 5 || index === 8) duration = .65;
    if (index === 5) { setActive(spoonWeld, false); arms.right.physics.setTarget('joint7', .046); }
    if (index === 8) { setActive(cupWeld, false); arms.left.physics.setTarget('joint7', .046); }
  }
  function start() {
    if (running) return false;
    reset();
    try { plans = buildPlans(); } catch (cause) { fail(cause.message); return false; }
    for (const arm of Object.values(arms)) arm.physics.setTarget('joint7', .046);
    running = true; enter(0); return true;
  }
  function update() {
    if (!running) return;
    maximumCupHeight = Math.max(maximumCupHeight, position(cup).z);
    const elapsed = data.time - started;
    const t = clamp(elapsed / duration, 0, 1), blend = t * t * (3 - 2 * t);
    for (const [side, arm] of Object.entries(arms)) {
      for (const joint of ARM_JOINTS) arm.physics.setTarget(joint.name, from[side][joint.name] + (plans[phase][side][joint.name] - from[side][joint.name]) * blend);
    }
    const reached = Object.entries(arms).every(([side, arm]) => ARM_JOINTS.every((joint) => Math.abs(data.qpos[arm.joints.byName[joint.name].qposadr] - plans[phase][side][joint.name]) < (phase === 1 ? .008 : .020)));
    if (elapsed > duration + 5) { fail(`motion-timeout:${COOP_PHASES[phase][0]}`); return; }
    if (elapsed < duration || !reached) return;
    if (phase === 1 && !attached) {
      if (arms.left.ik.tcpPosition && position(arms.left.ik.bodyId).distanceTo(new Vector3(...CUP_POINT)) > .015) { fail('cup-grasp-out-of-reach'); return; }
      if (position(arms.right.ik.bodyId).distanceTo(new Vector3(...SPOON_POINT)) > .015) { fail('spoon-grasp-out-of-reach'); return; }
      if (closingAt < 0) {
        for (const [side, arm] of Object.entries(arms)) arm.physics.setTarget('joint7', CLOSED_WIDTH[side]);
        closingAt = data.time;
      }
      const closed = Object.entries(arms).every(([side, arm]) =>
        ['joint7', 'joint_left', 'joint_right'].every(name => {
          const joint = arm.joints.byName[name];
          return Math.abs(data.qpos[joint.qposadr] - CLOSED_WIDTH[side]) < .001 &&
            Math.abs(data.qvel[model.jnt_dofadr[joint.id]]) < .002;
        }));
      // Wait for measured finger positions, then dwell before applying the grasp.
      const contact = spoonContact();
      graspContact = contact;
      if (!closed || !contact.bilateral || contact.penetration > .0008 || data.time - closingAt < .65) return;
      attach(cupWeld, arms.left.ik.bodyId, cup);
      attach(spoonWeld, arms.right.ik.bodyId, spoon);
      attached = true; attachedAt = data.time;
    }
    if (phase === 1 && data.time - attachedAt < .25) return;
    if (phase === 3 && (!active[cupWeld] || data.xmat[cup * 9 + 8] < .97)) { fail('cup-not-held-upright'); return; }
    if (phase === 4) {
      inserted = bowlInCup().inside && Boolean(active[cupWeld]);
      if (!inserted) { fail('spoon-missed-cup-opening'); return; }
    }
    if (phase === COOP_PHASES.length - 1) {
      const p = position(cup);
      if (!bowlInCup().inside || Math.abs(p.z - .882) > .012 || data.xmat[cup * 9 + 8] < .95) { fail('placement-not-stable'); return; }
      complete = true; running = false; phase = -1;
      for (const arm of Object.values(arms)) arm.physics.setTargets({ ...readAngles(data, arm.joints), joint7: .046 });
    } else enter(phase + 1);
  }
  return { start, update, reset, state, isRunning: () => running };
}
