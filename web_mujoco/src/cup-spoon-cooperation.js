import { Quaternion, Vector3 } from 'three';
import { ARM_JOINTS, readAngles } from './kinematics.js';
import { createPosePlanner } from './pose-ik.js';
import { FEEDING_LAYOUT } from './feeding-layout.js';
import { COFFEE_LAYOUT } from './coffee-view.js';

export const COOP_PHASES = Object.freeze([
  ['approach', '双臂靠近杯柄与勺柄', .08],
  ['grasp', '左臂持杯，右臂取勺', .16],
  ['lift', '同时抬起，保持杯口向上', .24],
  ['coffee-approach', '左臂将空杯移到咖啡机出液口下方', .25],
  ['dispense', '正在接咖啡，等待杯内液面升高', .29],
  ['coffee-clear', '左臂保持杯口向上，移离出液口', .32],
  ['raise-spoon', '先抬勺避开杯沿，再移到杯口上方', .33],
  ['align', '双臂在中央会合、对准杯口', .34],
  ['insert', '左臂持稳杯子，右臂缓慢插入', .42],
  ['stir', '右臂在杯中轻轻搅拌两圈', .48],
  ['withdraw', '抬勺离开杯口', .55],
  ['feed-approach', '保持勺面朝上，靠近就餐者', .60],
  ['feed', '缓慢送勺到嘴边', .66],
  ['taste', '在嘴边短暂停留', .70],
  ['feed-retreat', '沿原路收回餐勺', .74],
  ['return-spoon', '把餐勺移回杯口上方', .77],
  ['reinsert', '将餐勺重新放入杯中', .79],
  ['release-spoon', '右臂松勺，确认勺头入杯', .81],
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
const FEED_Q = new Quaternion().setFromAxisAngle(new Vector3(0, 0, 1), Math.PI / 2)
  .multiply(new Quaternion().setFromAxisAngle(new Vector3(0, 1, 0), Math.PI / 3));
// Bowl offset from the grasp centre when the spoon is horizontal toward +Y.
const FEED_POINT = [FEEDING_LAYOUT.mouth[0], FEEDING_LAYOUT.mouth[1]-.054, FEEDING_LAYOUT.mouth[2]-.012];
const SPOON_X = .1633;
const STIR_Q = new Quaternion().setFromAxisAngle(new Vector3(0, 1, 0), Math.PI*11/18);
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
  let coffeeFill = 0, spoonLoaded = false, minimumStirDepth = Infinity;
  let stirringTravel = 0, lastStirBowl = null, fed = false, minimumMouthDistance = Infinity;

  const position = (body) => new Vector3(...data.xpos.subarray(body * 3, body * 3 + 3));
  const quaternion = (body) => new Quaternion(data.xquat[body * 4 + 1], data.xquat[body * 4 + 2], data.xquat[body * 4 + 3], data.xquat[body * 4]);
  const bowlPosition = () => new Vector3(.094, 0, -.001).applyQuaternion(quaternion(spoon)).add(position(spoon));
  function bowlInCup() {
    const bowl = bowlPosition();
    const local = bowl.sub(position(cup)).applyQuaternion(quaternion(cup).invert());
    return { inside: Math.hypot(local.x, local.y) < .030 && local.z > -.045 && local.z < .040, local: local.toArray() };
  }
  function state() {
    const current = COOP_PHASES[phase];
    return { running, complete, error, time: data.time, coffeeFill, spoonLoaded,
      dispensing: running && current?.[0] === 'dispense', minimumStirDepth, phase: current?.[0] || (complete ? 'complete' : 'idle'),
      label: current?.[0] === 'dispense' ? `正在接咖啡 · ${Math.round(coffeeFill*100)}%` : current?.[1] || (complete ? '咖啡协作完成' : '准备协作'), progress: complete ? 1 : current?.[2] || 0,
      cupHeld: Boolean(active[cupWeld]), spoonHeld: Boolean(active[spoonWeld]),
      cupPosition: position(cup).toArray(), spoonPosition: position(spoon).toArray(),
      cupQuaternion: quaternion(cup).toArray(), spoonQuaternion: quaternion(spoon).toArray(),
      cupUpright: data.xmat[cup * 9 + 8], bowl: bowlInCup(), inserted, maximumCupHeight, graspContact,
      stirringTravel, fed, minimumMouthDistance, bowlPosition: bowlPosition().toArray() };
  }
  function detach() { setActive(cupWeld, false); setActive(spoonWeld, false); }
  function reset() {
    detach(); running = complete = attached = inserted = false;
    phase = -1; error = ''; maximumCupHeight = 0; plans = [];
    graspContact = { bilateral: false, penetration: 0 };
    coffeeFill = 0; spoonLoaded = false; minimumStirDepth = Infinity;
    stirringTravel = 0; lastStirBowl = null; fed = false; minimumMouthDistance = Infinity;
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
    const spoutId = id(mujoco.mjtObj.mjOBJ_SITE.value, 'coffee_spout');
    const spout = Array.from(data.site_xpos.subarray(spoutId*3, spoutId*3+3));
    if (new Vector3(...spout).distanceTo(new Vector3(...COFFEE_LAYOUT.spout)) > .001) throw new Error('coffee-layout-mismatch');
    const coffeePoint = [spout[0], spout[1]-.068, spout[2]-.280];
    const goals = [
      { left: [[.29, -.138, 1.015], LEFT_Q], right: [[.3787, .085, .944], RIGHT_Q] },
      { left: [CUP_POINT, LEFT_Q], right: [SPOON_POINT, RIGHT_Q] },
      { left: [[.29, -.138, .914], LEFT_Q], right: [[.3787, .085, .944], RIGHT_Q] },
      { left: [coffeePoint, LEFT_Q], right: [[.3787, .085, .944], RIGHT_Q] },
      null,
      { left: [[coffeePoint[0]-.06, coffeePoint[1], coffeePoint[2]], LEFT_Q], right: [[.3787, .085, .944], RIGHT_Q] },
      { left: [[coffeePoint[0]-.06, coffeePoint[1], coffeePoint[2]], LEFT_Q], right: [[.227, .085, 1.035], RIGHT_Q] },
      { left: [[.20, -.033, .914], LEFT_Q], right: [[.154, .035, 1.00], RIGHT_Q] },
      { left: [[.20, -.033, .947], LEFT_Q], right: [[SPOON_X, .035, .979], STIR_Q] },
      { left: [[.20, -.033, .947], LEFT_Q], right: [[SPOON_X, .035, .979], STIR_Q] },
      { left: [[.20, -.033, .914], LEFT_Q], right: [[.154, .035, 1.00], RIGHT_Q] },
      { left: [[.20, -.033, .947], LEFT_Q], right: [[FEED_POINT[0], FEED_POINT[1]-.065, FEED_POINT[2]+.02], FEED_Q] },
      { left: [[.20, -.033, .947], LEFT_Q], right: [FEED_POINT, FEED_Q] },
      null,
      { left: [[.20, -.033, .947], LEFT_Q], right: [[FEED_POINT[0], FEED_POINT[1]-.065, FEED_POINT[2]+.02], FEED_Q] },
      { left: [[.20, -.033, .914], LEFT_Q], right: [[.154, .035, 1.00], RIGHT_Q] },
      { left: [[.20, -.033, .947], LEFT_Q], right: [[SPOON_X, .035, .979], STIR_Q] },
      null,
      { left: [[.20, -.033, .947], LEFT_Q], right: [[SPOON_X, .15, 1.00], RIGHT_Q] },
      { left: [[.20, -.033, .884], LEFT_Q], right: [[SPOON_X, .15, 1.00], RIGHT_Q] },
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
          if (COOP_PHASES[index][0] === 'align') {
            // Keep the cup above the yarn and the spoon above the rim. Cartesian
            // paths avoid the low arcs produced by interpolating joint angles.
            const start = new Vector3(...goals[index-1][side][0]);
            const end = new Vector3(...goals[index][side][0]);
            const path = [{ ...seed }];
            for (let waypoint = 1; waypoint <= 40; waypoint++) {
              const t = waypoint/40;
              const point = start.clone().lerp(end, t);
              point.z += (side === 'left' ? .025 : .015)*Math.sin(Math.PI*t);
              const solved = planner.solve(point.toArray(), side === 'left' ? LEFT_Q : RIGHT_Q, seed);
              const jump = Math.max(...ARM_JOINTS.map(joint => Math.abs(solved.angles[joint.name]-seed[joint.name])));
              if (solved.positionError > .003 || solved.rotationError > .03 || jump > .2) {
                throw new Error(`${side}-clearance-path-unreachable:${waypoint}`);
              }
              seed = solved.angles; path.push({ ...seed });
            }
            result[index][`${side}Path`] = path;
            result[index][side] = { ...seed };
            continue;
          }
          if (goals[index]) {
            const [point, q] = goals[index][side];
            let solved = planner.solve(point, q, seed);
            if (solved.positionError > .006 || solved.rotationError > .07) {
              solved = planner.solve(point, q, readAngles(data, arm.joints));
            }
            if (solved.positionError > .006 || solved.rotationError > .07) {
              throw new Error(`${side}/${COOP_PHASES[index][0]} unreachable (${solved.positionError.toFixed(4)} m, ${solved.rotationError.toFixed(3)} rad)`);
            }
            seed = solved.angles;
          }
          result[index][side] = { ...seed };
          if (side === 'right' && COOP_PHASES[index][0] === 'stir') {
            const path = [{ ...seed }];
            for (let waypoint = 1; waypoint <= 48; waypoint++) {
              const angle = waypoint / 48 * Math.PI * 4;
              const point = [SPOON_X + .003*(Math.cos(angle)-1), .035+.003*Math.sin(angle), .979];
              const solved = planner.solve(point, STIR_Q, seed);
              if (solved.positionError > .003 || solved.rotationError > .07) throw new Error('stir-path-unreachable');
              seed = solved.angles; path.push({ ...seed });
            }
            result[index].rightPath = path;
          }
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
    const name = COOP_PHASES[index][0];
    duration = Math.max(['insert', 'reinsert', 'feed'].includes(name) ? 2 : 1.1, delta / .65);
    if (name === 'stir') duration = 5;
    if (name === 'align') duration = Math.max(duration, 6);
    if (name === 'dispense') duration = 3.5;
    if (name === 'taste') spoonLoaded = false;
    if (name === 'taste') duration = 1.2;
    if (name === 'release-spoon' || name === 'release-cup') duration = .65;
    if (name === 'release-spoon') { setActive(spoonWeld, false); arms.right.physics.setTarget('joint7', .046); }
    if (name === 'release-cup') { setActive(cupWeld, false); arms.left.physics.setTarget('joint7', .046); }
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
    const phaseName = COOP_PHASES[phase][0];
    for (const [side, arm] of Object.entries(arms)) {
      const path = plans[phase][`${side}Path`];
      if (path) {
        const cursor = (phaseName === 'stir' ? t : blend)*(path.length-1), index = Math.min(path.length-2, Math.floor(cursor));
        const amount = cursor-index;
        for (const joint of ARM_JOINTS) arm.physics.setTarget(joint.name, path[index][joint.name]+(path[index+1][joint.name]-path[index][joint.name])*amount);
      } else {
        for (const joint of ARM_JOINTS) arm.physics.setTarget(joint.name, from[side][joint.name] + (plans[phase][side][joint.name] - from[side][joint.name]) * blend);
      }
    }
    if (phaseName === 'dispense') {
      const cupPoint = position(cup);
      const horizontalError = Math.hypot(cupPoint.x-COFFEE_LAYOUT.spout[0], cupPoint.y-COFFEE_LAYOUT.spout[1]);
      if (horizontalError > .008 || data.xmat[cup*9+8] < .98) { fail('cup-not-under-coffee-spout'); return; }
      coffeeFill = Math.min(1, t);
    }
    if (phaseName === 'stir') {
      const bowl = bowlPosition();
      if (lastStirBowl) stirringTravel += bowl.distanceTo(lastStirBowl);
      lastStirBowl = bowl;
      const depth = .040-bowlInCup().local[2];
      minimumStirDepth = Math.min(minimumStirDepth, depth);
      if (coffeeFill > .9) spoonLoaded = true;
      if (!bowlInCup().inside) { fail('stir-left-cup'); return; }
    }
    if (phaseName === 'feed' || phaseName === 'taste') {
      minimumMouthDistance = Math.min(minimumMouthDistance, bowlPosition().distanceTo(new Vector3(...FEEDING_LAYOUT.mouth)));
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
    if (phaseName === 'align' && (!active[cupWeld] || data.xmat[cup * 9 + 8] < .97)) { fail('cup-not-held-upright'); return; }
    if (phaseName === 'insert' || phaseName === 'reinsert') {
      inserted = bowlInCup().inside && Boolean(active[cupWeld]);
      if (!inserted) { fail('spoon-missed-cup-opening'); return; }
    }
    if (phaseName === 'feed') {
      if (bowlPosition().distanceTo(new Vector3(...FEEDING_LAYOUT.mouth)) > .018) { fail('spoon-missed-mouth'); return; }
      fed = true;
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
