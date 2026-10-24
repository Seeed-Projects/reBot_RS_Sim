import { Quaternion, Vector3 } from 'three';
import { readAngles } from './kinematics.js';
import { createPosePlanner } from './pose-ik.js';
import { COOP_OBJECTS, createCooperationGeometry, relativePose } from './cooperation-geometry.js';
import { createMotionSafety } from './motion-safety.js';
import { createGraspContactSensor } from './grasp-contact-sensor.js';
import { createHoldMonitor } from './hold-monitor.js';
import {COOPERATIVE_TASKS,createMoveAction,createGraspAction,releaseObject,observeObjects,createStablePlacementAction,createCupInsertionAction} from './cooperative-actions.js';

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

const CLOSED_WIDTH = { left: .004, right: .0034 };

export function createCupSpoonCooperation({ mujoco, model, data, arms, graspMode='assisted' }) {
  if(!['assisted','contact'].includes(graspMode))throw new RangeError('Unknown grasp mode');
  const closedWidths={...CLOSED_WIDTH,left:graspMode==='contact'?.0028:CLOSED_WIDTH.left};
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
  const cupFingers = new Set(['left_gripper_left', 'left_gripper_right'].map(name => id(mujoco.mjtObj.mjOBJ_BODY.value, name)));
  const cupSensor=createGraspContactSensor(mujoco,model,data,cup,cupFingers);
  const cupSupportSensor=createGraspContactSensor(mujoco,model,data,cup,[id(mujoco.mjtObj.mjOBJ_BODY.value,'task_table')]);
  const spoonSensor=createGraspContactSensor(mujoco,model,data,spoon,spoonFingers);
  const spoonContact=()=>spoonSensor.sample();
  // The current WASM bool-array getter is unbound. The supported state API
  // updates equality flags while preserving the four joint-coupling equalities.
  const active = Array(model.neq).fill(1);
  const activeSpec = mujoco.mjtState.mjSTATE_EQ_ACTIVE.value;
  function setActive(index, enabled) {
    active[index] = enabled ? 1 : 0;
    mujoco.mj_setState(model, data, active, activeSpec);
  }
  let running = false, phase = -1,task='spoon-in-cup';
  let complete = false, error = '', plans = [], attached = false;
  let inserted = false, maximumCupHeight = 0;
  let attachedAt = -1;
  const moveAction=createMoveAction({arms,model,data});
  const graspAction=createGraspAction({arms,model,data,closedWidths,contactOnly:graspMode==='contact'});
  let graspContact = { bilateral: false, penetration: 0 };
  let cupGraspContact = { bilateral: false, penetration: 0 },holdContact=null,cupSupportForce=0;
  const holdMonitors={cup:createHoldMonitor(),spoon:createHoldMonitor()};
  let geometry = null, initialObjects = null, failure = null, taskStarted = 0;
  let history = [], metrics = {};
  const safety=createMotionSafety(mujoco,model,data,arms);
  let observe = null, visual = false, waiting = false, epoch = 0, observation = null;
  let visionHistory = [], corrections = 0;
  let visualGrips=null, visionStage='';
  let cupHeld=false,spoonHeld=false;
  const decoded = value => ({ position: [...value.position], quaternion: new Quaternion(...value.quaternion) });
  const insertionAction=createCupInsertionAction(COOP_OBJECTS.spoon.bowl);
  const visualBowl=insertionAction.measure;
  function visualPlacementValid(o) {
    if(COOPERATIVE_TASKS[task].destination==='source')return ['cup','spoon'].every(key=>
      new Vector3(...o[key].position).distanceTo(new Vector3(...initialObjects[key].position))<.010)&&
      new Vector3(0,0,1).applyQuaternion(new Quaternion(...o.cup.quaternion)).z>.95;
    return visualBowl(o).inside && Math.abs(o.cup.position[2]-geometry.restHeight)<.012 &&
      new Vector3(0,0,1).applyQuaternion(new Quaternion(...o.cup.quaternion)).z>.95;
  }
  const verifyAction=createStablePlacementAction(visualPlacementValid);
  const returnPhases={4:['return-above','回到杯子和勺子的支撑面上方',.66],
    5:['return-objects','双臂缓慢下降，将两件物品送回原位',.76],
    6:['release-spoon','右臂松勺，让餐勺落稳在支撑块上',.82],
    7:['clear-right','右臂抬起避让，左臂持稳杯子',.90]};
  const phaseInfo=index=>task==='present-and-return'&&returnPhases[index]?returnPhases[index]:COOP_PHASES[index];
  const makeGeometry=grips=>createCooperationGeometry(initialObjects.cup,initialObjects.spoon,grips,{task});
  function requestVision(stage, callback) {
    const requestEpoch = epoch;
    waiting = true;
    visionStage=stage;
    let prior=observation;
    if(task==='present-and-return'&&stage.includes('verification'))prior=Object.fromEntries(
      Object.entries(initialObjects).map(([key,value])=>[key,{position:[...value.position],quaternion:value.quaternion.toArray()}]));
    if(visualGrips && phase>=2 && phase<=4) {
      prior={...observation};
      for(const [side,key] of [['left','cup'],['right','spoon']]) {
        const hand=pose(arms[side].ik.bodyId),grip=visualGrips[side];
        prior[key]={...prior[key],position:new Vector3(...grip.position).applyQuaternion(hand.quaternion).add(new Vector3(...hand.position)).toArray(),
          quaternion:hand.quaternion.clone().multiply(grip.quaternion).toArray()};
      }
    }
    const source=stage==='initial'||stage==='pregrasp'||COOPERATIVE_TASKS[task].destination==='source'&&stage.includes('verification');
    observeObjects(observe,{stage,mode:source?'source':'held',destination:COOPERATIVE_TASKS[task].destination,prior},{
      now:()=>data.time,isCurrent:()=>epoch===requestEpoch&&running,
      onAttempt:({observation:o,attempt,time,valid,fresh})=>visionHistory.push({stage,attempt,time,valid,fresh,cup:o?.cup,spoon:o?.spoon,computeMs:o?.diagnostics?.computeMs})
    }).then(result=>{
        if(epoch!==requestEpoch || !running||!result) return;
        waiting=false;
        if(result.error) {fail(result.error,result.detail);return;}
        observation=result.observation;callback(observation);
      }).catch(cause=>{ if(epoch===requestEpoch) {waiting=false;fail('vision-error',cause.message);} });
  }

  const position = (body) => new Vector3(...data.xpos.subarray(body * 3, body * 3 + 3));
  const quaternion = (body) => new Quaternion(data.xquat[body * 4 + 1], data.xquat[body * 4 + 2], data.xquat[body * 4 + 3], data.xquat[body * 4]);
  const pose = (body) => ({ position: position(body).toArray(), quaternion: quaternion(body) });
  function bowlInCup() {
    const bowl = new Vector3(...COOP_OBJECTS.spoon.bowl).applyQuaternion(quaternion(spoon)).add(position(spoon));
    const local = bowl.sub(position(cup)).applyQuaternion(quaternion(cup).invert());
    return { inside: Math.hypot(local.x, local.y) < .030 && local.z > -.045 && local.z < .040, local: local.toArray() };
  }
  function state() {
    const current = phaseInfo(phase);
    return { running, complete, error,task, phase: current?.[0] || (complete ? 'complete' : 'idle'),
      label: waiting ? ({initial:'视觉定位杯子和勺子',pregrasp:'重新观察抓取位置',alignment:task==='present-and-return'?'观察双臂持稳状态':'视觉校正杯口与勺头',insertion:'观察勺头是否入杯',verification:'双臂退开后复核放置','settled-verification':'确认杯勺保持稳定'}[visionStage]||'视觉观察中') : current?.[1] || (complete ? `${COOPERATIVE_TASKS[task].label}完成` : '准备协作'), progress: complete ? 1 : current?.[2] || 0,
      cupHeld, spoonHeld,
      cupPosition: position(cup).toArray(), spoonPosition: position(spoon).toArray(),
      cupQuaternion: quaternion(cup).toArray(), spoonQuaternion: quaternion(spoon).toArray(),
      cupUpright: data.xmat[cup * 9 + 8], bowl: bowlInCup(), inserted, maximumCupHeight, graspContact,
      failure, history: history.map(entry => ({ ...entry })), metrics: { ...metrics, collision:{...safety.metrics} },
      controlMode:visual?'visual-rgbd':'ground-truth-benchmark', graspMode:graspMode==='assisted'?'assisted-weld':'contact-only',
      cupGraspContact:{...cupGraspContact},holdContact,cupSupportForce,
      vision:{ waiting, corrections, observation, history:visionHistory.map(item=>({...item})), bowl:observation?.valid?visualBowl(observation):null },
      graspTargets: geometry ? Object.fromEntries(Object.entries(geometry.grasp).map(([side, target]) => [side, [...target.position]])) : null };
  }
  function detach() { setActive(cupWeld, false); setActive(spoonWeld, false);cupHeld=spoonHeld=false; }
  function reset() {
    safety.resetMetrics();
    epoch++;waiting=false;observation=null;visionHistory=[];corrections=0;visualGrips=null;verifyAction.reset();graspAction.reset();
    detach(); running = complete = attached = inserted = false;
    for(const arm of Object.values(arms))arm.physics.setGripperProfile('web');
    phase = -1; error = ''; maximumCupHeight = 0; plans = [];
    graspContact = { bilateral: false, penetration: 0 };
    cupGraspContact={ bilateral:false, penetration:0 };holdContact=null;cupSupportForce=0;
    Object.values(holdMonitors).forEach(monitor=>monitor.reset());
    geometry = initialObjects = failure = null; history = [];
    metrics = { elapsed: 0, maxFingerPenetration: 0, maxJointTrackingError: 0, graspWaitSeconds: 0,
      maxSlip:{cup:0,spoon:0},minimumHoldForce:{cup:null,spoon:null} };
  }
  function fail(message, detail = message) {
    failure = { code: message, detail, phase: phaseInfo(phase)?.[0] || 'planning', time: data.time };
    error = message; running = false; detach();
    // Stop following the unfinished waypoint after a failed operation.
    for (const arm of Object.values(arms)) arm.physics.setTargets(readAngles(data, arm.joints));
  }
  function attach(weld, hand, object) {
    if(object===cup)cupHeld=true;else spoonHeld=true;
    if(graspMode==='contact')return;
    const handQ = quaternion(hand);
    const offset = position(object).sub(position(hand)).applyQuaternion(handQ.clone().invert());
    const relative = handQ.invert().multiply(quaternion(object)).normalize();
    // MuJoCo weld data: anchor(3), relative position(3), quaternion(wxyz), torque scale.
    model.eq_data.set([...offset.toArray(), relative.w, relative.x, relative.y, relative.z], weld * 11 + 3);
    setActive(weld, true);
  }
  function buildPlans(startIndex = 0) {
    const began=performance.now();
    const goals = geometry.goals;
    const result = plans.length ? plans.map(plan => ({ ...plan })) : Array.from({ length: goals.length }, () => ({}));
    for (const side of ['left', 'right']) {
      const arm = arms[side];
      const planner = createPosePlanner(mujoco, model, data, arm.joints, arm.ik.bodyId);
      let seed = readAngles(data, arm.joints);
      try {
        for (let index = startIndex; index < goals.length; index++) {
          if (goals[index]) {
            const { position: point, quaternion: q } = goals[index][side];
            const solved = planner.solve(point, q, seed);
            if (!solved || solved.positionError > .006 || solved.rotationError > .07) {
              throw new Error(`${side}/${COOP_PHASES[index][0]} unreachable (${solved?.positionError.toFixed(4) ?? 'unknown'} m, ${solved?.rotationError.toFixed(3) ?? 'unknown'} rad)`);
            }
            seed = solved.angles;
          }
          result[index][side] = { ...seed };
        }
      } finally { metrics.ikIterations=(metrics.ikIterations||0)+planner.statistics.iterations;planner.dispose(); }
    }
    metrics.planningMs=(metrics.planningMs||0)+performance.now()-began;return result;
  }
  function enter(index) {
    const collision=safety.validate(plans[index],index,{cup:cupHeld,spoon:spoonHeld});
    if(collision) {phase=index;fail('trajectory-collision',JSON.stringify(collision));return;}
    phase = index; attached = false;
    history.push({phase:phaseInfo(index)[0],time:data.time});
    attachedAt=-1;graspAction.reset();
    const releaseSpoon=index===(task==='present-and-return'?6:5);
    moveAction.begin(plans[index],{index,minimumSeconds:releaseSpoon||index===8 ? .65 : task==='present-and-return'&&index===5?2:0});
    if(releaseSpoon)releaseObject(arms,'right',()=>{setActive(spoonWeld,false);spoonHeld=false;});
    if(index===8)releaseObject(arms,'left',()=>{setActive(cupWeld,false);cupHeld=false;});
  }
  function start({task:nextTask='spoon-in-cup'}={}) {
    if (running) return false;
    if(!COOPERATIVE_TASKS[nextTask])throw new RangeError('Unknown cooperative task');
    reset();
    task=nextTask;
    if(graspMode==='contact')for(const arm of Object.values(arms))arm.physics.setGripperProfile('cooperation');
    taskStarted = data.time;
    visual=Boolean(observe);
    if(visual) {
      running=true;
      requestVision('initial',o=>{
        initialObjects={cup:decoded(o.cup),spoon:decoded(o.spoon)};
        if(new Vector3(0,0,1).applyQuaternion(initialObjects.cup.quaternion).z<.95) {fail('cup-not-upright');return;}
        geometry=makeGeometry();
        try { plans=buildPlans(); } catch(cause) {fail('planning-failed',cause.message);return;}
        for(const arm of Object.values(arms)) arm.physics.setTarget('joint7',.046);
        enter(0);
      });
      return true;
    }
    if (data.xmat[cup * 9 + 8] < .95) { fail('cup-not-upright'); return false; }
    initialObjects = { cup: pose(cup), spoon: pose(spoon) };
    geometry = makeGeometry();
    try { plans = buildPlans(); } catch (cause) { fail('planning-failed', cause.message); return false; }
    for (const arm of Object.values(arms)) arm.physics.setTarget('joint7', .046);
    running = true; enter(0); return true;
  }
  function update() {
    if (!running || waiting) return;
    const collision=safety.runtime(phase);
    if(collision) {fail('unexpected-collision',JSON.stringify(collision));return;}
    metrics.elapsed = data.time - taskStarted;
    const contact = phase >= 1 && phase <= 5 ? spoonContact() : null;
    if(contact)holdContact=contact;
    if(graspMode==='contact'&&phase>=1&&phase<=8)cupGraspContact=cupSensor.sample();
    if(graspMode==='contact') {
      cupSupportForce=cupHeld?cupSupportSensor.sample().normalForce:0;
      for(const [key,measurement,held] of [['cup',cupGraspContact,cupHeld],['spoon',contact,spoonHeld]]) {
        if(!held||!measurement)continue;
        // During placement the table takes the load and the handle's pressure
        // patch redistributes. Confirm actual support force before allowing
        // that transition; an unsupported cup is still monitored for slip.
        if(key==='cup'&&(phase===7||task==='present-and-return'&&phase>=5)&&cupSupportForce>.1)continue;
        const status=holdMonitors[key].sample(measurement,data.time);
        metrics.maxSlip[key]=Math.max(metrics.maxSlip[key],status.slip);
        metrics.minimumHoldForce[key]=Math.min(metrics.minimumHoldForce[key]??Infinity,measurement.minNormalForce);
        if(status.failure) {fail(status.failure,`${key}: tactile slip ${(status.slip*1000).toFixed(2)} mm, missing contact ${status.lostSeconds.toFixed(3)} s`);return;}
      }
      metrics.maxFingerPenetration=Math.max(metrics.maxFingerPenetration,cupGraspContact.penetration);
    }
    if (contact) metrics.maxFingerPenetration = Math.max(metrics.maxFingerPenetration, contact.penetration);
    maximumCupHeight = Math.max(maximumCupHeight, position(cup).z);
    const motion=moveAction.update();
    metrics.maxJointTrackingError=Math.max(metrics.maxJointTrackingError,motion.maxError);
    if(motion.timedOut) {fail(`motion-timeout:${COOP_PHASES[phase][0]}`);return;}
    if(!motion.finished)return;
    if(visual && phase===0) {
      requestVision('pregrasp',o=>{
        const drift=Math.max(...['cup','spoon'].map(key=>new Vector3(...o[key].position).distanceTo(new Vector3(...initialObjects[key].position))));
        if(drift>.020) {fail('object-moved-before-grasp');return;}
        if(drift>.0007) {
          corrections++; initialObjects={cup:decoded(o.cup),spoon:decoded(o.spoon)};
          geometry=makeGeometry();
          try {plans=buildPlans(1);} catch(cause) {fail('planning-failed',cause.message);return;}
        }
        enter(1);
      });return;
    }
    if (phase === 1 && !attached) {
      if (position(arms.left.ik.bodyId).distanceTo(new Vector3(...geometry.grasp.left.position)) > .015) { fail('cup-grasp-out-of-reach'); return; }
      if (position(arms.right.ik.bodyId).distanceTo(new Vector3(...geometry.grasp.right.position)) > .015) { fail('spoon-grasp-out-of-reach'); return; }
      const cupDrift = visual?0:position(cup).distanceTo(new Vector3(...initialObjects.cup.position));
      const spoonDrift = visual?0:position(spoon).distanceTo(new Vector3(...initialObjects.spoon.position));
      if (cupDrift > .005 || spoonDrift > .005) {
        fail('object-moved-before-grasp', `cup ${(cupDrift * 1000).toFixed(2)} mm, spoon ${(spoonDrift * 1000).toFixed(2)} mm`); return;
      }
      // Wait for measured finger positions, then dwell before applying the grasp.
      graspContact = contact;
      const grasp=graspAction.update({cup:cupGraspContact,spoon:contact});
      if(!grasp.ready)return;
      attach(cupWeld, arms.left.ik.bodyId, cup);
      attach(spoonWeld, arms.right.ik.bodyId, spoon);
      metrics.graspWaitSeconds = grasp.seconds;
      // Account for the measured hold, including tiny contact/IK offsets,
      // before computing the bowl's insertion target relative to the cup.
      visualGrips={
        left: relativePose(pose(arms.left.ik.bodyId), visual?initialObjects.cup:pose(cup)),
        right: relativePose(pose(arms.right.ik.bodyId), visual?initialObjects.spoon:pose(spoon))
      };
      geometry = makeGeometry(visualGrips);
      try { plans = buildPlans(2); } catch (cause) { fail('planning-failed', cause.message); return; }
      attached = true; attachedAt = data.time;
    }
    if (phase === 1 && data.time - attachedAt < .25) return;
    if(visual&&phase===3&&task==='present-and-return') {
      requestVision('alignment',o=>{
        if(new Vector3(0,0,1).applyQuaternion(new Quaternion(...o.cup.quaternion)).z<.95) {fail('cup-not-held-upright');return;}
        enter(4);
      });return;
    }
    if(visual && phase===3&&task==='spoon-in-cup') {
      requestVision('alignment',o=>{
        const correction=insertionAction.alignmentGoal(o,pose(arms.right.ik.bodyId));
        if(correction.error) {fail(correction.error);return;}
        // Correct from freshly observed bowl/opening error; no simulator object pose enters this target.
        geometry.goals[4]={left:pose(arms.left.ik.bodyId),right:correction.goal};
        corrections++;
        try {plans=buildPlans(4);} catch(cause) {fail('planning-failed',cause.message);return;}
        enter(4);
      });return;
    }
    if (!visual && phase === 3 && (!cupHeld || data.xmat[cup * 9 + 8] < .97)) { fail('cup-not-held-upright'); return; }
    if (phase === 4) {
      if(task==='present-and-return') {enter(5);return;}
      if(visual) {
        requestVision('insertion',o=>{inserted=visualBowl(o).inside;if(!inserted) fail('spoon-missed-cup-opening');else enter(5);});return;
      }
      inserted = bowlInCup().inside && cupHeld;
      if (!inserted) { fail('spoon-missed-cup-opening'); return; }
    }
    if (phase === COOP_PHASES.length - 1) {
      if(visual) {
        if(!verifyAction.ready(data.time))return;
        requestVision(verifyAction.pending?'settled-verification':'verification',o=>{
          const result=verifyAction.accept(o,data.time);
          if(result.error) {fail(result.error);return;}
          if(result.complete) {complete=true;running=false;phase=-1;}
        });return;
      }
      const p = position(cup);
      const placed=task==='spoon-in-cup'?bowlInCup().inside:['cup','spoon'].every(key=>
        position(key==='cup'?cup:spoon).distanceTo(new Vector3(...initialObjects[key].position))<.010);
      if (!placed || Math.abs(p.z - geometry.restHeight) > .012 || data.xmat[cup * 9 + 8] < .95) { fail('placement-not-stable'); return; }
      complete = true; running = false; phase = -1;
      for (const arm of Object.values(arms)) arm.physics.setTargets({ ...readAngles(data, arm.joints), joint7: .046 });
    } else enter(phase + 1);
  }
  return { start, update, reset, state, dispose() {reset();cupSensor.dispose();cupSupportSensor.dispose();spoonSensor.dispose();safety.dispose();}, setObservationProvider(provider) {observe=provider;}, isRunning: () => running };
}
