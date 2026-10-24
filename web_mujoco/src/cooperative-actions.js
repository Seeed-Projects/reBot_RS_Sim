import {ARM_JOINTS,readAngles} from './kinematics.js';
import {motionBlend,motionDuration} from './motion-safety.js';
import {Quaternion,Vector3} from 'three';

export const COOPERATIVE_TASKS=Object.freeze({
  'spoon-in-cup':{label:'勺子入杯',destination:'cup'},
  'present-and-return':{label:'会合展示后归位',destination:'source'}
});

// The same motor action drives approach, lift, alignment, insertion, return
// and retreat. Recipes supply goals; this primitive owns bounded timing.
export function createMoveAction({arms,model,data}) {
  let from=null,goal=null,started=0,duration=0,phase=0;
  return {
    begin(next,{index=0,minimumSeconds=0}={}) {
      phase=index;started=data.time;goal=next;
      from=Object.fromEntries(Object.entries(arms).map(([side,arm])=>[side,readAngles(data,arm.joints)]));
      const delta=Math.max(...Object.keys(arms).flatMap(side=>ARM_JOINTS.map(j=>Math.abs(goal[side][j.name]-from[side][j.name]))));
      duration=motionDuration(delta,minimumSeconds>0?minimumSeconds:index===4?2:1.1);
    },
    update() {
      const elapsed=data.time-started,blend=motionBlend(Math.max(0,Math.min(1,elapsed/duration)));
      let reached=true,maxError=0;
      for(const [side,arm] of Object.entries(arms))for(const joint of ARM_JOINTS) {
        const target=from[side][joint.name]+(goal[side][joint.name]-from[side][joint.name])*blend;
        arm.physics.setTarget(joint.name,target);
        const q=data.qpos[arm.joints.byName[joint.name].qposadr];
        maxError=Math.max(maxError,Math.abs(target-q));
        if(Math.abs(q-goal[side][joint.name])>(phase===1 ? .008 : .020))reached=false;
      }
      return {elapsed,duration,reached,maxError,finished:elapsed>=duration&&reached,timedOut:elapsed>duration+5};
    }
  };
}

export function createGraspAction({arms,model,data,closedWidths,contactOnly}) {
  let began=null;
  return {reset(){began=null;},update({cup,spoon}) {
    if(began===null) {
      for(const [side,arm] of Object.entries(arms))arm.physics.setTarget('joint7',closedWidths[side]);
      began=data.time;
    }
    const closed=Object.entries(arms).every(([side,arm])=>['joint7','joint_left','joint_right'].every(name=>{
      const joint=arm.joints.byName[name];return Math.abs(data.qpos[joint.qposadr]-closedWidths[side])<.001&&
        Math.abs(data.qvel[model.jnt_dofadr[joint.id]])<.002;
    }));
    const seconds=data.time-began;
    const forceReady=!contactOnly||(cup.bilateral&&cup.minNormalForce>=.6&&spoon.minNormalForce>=.2);
    return {seconds,closed,ready:closed&&spoon.bilateral&&spoon.penetration<=.0008&&seconds>=.65&&forceReady};
  }};
}

export function releaseObject(arms,side,detach) {detach();arms[side].physics.setTarget('joint7',.046);}

// Image-space estimates drive both the alignment correction and the insertion
// acceptance. The move action executes the resulting bounded motor trajectory.
export function createCupInsertionAction(bowlPoint) {
  function measure(observation) {
    const p=new Vector3(...bowlPoint).applyQuaternion(new Quaternion(...observation.spoon.quaternion))
      .add(new Vector3(...observation.spoon.position)).sub(new Vector3(...observation.cup.position))
      .applyQuaternion(new Quaternion(...observation.cup.quaternion).invert());
    return {inside:Math.hypot(p.x,p.y)<.027&&p.z>-.044&&p.z<.040,local:p.toArray()};
  }
  return {measure,alignmentGoal(observation,hand) {
    const bowl=measure(observation),delta=new Vector3(-bowl.local[0],-bowl.local[1],.050-bowl.local[2])
      .applyQuaternion(new Quaternion(...observation.cup.quaternion));
    if(delta.length()>.025)return {error:'vision-alignment-out-of-range'};
    return {goal:{position:new Vector3(...hand.position).add(delta).add(new Vector3(0,0,-.031)).toArray(),quaternion:hand.quaternion}};
  }};
}

function usablePose(p) {
  return p&&p.position?.length===3&&p.position.every(Number.isFinite)&&p.quaternion?.length===4&&
    p.quaternion.every(Number.isFinite)&&Math.abs(Math.hypot(...p.quaternion)-1)<.01&&p.confidence>.6&&p.confidence<=1;
}
export async function observeObjects(observe,options,{now,isCurrent,onAttempt,maxAttempts=3}) {
  // Let the current native step batch finish before freezing and imaging it.
  await Promise.resolve();
  for(let attempt=0;attempt<maxAttempts;attempt++) {
    if(!isCurrent())return null;
    const requestedAt=now();
    const observation=await observe(options);
    if(!isCurrent())return null;
    const time=now(),o=observation;
    const fresh=o&&o.source==='rgbd-pixels'&&Number.isFinite(o.timestamp)&&o.timestamp>=requestedAt-.05&&o.timestamp<=time+.002&&time-o.timestamp<.6;
    const valid=fresh&&o.valid&&usablePose(o.cup)&&usablePose(o.spoon);
    onAttempt({observation,attempt,time,fresh,valid});
    if(valid)return {observation};
    if(attempt===maxAttempts-1)return {error:fresh?'vision-occluded':'vision-stale',detail:`${options.stage}: no reliable fresh cup/spoon observation after ${maxAttempts} frames`};
  }
}

export function createStablePlacementAction(isValid,{dwellSeconds=.35,maxMovement=.003}={}) {
  let candidate=null,at=0;
  return {
    reset(){candidate=null;at=0;},
    get pending(){return Boolean(candidate);},
    ready(time){return !candidate||time-at>=dwellSeconds;},
    accept(observation,time) {
      if(!isValid(observation))return {error:'placement-not-stable'};
      if(!candidate) {candidate=observation;at=time;return {complete:false};}
      if(time-at<dwellSeconds)return {complete:false};
      const stable=['cup','spoon'].every(key=>Math.hypot(...observation[key].position.map((value,i)=>value-candidate[key].position[i]))<maxMovement);
      return stable?{complete:true}:{error:'placement-not-stable'};
    }
  };
}
