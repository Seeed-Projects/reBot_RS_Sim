import { ARM_JOINTS, readAngles } from './kinematics.js';
import { CONTROL_CONFIG } from './control-config.js';
import { Quaternion, Vector3 } from 'three';

export const motionBlend=t=>t*t*t*(10+t*(-15+6*t));
export function motionDuration(delta,minimum=1.1) {
  const limits=CONTROL_CONFIG.cooperation;
  return Math.max(minimum,1.875*delta/limits.maxVelocity,Math.sqrt(5.774*delta/limits.maxAcceleration));
}
// MuJoCo collision geometry is the safety boundary. Task targets and visual
// estimates never receive body positions through this checker.
export function createMotionSafety(mujoco,model,data,arms) {
  const names=Array.from({length:model.nbody},(_,id)=>mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_BODY.value,id)||'world');
  const side=id=>names[id].startsWith('left_')?'left':names[id].startsWith('right_')?'right':null;
  const scratch=new mujoco.MjData(model);
  const metrics={samples:0,checks:0,totalMs:0,maxPenetration:0};
  const pose=(sample,id)=>({p:new Vector3(...sample.xpos.subarray(id*3,id*3+3)),q:new Quaternion(sample.xquat[id*4+1],sample.xquat[id*4+2],sample.xquat[id*4+3],sample.xquat[id*4])});
  function check(sample,phase) {
    const contacts=sample.contact;let forbidden=null;
    try {for(let i=0;i<Math.min(sample.ncon,contacts.size());i++) {
      const c=contacts.get(i);
      try {
        const a=model.geom_bodyid[c.geom1],b=model.geom_bodyid[c.geom2],sa=side(a),sb=side(b);
        if(!sa&&!sb)continue;
        const an=names[a],bn=names[b],robot=sa?an:bn,other=sa?bn:an;
        // The bolted base rests on its mounting plate. Fingers may contact
        // their assigned utensil only during the deliberate grasp/hold.
        if(/_base_link$/.test(robot)&&other==='world')continue;
        const intentional=(/left_gripper_(left|right)$/.test(robot)&&other==='coop_cup')||(/right_gripper_(left|right)$/.test(robot)&&other==='coop_spoon');
        if(intentional && phase>=1 && phase<=8 && -c.dist<.0008)continue;
        metrics.maxPenetration=Math.max(metrics.maxPenetration,-c.dist);
        if(c.dist<-.0005) {forbidden={a:an,b:bn,penetration:-c.dist,phase};break;}
      } finally {c.delete();}
    }} finally {contacts.delete();}
    return forbidden;
  }
  return {
    metrics,
    resetMetrics() {for(const key of Object.keys(metrics))metrics[key]=0;},
    runtime(phase) {metrics.checks++;return check(data,phase);},
    validate(targets,phase,held={}) {
      const began=performance.now();scratch.qpos.set(data.qpos);scratch.mocap_pos.set(data.mocap_pos);scratch.mocap_quat.set(data.mocap_quat);
      const from=Object.fromEntries(Object.entries(arms).map(([key,arm])=>[key,readAngles(data,arm.joints)]));
      const delta=Math.max(...Object.keys(arms).flatMap(key=>ARM_JOINTS.map(j=>Math.abs(targets[key][j.name]-from[key][j.name]))));
      const samples=Math.max(1,Math.ceil(delta/CONTROL_CONFIG.cooperation.pathSampleRadians));
      const carried=[];
      for(const [key,side] of [['cup','left'],['spoon','right']]) if(held[key]) {
        const body=names.indexOf(`coop_${key}`),hand=arms[side].ik.bodyId,h=pose(data,hand),o=pose(data,body),inverse=h.q.clone().invert();
        carried.push({hand,address:model.jnt_qposadr[model.body_jntadr[body]],position:o.p.sub(h.p).applyQuaternion(inverse),quaternion:inverse.multiply(o.q)});
      }
      let collision=null;
      for(let i=0;i<=samples;i++) {
        const blend=i/samples;
        for(const [key,arm] of Object.entries(arms)) for(const joint of ARM_JOINTS) scratch.qpos[arm.joints.byName[joint.name].qposadr]=from[key][joint.name]+(targets[key][joint.name]-from[key][joint.name])*blend;
        mujoco.mj_kinematics(model,scratch);
        for(const object of carried) {
          const hand=pose(scratch,object.hand),p=object.position.clone().applyQuaternion(hand.q).add(hand.p),q=hand.q.multiply(object.quaternion);
          scratch.qpos.set([p.x,p.y,p.z,q.w,q.x,q.y,q.z],object.address);
        }
        if(carried.length)mujoco.mj_kinematics(model,scratch);
        mujoco.mj_collision(model,scratch);metrics.samples++;
        collision=check(scratch,phase);if(collision)break;
      }
      metrics.totalMs+=performance.now()-began;
      return collision;
    },
    dispose() {scratch.delete();}
  };
}
