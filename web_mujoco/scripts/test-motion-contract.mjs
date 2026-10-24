import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import loadMujoco from '@mujoco/mujoco';
import { CONTROL_CONFIG } from '../src/control-config.js';
import { ARM_JOINTS } from '../src/kinematics.js';
import { createMotionSafety,motionBlend,motionDuration } from '../src/motion-safety.js';

assert.deepEqual(ARM_JOINTS.map(j=>[j.min,j.max]),CONTROL_CONFIG.jointLimits);
const xml=await readFile(new URL('../../rebotarm_mujoco_rs/models/rs_dual_tabletop_scene.xml',import.meta.url),'utf8');
for(const side of ['left','right'])for(const [i,joint] of ARM_JOINTS.entries()) {
  const element=xml.match(new RegExp(`<joint name="${side}_${joint.name}"[^>]+`))?.[0];
  assert(element,`${side}/${joint.name}`);
  const range=element.match(/range="([^"]+)"/)?.[1].split(/\s+/).map(Number);
  const allowed=CONTROL_CONFIG.jointLimits[i];
  assert(range[0]<=allowed[0]&&range[1]>=allowed[1],`${side}/${joint.name}: commanded range must stay inside model hard stops`);
}
for(const delta of [.01,.3,1,3]) {
  const duration=motionDuration(delta),dt=1e-4;let lastV=0,maxV=0,maxA=0;
  for(let t=dt;t<1-dt;t+=dt) {
    const v=delta*(motionBlend(t+dt)-motionBlend(t-dt))/(2*dt*duration);
    maxV=Math.max(maxV,v);if(t>2*dt)maxA=Math.max(maxA,Math.abs(v-lastV)/(dt*duration));lastV=v;
  }
  assert(maxV<=CONTROL_CONFIG.cooperation.maxVelocity+1e-5);
  assert(maxA<=CONTROL_CONFIG.cooperation.maxAcceleration+1e-3);
}
const mujoco=await loadMujoco();
const tiny='<mujoco><option gravity="0 0 0"/><worldbody><geom type="box" size=".1 .1 .1" pos=".5 0 0"/><body name="left_link1"><joint name="j" type="slide" axis="1 0 0"/><geom type="sphere" size=".04"/></body><body name="right_link1" pos="1 0 0"><joint type="slide" axis="0 1 0"/><geom type="sphere" size=".04"/></body></worldbody></mujoco>';
const model=mujoco.MjModel.from_xml_string(tiny),data=new mujoco.MjData(model);
mujoco.mj_forward(model,data);
const guard=createMotionSafety(mujoco,model,data,{});
assert.equal(guard.runtime(0),null);
data.qpos[0]=.5;mujoco.mj_forward(model,data);
const environment=guard.runtime(0);
assert.deepEqual(new Set([environment.a,environment.b]),new Set(['world','left_link1']),'Detect contact with the environment');
data.qpos[0]=1;mujoco.mj_forward(model,data);
const pair=guard.runtime(0);assert(new Set([pair.a,pair.b]).has('right_link1'),'Detect inter-arm collision');
guard.dispose();
// Native analytic Jacobian agrees with independent finite differences.
data.qpos[0]=.2;mujoco.mj_forward(model,data);
const jp=new mujoco.DoubleBuffer(3*model.nv),jr=new mujoco.DoubleBuffer(3*model.nv);
mujoco.mj_jacBody(model,data,jp,jr,1);
const before=data.xpos[3],analytic=jp.GetView()[0];
data.qpos[0]+=.00001;mujoco.mj_kinematics(model,data);
assert(Math.abs((data.xpos[3]-before)/.00001-analytic)<1e-8);
jp.delete();jr.delete();data.delete();model.delete();
const chain=ARM_JOINTS.map((joint,i)=>`<joint name="${joint.name}" type="${i<3?'slide':'hinge'}" axis="${i%3===0?'1 0 0':i%3===1?'0 1 0':'0 0 1'}"/>`).join('');
const pathModel=mujoco.MjModel.from_xml_string(`<mujoco><option gravity="0 0 0"/><worldbody><geom type="box" size=".1 .1 .1" pos=".5 0 0"/><body name="left_link1">${chain}<geom type="sphere" size=".04"/></body><body name="left_link3" pos="2 0 0"><freejoint/><geom type="sphere" size=".04"/></body></worldbody></mujoco>`);
const pathData=new mujoco.MjData(pathModel);mujoco.mj_forward(pathModel,pathData);
const entries=ARM_JOINTS.map((joint,i)=>({...joint,id:i,qposadr:i}));
const arm={joints:{all:entries,byName:Object.fromEntries(entries.map(j=>[j.name,j]))}};
const pathGuard=createMotionSafety(mujoco,pathModel,pathData,{left:arm});
const target=Object.fromEntries(ARM_JOINTS.map((j,i)=>[j.name,i===0?1:0]));
const beforePath=Array.from(pathData.qpos),hit=pathGuard.validate({left:target},0);
assert(hit&&hit.penetration>.0005,'Detect an obstacle between clear endpoints');
assert.deepEqual(Array.from(pathData.qpos),beforePath,'Preflight must never move the live robot');
pathData.qpos[0]=2;mujoco.mj_forward(pathModel,pathData);
const self=pathGuard.runtime(0);assert([self.a,self.b].includes('left_link3'),'Detect nonadjacent self contact');
pathGuard.dispose();pathData.delete();pathModel.delete();
console.log('Shared SI limits, quintic velocity/acceleration, environment/inter-arm guards and analytic Jacobian passed');
