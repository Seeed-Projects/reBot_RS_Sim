import assert from 'node:assert/strict';
import { readFile,mkdir,writeFile } from 'node:fs/promises';
import loadMujoco from '@mujoco/mujoco';
import { createDualSimulation } from '../src/dual-simulation.js';

const root=new URL('../../rebotarm_mujoco_rs/models/',import.meta.url);
const xml=await readFile(new URL('rs_dual_tabletop_scene.xml',root),'utf8');
assert(!/<exclude body1="left_gripper_(?:left|right)" body2="coop_cup"/.test(xml),
  'The actual model must enable cup/finger contacts');
const mujoco=await loadMujoco(),vfs=new mujoco.MjVFS();
for(const file of new Set([...xml.matchAll(/file="([^"]+)"/g)].map(m=>m[1])))vfs.addBuffer(`meshes/${file}`,await readFile(new URL(`meshes/${file}`,root)));
const model=mujoco.MjModel.from_xml_string(xml,vfs),data=new mujoco.MjData(model);
const simulation=createDualSimulation(mujoco,model,data,{graspMode:'contact'});
const stateSpec=mujoco.mjtState.mjSTATE_EQ_ACTIVE.value,flags=new mujoco.DoubleBuffer(model.neq);
const cup=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_EQUALITY.value,'cup_grasp'),spoon=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_EQUALITY.value,'spoon_grasp');
const body=name=>mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY.value,name);
const objects=new Set([body('coop_cup'),body('coop_spoon')]);
const friction=Array.from(model.geom_friction),results=[],faults=[],returnTasks=[];
const returnOnly=process.argv.includes('--return-only');
const seeds=returnOnly?[]:process.argv.includes('--quick')?[null,1,42]:[null,0,1,2,7,17,42,99,123,999,2026,65535,4294967295];
try {
  for(const [index,seed] of seeds.entries()) {
    const frictionScale=[1,.9,1.1][index%3],contacts=[],began=performance.now();
    model.geom_friction.set(friction);
    for(let geom=0;geom<model.ngeom;geom++)if(objects.has(model.geom_bodyid[geom]))
      for(let axis=0;axis<3;axis++)model.geom_friction[geom*3+axis]*=frictionScale;
    const started=simulation.startCooperation({seed});
    for(let step=0;step<5000&&simulation.state().running;step++) {
      simulation.step([10,20,33,47][step%4]);
      mujoco.mj_getState(model,data,flags,stateSpec);
      assert.equal(flags.GetView()[cup],0,'Contact-only mode must never activate the cup weld');
      assert.equal(flags.GetView()[spoon],0,'Contact-only mode must never activate the spoon weld');
      const state=simulation.state().cooperation;
      if(step%5===0)contacts.push({time:data.time,phase:state.phase,cup:state.cupGraspContact,spoon:state.holdContact||state.graspContact});
    }
    simulation.step(1000);
    const state=simulation.state().cooperation;
    const success=started&&state.complete&&!state.error&&state.maximumCupHeight>.90&&state.bowl.inside&&state.cupUpright>.95;
    results.push({seed,frictionScale,success,complete:state.complete,failure:state.failure,metrics:state.metrics,
      bowl:state.bowl,milliseconds:performance.now()-began,contacts});
    await mkdir('.reports',{recursive:true});
    await writeFile('.reports/contact-grasp.json',JSON.stringify({version:2,assistance:false,cupFingerCollisions:true,
      passed:results.filter(result=>result.success).length,total:results.length,results},null,2));
    console.log(JSON.stringify({seed,frictionScale,success,failure:state.failure}));
    assert(contacts.some(sample=>sample.cup.bilateral&&sample.spoon.bilateral),'Measure both actual grasps');
  }
  assert(results.every(result=>result.success),'Every contact-only case must complete placement and stay stable after release');
  for(const fault of returnOnly?[]:['slide','lost']) {
    model.geom_friction.set(friction);
    assert(simulation.startCooperation({seed:17}));
    for(let i=0;i<2000&&simulation.state().cooperation.phase!=='lift'&&simulation.state().running;i++)simulation.step(10);
    assert.equal(simulation.state().cooperation.phase,'lift');
    // Test-only physical perturbation. Production control receives contact
    // forces / tactile patch motion, never this free-joint position.
    const spoonBody=body('coop_spoon');
    const joint=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_JOINT.value,'coop_spoon_free');
    for(let axis=0;axis<3;axis++)data.qpos[model.jnt_qposadr[joint]+axis]+=
      data.xmat[spoonBody*9+axis*3+(fault==='slide'?0:1)]*(fault==='slide'?.009:.04);
    mujoco.mj_forward(model,data);
    for(let i=0;i<200&&simulation.state().running;i++)simulation.step(10);
    const state=simulation.state().cooperation;
    faults.push({fault,failure:state.failure,metrics:state.metrics});
    assert.equal(state.running,false);
    assert.equal(state.complete,false);
    assert.equal(state.failure.code,fault==='slide'?'contact-slip-detected':'contact-grasp-lost');
  }
  for(const seed of process.argv.includes('--quick')?[null]:[null,17,42]) {
    model.geom_friction.set(friction);
    assert(simulation.startCooperation({seed,task:'present-and-return'}));
    const initial=simulation.state().cooperation;
    for(let step=0;step<5000&&simulation.state().running;step++) {
      simulation.step([10,20,33,47][step%4]);
      mujoco.mj_getState(model,data,flags,stateSpec);
      assert.equal(flags.GetView()[cup],0);assert.equal(flags.GetView()[spoon],0);
    }
    simulation.step(1000);
    const state=simulation.state().cooperation;
    const errors=Object.fromEntries(['cup','spoon'].map(key=>[key,Math.hypot(...state[`${key}Position`].map((value,i)=>value-initial[`${key}Position`][i]))]));
    const success=state.complete&&!state.error&&state.maximumCupHeight>.90&&Object.values(errors).every(error=>error<.010);
    returnTasks.push({seed,success,errors,failure:state.failure,metrics:state.metrics,history:state.history});
    await mkdir('.reports',{recursive:true});
    await writeFile('.reports/contact-return.json',JSON.stringify({assistance:false,returnTasks},null,2));
    console.log(JSON.stringify({task:'present-and-return',seed,success,errors,failure:state.failure}));
    assert(success,'The composed task must return both released objects to their supports');
    assert(state.history.some(entry=>entry.phase==='return-objects'));
    assert.equal(state.inserted,false);
  }
  if(!returnOnly)await writeFile('.reports/contact-grasp.json',JSON.stringify({version:2,assistance:false,cupFingerCollisions:true,
    passed:results.filter(result=>result.success).length,total:results.length,results,faults,returnTasks},null,2));
} finally {simulation.dispose();flags.delete();data.delete();model.delete();vfs.delete();}
