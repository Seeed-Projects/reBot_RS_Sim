import {readFile,writeFile,mkdir} from 'node:fs/promises';
import loadMujoco from '@mujoco/mujoco';
import {createDualSimulation} from '../src/dual-simulation.js';

const root=new URL('../../rebotarm_mujoco_rs/models/',import.meta.url);
const source=await readFile(new URL('rs_dual_tabletop_scene.xml',root),'utf8');
const xml=source.replace(/\s*<exclude body1="left_gripper_(?:left|right)" body2="coop_cup"\s*\/>/g,'');
const mujoco=await loadMujoco(),vfs=new mujoco.MjVFS();
for(const file of new Set([...xml.matchAll(/file="([^"]+)"/g)].map(m=>m[1])))vfs.addBuffer(`meshes/${file}`,await readFile(new URL(`meshes/${file}`,root)));
const model=mujoco.MjModel.from_xml_string(xml,vfs),data=new mujoco.MjData(model);
const force=new mujoco.DoubleBuffer(6),flags=new mujoco.DoubleBuffer(model.neq);
const name=(type,id)=>mujoco.mj_id2name(model,type,id);
const bodyNames=Array.from({length:model.nbody},(_,i)=>name(mujoco.mjtObj.mjOBJ_BODY.value,i));
const geomNames=Array.from({length:model.ngeom},(_,i)=>name(mujoco.mjtObj.mjOBJ_GEOM.value,i));
const reports=[];
const initialFriction=Float64Array.from(model.geom_friction);
function contacts() {
  const result=[],native=data.contact;
  try {for(let i=0;i<Math.min(data.ncon,native.size());i++) {
    const c=native.get(i);
    try {
      const a=bodyNames[model.geom_bodyid[c.geom1]],b=bodyNames[model.geom_bodyid[c.geom2]];
      if(![a,b].some(n=>n==='coop_cup'||n==='coop_spoon'))continue;
      mujoco.mj_contactForce(model,data,i,force);
      result.push({a,b,geomA:geomNames[c.geom1],geomB:geomNames[c.geom2],distance:c.dist,force:Array.from(force.GetView())});
    } finally {c.delete();}
  }} finally {native.delete();}
  return result;
}
try {
  const mode=process.argv.includes('--global')?'global':process.argv.includes('--solver')?'solver':'baseline';
  const cases=mode==='global'?[
    {label:'global-10',kp:1800,kd:18,impratio:10,preload:2,global:true,settle:2500},
    {label:'global-10-seed1',kp:1800,kd:18,impratio:10,preload:2,global:true,settle:2500,seed:1},
    {label:'held-10-preload4',kp:1800,kd:18,impratio:10,preload:4},
    {label:'held-10-seed42-low-friction',kp:1800,kd:18,impratio:10,preload:2,seed:42,friction:.9},
    {label:'held-10-seed17-high-friction',kp:1800,kd:18,impratio:10,preload:2,seed:17,friction:1.1}
  ]:mode==='solver'?[
    {label:'elliptic-10',kp:1800,kd:18,impratio:10},
    {label:'elliptic-100',kp:1800,kd:18,impratio:100},
    {label:'preload-2N',kp:1800,kd:18,preload:2},
    {label:'preload-4N',kp:1800,kd:18,preload:4},
    {label:'elliptic-preload-2N',kp:1800,kd:18,impratio:10,preload:2}
  ]:[{label:'baseline'},{label:'ros-gripper',kp:1800,kd:18},{label:'firm-gripper',kp:3000,kd:20}];
  for(const config of cases) {
    model.geom_friction.set(initialFriction);
    for(let geom=0;geom<model.ngeom;geom++)if(['coop_cup','coop_spoon'].includes(bodyNames[model.geom_bodyid[geom]]))
      for(let axis=0;axis<3;axis++)model.geom_friction[geom*3+axis]*=config.friction||1;
    model.opt.cone=config.global?mujoco.mjtCone.mjCONE_ELLIPTIC.value:mujoco.mjtCone.mjCONE_PYRAMIDAL.value;
    model.opt.impratio=config.global?config.impratio:1;
    const sim=createDualSimulation(mujoco,model,data,{graspMode:'contact'});
    if(config.kp)for(const [side,arm] of Object.entries(sim.arms)) {
      const joint=arm.joints.byName.joint7,actuator=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_ACTUATOR.value,`${side}_joint7_motor`);
      const original=arm.physics.applyCtrl;let closingAt=null;
      arm.physics.applyCtrl=()=>{
        original();
        if(arm.physics.targets.joint7<.01&&data.qpos[joint.qposadr]<.006)closingAt??=data.time;else closingAt=null;
        const preload=closingAt===null?0:(config.preload||0)*Math.min(1,(data.time-closingAt)/.5);
        data.ctrl[actuator]=Math.max(-64,Math.min(64,config.kp*(arm.physics.targets.joint7-data.qpos[joint.qposadr])-config.kd*data.qvel[model.jnt_dofadr[joint.id]]-preload));
      };
    }
    const samples=[];let phase='';
    sim.reset({seed:config.seed??null});sim.step(config.settle||0);
    sim.startCooperation({seed:config.seed??null,resetScene:!config.settle});
    for(let i=0;i<4500&&sim.state().running;i++) {
      if(config.impratio&&!config.global&&sim.state().cooperation.cupHeld) {
        model.opt.cone=mujoco.mjtCone.mjCONE_ELLIPTIC.value;model.opt.impratio=config.impratio;
      }
      sim.step(10);
      mujoco.mj_getState(model,data,flags,mujoco.mjtState.mjSTATE_EQ_ACTIVE.value);
      if(flags.GetView().slice(-2).some(v=>v!==0))throw new Error('Grasp weld activated');
      const s=sim.state().cooperation;
      if(s.phase!==phase || i%10===0&&s.phase!=='approach') {
        samples.push({time:data.time,phase:s.phase,cup:s.cupPosition,spoon:s.spoonPosition,cupQuaternion:s.cupQuaternion,
          joints:Object.fromEntries(Object.entries(sim.arms).map(([side,arm])=>[side,arm.physics.telemetry().joints.at(-1)])),contacts:contacts()});
        phase=s.phase;
      }
    }
    reports.push({config,state:sim.state().cooperation,samples});
    console.log(JSON.stringify({config,complete:reports.at(-1).state.complete,failure:reports.at(-1).state.failure,cup:reports.at(-1).state.cupPosition}));
    sim.dispose();
  }
  await mkdir('.reports',{recursive:true});await writeFile(`.reports/contact-diagnosis-${mode}.json`,JSON.stringify({reports},null,2));
} finally {force.delete();flags.delete();data.delete();model.delete();vfs.delete();}
