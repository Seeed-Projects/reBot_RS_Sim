import { detectTabletopObjects } from '../src/rgbd-perception.js';

export async function runVisionCase(h,config={}) {
  h.view.setSceneAppearance(config.appearance||{});
  const observations=[],began=performance.now();let count=0,corrupted=false,moved=false,removed=false;
  h.baseFriction??=Float64Array.from(h.model.geom_friction);
  h.model.geom_friction.set(h.baseFriction);
  if(config.frictionScale) {
    const bodies=['coop_cup','coop_spoon'].map(name=>h.mujoco.mj_name2id(h.model,h.mujoco.mjtObj.mjOBJ_BODY.value,name));
    for(let geom=0;geom<h.model.ngeom;geom++)if(bodies.includes(h.model.geom_bodyid[geom]))for(let axis=0;axis<3;axis++)h.model.geom_friction[geom*3+axis]*=config.frictionScale;
  }
  h.simulation.setObservationProvider(async options=>{
    if(config.missingObject&&!removed) {
      // Move a real object outside both cameras. The perception module still
      // receives only rendered pixels; fruit, fixtures and room remain visible.
      const id=h.mujoco.mj_name2id(h.model,h.mujoco.mjtObj.mjOBJ_JOINT.value,`coop_${config.missingObject}_free`);
      h.data.qpos[h.model.jnt_qposadr[id]]+=3;
      h.mujoco.mj_forward(h.model,h.data);removed=true;
    }
    if(config.moveBeforeGrasp&&options.stage==='pregrasp'&&!moved) {
      const id=h.mujoco.mj_name2id(h.model,h.mujoco.mjtObj.mjOBJ_JOINT.value,'coop_cup_free');
      h.data.qpos[h.model.jnt_qposadr[id]]+=config.moveBeforeGrasp;
      h.mujoco.mj_forward(h.model,h.data);moved=true;
    }
    h.view.sync(h.data);
    const frames=[h.view.captureCamera('overhead',{zoom:3}),h.view.captureWorkspaceRgbd()];
    for(const frame of frames) {
      if(config.fault==='occluded'||config.fault==='transient-occlusion'&&count===0) {frame.rgb.fill(0);frame.depth.fill(Infinity);corrupted=true;}
      if(config.fault==='stale')frame.timestamp-=2;
      if(config.noise)for(let i=0;i<frame.depth.length;i++) {
        const noise=Math.sin(i*12.9898+count*78.233)*43758.5453%1;
        frame.depth[i]+=noise*.00015;
        for(let c=0;c<3;c++)frame.rgb[i*4+c]=Math.max(0,Math.min(255,frame.rgb[i*4+c]+Math.round(noise*2)));
        if(i%37===0)frame.depth[i]=Infinity;
      }
      if(config.partialOcclusion)for(let row=0;row<frame.height;row++)for(let column=0;column<frame.width;column++) {
        // Missing stripes model partial sensor occlusion consistently in RGB
        // and depth; all remaining pixels still come from the renderer.
        if(column%40>=config.partialOcclusion*40)continue;
        const pixel=row*frame.width+column;frame.rgb.fill(0,pixel*4,pixel*4+4);frame.depth[pixel]=Infinity;
      }
    }
    count++;
    const observation=detectTabletopObjects(frames,options);
    const truth=h.truth();
    // Ground truth is used only here, after image inference, to score errors.
    observations.push({stage:options.stage,observation,errors:observation.valid?Object.fromEntries(['cup','spoon'].map(key=>[key,Math.hypot(...observation[key].position.map((v,i)=>v-truth[`${key}Position`][i]))])):null});
    return observation;
  });
  h.simulation.startCooperation({seed:config.seed??null,task:config.task||'spoon-in-cup'});
  const phases=[];
  for(let i=0;i<5000&&h.simulation.state().running;i++) {
    if(!h.simulation.state().cooperation.vision.waiting)h.step([10,20,33,47][i%4]);
    const state=h.truth();
    if(phases.at(-1)!==state.phase)phases.push(state.phase);
    await new Promise(resolve=>setTimeout(resolve,0));
  }
  const state=h.truth();
  return {config,corrupted,moved,observations,phases,state,milliseconds:performance.now()-began,render:h.view.getRenderMetrics(),performance:h.simulation.state().performance};
}
