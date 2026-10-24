// Simulated tactile measurements. These are contact forces, not object poses;
// image perception and visual task planning never receive this module's data.
export function createGraspContactSensor(mujoco,model,data,object,fingers) {
  const ids=new Set(fingers),force=new mujoco.DoubleBuffer(6);
  function sample() {
    const normal=Object.fromEntries([...ids].map(id=>[id,0]));
    const centres=Object.fromEntries([...ids].map(id=>[id,[0,0,0]]));
    let penetration=0,count=0,tangential=0;
    const contacts=data.contact;
    try {
      for(let index=0;index<Math.min(data.ncon,contacts.size());index++) {
        const contact=contacts.get(index);
        try {
          const a=model.geom_bodyid[contact.geom1],b=model.geom_bodyid[contact.geom2];
          const finger=a===object?b:b===object?a:-1;
          if(!ids.has(finger)||contact.dist>=.0003)continue;
          penetration=Math.max(penetration,-contact.dist);
          // Detected contacts inside the margin can have zero load. They must
          // not count as a force-bearing grasp merely because they exist.
          mujoco.mj_contactForce(model,data,index,force);
          const values=force.GetView(),load=Math.max(0,values[0]);
          normal[finger]+=load;tangential+=Math.hypot(values[1],values[2]);count++;
          // A tactile patch is expressed in the finger frame so arm motion
          // alone cannot look like an object sliding across its surface.
          for(let axis=0;axis<3;axis++) {
            let local=0;
            for(let row=0;row<3;row++)local+=data.xmat[finger*9+row*3+axis]*(contact.pos[row]-data.xpos[finger*3+row]);
            centres[finger][axis]+=local*load;
          }
        } finally {contact.delete();}
      }
    } finally {contacts.delete();}
    const loads=Object.values(normal),normalForce=loads.reduce((sum,value)=>sum+value,0);
    return {bilateral:loads.every(value=>value>.01),normalForce,minNormalForce:Math.min(...loads),
      fingerForces:normal,contactCentres:Object.fromEntries([...ids].map(id=>
        [id,normal[id]>.01?centres[id].map(value=>value/normal[id]):null])),
      tangentialForce:tangential,penetration,count};
  }
  return {sample,dispose(){force.delete();}};
}
