// Simulated tactile feedback only: no object pose or vision truth enters here.
// Contact patches are low-pass filtered because individual solver contacts can
// appear/disappear while the same physical surface remains firmly held.
export function createHoldMonitor({slipLimit=.004,filterSeconds=.12,lossSeconds=.10}={}) {
  let origin=null,filtered=null,previousAt=null,lostAt=null,slippingAt=null;
  function reset() {origin=filtered=previousAt=lostAt=slippingAt=null;}
  function sample(contact,time) {
    const dt=previousAt===null?0:Math.max(0,time-previousAt);previousAt=time;
    if(!contact.bilateral) {
      lostAt??=time;
      return {slip:0,lostSeconds:time-lostAt,failure:time-lostAt>lossSeconds?'contact-grasp-lost':null};
    }
    lostAt=null;
    const centres=contact.contactCentres;
    if(!origin) {
      origin=structuredClone(centres);filtered=structuredClone(centres);
    }
    const drifts=[];
    for(const [key,point] of Object.entries(centres)) {
      if(!point||!origin[key])continue;
      const alpha=1-Math.exp(-dt/filterSeconds);
      filtered[key]=point.map((value,axis)=>filtered[key][axis]+alpha*(value-filtered[key][axis]));
      drifts.push(Math.hypot(...filtered[key].map((value,axis)=>value-origin[key][axis])));
    }
    // One pressure centroid can move as load transfers between contacts.
    // Require both opposed tactile patches to agree on material movement.
    const slip=drifts.length>=2?Math.min(...drifts):0;
    if(slip>slipLimit)slippingAt??=time;else slippingAt=null;
    return {slip,lostSeconds:0,failure:slippingAt!==null&&time-slippingAt>.06?'contact-slip-detected':null};
  }
  return {sample,reset};
}
