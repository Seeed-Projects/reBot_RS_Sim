import assert from 'node:assert/strict';
import {observeObjects,createStablePlacementAction} from '../src/cooperative-actions.js';

const pose=x=>({position:[x,0,.9],quaternion:[0,0,0,1],confidence:.9});
const frame=(timestamp=1,x=0)=>({source:'rgbd-pixels',valid:true,timestamp,cup:pose(x),spoon:pose(x+.1)});
let calls=0,attempts=[];
const recovered=await observeObjects(async()=>++calls===1?frame(-1):frame(),{stage:'initial'},
  {now:()=>1,isCurrent:()=>true,onAttempt:attempt=>attempts.push(attempt)});
assert.equal(calls,2);assert.equal(recovered.observation.valid,true);assert.equal(attempts[0].fresh,false);
calls=0;
const stale=await observeObjects(async()=>{calls++;return frame(-1);},{stage:'initial'},
  {now:()=>1,isCurrent:()=>true,onAttempt:()=>{}});
assert.equal(stale.error,'vision-stale');assert.equal(calls,3);
const invalid=await observeObjects(async()=>({...frame(),spoon:{...pose(.1),quaternion:[0,0,0,0]}}),{stage:'initial'},
  {now:()=>1,isCurrent:()=>true,onAttempt:()=>{}});
assert.equal(invalid.error,'vision-occluded');
calls=0;
assert.equal(await observeObjects(async()=>{calls++;return frame();},{stage:'initial'},
  {now:()=>1,isCurrent:()=>false,onAttempt:()=>{}}),null);assert.equal(calls,0);

const verification=createStablePlacementAction(observation=>observation.valid);
assert.equal(verification.accept(frame(),1).complete,false);
assert.equal(verification.ready(1.2),false);
assert.equal(verification.accept(frame(1.2),1.2).complete,false);
assert.equal(verification.accept(frame(1.4,.002),1.4).complete,true);
verification.reset();verification.accept(frame(),1);
assert.equal(verification.accept(frame(1.4,.005),1.4).error,'placement-not-stable');
verification.reset();assert.equal(verification.pending,false);
assert.equal(verification.accept({...frame(),valid:false},1).error,'placement-not-stable');
console.log('Shared actions: fresh RGB-D retries, cancellation, pose validation and independent stable-placement checks passed');
