import { Quaternion, Vector3 } from 'three';

// Calibrated in each object's local frame. The spoon point sits between its
// rests, 18 mm inside the fingers; changing the world pose keeps that depth.
export const COOP_OBJECTS = Object.freeze({
  cup: { body: 'coop_cup', joint: 'coop_cup_free', grasp: [0, -.067994, .0020204] },
  spoon: { body: 'coop_spoon', joint: 'coop_spoon_free', grasp: [.0406577, 0, -.015304], bowl: [.094, 0, -.001] }
});
const Z = new Vector3(0, 0, 1);
const LEFT_GRASP = new Quaternion().setFromAxisAngle(Z, Math.PI / 2)
  .multiply(new Quaternion().setFromAxisAngle(new Vector3(0, 1, 0), Math.PI / 6));
const RIGHT_GRASP = new Quaternion().setFromAxisAngle(new Vector3(0, 1, 0), Math.PI / 2);
const point = (p) => new Vector3(...p);
const translated = (pose, delta) => ({ position: point(pose.position).add(point(delta)).toArray(), quaternion: pose.quaternion.clone() });
export function worldPoint(pose, local) { return point(local).applyQuaternion(pose.quaternion).add(point(pose.position)); }
export function yawQuaternion(q) {
  const heading = new Vector3(1, 0, 0).applyQuaternion(q);
  return new Quaternion().setFromAxisAngle(Z, Math.atan2(heading.y, heading.x));
}
export function relativePose(hand, object) {
  const inverse = hand.quaternion.clone().invert();
  return { position: point(object.position).sub(point(hand.position)).applyQuaternion(inverse).toArray(),
    quaternion: inverse.multiply(object.quaternion).normalize() };
}
function handForObject(object, grip) {
  const quaternion = object.quaternion.clone().multiply(grip.quaternion.clone().invert()).normalize();
  return { position: point(object.position).sub(point(grip.position).applyQuaternion(quaternion)).toArray(), quaternion };
}

export function createCooperationGeometry(cup, spoon, measuredGrips = null,{task='spoon-in-cup'}={}) {
  const grasp = {
    left: { position: worldPoint(cup, COOP_OBJECTS.cup.grasp).toArray(), quaternion: yawQuaternion(cup.quaternion).multiply(LEFT_GRASP) },
    right: { position: worldPoint(spoon, COOP_OBJECTS.spoon.grasp).toArray(), quaternion: yawQuaternion(spoon.quaternion).multiply(RIGHT_GRASP) }
  };
  const grips = measuredGrips || { left: relativePose(grasp.left, cup), right: relativePose(grasp.right, spoon) };
  // A shared rendezvous area is a workspace policy. Object-to-hand and
  // bowl-to-opening transforms determine the actual two-arm targets.
  const cupAtMeeting = { position: [.28, .035, cup.position[2] + .030], quaternion: yawQuaternion(cup.quaternion) };
  const leftMeeting = handForObject(cupAtMeeting, grips.left);
  function spoonAtOpening(height) {
    const bowl = worldPoint(cupAtMeeting, [0, 0, height]);
    const pose = { position: bowl.sub(point(COOP_OBJECTS.spoon.bowl).applyQuaternion(spoon.quaternion)).toArray(), quaternion: spoon.quaternion };
    return handForObject(pose, grips.right);
  }
  const aligned = spoonAtOpening(.050), inserted = spoonAtOpening(.019);
  const rightClear = translated(inserted, [0, .115, .052]);
  const leftPlaced = handForObject(translated(cupAtMeeting, [0, 0, -.030]), grips.left);
  const goals=[
    { left: translated(grasp.left, [0, 0, .131]), right: translated(grasp.right, [0, 0, .061]) },
    grasp,
    { left: translated(grasp.left, [0, 0, .030]), right: translated(grasp.right, [0, 0, .061]) },
    { left: leftMeeting, right: aligned },
    { left: leftMeeting, right: inserted },
    null,
    { left: leftMeeting, right: rightClear },
    { left: leftPlaced, right: rightClear },
    null,
    { left: translated(leftPlaced, [0, 0, .151]), right: rightClear }
  ];
  if(task==='present-and-return') {
    // Return above the supports first, then descend. Direct joint-space
    // interpolation from the meeting point crosses the utensil end stop.
    const leftOriginal=handForObject(cup,grips.left),rightOriginal=handForObject(spoon,grips.right);
    const leftSource=translated(leftOriginal,[0,0,.002]),rightSource=translated(rightOriginal,[0,0,.004]);
    const rightWithdrawn=translated(rightOriginal,[0,0,.070]);
    goals[4]={left:translated(leftOriginal,[0,0,.060]),right:rightWithdrawn};
    goals[5]={left:leftSource,right:rightSource};
    goals[6]=null;
    goals[7]={left:leftSource,right:rightWithdrawn};
    goals[9]={left:translated(leftSource,[0,0,.151]),right:translated(rightWithdrawn,[0,.115,.040])};
  }
  return {grasp,restHeight:cup.position[2],goals};
}
