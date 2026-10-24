import { Quaternion, Vector3 } from 'three';
import { COOP_OBJECTS } from './cooperation-geometry.js';

export function validateLayoutSeed(seed) {
  if (seed !== null && (!Number.isInteger(seed) || seed < 0 || seed > 0xffffffff)) throw new RangeError('Layout seed must be a uint32 or null');
  return seed;
}
export function createCooperationLayout(mujoco, model, data) {
  const bodies = Object.fromEntries(Object.entries(COOP_OBJECTS).map(([key, spec]) => {
    const joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT.value, spec.joint);
    if (joint < 0) throw new Error(`Missing layout joint: ${spec.joint}`);
    const address = model.jnt_qposadr[joint];
    return [key, { address, initial: Array.from(model.qpos0.subarray(address, address + 7)) }];
  }));
  const fixtureBody = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY.value, 'spoon_fixture');
  const fixture = fixtureBody < 0 ? -1 : model.body_mocapid[fixtureBody];
  if (fixture < 0) throw new Error('Missing spoon fixture mocap body');
  function apply(seed) {
    validateLayoutSeed(seed);
    let state = seed;
    const random = () => { state = (Math.imul(state, 1664525) + 1013904223) >>> 0; return state / 4294967296 * 2 - 1; };
    const offsets = Object.fromEntries(['cup', 'spoon'].map(key => [key, seed === null
      ? { x: 0, y: 0, yaw: 0 } : { x: random() * .010, y: random() * .008, yaw: random() * .08 }]));
    for (const [key, body] of Object.entries(bodies)) {
      const { x, y, yaw } = offsets[key];
      const q = new Quaternion().setFromAxisAngle(new Vector3(0, 0, 1), yaw)
        .multiply(new Quaternion(body.initial[4], body.initial[5], body.initial[6], body.initial[3]));
      data.qpos.set([body.initial[0] + x, body.initial[1] + y, body.initial[2], q.w, q.x, q.y, q.z], body.address);
    }
    // Move the complete spoon fixture with the utensil. It must remain under
    // both ends after translation/rotation, in the physics and camera views.
    const { x, y, yaw } = offsets.spoon;
    data.mocap_pos.set([bodies.spoon.initial[0] + x, bodies.spoon.initial[1] + y, bodies.spoon.initial[2]], fixture * 3);
    data.mocap_quat.set([Math.cos(yaw / 2), 0, 0, Math.sin(yaw / 2)], fixture * 4);
    mujoco.mj_forward(model, data);
    return { seed, offsets };
  }
  return { apply };
}
