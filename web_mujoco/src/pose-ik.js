import { Quaternion, Vector3 } from 'three';
import { ARM_JOINTS } from './kinematics.js';

const ROTATION_WEIGHT = .13;
const clamp = (x, a, b) => Math.max(a, Math.min(b, x));

function rotationVector(target, actual) {
  const q = target.clone().multiply(actual.clone().invert()).normalize();
  if (q.w < 0) q.set(-q.x, -q.y, -q.z, -q.w);
  const norm = Math.hypot(q.x, q.y, q.z);
  const scale = norm < 1e-9 ? 2 : 2 * Math.atan2(norm, q.w) / norm;
  return [q.x * scale, q.y * scale, q.z * scale];
}

function solveLinear(matrix, rhs) {
  const rows = matrix.map((row, i) => [...row, rhs[i]]);
  for (let col = 0; col < rhs.length; col++) {
    let pivot = col;
    for (let row = col + 1; row < rhs.length; row++) if (Math.abs(rows[row][col]) > Math.abs(rows[pivot][col])) pivot = row;
    [rows[col], rows[pivot]] = [rows[pivot], rows[col]];
    if (Math.abs(rows[col][col]) < 1e-12) return null;
    const div = rows[col][col];
    for (let i = col; i <= rhs.length; i++) rows[col][i] /= div;
    for (let row = 0; row < rhs.length; row++) if (row !== col) {
      const factor = rows[row][col];
      for (let i = col; i <= rhs.length; i++) rows[row][i] -= factor * rows[col][i];
    }
  }
  return rows.map((row) => row[rhs.length]);
}

// Solve against scratch data so trajectory planning never teleports a live arm
// or changes the other arm/object state. Only motor targets use the result.
export function createPosePlanner(mujoco, model, data, joints, bodyId) {
  const scratch = new mujoco.MjData(model);
  const jacp = new mujoco.DoubleBuffer(3 * model.nv), jacr = new mujoco.DoubleBuffer(3 * model.nv);
  const statistics = { solves: 0, iterations: 0, totalMs: 0, maxMs: 0 };
  const arm = ARM_JOINTS.map((joint) => joints.byName[joint.name]);
  function pose() {
    const i = bodyId * 3, q = bodyId * 4;
    return { position: new Vector3(...scratch.xpos.subarray(i, i + 3)),
      quaternion: new Quaternion(scratch.xquat[q + 1], scratch.xquat[q + 2], scratch.xquat[q + 3], scratch.xquat[q]) };
  }
  function apply(angles) {
    for (const joint of arm) scratch.qpos[joint.qposadr] = angles[joint.name];
    mujoco.mj_kinematics(model, scratch);
    mujoco.mj_comPos(model, scratch);
  }
  function solve(position, quaternion, seed) {
    const began = performance.now(); statistics.solves++;
    const finish = result => { const ms = performance.now() - began; statistics.totalMs += ms; statistics.maxMs = Math.max(statistics.maxMs, ms); return result; };
    scratch.qpos.set(data.qpos);
    const target = new Vector3(...position);
    const seeds = [seed, { ...seed, joint2: .7, joint3: .8, joint4: -.4 },
      { ...seed, joint2: 1.1, joint3: 1.2, joint4: .8 }];
    let best = null;
    for (const initial of seeds) {
      let angles = { ...initial };
      for (let iteration = 0; iteration < 180; iteration++) {
        statistics.iterations++;
        apply(angles);
        const current = pose();
        const dp = target.clone().sub(current.position).toArray();
        const dr = rotationVector(quaternion, current.quaternion);
        const positionError = Math.hypot(...dp), rotationError = Math.hypot(...dr);
        const score = positionError + ROTATION_WEIGHT * rotationError;
        if (!best || score < best.score) best = { angles: { ...angles }, positionError, rotationError, score };
        if (positionError < .0006 && rotationError < .012) return finish(best);
        const error = [...dp, ...dr.map((x) => x * ROTATION_WEIGHT)];
        mujoco.mj_jacBody(model, scratch, jacp, jacr, bodyId);
        const jp = jacp.GetView(), jr = jacr.GetView();
        const columns = arm.map(joint => {
          const dof = model.jnt_dofadr[joint.id];
          return [jp[dof], jp[model.nv+dof], jp[2*model.nv+dof],
            jr[dof]*ROTATION_WEIGHT, jr[model.nv+dof]*ROTATION_WEIGHT, jr[2*model.nv+dof]*ROTATION_WEIGHT];
        });
        const dot = (a, b) => a.reduce((sum, x, i) => sum + x * b[i], 0);
        const gram = columns.map((a, i) => columns.map((b, j) => dot(a, b) + (i === j ? .000015 : 0)));
        const delta = solveLinear(gram, columns.map((column) => dot(column, error)));
        if (!delta) break;
        const gain = Math.min(1, .15 / Math.max(...delta.map(Math.abs)));
        arm.forEach((joint, i) => { angles[joint.name] = clamp(angles[joint.name] + delta[i] * gain, joint.min, joint.max); });
      }
    }
    return finish(best);
  }
  return { solve, statistics, dispose() { jacp.delete(); jacr.delete(); scratch.delete(); } };
}
