import { ARM_JOINTS, homePose } from './kinematics.js';
import { CONTROL_CONFIG } from './control-config.js';

/** Shared SI defaults, with the validated browser gripper profile. */
export const ARM_KP = CONTROL_CONFIG.arm.kp;
export const ARM_KD = CONTROL_CONFIG.arm.kd;
export const ARM_TAU_LIMIT = CONTROL_CONFIG.arm.tauLimit;
export const GRIPPER_KP = CONTROL_CONFIG.gripperProfiles.web.kp;
export const GRIPPER_KD = CONTROL_CONFIG.gripperProfiles.web.kd;
export const GRIPPER_TAU_LIMIT = CONTROL_CONFIG.gripperProfiles.web.tauLimit;

/** Default for explicit manual stepping; live pages use the wall-time clock. */
export const STEPS_PER_FRAME = 4;

function clip(value, min, max) {
  return Math.min(max, Math.max(min, value));
}

function namedId(mujoco, model, type, name) {
  const id = mujoco.mj_name2id(model, type, name);
  if (id < 0) throw new Error(`找不到 ${name}`);
  return id;
}

export function createPhysicsController(
  mujoco,
  model,
  data,
  joints,
  { beforeStep, afterStep } = {}
) {
  const jointType = mujoco.mjtObj.mjOBJ_JOINT.value;
  const actuatorType = mujoco.mjtObj.mjOBJ_ACTUATOR.value;
  const prefix = joints.prefix || '';

  const arm = ARM_JOINTS.map((joint, index) => {
    const id = namedId(mujoco, model, jointType, `${prefix}${joint.name}`);
    return {
      name: joint.name,
      qposadr: model.jnt_qposadr[id],
      dofadr: model.jnt_dofadr[id],
      actuatorId: namedId(mujoco, model, actuatorType, `${prefix}${joint.name}_motor`),
      kp: ARM_KP[index],
      kd: ARM_KD[index],
      tauLimit: ARM_TAU_LIMIT[index]
    };
  });

  const gripperJointId = namedId(mujoco, model, jointType, `${prefix}joint7`);
  const gripper = {
    qposadr: model.jnt_qposadr[gripperJointId],
    dofadr: model.jnt_dofadr[gripperJointId],
    actuatorId: namedId(mujoco, model, actuatorType, `${prefix}joint7_motor`)
  };

  const targets = homePose(joints);
  let gripperProfile=CONTROL_CONFIG.gripperProfiles.web;
  let gripperCommand=null,gripperControlAt=null,preloadAt=null,preload=0;

  function setGripperProfile(name='web') {
    const profile=CONTROL_CONFIG.gripperProfiles[name];
    if(!profile)throw new RangeError(`Unknown gripper profile: ${name}`);
    gripperProfile=profile;gripperCommand=null;gripperControlAt=preloadAt=null;preload=profile.preload||0;
  }
  function setGripperPreload(value) {
    if(!Number.isFinite(value)||value<0||value>(gripperProfile.maxPreload||0))throw new RangeError('Invalid gripper preload');
    preload=value;
  }

  function setTarget(name, value) {
    const joint = joints.byName[name];
    if (!joint || !Number.isFinite(value)) return;
    const next = clip(value, joint.min, joint.max);
    targets[name] = next;
    if (name === 'joint7') {
      targets.joint_left = next;
      targets.joint_right = next;
    }
  }

  function setTargets(pose) {
    Object.entries(pose).forEach(([name, value]) => setTarget(name, value));
  }

  function applyCtrl() {
    for (const joint of arm) {
      const q = data.qpos[joint.qposadr];
      const qd = data.qvel[joint.dofadr];
      const bias = data.qfrc_bias[joint.dofadr];
      const tau = bias + joint.kp * (targets[joint.name] - q) - joint.kd * qd;
      data.ctrl[joint.actuatorId] = clip(tau, -joint.tauLimit, joint.tauLimit);
    }
    const position=data.qpos[gripper.qposadr];
    gripperCommand??=position;
    const elapsed=gripperControlAt===null?0:Math.max(0,data.time-gripperControlAt);
    gripperControlAt=data.time;
    if(gripperProfile.closingRate) {
      const delta=targets.joint7-gripperCommand,rate=delta<0?gripperProfile.closingRate:gripperProfile.openingRate;
      gripperCommand+=clip(delta,-rate*elapsed,rate*elapsed);
    } else gripperCommand=targets.joint7;
    if(preload&&targets.joint7<gripperProfile.preloadWidth&&position<gripperProfile.preloadWidth&&gripperCommand<gripperProfile.preloadWidth)preloadAt??=data.time;
    else preloadAt=null;
    const closingForce=preloadAt===null?0:preload*Math.min(1,(data.time-preloadAt)/gripperProfile.preloadRampSeconds);
    const gripperTau =gripperProfile.kp*(gripperCommand-position)-gripperProfile.kd*data.qvel[gripper.dofadr]-closingForce;
    data.ctrl[gripper.actuatorId] = clip(
      gripperTau,
      -gripperProfile.tauLimit,
      gripperProfile.tauLimit
    );
  }

  function step(steps = STEPS_PER_FRAME) {
    for (let i = 0; i < steps; i += 1) {
      applyCtrl();
      beforeStep?.();
      mujoco.mj_step(model, data);
      afterStep?.();
    }
  }

  function reset() {
    mujoco.mj_resetData(model, data);
    Object.assign(targets, homePose(joints));
    setGripperProfile();
    mujoco.mj_forward(model, data);
  }

  function telemetry() {
    const samples = arm.map((joint) => ({
      name: joint.name,
      target: targets[joint.name],
      position: data.qpos[joint.qposadr],
      error: targets[joint.name] - data.qpos[joint.qposadr],
      torque: data.qfrc_actuator?.[joint.dofadr] ?? data.ctrl[joint.actuatorId],
      torqueLimit: joint.tauLimit
    }));
    samples.push({
      name: 'joint7',
      target: targets.joint7,
      position: data.qpos[gripper.qposadr],
      error: targets.joint7 - data.qpos[gripper.qposadr],
      torque: data.qfrc_actuator?.[gripper.dofadr] ?? data.ctrl[gripper.actuatorId],
      torqueLimit: gripperProfile.tauLimit,
      command:gripperCommand,preload
    });
    const rmsError = Math.sqrt(
      samples.reduce((sum, sample) => sum + sample.error * sample.error, 0) / samples.length
    );
    return { joints: samples, rmsError };
  }

  return { targets, setTarget, setTargets, setGripperProfile,setGripperPreload,applyCtrl, step, reset, telemetry };
}
