import { loadMujocoModule, loadRsScene } from '../src/load-model.js';
import { createSceneView } from '../src/scene-view.js';
import { createDualSimulation } from '../src/dual-simulation.js';
import { MODEL_VARIANTS } from '../src/model-variants.js';
import { paintCameraFrame } from '../src/camera-sampler.js';

const { mujoco, model, data, materialProps } = await loadRsScene(loadMujocoModule(), progress => { window.loadProgress = progress; }, { sceneXml: 'rs_dual_tabletop_scene.xml' });
const view = createSceneView(document.getElementById('viewport'), MODEL_VARIANTS.dual.view);
view.build(mujoco, model, materialProps);
const simulation = createDualSimulation(mujoco, model, data,{graspMode:new URLSearchParams(location.search).get('grasp')==='contact'?'contact':'assisted'});
simulation.step(300);
function capture() {
  view.sync(data);
  const frame = view.captureCamera('overhead', { zoom: 3 });
  paintCameraFrame(document.getElementById('sensor').getContext('2d'), frame);
  return frame;
}
window.visionHarness = {
  simulation, mujoco, model, data, view, capture,
  layout(seed) { simulation.setCooperationLayout(seed); view.sync(data); return simulation.state(); },
  step(count) { simulation.step(count); view.sync(data); },
  // Test-only ground truth remains outside the image perception/control API.
  truth() { return simulation.state().cooperation; }
};
capture();
window.ready = true;
