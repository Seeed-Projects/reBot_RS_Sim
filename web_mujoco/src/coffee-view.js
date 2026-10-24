import * as THREE from 'three';
import { mergeGeometries } from 'three/addons/utils/BufferGeometryUtils.js';

export const COFFEE_LAYOUT = Object.freeze({ machine: [.68, -.30, .84], spout: [.29, -.20, 1.265] });

export function createCoffeeMachine() {
  const group = new THREE.Group();
  const steel = new THREE.MeshStandardMaterial({ color: 0xaab2ae, metalness: .75, roughness: .28 });
  const dark = new THREE.MeshStandardMaterial({ color: 0x283732, metalness: .25, roughness: .4 });
  function part(geometry, material, position) {
    const mesh = new THREE.Mesh(geometry, material);
    mesh.position.set(...position); mesh.castShadow = mesh.receiveShadow = true; group.add(mesh); return mesh;
  }
  part(new THREE.BoxGeometry(.13, .17, .40), dark, [0, 0, .20]);
  part(new THREE.BoxGeometry(.16, .18, .014), steel, [0, 0, .007]);

  part(new THREE.BoxGeometry(.16, .17, .055), dark, [0, 0, .395]);
  part(new THREE.BoxGeometry(.005, .135, .105), steel, [-.068, 0, .115]);
  part(new THREE.CylinderGeometry(.017, .017, .018, 24).rotateX(Math.PI/2), steel, [-.39, .10, .441]);
  part(new THREE.CylinderGeometry(.004, .004, .014, 16).rotateX(Math.PI/2), steel, [-.39, .10, .432]);
  const pipeStart = new THREE.Vector3(-.07, 0, .46), pipeEnd = new THREE.Vector3(-.39, .10, .46);
  const pipeDelta = pipeEnd.clone().sub(pipeStart);
  const pipe = part(new THREE.CylinderGeometry(.006, .006, pipeDelta.length(), 16), steel, pipeStart.clone().add(pipeEnd).multiplyScalar(.5).toArray());
  pipe.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), pipeDelta.normalize());
  part(new THREE.CylinderGeometry(.006, .006, .10, 16).rotateX(Math.PI/2), steel, [-.07, 0, .41]);
  part(new THREE.CylinderGeometry(.006, .006, .03, 16).rotateX(Math.PI/2), steel, [-.39, .10, .448]);
  const glass = new THREE.MeshPhysicalMaterial({ color: 0x625441, roughness: .2, transparent: true, opacity: .7 });
  part(new THREE.CylinderGeometry(.046, .035, .045, 32).rotateX(Math.PI/2), glass, [.015, 0, .446]);
  const bean = new THREE.MeshStandardMaterial({ color: 0x36251b, roughness: .8 });
  for (let i = 0; i < 18; i++) {
    const a = i*2.4, r = .034*Math.sqrt(i/18);
    part(new THREE.SphereGeometry(.006, 8, 6).scale(1, .7, .6), bean, [.015+r*Math.cos(a), r*Math.sin(a), .447]);
  }
  part(new THREE.BoxGeometry(.003, .038, .014), new THREE.MeshStandardMaterial({ color: 0x9bc7a4, emissive: 0x4d8d64, emissiveIntensity: .5 }), [-.083, -.035, .40]);
  part(new THREE.CylinderGeometry(.009, .009, .005, 16).rotateZ(Math.PI/2), steel, [-.084, .04, .40]);
  // Keep the machine's many details to one draw call per finish.
  const batches = new Map();
  for (const object of group.children) {
    object.updateMatrix();
    const geometry = object.geometry.toNonIndexed();
    geometry.applyMatrix4(object.matrix);
    if (!batches.has(object.material)) batches.set(object.material, []);
    batches.get(object.material).push(geometry);
    object.geometry.dispose();
  }
  group.clear();
  for (const [material, parts] of batches) {
    const merged = new THREE.Mesh(mergeGeometries(parts), material);
    parts.forEach(geometry => geometry.dispose());
    merged.castShadow = merged.receiveShadow = true; group.add(merged);
  }
  return group;
}

// Visual liquid follows measured cup/spoon poses; filling is driven by the
// verified dispensing phase, rather than starting with a painted brown cup.
export function createCoffeeVisuals(scene) {
  const coffee = new THREE.MeshPhysicalMaterial({ color: 0x381a0a, roughness: .24, clearcoat: .65 });
  const surface = new THREE.Mesh(new THREE.CircleGeometry(.0305, 64), coffee);
  const crema = new THREE.Mesh(new THREE.TorusGeometry(.0285, .0011, 8, 64), new THREE.MeshStandardMaterial({ color: 0xb78b52, roughness: .6 }));
  surface.add(crema);
  const ripple = new THREE.Mesh(new THREE.TorusGeometry(.012, .00055, 6, 48), new THREE.MeshStandardMaterial({ color: 0x98724a, roughness: .4 }));
  ripple.position.z = .0004; surface.add(ripple);
  const stream = new THREE.Mesh(new THREE.CylinderGeometry(.0017, .0022, 1, 12), coffee.clone());
  const spoonCoffee = new THREE.Mesh(new THREE.SphereGeometry(1, 24, 12), coffee.clone());
  spoonCoffee.scale.set(.015, .009, .00065);
  scene.add(surface, stream, spoonCoffee);
  function update(state) {
    const fill = state?.coffeeFill || 0;
    surface.visible = fill > .001; stream.visible = Boolean(state?.dispensing);
    spoonCoffee.visible = Boolean(state?.spoonLoaded);
    if (!state || !surface.visible && !stream.visible && !spoonCoffee.visible) return;
    const cupQ = new THREE.Quaternion(...state.cupQuaternion);
    const level = -.032 + .066*fill;
    const surfacePoint = new THREE.Vector3(0, 0, level).applyQuaternion(cupQ).add(new THREE.Vector3(...state.cupPosition));
    surface.position.copy(surfacePoint); surface.quaternion.copy(cupQ);
    const stirring = state.phase === 'stir';
    surface.rotateX(stirring ? .008*Math.sin(state.time*7) : 0);
    ripple.visible = stirring || state.dispensing;
    ripple.scale.setScalar(.7+.25*Math.sin(state.time*9));
    ripple.position.x = stirring ? .006*Math.cos(state.time*2.5) : 0;
    ripple.position.y = stirring ? .006*Math.sin(state.time*2.5) : 0;
    if (state.dispensing) {
      const top = new THREE.Vector3(...COFFEE_LAYOUT.spout);
      const delta = surfacePoint.clone().sub(top);
      stream.position.copy(top).addScaledVector(delta, .5);
      stream.scale.set(1, delta.length(), 1);
      stream.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), delta.normalize());
    }
    const spoonQ = new THREE.Quaternion(...state.spoonQuaternion);
    spoonCoffee.position.copy(new THREE.Vector3(.094, 0, -.0018).applyQuaternion(spoonQ).add(new THREE.Vector3(...state.spoonPosition)));
    spoonCoffee.quaternion.copy(spoonQ);
  }
  update(null);
  return { update, dispose() {
    for (const mesh of [surface, stream, spoonCoffee]) {
      scene.remove(mesh);
      mesh.traverse(object => { object.geometry?.dispose(); object.material?.dispose(); });
    }
  } };
}
