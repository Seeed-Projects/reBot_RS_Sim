import * as THREE from 'three';
import { mergeGeometries } from 'three/addons/utils/BufferGeometryUtils.js';
import { createHomeRoom } from './home-room-view.js';
import { createTabletopProp } from './tabletop-props-view.js';
import { createCoffeeMachine } from './coffee-view.js';

export function createHomeEnvironmentTexture() {
  const canvas = document.createElement('canvas');
  canvas.width = 512; canvas.height = 256;
  const ctx = canvas.getContext('2d');
  const gradient = ctx.createLinearGradient(0, 0, 0, 256);
  gradient.addColorStop(0, '#faf4e6');
  gradient.addColorStop(.45, '#b3beb8');
  gradient.addColorStop(.65, '#797970');
  gradient.addColorStop(1, '#bba383');
  ctx.fillStyle = gradient; ctx.fillRect(0, 0, 512, 256);
  ctx.fillStyle = '#fff9eb'; ctx.fillRect(38, 18, 100, 115);
  ctx.fillStyle = '#e1e4dc'; ctx.fillRect(325, 30, 58, 105);
  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  texture.mapping = THREE.EquirectangularReflectionMapping;
  return texture;
}

// Generated locally: no remote textures or models are required for the room.
export function createWoodTexture(floor = false) {
  const canvas = document.createElement('canvas');
  canvas.width = canvas.height = 512;
  const ctx = canvas.getContext('2d');
  let seed = 718;
  const random = () => { seed = (1664525 * seed + 1013904223) >>> 0; return seed / 4294967296; };
  ctx.fillStyle = floor ? '#b39572' : '#d9b98b';
  ctx.fillRect(0, 0, 512, 512);
  for (let i = 0; i < 2800; i++) {
    const y = random() * 512;
    ctx.strokeStyle = `rgba(${floor ? '74,43,20' : '113,71,30'},${.025 + random() * .12})`;
    ctx.lineWidth = .25 + random() * 1.2;
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.bezierCurveTo(170, y + random() * 6 - 3, 340, y + random() * 6 - 3, 512, y);
    ctx.stroke();
  }
  if (floor) {
    ctx.strokeStyle = 'rgba(62,44,29,.25)';
    ctx.lineWidth = 1.5;
    for (let y = 0; y < 512; y += 64) {
      ctx.strokeRect(0, y, 512, 64);
      ctx.beginPath(); ctx.moveTo((y * 7) % 512, y); ctx.lineTo((y * 7) % 512, y + 64); ctx.stroke();
    }
  }
  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  texture.wrapS = texture.wrapT = THREE.RepeatWrapping;
  texture.repeat.set(floor ? 3 : 1, floor ? 3 : 1);
  texture.anisotropy = 4;
  return texture;
}

export function createHomeProp(name) {
  if (name === 'coffee_machine_visual') return createCoffeeMachine();
  if (name === 'home_room_anchor') return createHomeRoom(createWoodTexture());
  const tabletop = createTabletopProp(name);
  if (tabletop) return tabletop;
  if (name !== 'basket_bottom') return null;
  const group = new THREE.Group();
  const parts = [];
  function rod(a, b, radius = .0016) {
    const start = new THREE.Vector3(...a), end = new THREE.Vector3(...b);
    const delta = end.clone().sub(start);
    const g = new THREE.CylinderGeometry(radius, radius, delta.length(), 5).toNonIndexed();
    g.applyQuaternion(new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 1, 0), delta.normalize()));
    g.translate(...start.add(end).multiplyScalar(.5).toArray());
    parts.push(g);
  }
  const base = new THREE.BoxGeometry(.24, .29, .008).toNonIndexed();
  parts.push(base);
  for (let i = -11; i <= 11; i++) {
    const y = i * .012;
    rod([-.119, y, .004], [-.119, y, .05]);
    rod([.119, y, .004], [.119, y, .05]);
  }
  for (let i = -9; i <= 9; i++) {
    const x = i * .012;
    rod([x, -.146, .004], [x, -.146, .05]);
    rod([x, .146, .004], [x, .146, .05]);
  }
  for (let z = .008; z <= .05; z += .006) {
    rod([-.12, -.146, z], [.12, -.146, z]);
    rod([-.12, .146, z], [.12, .146, z]);
    rod([-.12, -.146, z], [-.12, .146, z]);
    rod([.12, -.146, z], [.12, .146, z]);
  }
  const mesh = new THREE.Mesh(mergeGeometries(parts), new THREE.MeshStandardMaterial({ color: 0xb79561, roughness: .94 }));
  parts.forEach((g) => g.dispose());
  mesh.castShadow = mesh.receiveShadow = true;
  group.add(mesh);
  return group;
}
