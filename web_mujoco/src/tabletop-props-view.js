import * as THREE from 'three';

function mesh(group, geometry, material) {
  const object = new THREE.Mesh(geometry, material);
  object.castShadow = object.receiveShadow = true;
  group.add(object);
  return object;
}

function speckleTexture(base, dot, seed, count, size = 256) {
  const canvas = document.createElement('canvas'); canvas.width = canvas.height = size;
  const ctx = canvas.getContext('2d');
  const random = () => { seed = (Math.imul(seed, 1664525) + 1013904223) >>> 0; return seed / 4294967296; };
  ctx.fillStyle = base; ctx.fillRect(0, 0, size, size);
  for (let i = 0; i < count; i++) {
    ctx.globalAlpha = .15 + random() * .4; ctx.fillStyle = dot;
    ctx.beginPath(); ctx.ellipse(random() * size, random() * size, .3 + random() * .8, .4 + random() * 1.4, 0, 0, Math.PI * 2); ctx.fill();
  }
  ctx.globalAlpha = 1;
  const texture = new THREE.CanvasTexture(canvas); texture.colorSpace = THREE.SRGBColorSpace; return texture;
}

function fruit(name) {
  const group = new THREE.Group(), apple = name === 'apple';
  const geometry = new THREE.SphereGeometry(1, 48, 32).rotateX(Math.PI / 2);
  const positions = geometry.attributes.position;
  for (let i = 0; i < positions.count; i++) {
    const x = positions.getX(i), y = positions.getY(i), z = positions.getZ(i);
    const theta = Math.atan2(y, x), radius = Math.hypot(x, y);
    const lobe = apple ? 1 + .055 * Math.cos(theta * 5) * (1 - z * z) : 1 + .012 * Math.sin(theta * 17 + z * 13);
    const pz = apple ? z * .023 - Math.sign(z) * .004 * Math.exp(-radius * radius / .065) : z * .021;
    positions.setXYZ(i, x * (apple ? .023 : .021) * lobe, y * (apple ? .023 : .018) * lobe, pz);
  }
  geometry.computeVertexNormals();
  const map = speckleTexture(apple ? '#b93223' : '#91b82e', apple ? '#f2bd69' : '#4c751b', apple ? 91 : 14, apple ? 1200 : 4000);
  mesh(group, geometry, new THREE.MeshPhysicalMaterial({ map, roughness: apple ? .38 : .63, clearcoat: apple ? .36 : .1 }));
  const leaf = mesh(group, new THREE.SphereGeometry(1, 16, 8).scale(.010, .004, .0007), new THREE.MeshStandardMaterial({ color: apple ? 0x48663b : 0x3f651e, roughness: .85 }));
  leaf.position.set(.007, .002, .025); leaf.rotation.y = -.35;
  return group;
}

function egg() {
  const group = new THREE.Group(), geometry = new THREE.SphereGeometry(1, 40, 28).rotateX(Math.PI / 2);
  const p = geometry.attributes.position;
  for (let i = 0; i < p.count; i++) { const z = p.getZ(i), scale = 1 - z * .17; p.setXYZ(i, p.getX(i) * .018 * scale, p.getY(i) * .021 * scale, z * .024); }
  geometry.computeVertexNormals();
  mesh(group, geometry, new THREE.MeshStandardMaterial({ map: speckleTexture('#e6caa0', '#9f7b51', 888, 1800), roughness: .75 }));
  return group;
}

function spool() {
  const group = new THREE.Group(), cap = new THREE.MeshStandardMaterial({ color: 0xe4d3a8, roughness: .6 });
  mesh(group, new THREE.CylinderGeometry(.012, .012, .034, 32).rotateX(Math.PI / 2), cap);
  for (const z of [-.0185, .0185]) {
    const flange = mesh(group, new THREE.CylinderGeometry(.022, .022, .003, 40).rotateX(Math.PI / 2), cap); flange.position.z = z;
    const ring = mesh(group, new THREE.TorusGeometry(.006, .0012, 8, 24), new THREE.MeshStandardMaterial({ color: 0xb59f74, roughness: .7 })); ring.position.z = z + Math.sign(z) * .0016;
  }
  const points = [];
  for (let i = 0; i <= 1000; i++) { const t = i / 1000, a = t * Math.PI * 2 * 27; points.push(new THREE.Vector3(.017 * Math.cos(a), .017 * Math.sin(a), -.016 + .032 * t)); }
  const path = new THREE.CatmullRomCurve3(points);
  mesh(group, new THREE.TubeGeometry(path, 650, .00065, 5, false), new THREE.MeshStandardMaterial({ color: 0x4c8196, roughness: .93 }));
  return group;
}

function cup() {
  const group = new THREE.Group();
  const material = new THREE.MeshPhysicalMaterial({ color: 0xa9c5b5, roughness: .23, clearcoat: .6, clearcoatRoughness: .17 });
  // Closed cross-section: exterior, rolled lip, interior and a solid bottom.
  const profile = [[0, -.042], [.027, -.042], [.031, -.040], [.033, -.034], [.036, .025], [.037, .038], [.0365, .042], [.033, .042], [.032, .038], [.031, -.032], [.028, -.035], [0, -.035]];
  mesh(group, new THREE.LatheGeometry(profile.map(([r, z]) => new THREE.Vector2(r, z)), 64).rotateX(Math.PI / 2), material);
  // Open C curve: both ends stop in the outer wall, never cross the cup cavity.
  const handlePath = new THREE.CubicBezierCurve3(new THREE.Vector3(0, -.036, .026),
    new THREE.Vector3(0, -.086, .047), new THREE.Vector3(0, -.086, -.047), new THREE.Vector3(0, -.036, -.026));
  const handle = mesh(group, new THREE.TubeGeometry(handlePath, 80, .0035, 16, false), material.clone());
  handle.name = 'cup_handle';
  const ring = mesh(group, new THREE.TorusGeometry(.0347, .0011, 10, 64), new THREE.MeshPhysicalMaterial({ color: 0xe2e8d7, roughness: .24, clearcoat: .65 })); ring.position.z = .041;
  const foot = mesh(group, new THREE.TorusGeometry(.0265, .0015, 8, 48), material.clone()); foot.position.z = -.040;
  return group;
}

function spoon() {
  const group = new THREE.Group(), steel = new THREE.MeshPhysicalMaterial({ color: 0xc6ccca, metalness: .96, roughness: .19, clearcoat: .24 });
  const shape = new THREE.Shape();
  shape.moveTo(-.036, 0); shape.bezierCurveTo(-.036, -.006, -.025, -.006, -.016, -.004);
  shape.bezierCurveTo(.013, -.003, .054, -.003, .078, -.005);
  shape.lineTo(.078, .005); shape.bezierCurveTo(.054, .003, .013, .003, -.016, .004);
  shape.bezierCurveTo(-.025, .006, -.036, .006, -.036, 0);
  const handle = new THREE.ExtrudeGeometry(shape, { depth: .0015, bevelEnabled: true, bevelSegments: 3, steps: 1, bevelSize: .00065, bevelThickness: .0006, curveSegments: 20 });
  handle.translate(0, 0, -.00075); mesh(group, handle, steel);
  // Two curved surfaces joined around the rim create a thin, concave bowl.
  const vertices = [], indices = [], rings = 12, segments = 48, layerSize = (rings + 1) * (segments + 1);
  for (let side = 0; side < 2; side++) for (let r = 0; r <= rings; r++) for (let s = 0; s <= segments; s++) {
    const t = r / rings, a = s / segments * Math.PI * 2;
    vertices.push(.094 + .020 * t * Math.cos(a), .013 * t * Math.sin(a), -.004 * (1 - t * t) + (side ? -.001 : 0));
  }
  for (let side = 0; side < 2; side++) for (let r = 0; r < rings; r++) for (let s = 0; s < segments; s++) {
    const a = side * layerSize + r * (segments + 1) + s, b = a + segments + 1;
    if (!side) indices.push(a, b, a + 1, a + 1, b, b + 1); else indices.push(a, a + 1, b, a + 1, b + 1, b);
  }
  for (let s = 0; s < segments; s++) {
    const a = rings * (segments + 1) + s, b = a + layerSize;
    indices.push(a, a + 1, b, a + 1, b + 1, b);
  }
  const bowl = new THREE.BufferGeometry(); bowl.setAttribute('position', new THREE.Float32BufferAttribute(vertices, 3)); bowl.setIndex(indices); bowl.computeVertexNormals();
  mesh(group, bowl, steel.clone());
  return group;
}

export function createTabletopProp(name) {
  if (name === 'apple_visual') return fruit('apple');
  if (name === 'lime_visual') return fruit('lime');
  if (name === 'egg_visual') return egg();
  if (name === 'spool_visual') return spool();
  if (name === 'cup_visual') return cup();
  if (name === 'spoon_visual') return spoon();
  return null;
}
