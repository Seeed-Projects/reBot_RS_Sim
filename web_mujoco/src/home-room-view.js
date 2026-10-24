import * as THREE from 'three';
import { mergeGeometries } from 'three/addons/utils/BufferGeometryUtils.js';
import { RoundedBoxGeometry } from 'three/addons/geometries/RoundedBoxGeometry.js';

// Room furniture is built in world metres and batched by finish. Adding a
// complete room does not introduce hundreds of separate shadow draw calls.
export function createHomeRoom(woodTexture,{detail='balanced'}={}) {
  const full=detail==='full';
  const group = new THREE.Group();
  const batches = new Map();
  function add(finish, color, geometry, position, rotation = null) {
    let g = geometry.index ? geometry.toNonIndexed() : geometry;
    if (g !== geometry) geometry.dispose();
    if (!g.attributes.uv) g.setAttribute('uv', new THREE.Float32BufferAttribute(new Float32Array(g.attributes.position.count * 2), 2));
    if (rotation) g.applyQuaternion(new THREE.Quaternion().setFromEuler(new THREE.Euler(...rotation)));
    g.translate(...position);
    const rgb = new THREE.Color(color), values = new Float32Array(g.attributes.position.count * 3);
    for (let i = 0; i < values.length; i += 3) rgb.toArray(values, i);
    g.setAttribute('color', new THREE.BufferAttribute(values, 3));
    if (!batches.has(finish)) batches.set(finish, []);
    batches.get(finish).push(g);
  }
  const box = (finish, color, p, size, radius = 0) => add(finish, color,
    radius ? new RoundedBoxGeometry(...size, full?2:1, radius) : new THREE.BoxGeometry(...size), p);
  const cylinder = (finish, color, p, radius, height, top = radius) => add(finish, color,
    new THREE.CylinderGeometry(top, radius, height, full?24:16).rotateX(Math.PI / 2), p);
  const sphere = (finish, color, p, size) => add(finish, color,
    new THREE.SphereGeometry(1, full?16:12, full?10:8).scale(...size), p);
  function rod(finish, color, a, b, radius) {
    const start = new THREE.Vector3(...a), end = new THREE.Vector3(...b), delta = end.clone().sub(start);
    const g = new THREE.CylinderGeometry(radius * .84, radius, delta.length(), full?10:8);
    g.applyQuaternion(new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 1, 0), delta.normalize()));
    add(finish, color, g, start.add(end).multiplyScalar(.5).toArray());
  }
  function torus(finish, color, p, radius, tube, rotation = [0, 0, 0]) {
    add(finish, color, new THREE.TorusGeometry(radius, tube, full?8:6, full?32:24), p, rotation);
  }
  function plant(x, y, z, scale = 1) {
    // An open vessel avoids a solid top cap overlapping the soil at z + .09.
    const profile = [[0, 0], [.043, 0], [.045, .004], [.052, .087],
      [.052, .090], [.047, .090], [.046, .083], [.040, .008], [0, .008]];
    const pot = new THREE.LatheGeometry(profile.map(([r, h]) =>
      new THREE.Vector2(r * scale, h * scale)), full?40:24).rotateX(Math.PI / 2);
    add('ceramic', 0xd1c3a4, pot, [x, y, z]);
    cylinder('matte', 0x403426, [x, y, z + .081 * scale], .0455 * scale, .004 * scale);
    for (let i = 0; i < 9; i++) {
      const a = i * 2.4;
      const end = [x + Math.cos(a) * .05 * scale, y + Math.sin(a) * .05 * scale, z + (.17 + i % 3 * .025) * scale];
      rod('matte', 0x4f6541, [x, y, z + .09 * scale], end, .002 * scale);
      const geometry = new THREE.SphereGeometry(1, full?12:10, full?8:6).scale(.024 * scale, .012 * scale, .052 * scale);
      add('leaf', i % 2 ? 0x6c8953 : 0x3e633e, geometry, end, [.35 * Math.cos(a), .35 * Math.sin(a), a]);
    }
  }
  function jar(x, y, z, color, height = .13) {
    cylinder('ceramic', color, [x, y, z + height / 2], .045, height);
    cylinder('wood', 0xc3a16f, [x, y, z + height + .008], .048, .016);
    box('matte', 0xe7ddbf, [x + .045, y, z + height * .52], [.003, .052, .045]);
    for (let i = 0; i < 3; i++) box('matte', 0x7a775f, [x + .047, y, z + height * .52 + (i - 1) * .008], [.002, .030 - i * .004, .002]);
  }
  // Sideboard: feet, carcass, inset panel doors, drawer seams and brass pulls.
  for (const x of [-.56, -.22]) for (const y of [.99, 1.97]) cylinder('metal', 0x585d4f, [x, y, .065], .018, .13);
  box('wood', 0x977650, [-.39, 1.48, .43], [.46, 1.16, .64], .008);
  box('wood', 0xe4cda6, [-.38, 1.48, .774], [.50, 1.21, .048], .009);
  for (const y of [1.105, 1.48, 1.855]) {
    box('matte', 0x657a68, [-.151, y, .40], [.025, .355, .50], .006);
    box('matte', 0x758977, [-.133, y, .40], [.012, .285, .425], .004);
    box('wood', 0xb19368, [-.148, y, .690], [.025, .355, .075], .003);
    rod('metal', 0xc6aa73, [-.112, y - .045, .69], [-.112, y + .045, .69], .004);
    cylinder('metal', 0xc6aa73, [-.113, y + .10, .43], .008, .009);
  }
  // Countertop: kettle with spout and handle, cutting boards and labelled jars.
  cylinder('metal', 0xc5c5b3, [-.38, 1.12, .87], .067, .13, .056);
  cylinder('metal', 0x4c5349, [-.38, 1.12, .941], .058, .010);
  sphere('metal', 0x4b5145, [-.38, 1.12, .96], [.013, .013, .015]);
  rod('metal', 0xa3ada0, [-.35, 1.075, .90], [-.285, 1.03, .935], .013);
  torus('matte', 0x39483c, [-.40, 1.16, .91], .067, .010, [Math.PI / 2, 0, 0]);
  box('wood', 0xc5a371, [-.57, 1.52, .95], [.025, .20, .29], .018);
  box('wood', 0xe0c49a, [-.54, 1.70, .93], [.025, .18, .25], .025);
  jar(-.35, 1.48, .798, 0xe6d8b9);
  jar(-.34, 1.69, .798, 0x8f9a7c, .15);
  jar(-.35, 1.89, .798, 0x9f7354, .11);
  // Floating shelf with brackets, stacked plates and trailing greenery.
  box('wood', 0xcbb083, [-.52, 1.48, 1.49], [.27, 1.18, .035], .004);
  for (const y of [1.02, 1.94]) {
    box('metal', 0x586653, [-.65, y, 1.41], [.02, .02, .18]);
    rod('metal', 0x586653, [-.65, y, 1.33], [-.40, y, 1.47], .006);
  }
  for (let i = 0; i < 4; i++) {
    cylinder('ceramic', 0xe4dfc9, [-.48, 1.12, 1.518 + i * .012], .084, .012);
    torus('ceramic', 0x7f927e, [-.48, 1.12, 1.525 + i * .012], .074, .0017);
  }
  plant(-.51, 1.82, 1.509, .9);
  for (let i = 0; i < 8; i++) {
    const z = 1.5 - i * .037, y = 1.85 + Math.sin(i * .8) * .035;
    rod('leaf', 0x506f42, [-.43, y, z], [-.43, y, z - .04], .002);
    sphere('leaf', 0x688a50, [-.42, y + (i % 2 ? -.025 : .025), z], [.012, .026, .012]);
  }
  cylinder('ceramic', 0x967258, [-.51, 1.46, 1.59], .046, .16, .025);
  for (let i = 0; i < 4; i++) {
    const y = 1.46 + (i - 1.5) * .02;
    rod('wood', 0x9c8b61, [-.51, 1.46, 1.65], [-.48 + i * .008, y, 1.92 - i * .018], .0015);
    sphere('matte', 0xd4c6a8, [-.48 + i * .008, y, 1.92 - i * .018], [.018, .012, .035]);
  }
  // Low bookcase and a mix of books, storage baskets and ceramics.
  for (const y of [-2.04, -1.26]) box('wood', 0xb69a72, [-.47, y, .66], [.34, .025, 1.22]);
  box('wood', 0xa38860, [-.635, -1.65, .66], [.015, .80, 1.22]);
  for (const z of [.10, .47, .84, 1.24]) box('wood', 0xcebb94, [-.47, -1.65, z], [.36, .82, .027]);
  const bookColors = [0x647663, 0xc09c68, 0xb46f52, 0xdbceb0, 0x526b70];
  for (let i = 0; i < 12; i++) {
    const y = -1.99 + i * .045, height = .21 + (i % 4) * .022;
    box('matte', bookColors[i % bookColors.length], [-.44, y, .485 + height / 2], [.22, .036, height], .001);
    for (const z of [.53, .64]) box('matte', 0xe1d5b9, [-.328, y, z], [.003, .025, .003]);
  }
  for (let i = 0; i < 3; i++) box('matte', bookColors[i], [-.45, -1.73, .875 + i * .026], [.22, .30, .022], .002);
  jar(-.46, -1.39, .855, 0xb2bca2, .19);
  for (const y of [-1.83, -1.45]) {
    box('matte', 0xa8926a, [-.45, y, .26], [.26, .30, .26], .008);
    box('matte', 0x665e46, [-.315, y, .28], [.006, .095, .027], .007);
    for (let z = .16; z < .39; z += .025) box('matte', 0xc3b28d, [-.317, y, z], [.006, .29, .003]);
  }
  plant(-.47, -1.86, 1.26, .85);
  cylinder('ceramic', 0x8f9f86, [-.46, -1.40, 1.34], .055, .16, .031);
  // Keep the folds in front of the sill's x=-.50 edge and the right panel
  // clear of the shelf/sideboard, which start at y=.89/.90 respectively.
  const curtainX = -.46, curtainWidth = .18;
  rod('metal', 0x807b64, [curtainX, -.85, 2.00], [curtainX, .98, 2.00], .012);
  for (const y of [-.80, .93]) {
    rod('metal', 0x807b64, [-.68, y, 2.00], [curtainX, y, 2.00], .006);
  }
  for (const y of [-.66, .78]) {
    const points = [], indices = [], cols = 32;
    for (let row = 0; row <= 12; row++) for (let col = 0; col <= cols; col++) {
      const u = col / cols, z = .13 + row / 12 * 1.84;
      points.push(curtainX + Math.sin(u * Math.PI * 12) * .018, y + (u - .5) * curtainWidth, z);
    }
    for (let row = 0; row < 12; row++) for (let col = 0; col < cols; col++) {
      const a = row * (cols + 1) + col, b = a + cols + 1;
      indices.push(a, b, a + 1, a + 1, b, b + 1);
    }
    const g = new THREE.BufferGeometry(); g.setAttribute('position', new THREE.Float32BufferAttribute(points, 3)); g.setIndex(indices); g.computeVertexNormals();
    add('fabric', 0xe7e0c9, g, [0, 0, 0]);
    for (let i = 0; i < 7; i++) torus('metal', 0xa79f84, [curtainX, y + (i - 3) * .026, 1.985], .018, .002, [Math.PI / 2, 0, 0]);
  }
  plant(-.55, -.30, .967, .62);
  plant(-.55, .55, .967, .78);
  // Two dining chairs with rounded upholstered seats and slatted backs.
  for (const side of [-1, 1]) {
    const y = side * 1.13, back = y + side * .19;
    for (const x of [.48, .82]) for (const offset of [-.15, .15]) rod('wood', 0xab8a5d, [x + (x < .6 ? -.025 : .025), y + offset * 1.16, .02], [x, y + offset, .46], .018);
    box('wood', 0xbaa077, [.65, y, .45], [.43, .40, .04], .016);
    box('fabric', side < 0 ? 0xb28c65 : 0x8c9e83, [.65, y, .487], [.405, .38, .055], .025);
    for (const x of [.48, .82]) rod('wood', 0xaf926a, [x, back, .44], [x, back + side * .045, .95], .016);
    box('wood', 0xbfa67d, [.65, back + side * .04, .925], [.39, .035, .09], .014);
    for (const x of [.55, .65, .75]) rod('wood', 0xbea780, [x, back, .53], [x, back + side * .035, .88], .010);
  }
  // Floor lamp, framed prints and rug fringes complete the surrounding space.
  cylinder('metal', 0x5e6658, [.85, 1.80, .025], .12, .05);
  rod('metal', 0x747665, [.85, 1.80, .04], [.85, 1.80, 1.55], .009);
  cylinder('fabric', 0xe9d9b6, [.85, 1.80, 1.53], .19, .24, .11);
  cylinder('ceramic', 0xf7e8c5, [.85, 1.80, 1.405], .165, .004);
  const light = new THREE.PointLight(0xffd29a, .35, 2); light.position.set(.85, 1.80, 1.43); group.add(light);
  for (const x of [.55, 1.25]) {
    box('wood', 0xa88b60, [x, 2.193, 1.60], [.48, .025, .65], .003);
    box('matte', 0xe7dfc9, [x, 2.175, 1.60], [.43, .006, .60]);
    sphere('matte', 0xa9b39a, [x - .05, 2.169, 1.64], [.13, .004, .19]);
    sphere('matte', 0xb97953, [x + .08, 2.163, 1.48], [.09, .004, .10]);
    rod('matte', 0x697962, [x - .12, 2.157, 1.38], [x + .09, 2.157, 1.83], .003);
  }
  for (const x of [-.637, 1.297]) for (let y = -1.02; y <= 1.02; y += .021) rod('fabric', 0xd4c9ad, [x - .022, y, .007], [x + .022, y, .007], .0015);

  for (const [finish, geometries] of batches) {
    const material = new THREE.MeshStandardMaterial({ vertexColors: true,
      roughness: finish === 'metal' ? .34 : finish === 'ceramic' ? .26 : .88,
      metalness: finish === 'metal' ? .65 : .02,
      side: finish === 'fabric' ? THREE.DoubleSide : THREE.FrontSide,
      map: finish === 'wood' ? woodTexture : null });
    const geometry = mergeGeometries(geometries);
    geometries.forEach((g) => g.dispose());
    const mesh = new THREE.Mesh(geometry, material); mesh.castShadow = mesh.receiveShadow = true; group.add(mesh);
  }
  return group;
}
