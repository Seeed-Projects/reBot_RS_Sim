import * as THREE from 'three';
import { mergeGeometries } from 'three/addons/utils/BufferGeometryUtils.js';

// Local coordinates match the original solid collision envelopes. Details are
// batched by finish, so shelves, ribs and packaging do not add individual draws.
export function createLabProp(geomName, halfSize) {
  if (geomName !== 'storage_rack' && geomName !== 'dynamic_pallet_geom') return null;

  const group = new THREE.Group();
  group.name = `lab-detail-${geomName}`;
  const batches = new Map();

  function box(finish, color, position, size, rotation = 0) {
    const geometry = new THREE.BoxGeometry(...size).toNonIndexed();
    geometry.rotateY(rotation);
    geometry.translate(...position);
    const rgb = new THREE.Color(color);
    const colors = new Float32Array(geometry.attributes.position.count * 3);
    for (let i = 0; i < colors.length; i += 3) rgb.toArray(colors, i);
    geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
    if (!batches.has(finish)) batches.set(finish, []);
    batches.get(finish).push(geometry);
  }

  function crate(x, y, bottom, width, depth, height, color) {
    const centerZ = bottom + height / 2;
    // The lid occupies the top 16 mm. A full-height body would put two
    // differently coloured faces on the same plane and shimmer as we orbit.
    const bodyHeight = height - 0.016;
    box('matte', color, [x, y, bottom + bodyHeight / 2], [width, depth, bodyHeight]);
    box('matte', 0x263d4a, [x, y, bottom + height - 0.008], [width + 0.012, depth + 0.012, 0.016]);
    // Moulded corner ribs, lid lip, recessed handle and a small inventory label.
    for (const side of [-1, 1]) {
      for (const offset of [-0.33, 0.33]) {
        box('matte', color, [x + offset * width, y + side * (depth / 2 + 0.004), centerZ], [0.012, 0.012, height * 0.83]);
      }
    }
    box('matte', 0x172a35, [x, y - depth / 2 - 0.004, bottom + height * 0.73], [width * 0.28, 0.008, height * 0.10]);
    box('matte', 0xe6ecea, [x, y - depth / 2 - 0.009, bottom + height * 0.36], [width * 0.30, 0.003, height * 0.23]);
    for (let i = 0; i < 6; i += 1) {
      box('matte', 0x324852, [x + (i - 2.5) * width * 0.031, y - depth / 2 - 0.011, bottom + height * 0.36], [width * (i % 2 ? 0.01 : 0.018), 0.002, height * 0.15]);
    }
  }

  if (geomName === 'storage_rack') {
    // 1.20 × 1.10 × 1.10 m shelving, open toward the laboratory aisle (-Y).
    for (const x of [-0.565, 0.565]) {
      for (const y of [-0.515, 0.515]) {
        box('metal', 0x54718c, [x, y, 0], [0.055, 0.055, 1.10]);
        box('matte', 0x203340, [x, y, -0.532], [0.070, 0.070, 0.036]);
      }
    }
    for (const z of [-0.44, -0.02, 0.40]) {
      box('metal', 0x9baeba, [0, 0, z], [1.08, 1.01, 0.026]);
      for (const y of [-0.514, 0.514]) {
        box('metal', 0xe2a653, [0, y, z - 0.025], [1.10, 0.024, 0.064]);
      }
      for (const x of [-0.55, 0.55]) {
        box('metal', 0x54718c, [x, 0, z - 0.025], [0.024, 1.03, 0.064]);
      }
    }
    // Rear cross brace and low kick plates make the reserved footprint legible.
    const braceLength = Math.hypot(1.05, 0.88);
    for (const direction of [-1, 1]) {
      box('metal', 0x698296, [0, 0.529, 0], [0.022, 0.018, braceLength], direction * Math.atan2(1.05, 0.88));
    }
    for (const y of [-0.53, 0.53]) {
      box('metal', 0x3a5367, [0, y, -0.493], [1.10, 0.025, 0.070]);
    }
    crate(-0.285, -0.13, -0.427, 0.43, 0.66, 0.29, 0x467c92);
    crate(0.285, -0.13, -0.427, 0.43, 0.66, 0.29, 0x467c92);
    crate(-0.285, -0.10, -0.007, 0.43, 0.66, 0.28, 0x819491);
    crate(0.285, -0.10, -0.007, 0.43, 0.66, 0.28, 0x819491);
    // Compact equipment cases on the top shelf, below the original rack height.
    for (const x of [-0.30, 0.08, 0.37]) {
      const width = x < 0 ? 0.32 : 0.20;
      box('matte', 0xc7d1ce, [x, 0.03, 0.474], [width, 0.60, 0.122]);
      box('matte', 0x314c60, [x, -0.273, 0.474], [width * 0.44, 0.006, 0.025]);
    }
    group.scale.set(halfSize[0] / 0.60, halfSize[1] / 0.55, halfSize[2] / 0.55);
  } else {
    // 0.70 × 0.45 × 0.40 m loaded pallet; the whole assembly follows its free body.
    for (const x of [-0.275, 0, 0.275]) {
      box('wood', 0x8f6540, [x, 0, -0.171], [0.10, 0.45, 0.058]);
    }
    for (let i = 0; i < 5; i += 1) {
      box('wood', i % 2 ? 0xc39761 : 0xb78955, [0, (i - 2) * 0.089, -0.128], [0.70, 0.075, 0.028]);
    }
    crate(0, 0.005, -0.114, 0.60, 0.385, 0.148, 0x54848a);
    crate(0, 0.005, 0.034, 0.60, 0.385, 0.158, 0x75999a);
    // Two packing straps run over the lids and down both ends.
    for (const x of [-0.19, 0.19]) {
      box('matte', 0xd7b76a, [x, 0.005, 0.197], [0.017, 0.40, 0.006]);
      for (const y of [-0.195, 0.205]) {
        box('matte', 0xd7b76a, [x, y, 0.040], [0.017, 0.005, 0.31]);
      }
    }
    group.scale.set(halfSize[0] / 0.35, halfSize[1] / 0.225, halfSize[2] / 0.20);
  }

  for (const [finish, geometries] of batches) {
    const geometry = mergeGeometries(geometries);
    geometries.forEach((part) => part.dispose());
    const mesh = new THREE.Mesh(geometry, new THREE.MeshStandardMaterial({
      vertexColors: true,
      metalness: finish === 'metal' ? 0.55 : 0.02,
      roughness: finish === 'metal' ? 0.46 : finish === 'wood' ? 0.94 : 0.76
    }));
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    group.add(mesh);
  }
  // Keep scaling on children: the parent matrix is replaced by the live geom pose.
  for (const mesh of group.children) mesh.scale.copy(group.scale);
  group.scale.set(1, 1, 1);
  return group;
}

export function disposeLabProp(group) {
  group.traverse((object) => {
    object.geometry?.dispose();
    object.material?.map?.dispose();
    object.material?.dispose();
  });
}
