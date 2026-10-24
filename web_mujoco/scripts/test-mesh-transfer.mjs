import assert from 'node:assert/strict';
import {readFile,readdir} from 'node:fs/promises';
import {gzipSync} from 'node:zlib';
import {packStl} from './pack-stl.mjs';
import {unpackStl} from '../src/stl-transfer.js';
const root=new URL('../../rebotarm_mujoco_rs/models/meshes/',import.meta.url);
let meshes=0,originalBytes=0,packedBytes=0,decodeMs=0;
for(const name of await readdir(root)) {
  if(!/\.stl$/i.test(name))continue;
  const bytes=await readFile(new URL(name,root)),packed=packStl(bytes);if(!packed)continue;
  const began=performance.now(),decoded=unpackStl(packed);decodeMs+=performance.now()-began;
  assert.equal(Buffer.compare(bytes,decoded),0,`${name} must retain every original byte`);
  originalBytes+=gzipSync(bytes,{level:9}).length;packedBytes+=gzipSync(packed,{level:9}).length;meshes++;
}
assert(meshes>=40);assert(packedBytes<originalBytes*.7,'The deployment payload must shrink by at least 30%');
assert.equal(packStl(Buffer.from('solid ascii mesh')),null);
assert.throws(()=>unpackStl(new Uint8Array(100)),/header/);
const tiny=Buffer.alloc(84+50);tiny.writeUInt32LE(1,80);tiny.writeUInt16LE(347,132);
const packed=packStl(tiny);assert.equal(Buffer.compare(tiny,unpackStl(packed)),0);
assert.throws(()=>unpackStl(packed.subarray(0,packed.length-1)),/indices|lengths|data/);
const invalid=Buffer.from(packed);invalid.writeUInt32LE(0xffffffff,96);assert.throws(()=>unpackStl(invalid),/lengths/);
const index=100+12*(packed.readUInt32LE(8)+packed.readUInt32LE(12));
invalid.set(packed);invalid[index]=126;assert.throws(()=>unpackStl(invalid),/bounds/);
invalid.set(packed);invalid.fill(255,index,index+4);assert.throws(()=>unpackStl(invalid),/indices|overflow/);
console.log(JSON.stringify({meshes,originalBytes,packedBytes,reduction:1-packedBytes/originalBytes,decodeMs}));
