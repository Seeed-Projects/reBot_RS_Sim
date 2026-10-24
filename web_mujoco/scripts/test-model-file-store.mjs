import assert from 'node:assert/strict';
import loadMujoco from '@mujoco/mujoco';
import {createModelFileStore} from '../src/model-file-store.js';

const mujoco=await loadMujoco(),fs=mujoco.FS;
const roots=()=>fs.readdir('/').filter(name=>name.startsWith('rebot-model-')).sort();
const before=roots(),stores=[],models=[];
try {
  for(const mass of [2,5]) {
    const store=createModelFileStore(mujoco);stores.push(store);
    assert.equal(store.statistics.backend,'memfs');
    store.add('scene.xml',new TextEncoder().encode('<mujoco><include file="parts/body.xml"/></mujoco>'));
    store.add('parts/body.xml',new TextEncoder().encode(`<mujoco><worldbody><body><freejoint/><geom type="sphere" size=".1" mass="${mass}"/></body></worldbody></mujoco>`));
    models.push(store.load('scene.xml'));
  }
  assert.equal(roots().length,before.length+2);
  assert.equal(models[0].body_mass[1],2);assert.equal(models[1].body_mass[1],5);
  for(const path of ['../x','/x','parts/../x','parts//x','x\0.xml'])assert.throws(()=>stores[0].add(path,new Uint8Array()),/path|directory/);
  stores[0].dispose();stores[0].dispose();assert.equal(roots().length,before.length+1);
  assert.equal(models[1].body_mass[1],5,'One load must not remove another load’s files');
  assert.throws(()=>stores[0].add('x',new Uint8Array()),/disposed/);
} finally {models.forEach(model=>model.delete());stores.forEach(store=>store.dispose());}
assert.deepEqual(roots(),before);

// A partially failed write must also be owned and cleaned up.
const failingFs={...fs,writeFile(path,bytes){fs.writeFile(path,bytes);throw new Error('Injected write failure');}};
const failing=createModelFileStore({FS:failingFs});
assert.throws(()=>failing.add('nested/partial.bin',new Uint8Array(64)),/Injected/);
failing.dispose();assert.deepEqual(roots(),before);

let destroyed=false;
const fake={MjVFS:class {addBuffer(name,bytes){assert.equal(name,'scene.xml');assert.equal(bytes.length,4);}delete(){destroyed=true;}},
  MjModel:{from_xml_path(name,vfs){assert(vfs);return {name};}}};
Object.defineProperty(fake,'FS',{get(){throw new Error('Unexported FS getter must not run');}});
const fallback=createModelFileStore(fake);assert.equal(fallback.statistics.backend,'vfs');
fallback.add('scene.xml',new Uint8Array(4));assert.equal(fallback.load('scene.xml').name,'scene.xml');fallback.dispose();assert(destroyed);
assert.throws(()=>createModelFileStore(fake,{backend:'memfs'}),/bulk file API/);
assert.throws(()=>createModelFileStore(fake,{backend:'unknown'}),RangeError);
console.log('Model files: native include loading, independent directories, failure cleanup, path containment and VFS fallback passed');
