import assert from 'node:assert/strict';
import {readFile,mkdir,writeFile} from 'node:fs/promises';
import {gzipSync} from 'node:zlib';
import loadMujoco from '@mujoco/mujoco';

const began=performance.now(),mujoco=await loadMujoco();
const runtimeMs=performance.now()-began;
const tiny=process.argv.includes('--tiny');
const scene=process.argv.slice(2).find(value=>value.endsWith('.xml'))||'rs_dual_tabletop_scene.xml';
const root=new URL('../../rebotarm_mujoco_rs/models/',import.meta.url),vfs=new mujoco.MjVFS();
const files=new Set(),xmls=[scene];
for(const relative of tiny?[]:xmls) {
  if(files.has(relative))continue;
  const bytes=await readFile(new URL(relative,root));vfs.addBuffer(relative,bytes);files.add(relative);
  const xml=bytes.toString('utf8');
  for(const [,file] of xml.matchAll(/<include\s+file="([^"]+)"/g))xmls.push(file);
  for(const [,file] of xml.matchAll(/\sfile="([^"]+\.(?:STL|stl|obj|msh))"/g)) {
    const path=file.startsWith('meshes/')?file:`meshes/${file}`;
    if(!files.has(path)){vfs.addBuffer(path,await readFile(new URL(path,root)));files.add(path);}
  }
}
const at=performance.now(),model=tiny?mujoco.MjModel.from_xml_string('<mujoco><worldbody><body><freejoint/><geom type="sphere" size=".1"/></body></worldbody></mujoco>'):mujoco.MjModel.from_xml_path(scene,vfs),compileMs=performance.now()-at;
const length=Number(mujoco.mj_sizeModel(model)),buffer=new mujoco.Uint8Buffer(length);
const expected={nmesh:model.nmesh,ngeom:model.ngeom,nq:model.nq,masses:Array.from(model.body_mass)};
console.log(JSON.stringify({stage:'compiled',runtimeMs,compileMs,binaryBytes:length,nbuffer:String(model.nbuffer)}));
let restored,originalFreed=false,bufferFreed=false;
try {
  mujoco.mj_saveModel(model,null,buffer);
  const bytes=Uint8Array.from(buffer.GetView()),packed=gzipSync(bytes);
  console.log(JSON.stringify({stage:'serialized',bytes:length,gzipBytes:packed.length,header:[...bytes.slice(0,24)]}));
  assert.equal(bytes.length,length);
  model.delete();originalFreed=true;buffer.delete();bufferFreed=true;
  vfs.addBuffer('probe.mjb',bytes);
  const at=performance.now();restored=mujoco.MjModel.from_binary_path('probe.mjb',vfs);
  const report={scene,runtimeMs,compileMs,binaryLoadMs:performance.now()-at,binaryBytes:length,gzipBytes:packed.length,
    ...expected,masses:undefined,wasmBytes:restored.qpos0.buffer.byteLength};
  assert.equal(restored.ngeom,expected.ngeom);assert.deepEqual(Array.from(restored.body_mass),expected.masses);
  await mkdir('.reports',{recursive:true});await writeFile('.reports/startup-diagnosis.json',JSON.stringify(report,null,2));
  console.log(JSON.stringify(report));
} finally {restored?.delete();if(!bufferFreed)buffer.delete();if(!originalFreed)model.delete();vfs.delete();}
