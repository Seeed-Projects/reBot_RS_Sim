import {BufferGeometry,BufferAttribute} from 'three';

// Linear vertex clustering, used only for display meshes. Physics retains the
// original MuJoCo mesh. Attribute seams / sharp normal changes stay separate.
export function createDisplayLod(source,cell=.001) {
  const position=source.attributes.position,index=source.index;
  if(!index||index.count<9000)return null;
  const attributes=Object.entries(source.attributes),normal=source.attributes.normal;
  const groups=new Map(),remap=new Uint32Array(position.count),clusters=[];
  for(let vertex=0;vertex<position.count;vertex++) {
    const key=[0,1,2].map(axis=>Math.floor(position.array[vertex*3+axis]/cell));
    if(normal)for(let axis=0;axis<3;axis++)key.push(Math.round(normal.array[vertex*3+axis]*2));
    for(const name of ['uv','color']) {
      const attribute=source.attributes[name];
      if(attribute)for(let axis=0;axis<attribute.itemSize;axis++)key.push(Math.round(attribute.array[vertex*attribute.itemSize+axis]*4096));
    }
    const hash=key.join(',');let group=groups.get(hash);
    if(group===undefined) {group=clusters.length;groups.set(hash,group);clusters.push({count:0,sums:attributes.map(([,a])=>Array(a.itemSize).fill(0))});}
    remap[vertex]=group;const cluster=clusters[group];cluster.count++;
    attributes.forEach(([,attribute],a)=>{
      for(let axis=0;axis<attribute.itemSize;axis++)cluster.sums[a][axis]+=attribute.array[vertex*attribute.itemSize+axis];
    });
  }
  const faces=[],seen=new Set();
  for(let triangle=0;triangle<index.count;triangle+=3) {
    const a=remap[index.array[triangle]],b=remap[index.array[triangle+1]],c=remap[index.array[triangle+2]];
    if(a===b||b===c||c===a)continue;
    const key=[a,b,c].sort((x,y)=>x-y).join(',');if(seen.has(key))continue;seen.add(key);faces.push(a,b,c);
  }
  if(faces.length>index.count*.9)return null;
  const geometry=new BufferGeometry();
  attributes.forEach(([name,attribute],a)=>{
    const values=new Float32Array(clusters.length*attribute.itemSize);
    clusters.forEach((cluster,i)=>cluster.sums[a].forEach((value,axis)=>{values[i*attribute.itemSize+axis]=value/cluster.count;}));
    geometry.setAttribute(name,new BufferAttribute(values,attribute.itemSize));
  });
  geometry.setIndex(faces);geometry.normalizeNormals();geometry.computeBoundingSphere();geometry.computeBoundingBox();
  let maxDisplacement=0;
  for(let vertex=0;vertex<position.count;vertex++)maxDisplacement=Math.max(maxDisplacement,Math.hypot(...[0,1,2].map(axis=>
    position.array[vertex*3+axis]-geometry.attributes.position.array[remap[vertex]*3+axis])));
  return {geometry,maxDisplacement,originalTriangles:index.count/3,reducedTriangles:faces.length/3};
}
