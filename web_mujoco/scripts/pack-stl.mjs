// Build-time lossless transport only: neither coordinates nor faces change.
const MAGIC='RBSTLP01';
export function packStl(source) {
  const bytes=Buffer.from(source.buffer,source.byteOffset,source.byteLength);
  if(bytes.length<84)return null;
  const count=bytes.readUInt32LE(80);
  if(!count||84+50*count!==bytes.length)return null;
  const vertices=new Map(),normals=new Map(),points=[],directions=[];
  const indices=Buffer.allocUnsafe(count*20),attributes=Buffer.allocUnsafe(count*2),previous=[0,0,0,0];
  let cursor=0;
  function index(map,list,offset) {
    // Compare the original float bits, including signed zero and NaN payloads.
    const key=bytes.toString('hex',offset,offset+12);let id=map.get(key);
    if(id===undefined){id=list.length;map.set(key,id);list.push(bytes.subarray(offset,offset+12));}
    return id;
  }
  function emit(value,slot) {
    const delta=value-previous[slot];previous[slot]=value;
    let encoded=((delta<<1)^(delta>>31))>>>0;
    while(encoded>127){indices[cursor++]=(encoded&127)|128;encoded>>>=7;}
    indices[cursor++]=encoded;
  }
  for(let face=0;face<count;face++) {
    const offset=84+face*50;emit(index(normals,directions,offset),0);
    for(let vertex=0;vertex<3;vertex++)emit(index(vertices,points,offset+12+vertex*12),vertex+1);
    bytes.copy(attributes,face*2,offset+48,offset+50);
  }
  const header=Buffer.alloc(100);header.write(MAGIC);header.writeUInt32LE(points.length,8);header.writeUInt32LE(directions.length,12);
  bytes.copy(header,16,0,84);
  return Buffer.concat([header,...points,...directions,indices.subarray(0,cursor),attributes]);
}
