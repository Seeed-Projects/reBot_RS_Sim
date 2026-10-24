const MAGIC='RBSTLP01',MAX_STL_BYTES=256*1024*1024;

// Reconstruct the exact original binary STL before MuJoCo sees it. The build
// stores repeated vertices/normals once and delta-encodes only their indices.
export function unpackStl(bytes) {
  if(bytes.length<100||[...MAGIC].some((char,index)=>bytes[index]!==char.charCodeAt(0)))throw new Error('Invalid packed STL header');
  const view=new DataView(bytes.buffer,bytes.byteOffset,bytes.byteLength);
  const vertices=view.getUint32(8,true),normals=view.getUint32(12,true),faces=view.getUint32(96,true);
  const size=84+50*faces,tableEnd=100+12*(vertices+normals),indexEnd=bytes.length-2*faces;
  if(!faces||!vertices||!normals||vertices>3*faces||normals>faces||size>MAX_STL_BYTES||tableEnd+4*faces>indexEnd)throw new Error('Invalid packed STL lengths');
  const result=new Uint8Array(size),previous=[0,0,0,0];result.set(bytes.subarray(16,100));let cursor=tableEnd;
  function next(slot) {
    let encoded=0;
    for(let part=0;part<5;part++) {
      if(cursor>=indexEnd)throw new Error('Truncated packed STL indices');
      const byte=bytes[cursor++];
      if(part===4&&byte>15)throw new Error('Packed STL index overflow');
      encoded|=(byte&127)<<(7*part);
      if(!(byte&128)) {
        const index=previous[slot]+((encoded>>>1)^-(encoded&1));
        if(index<0||index>=(slot?vertices:normals))throw new Error('Packed STL index out of bounds');
        previous[slot]=index;return index;
      }
    }
    throw new Error('Invalid packed STL index');
  }
  for(let face=0;face<faces;face++)for(let slot=0;slot<4;slot++) {
    const index=next(slot),offset=100+(slot?0:12*vertices)+12*index;
    result.set(bytes.subarray(offset,offset+12),84+face*50+12*slot);
  }
  if(cursor!==indexEnd)throw new Error('Unexpected packed STL data');
  for(let face=0;face<faces;face++) {
    result[84+face*50+48]=bytes[cursor++];result[84+face*50+49]=bytes[cursor++];
  }
  return result;
}
