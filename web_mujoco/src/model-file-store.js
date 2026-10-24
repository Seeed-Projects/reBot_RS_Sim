// The current MjVFS JS binding converts each byte through Embind. MEMFS uses
// typed-array bulk copies and mj_loadXML reads it through the same native parser.
let sequence=0;
export function createModelFileStore(mujoco,{backend='auto'}={}) {
  if(!['auto','memfs','vfs'].includes(backend))throw new RangeError('Unknown model file backend');
  // Unexported Emscripten runtime properties can have aborting getters.
  const fs=Object.getOwnPropertyDescriptor(mujoco,'FS')?.value;
  const supportsFs=fs&&['mkdir','writeFile','unlink','rmdir'].every(name=>typeof fs[name]==='function');
  if(backend==='memfs'&&!supportsFs)throw new Error('MuJoCo runtime has no bulk file API');
  const useFs=backend!=='vfs'&&supportsFs;
  const statistics={backend:useFs?'memfs':'vfs',writeMs:0,bytes:0};
  const vfs=useFs?null:new mujoco.MjVFS(),files=new Set(),directories=[];
  let base='',disposed=false;
  if(useFs) {
    for(;;) {
      base=`/rebot-model-${++sequence}`;
      try {fs.mkdir(base);directories.push(base);break;}
      catch(error) {if(!fs.analyzePath?.(base).exists)throw error;}
    }
  }
  function relativePath(value) {
    if(typeof value!=='string'||!value||value.includes('\0'))throw new Error('Invalid model resource path');
    const path=value.replaceAll('\\','/').replace(/^\.\//,'');
    if(path.startsWith('/')||path.split('/').some(part=>!part||part==='.'||part==='..'))throw new Error('Model resource escapes its load directory');
    return path;
  }
  return {
    statistics,
    add(name,bytes) {
      if(disposed)throw new Error('Model file store is disposed');
      const relative=relativePath(name),began=performance.now();
      if(useFs) {
        const parts=relative.split('/');let directory=base;
        for(const part of parts.slice(0,-1)) {
          directory+=`/${part}`;
          if(!directories.includes(directory)){fs.mkdir(directory);directories.push(directory);}
        }
        const path=`${base}/${relative}`;
        files.add(path);fs.writeFile(path,bytes);
      } else vfs.addBuffer(relative,bytes);
      statistics.bytes+=bytes.byteLength;statistics.writeMs+=performance.now()-began;
    },
    load(scene) {
      if(disposed)throw new Error('Model file store is disposed');
      const relative=relativePath(scene);
      return useFs?mujoco.MjModel.from_xml_path(`${base}/${relative}`):mujoco.MjModel.from_xml_path(relative,vfs);
    },
    dispose() {
      if(disposed)return;disposed=true;
      if(useFs) {
        // Only remove this load's own files and directories, in reverse order.
        for(const path of [...files].reverse()) {
          try {fs.unlink(path);}catch(error){if(error.errno!==44&&error.code!=='ENOENT')throw error;}
        }
        for(const path of directories.reverse())fs.rmdir(path);
      } else vfs.delete();
    }
  };
}
