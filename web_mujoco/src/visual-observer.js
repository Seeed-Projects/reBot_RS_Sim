import { detectTabletopObjects } from './rgbd-perception.js';

// Image processing lives off the UI thread. Calibration and RGB-D pixels are
// the entire worker message; simulation/model handles never cross this API.
export function createVisualObserver(view,data,{worker=true}={}) {
  const processor=worker?new Worker(new URL('./vision-worker.js',import.meta.url),{type:'module'}):null;
  let sequence=0,disposed=false;
  const pending=new Map();
  if(processor) processor.onmessage=({data:result})=>{
    const item=pending.get(result.id);if(!item)return;pending.delete(result.id);clearTimeout(item.timer);
    if(result.error)item.reject(new Error(result.error));else item.resolve(result.observation);
  };
  if(processor) processor.onerror=event=>{
    event.preventDefault();
    for(const item of pending.values()){clearTimeout(item.timer);item.reject(new Error(event.message||'Vision worker failed'));}pending.clear();
  };
  return {
    async observe(options={}) {
      if(disposed)throw new Error('Vision observer disposed');
      view.sync(data);
      const began=performance.now(),frames=[await view.captureCamera('overhead',{zoom:3,asyncReadback:true}),
        await view.captureWorkspaceRgbd({asyncReadback:true})];
      if(disposed)throw new Error('Vision observer disposed');
      let observation;
      if(!processor) observation=detectTabletopObjects(frames,options);
      else observation=await new Promise((resolve,reject)=>{
        const id=++sequence,timer=setTimeout(()=>{pending.delete(id);reject(new Error('Vision worker timeout'));},10000);
        pending.set(id,{resolve,reject,timer});
        processor.postMessage({id,frames,options},frames.flatMap(f=>[f.rgb.buffer,f.depth.buffer]));
      });
      observation.diagnostics.totalMs=performance.now()-began;
      return observation;
    },
    dispose() {disposed=true;processor?.terminate();for(const item of pending.values()){clearTimeout(item.timer);item.reject(new Error('Vision observer disposed'));}pending.clear();}
  };
}
