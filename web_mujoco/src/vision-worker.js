import { detectTabletopObjects } from './rgbd-perception.js';
self.onmessage=({data:{id,frames,options}})=>{
  try {self.postMessage({id,observation:detectTabletopObjects(frames,options)});}
  catch(error){self.postMessage({id,error:error.message});}
};
