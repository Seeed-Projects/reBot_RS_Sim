// GPU time measures completed main-view draws. CPU submission time, pixel
// readback latency and WASM memory are reported separately by the caller.
export function createRenderProfiler(renderer) {
  const gl=renderer.getContext(),debug=gl.getExtension('WEBGL_debug_renderer_info');
  const timer=gl.getExtension('EXT_disjoint_timer_query_webgl2'),pending=[];
  const graphics={vendor:debug?gl.getParameter(debug.UNMASKED_VENDOR_WEBGL):gl.getParameter(gl.VENDOR),
    renderer:debug?gl.getParameter(debug.UNMASKED_RENDERER_WEBGL):gl.getParameter(gl.RENDERER),
    version:gl.getParameter(gl.VERSION),gpuTimerSupported:Boolean(timer)};
  let active=null,samples=0,total=0,max=0,last=null;
  function poll() {
    const disjoint=timer&&gl.getParameter(timer.GPU_DISJOINT_EXT);
    for(let i=pending.length-1;i>=0;i--) {
      const query=pending[i];
      if(!disjoint&&!gl.getQueryParameter(query,gl.QUERY_RESULT_AVAILABLE))continue;
      if(!disjoint) {
        last=gl.getQueryParameter(query,gl.QUERY_RESULT)/1e6;
        samples++;total+=last;max=Math.max(max,last);
      }
      gl.deleteQuery(query);pending.splice(i,1);
    }
  }
  return {
    begin() {poll();if(timer&&pending.length<4) {active=gl.createQuery();gl.beginQuery(timer.TIME_ELAPSED_EXT,active);}},
    end() {if(active) {gl.endQuery(timer.TIME_ELAPSED_EXT);pending.push(active);active=null;}},
    state() {poll();return {graphics,gpu:{samples,meanMs:samples?total/samples:null,maxMs:samples?max:null,lastMs:last}};},
    dispose() {if(active) {gl.endQuery(timer.TIME_ELAPSED_EXT);gl.deleteQuery(active);}pending.forEach(query=>gl.deleteQuery(query));pending.length=0;}
  };
}

export function sceneGeometryMetrics(scene,retainedGeometries=[]) {
  const unique=new Set(),byBody=new Map();let bytes=0,triangles=0;
  scene.traverse(object=>{
    const g=object.geometry;if(!g||!object.isMesh)return;
    const count=(g.index?.count??g.attributes.position?.count??0)/3;
    triangles+=count;
    const body=object.userData.bodyName||'room / props';byBody.set(body,(byBody.get(body)||0)+count);
    if(unique.has(g))return;unique.add(g);
    for(const attribute of Object.values(g.attributes))bytes+=attribute.array.byteLength;
    bytes+=g.index?.array.byteLength||0;
  });
  let retainedBytes=bytes;
  for(const g of retainedGeometries) {
    if(!g||unique.has(g))continue;unique.add(g);
    for(const attribute of Object.values(g.attributes))retainedBytes+=attribute.array.byteLength;
    retainedBytes+=g.index?.array.byteLength||0;
  }
  return {triangles,geometryArrayBytes:bytes,retainedGeometryArrayBytes:retainedBytes,uniqueGeometries:unique.size,
    largestBodies:[...byBody].sort((a,b)=>b[1]-a[1]).slice(0,12).map(([body,triangles])=>({body,triangles}))};
}
