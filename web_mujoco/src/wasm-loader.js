// Bound connection and stalled body waits, while allowing slow connections
// that continue to receive bytes to finish without a second WASM download.
export async function compileRemoteWasm(url, timeoutMs = 8000) {
  const controller = new AbortController();
  let timeout,finished=false;
  const progress=()=>{if(finished)return;clearTimeout(timeout);timeout=setTimeout(()=>controller.abort(),timeoutMs);};
  progress();
  try {
    const response = await fetch(url, { mode: 'cors', signal: controller.signal });
    if (!response.ok) throw new Error(`WASM returned HTTP ${response.status}`);
    progress();
    const monitored=response.body?new Response(response.body.pipeThrough(new TransformStream({
      transform(chunk,output){progress();output.enqueue(chunk);}
    })),{headers:response.headers}):response;
    if (typeof WebAssembly.compileStreaming === 'function' &&
        response.headers.get('content-type')?.split(';')[0].trim() === 'application/wasm') {
      return await WebAssembly.compileStreaming(monitored);
    }
    return await WebAssembly.compile(await monitored.arrayBuffer());
  } finally {
    finished=true;controller.abort();clearTimeout(timeout);
  }
}

// Chrome restricts synchronous instantiation of large modules on the main
// thread. Emscripten supports an asynchronous success callback; reject the
// outer initialization as well if instantiation fails before that callback.
export async function initializeCompiledWasm(loadRuntime, module) {
  let rejectInstantiation;
  const failed=new Promise((_,reject)=>{rejectInstantiation=reject;});
  const ready=loadRuntime({
    instantiateWasm(imports,receiveInstance) {
      void WebAssembly.instantiate(module,imports)
        .then(instance=>receiveInstance(instance,module))
        .catch(rejectInstantiation);
      return {};
    }
  });
  return Promise.race([ready,failed]);
}
