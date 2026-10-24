import assert from 'node:assert/strict';
import {createServer} from 'vite';
import {chromium} from 'playwright';

const server=await createServer({server:{host:'127.0.0.1',port:5198,strictPort:true},plugins:[{
  name:'runtime-test-page',configureServer(vite){
    vite.middlewares.use('/runtime-test.html',(_request,response)=>{response.setHeader('Content-Type','text/html');response.end('<!doctype html><title>Runtime delivery test</title>');});
  }
}]});
await server.listen();
const browser=await chromium.launch({channel:process.env.PLAYWRIGHT_CHANNEL||(process.platform==='win32'?'chrome':undefined),headless:true});
try {
  const page=await browser.newPage(),requests=[],errors=[];
  page.on('request',request=>{if(request.url().endsWith('.wasm'))requests.push(request.url());});
  page.on('pageerror',error=>errors.push(error.message));
  await page.goto('http://127.0.0.1:5198/runtime-test.html');
  assert.equal(await page.title(),'Runtime delivery test');
  const result=await page.evaluate(async()=>{
    const {compileRemoteWasm,initializeCompiledWasm}=await import('/src/wasm-loader.js');
    const {default:loadMujoco}=await import('/node_modules/@mujoco/mujoco/mujoco.js');
    const url='/node_modules/@mujoco/mujoco/mujoco.wasm';
    const response=await fetch(url);const bytes=await response.arrayBuffer();
    const compiled=await WebAssembly.compile(bytes),runtime=await initializeCompiledWasm(loadMujoco,compiled);
    const model=runtime.MjModel.from_xml_string('<mujoco><worldbody><body><joint/><geom type="sphere" size=".1"/></body></worldbody></mujoco>');
    const nq=model.nq;model.delete();
    const streamed=await compileRemoteWasm(url);let rejected=false;
    try {await initializeCompiledWasm(options=>new Promise(resolve=>options.instantiateWasm({},resolve)),compiled);}catch{rejected=true;}
    return {bytes:bytes.byteLength,nq,streamed:streamed instanceof WebAssembly.Module,rejected};
  });
  assert(result.bytes>8*1024*1024);assert.equal(result.nq,1);assert(result.streamed);assert(result.rejected);
  assert.equal(requests.length,2,'One explicit binary download and one streaming test; no SDK fallback download');
  assert.deepEqual(errors,[]);console.log('Browser runtime delivery: >8 MB async initialization, native model, streaming compile and rejection propagation passed');
} finally {await browser.close();await server.close();}
