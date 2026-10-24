import assert from 'node:assert/strict';
import {mkdir,writeFile} from 'node:fs/promises';
import {createServer} from 'vite';
import {chromium} from 'playwright';

const arg=name=>process.argv.find(value=>value.startsWith(`--${name}=`))?.slice(name.length+3);
const external=arg('url'),label=arg('label')||'baseline';
const mbps=Number(arg('mbps')||0),latency=Number(arg('latency')||0);
const server=external?null:await createServer({server:{host:'127.0.0.1',port:5196,strictPort:true}});await server?.listen();
const browser=await chromium.launch({channel:process.env.PLAYWRIGHT_CHANNEL||(process.platform==='win32'?'chrome':undefined),headless:true,
  args:['--enable-webgl','--ignore-gpu-blocklist']});
const report={label,mbps,latency,runs:[],errors:[]};
await mkdir('.reports',{recursive:true});
try {
  const context=await browser.newContext({viewport:{width:1100,height:850}}),page=await context.newPage();
  await page.routeWebSocket('**',socket=>socket.close());page.on('pageerror',error=>report.errors.push(error.message));
  if(mbps||latency) {
    const session=await context.newCDPSession(page);await session.send('Network.enable');
    await session.send('Network.emulateNetworkConditions',{offline:false,latency,downloadThroughput:mbps?mbps*1e6/8:-1,uploadThroughput:-1});
  }
  for(const cache of ['cold','warm']) {
    const began=performance.now();
    await page.goto(external||'http://127.0.0.1:5196/?variant=dual',{waitUntil:'domcontentloaded'});
    await page.waitForFunction(()=>window.rebotDual&&document.getElementById('dual-loading').hidden,null,{timeout:240000});
    const controlsMs=performance.now()-began;
    await page.waitForFunction(()=>Number.isFinite(window.rebotDual.startupMetrics().firstFrameMs),null,{timeout:90000});
    const firstSceneMs=performance.now()-began;
    await page.waitForFunction(()=>['overhead-camera-canvas','left-wrist-camera-canvas','right-wrist-camera-canvas'].every(id=>{
      const canvas=document.getElementById(id);return canvas.getContext('2d').getImageData(160,120,1,1).data.slice(0,3).some(value=>value>0);
    }),null,{timeout:90000});
    const run={cache,controlsMs,firstSceneMs,threeCamerasMs:performance.now()-began,...await page.evaluate(async()=>{
      let cachedModels=0;
      for(const name of await caches.keys()) {
        if(!name.startsWith('rebot-mujoco-'))continue;
        const requests=await (await caches.open(name)).keys();
        cachedModels+=requests.filter(request=>new URL(request.url).pathname.includes('/models/')).length;
      }
      const metrics=window.rebotDual.renderMetrics();
      return {startup:window.rebotDual.startupMetrics(),graphics:metrics.graphics,wasmBytes:metrics.wasmMemoryBytes,
        serviceWorker:Boolean(navigator.serviceWorker?.controller),cachedModels,
        resources:performance.getEntriesByType('resource').map(entry=>({name:new URL(entry.name).pathname,
          transferSize:entry.transferSize,encodedBodySize:entry.encodedBodySize,duration:entry.duration}))};
    })};
    report.runs.push(run);console.log(JSON.stringify({cache,controlsMs,firstSceneMs,threeCamerasMs:run.threeCamerasMs,startup:run.startup}));
    assert.equal(await page.locator('#dual-start').isEnabled(),true);
  }
  assert.deepEqual(report.errors,[]);
} finally {await writeFile(`.reports/startup-${label}.json`,JSON.stringify(report,null,2));await browser.close();await server?.close();}
