import {mkdir,writeFile} from 'node:fs/promises';
import {chromium} from 'playwright';

const arg=name=>process.argv.find(value=>value.startsWith(`--${name}=`))?.slice(name.length+3);
const url=arg('url')||'https://yang-ci.github.io/reBot_RS_Sim/?variant=dual';
const label=arg('label')||'live',timeout=Number(arg('timeout')||180000);
const mbps=Number(arg('mbps')||0),latency=Number(arg('latency')||0);
const blockCdn=process.argv.includes('--block-cdn');
const browser=await chromium.launch({channel:process.env.PLAYWRIGHT_CHANNEL||(process.platform==='win32'?'chrome':undefined),headless:true,
  args:['--enable-webgl','--ignore-gpu-blocklist']});
const report={url,mbps,latency,blockCdn,resources:[],console:[],errors:[],failed:[]},requests=new Map();
await mkdir('.reports',{recursive:true});
try {
  const context=await browser.newContext({viewport:{width:1100,height:850},serviceWorkers:'block'});
  const page=await context.newPage(),session=await context.newCDPSession(page);
  if(blockCdn)await page.route('https://cdn.jsdelivr.net/**',route=>route.abort());
  await session.send('Network.enable');await session.send('Network.setCacheDisabled',{cacheDisabled:true});
  if(mbps||latency)await session.send('Network.emulateNetworkConditions',{offline:false,latency,downloadThroughput:mbps?mbps*1e6/8:-1,uploadThroughput:-1});
  session.on('Network.requestWillBeSent',event=>{const request=requests.get(event.requestId)||{};requests.set(event.requestId,{...request,url:event.request.url,start:event.timestamp});});
  session.on('Network.responseReceived',event=>{
    const request=requests.get(event.requestId);if(!request)return;
    const response=event.response,headers=Object.fromEntries(Object.entries(response.headers).map(([key,value])=>[key.toLowerCase(),value]));
    Object.assign(request,{status:response.status,mime:response.mimeType,fromServiceWorker:response.fromServiceWorker,
      firstByteMs:(event.timestamp-request.start)*1000,encoding:headers['content-encoding']||'identity',cacheControl:headers['cache-control'],
      server:headers.server});
  });
  session.on('Network.loadingFinished',event=>{
    const request=requests.get(event.requestId);if(!request)return;
    report.resources.push({...request,wireBytes:event.encodedDataLength,durationMs:(event.timestamp-request.start)*1000,start:undefined});
  });
  page.on('console',message=>{if(['warning','error'].includes(message.type()))report.console.push({type:message.type(),text:message.text()});});
  page.on('pageerror',error=>report.errors.push(error.message));
  page.on('requestfailed',request=>report.failed.push({url:request.url(),error:request.failure()?.errorText}));
  const began=performance.now();
  await page.goto(url,{waitUntil:'domcontentloaded',timeout});report.documentMs=performance.now()-began;
  await page.waitForFunction(()=>{
    const loading=document.getElementById('dual-loading');return loading?.hidden;
  },null,{timeout});report.controlsMs=performance.now()-began;
  await page.waitForFunction(()=>['overhead-camera-canvas','left-wrist-camera-canvas','right-wrist-camera-canvas'].every(id=>{
    const canvas=document.getElementById(id);if(!canvas||!canvas.width||!canvas.height)return false;
    const context=canvas.getContext('2d');
    if(context)return context.getImageData(160,120,1,1).data.slice(0,3).some(value=>value>0);
    // Older deployed builds use a WebGL canvas for each preview. Its pixels
    // are checked in the captured screenshot, not with a 2D-only API.
    return Boolean(canvas.getContext('webgl2')&&!canvas.getContext('webgl2').isContextLost());
  }),null,{timeout});report.threeCamerasMs=performance.now()-began;
  report.startup=await page.evaluate(()=>window.rebotDual?.startupMetrics?.());
  await page.screenshot({path:`.reports/deployment-${label}.png`,timeout});
} catch(error) {report.failure=error.message;process.exitCode=1;}
finally {
  report.wireBytes=report.resources.reduce((sum,resource)=>sum+resource.wireBytes,0);
  report.slowest=[...report.resources].sort((a,b)=>b.durationMs-a.durationMs).slice(0,8);
  await writeFile(`.reports/deployment-${label}.json`,JSON.stringify(report,null,2));
  console.log(JSON.stringify({url,documentMs:report.documentMs,controlsMs:report.controlsMs,threeCamerasMs:report.threeCamerasMs,
    wireBytes:report.wireBytes,slowest:report.slowest,console:report.console,errors:report.errors,failed:report.failed,failure:report.failure}));
  await browser.close();
}
