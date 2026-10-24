import assert from 'node:assert/strict';
import {mkdir,writeFile} from 'node:fs/promises';
import {createServer} from 'vite';
import {chromium} from 'playwright';

const label=process.argv.find(arg=>arg.startsWith('--label='))?.slice(8)||'baseline';
const software=process.argv.includes('--software');
const server=await createServer({server:{host:'127.0.0.1',port:5195,strictPort:true}});await server.listen();
const browser=await chromium.launch({channel:process.env.PLAYWRIGHT_CHANNEL||(process.platform==='win32'?'chrome':undefined),headless:true,
  args:[...(software?['--use-angle=swiftshader']:[]),'--enable-webgl','--ignore-gpu-blocklist']});
const errors=[],report={label,forcedSoftware:software};await mkdir('.reports',{recursive:true});
try {
  const page=await browser.newPage({viewport:{width:1100,height:850}});
  page.on('pageerror',error=>errors.push(error.message));await page.routeWebSocket('**',socket=>socket.close());
  const began=performance.now();
  await page.goto(`http://127.0.0.1:5195/?variant=dual${label.includes('full')?'&meshDetail=full':''}`,{waitUntil:'domcontentloaded'});
  await page.waitForFunction(()=>window.rebotDual&&document.getElementById('dual-loading').hidden,null,{timeout:180000});
  report.loadMs=performance.now()-began;
  await page.waitForFunction(()=>['overhead-camera-canvas','left-wrist-camera-canvas','right-wrist-camera-canvas'].every(id=>{
    const canvas=document.getElementById(id);return canvas.getContext('2d').getImageData(160,120,1,1).data.slice(0,3).some(value=>value>0);
  }),null,{timeout:90000});
  report.before=await page.evaluate(()=>window.rebotDual.renderMetrics());
  report.softwareRenderer=/SwiftShader|llvmpipe|software/i.test(report.before.graphics.renderer);
  console.log(JSON.stringify({loadMs:report.loadMs,graphics:report.before.graphics,geometry:report.before.geometry}));
  const samples=[];
  for(const preset of ['dual-room','dual-close','dual-overhead','dual-perspective','dual-close','dual-perspective']) {
    const at=performance.now();await page.locator(`#${preset}`).click({force:true});
    await page.waitForTimeout(500);
    samples.push({preset,wallMs:performance.now()-at,render:await page.evaluate(()=>window.rebotDual.renderMetrics())});
  }
  report.samples=samples;report.after=await page.evaluate(()=>window.rebotDual.renderMetrics());
  const beforeGpu=report.before.gpu,afterGpu=report.after.gpu;
  report.steadyMainGpuMs=afterGpu.samples>beforeGpu.samples?
    (afterGpu.meanMs*afterGpu.samples-beforeGpu.meanMs*beforeGpu.samples)/(afterGpu.samples-beforeGpu.samples):null;
  report.steadyMainCpuMs=report.after.mainDraws>report.before.mainDraws?
    (report.after.mainTotalMs-report.before.mainTotalMs)/(report.after.mainDraws-report.before.mainDraws):null;
  await page.screenshot({path:`.reports/render-${label}.png`,timeout:90000});
  assert(report.after.frames>report.before.frames);assert(!report.after.previewError);
  assert.deepEqual(errors,[]);
  console.log(JSON.stringify({label,gpu:report.after.gpu,mainMs:report.after.mainMs,previewMs:report.after.previewMs,errors}));
} finally {
  report.errors=errors;await writeFile(`.reports/render-${label}.json`,JSON.stringify(report,null,2));await browser.close();await server.close();
}
