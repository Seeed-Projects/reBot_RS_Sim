import assert from 'node:assert/strict';
import { mkdir, writeFile } from 'node:fs/promises';
import { createServer } from 'vite';
import { chromium } from 'playwright';

const reports=[],errors=[];
const perturbedOnly=process.argv.includes('--perturbed');
const contact=process.argv.includes('--contact'),hardware=process.argv.includes('--hardware');
const extended=process.argv.includes('--extended');
const returning=process.argv.includes('--return');
const reportName=returning?'visual-return':extended?'visual-extended':contact?'visual-contact':perturbedOnly?'visual-closed-loop-perturbed':'visual-closed-loop';
await mkdir('.reports',{recursive:true});
const server=await createServer({server:{host:'127.0.0.1',port:5193,strictPort:true}});
await server.listen();
const browser=await chromium.launch({channel:process.env.PLAYWRIGHT_CHANNEL||(process.platform==='win32'?'chrome':undefined),headless:true,args:[...(hardware?[]:['--use-angle=swiftshader']),'--enable-webgl','--ignore-gpu-blocklist']});
try {
  const page=await browser.newPage({viewport:{width:1280,height:900}});
  await page.routeWebSocket('**',socket=>socket.close());
  page.on('pageerror',e=>errors.push(e.message));
  await page.goto(`http://127.0.0.1:5193/vision-harness.html${contact?'?grasp=contact':''}`,{waitUntil:'domcontentloaded'});
  await page.waitForFunction(()=>window.ready,null,{timeout:180000});
  const cases=returning?[{task:'present-and-return'},{task:'present-and-return',seed:17,noise:true}]:extended?[
    {appearance:{lightScale:.65,exposure:.9}},
    {appearance:{lightScale:1.35,exposure:1.15}},
    {seed:1,appearance:{cupColor:'#7c9cce',cupRoughness:.6}},
    {seed:42,appearance:{cupColor:'#bc7963',cupRoughness:.18}},
    {appearance:{cupWallStyle:'straight',cupRoughness:.65}},
    {moveBeforeGrasp:.012,noise:true},
    {partialOcclusion:.15},
    {missingObject:'cup'},
    {missingObject:'spoon'},
    {fault:'occluded'},
    {fault:'stale'}
  ]:[{seed:null},{seed:1},{seed:42,frictionScale:1.1},{seed:17,noise:true,frictionScale:.9},{moveBeforeGrasp:.004},{fault:'transient-occlusion'},{fault:'occluded'},{fault:'stale'}];
  for(const config of cases.filter((config,index)=>(!perturbedOnly||config.frictionScale)&&(!process.argv.includes('--smoke')||index===0))) {
    const report=await page.evaluate(async config=>{
      const {runVisionCase}=await import('/scripts/visual-suite.js');return runVisionCase(window.visionHarness,config);
    },config);
    reports.push(report);
    await writeFile(`.reports/${reportName}.json`,JSON.stringify({reports,errors},null,2));
    if(report.state.error&&!config.fault&&!config.missingObject) {
      await page.evaluate(()=>window.visionHarness.capture());
      await page.locator('#sensor').screenshot({path:`.reports/${reportName}-failure.png`});
    }
    if(config.fault==='occluded'||config.fault==='stale'||config.missingObject) {
      assert.equal(report.state.complete,false);assert.equal(report.state.running,false);
      assert.equal(report.state.error,config.fault==='stale'?'vision-stale':'vision-occluded');
      assert.equal(report.state.vision.history.length,3,'Bound observation retries');
      if(config.missingObject)assert(report.observations.every(o=>!o.observation.valid&&
        (!o.observation[config.missingObject]||o.observation[config.missingObject].confidence<=.6)),
        'Do not accept fixtures or fruit as the missing object');
    } else {
      assert.equal(report.state.error,'',JSON.stringify(report.state.failure));assert.equal(report.state.complete,true);
      assert.equal(report.state.controlMode,'visual-rgbd');
      if(!returning)assert.equal(report.state.vision.bowl.inside,true);
      else {
        const initial=report.observations.find(o=>o.stage==='initial').observation;
        const final=report.observations.at(-1).observation;
        for(const key of ['cup','spoon'])assert(Math.hypot(...final[key].position.map((value,i)=>value-initial[key].position[i]))<.010);
        assert.equal(report.state.inserted,false);
        assert(report.phases.includes('return-objects'));
      }
      assert(report.observations.some(o=>o.stage==='alignment'));
      assert(report.observations.some(o=>o.stage==='settled-verification'));
      const final=report.observations.filter(o=>o.stage.includes('verification'));
      assert(final.at(-1).observation.timestamp-final[0].observation.timestamp>=.3,'Verify stability across separate observations');
      for(const o of report.observations.filter(o=>o.observation.valid)) {
        assert(o.errors.cup<.005,`${o.stage}: cup pose error ${o.errors.cup}`);
        assert(o.errors.spoon<.006,`${o.stage}: spoon pose error ${o.errors.spoon}`);
      }
      if(!returning)assert(report.state.vision.corrections>=1);
      assert(report.state.metrics.collision.samples>0&&report.state.metrics.collision.checks>0);
    }
    console.log(JSON.stringify({config,complete:report.state.complete,error:report.state.error,milliseconds:report.milliseconds}));
  }
  assert.deepEqual(errors,[]);
  await page.locator('#sensor').screenshot({path:'.reports/vision-final.png'});
} finally {await browser.close();await server.close();}
console.log('Rendered RGB-D: localization, dual execution, fresh alignment correction, post-release verification and bounded failure passed');
