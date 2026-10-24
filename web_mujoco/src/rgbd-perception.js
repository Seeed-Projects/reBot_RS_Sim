import { Matrix4, Quaternion, Vector3 } from 'three';

// Pure image geometry. This module deliberately has no MuJoCo dependency and
// accepts no scene graph, body IDs, object-state messages or ground-truth poses.
const dot = (a, b) => a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
const mean = values => values.reduce((a, b) => a + b, 0) / values.length;
const percentile = (values, fraction) => values.sort((a,b) => a-b)[Math.min(values.length-1, Math.floor(values.length*fraction))];
function linear3(rows, values) {
  const a = Array.from({ length: 3 }, () => [0,0,0,0]);
  rows.forEach((row, i) => row.forEach((x, j) => { for (let k=0;k<3;k++) a[j][k] += x*row[k]; a[j][3] += x*values[i]; }));
  for (let i=0;i<3;i++) {
    let pivot=i; for (let j=i+1;j<3;j++) if (Math.abs(a[j][i])>Math.abs(a[pivot][i])) pivot=j;
    [a[i],a[pivot]]=[a[pivot],a[i]];
    if (Math.abs(a[i][i])<1e-12) return null;
    const div=a[i][i]; for(let k=i;k<4;k++) a[i][k]/=div;
    for(let j=0;j<3;j++) if(j!==i) { const scale=a[j][i]; for(let k=i;k<4;k++) a[j][k]-=scale*a[i][k]; }
  }
  return a.map(row=>row[3]);
}
function principalAxis(points) {
  const centre = [0,1,2].map(axis=>mean(points.map(p=>p[axis])));
  const covariance = Array.from({length:3},()=>[0,0,0]);
  for(const p of points) for(let i=0;i<3;i++) for(let j=0;j<3;j++) covariance[i][j]+=(p[i]-centre[i])*(p[j]-centre[j]);
  let axis=[1, .01, -.1];
  for(let iteration=0;iteration<24;iteration++) {
    const next=covariance.map(row=>dot(row,axis)), n=Math.hypot(...next);
    if(n<1e-15) return null; axis=next.map(v=>v/n);
  }
  if(axis[2]>0 || (Math.abs(axis[2])<.01 && axis[0]<0)) axis=axis.map(v=>-v);
  return { axis, centre };
}
export function pixelToWorld(frame, column, row) {
  const d=frame.depth[row*frame.width+column];
  if(!Number.isFinite(d)) return null;
  const c=frame.camera,m=c.matrixWorld;
  const x=(column-c.cx)*d/c.fx, y=(row-c.cy)*d/c.fy, z=-d;
  return [m[0]*x+m[4]*y+m[8]*z+m[12],m[1]*x+m[5]*y+m[9]*z+m[13],m[2]*x+m[6]*y+m[10]*z+m[14]];
}
export function imagePointCloud(frames) {
  const green=[],metal=[];
  for(const frame of frames) {
    if(frame.source!=='rendered-rgbd' || !frame.depth || frame.depth.length!==frame.width*frame.height || frame.rgb.length!==frame.depth.length*4) throw new Error('Invalid RGB-D camera frame');
    for(let row=0;row<frame.height;row++) for(let column=0;column<frame.width;column++) {
      const i=(row*frame.width+column)*4, r=frame.rgb[i],g=frame.rgb[i+1],b=frame.rgb[i+2];
      const isGreen=g>r*1.025 && g>b*1.005 && g>100 && r>g*.75 && b>g*.82;
      const isBlue=b>r*1.07&&b>g*1.03&&b>60;
      const isWarm=r>g*1.08&&r>b*1.08&&r>80;
      const isCup=isGreen||isBlue||isWarm;
      const isMetal=Math.min(r,g,b)>55 && Math.min(r,g,b)/Math.max(r,g,b)>.91;
      if(!isCup&&!isMetal) continue;
      const p=pixelToWorld(frame,column,row);
      if(!p || p[0]<.10 || p[0]>.57 || p[1]<-.20 || p[1]>.19 || p[2]<.845 || p[2]>1.10) continue;
      if(isCup) green.push(p);
      if(isMetal) metal.push(p);
    }
  }
  return {green,metal};
}
function detectCup(points, prior) {
  let candidates=largestCluster(points);
  if(prior) candidates=points.filter(p=>Math.hypot(p[0]-prior.position[0],p[1]-prior.position[1])<.052 && p[2]<prior.position[2]+.052);
  if(candidates.length<80) return null;
  const top=percentile(candidates.map(p=>p[2]),.97);
  let rim=candidates.filter(p=>Math.abs(p[2]-top)<.002);
  if(rim.length<40) return null;
  let x,y,radius;
  for(let iteration=0;iteration<3;iteration++) {
    const origin=[mean(rim.map(p=>p[0])),mean(rim.map(p=>p[1]))];
    const rows=rim.map(p=>[p[0]-origin[0],p[1]-origin[1],1]);
    const solved=linear3(rows,rows.map(([x,y])=>-x*x-y*y));
    if(!solved) return null;
    x=origin[0]-solved[0]/2;y=origin[1]-solved[1]/2;
    radius=Math.sqrt((solved[0]**2+solved[1]**2)/4-solved[2]);
    if(iteration<2) rim=rim.filter(p=>Math.abs(Math.hypot(p[0]-x,p[1]-y)-radius)<.0017);
    if(rim.length<40) return null;
  }
  if(radius<.028 || radius>.043) return null;
  const plane=linear3(rim.map(p=>[p[0]-x,p[1]-y,1]),rim.map(p=>p[2]));
  if(!plane) return null;
  const up=new Vector3(-plane[0],-plane[1],1).normalize();
  const handle=candidates.filter(p=>Math.hypot(p[0]-x,p[1]-y)>.045 && Math.hypot(p[0]-x,p[1]-y)<.085);
  if(handle.length<12 && !prior) return null;
  const outward=handle.length>=12 ? new Vector3(mean(handle.map(p=>p[0]))-x,mean(handle.map(p=>p[1]))-y,0)
    : new Vector3(0,-1,0).applyQuaternion(new Quaternion(...prior.quaternion));
  outward.addScaledVector(up,-outward.dot(up)).normalize();
  const axisY=outward.negate(),axisX=axisY.clone().cross(up).normalize();
  const quaternion=new Quaternion().setFromRotationMatrix(new Matrix4().makeBasis(axisX,axisY,up));
  const centre=new Vector3(x,y,plane[2]).addScaledVector(up,-.0415);
  const residual=Math.sqrt(mean(rim.map(p=>(Math.hypot(p[0]-x,p[1]-y)-radius)**2)));
  return { position:centre.toArray(), quaternion:quaternion.toArray(), confidence:Math.min(1,rim.length/150)*Math.exp(-residual/.003)*(handle.length>=12?1:.75),
    pixels:candidates.length,radius,residual,handlePixels:handle.length };
}
function largestCluster(points) {
  const voxels=new Map(),size=.006;
  for(const p of points) {const key=p.map(v=>Math.floor(v/size)).join(',');if(!voxels.has(key)) voxels.set(key,[]);voxels.get(key).push(p);}
  let largest=[];
  while(voxels.size) {
    const first=voxels.keys().next().value,queue=[first],cluster=[];
    for(let i=0;i<queue.length;i++) {
      const key=queue[i],cell=voxels.get(key);if(!cell) continue;voxels.delete(key);cluster.push(...cell);
      const xyz=key.split(',').map(Number);
      for(let x=-1;x<=1;x++) for(let y=-1;y<=1;y++) for(let z=-1;z<=1;z++) {
        const next=[xyz[0]+x,xyz[1]+y,xyz[2]+z].join(',');if(voxels.has(next)) queue.push(next);
      }
    }
    if(cluster.length>largest.length) largest=cluster;
  }
  return largest;
}
function detectSpoon(points,cup,{mode='source',prior,stage,destination='cup'}={}) {
  const released=stage==='verification'||stage==='settled-verification';
  const placed=released&&destination==='cup';
  if(prior?.spoon && !released) return trackSpoon(points,prior.spoon);
  const candidates=largestCluster(points.filter(p=>mode==='source'
    ? p[0]>.22 && p[0]<.50 && p[1]>Math.max(-.025,(cup?.position[1]??-.08)+.055) && p[1]<.17 && p[2]<.98
    : cup && Math.abs(p[0]-cup.position[0])<.14 && Math.abs(p[1]-cup.position[1])<.055 && p[2]<cup.position[2]+.14 &&
      (!placed || p[2]>cup.position[2]+.044)));
  if(candidates.length<40) return null;
  let fit=principalAxis(candidates); if(!fit) return null;
  let minimum=percentile(candidates.map(p=>dot(p,fit.axis)),.01);
  let stem=candidates.filter(p=>dot(p,fit.axis)<minimum+.085);
  if(stem.length<20) return null;
  fit=principalAxis(stem); if(!fit) return null;
  const axisX=new Vector3(...fit.axis),axisY=new Vector3(0,0,1).cross(axisX).normalize(),axisZ=axisX.clone().cross(axisY).normalize();
  const x=axisX.toArray(),y=axisY.toArray(),z=axisZ.toArray();
  minimum=percentile(candidates.map(p=>dot(p,x)),.005);
  stem=candidates.filter(p=>dot(p,x)<minimum+.080);
  const coordinates=candidates.map(p=>[dot(p,x),dot(p,y),dot(p,z)]);
  const extent=percentile(coordinates.map(p=>p[0]),.995)-minimum;
  if(extent<.025 || extent>.19) return null;
  let origin=[minimum+.03665,mean(stem.map(p=>dot(p,y))),percentile(stem.map(p=>dot(p,z)),.5)-.00135];
  let method='visible-handle';
  const bowl=coordinates.filter(p=>p[0]>minimum+.108);
  if(bowl.length>=60 && extent>.12) {
    const u=mean(bowl.map(p=>p[0])),v=mean(bowl.map(p=>p[1])),a=10,b=.004/.013**2;
    const parabola=linear3(bowl.map(p=>[p[0]-u,p[1]-v,1]),bowl.map(p=>p[2]-a*(p[0]-u)**2-b*(p[1]-v)**2));
    if(parabola) {
      const du=-parabola[0]/(2*a),dv=-parabola[1]/(2*b);
      const refined=[u+du-.094,v+dv,parabola[2]+.004-a*du**2-b*dv**2];
      if(Math.hypot(...refined.map((value,i)=>value-origin[i]))<.008) { origin=refined; method='bowl-surface'; }
    }
  }
  const position=axisX.clone().multiplyScalar(origin[0]).addScaledVector(axisY,origin[1]).addScaledVector(axisZ,origin[2]);
  const quaternion=new Quaternion().setFromRotationMatrix(new Matrix4().makeBasis(axisX,axisY,axisZ));
  const residual=Math.sqrt(mean(stem.map(p=>(dot(p,z)-origin[2]-.00135)**2)));
  const confidence=Math.min(1,candidates.length/160)*Math.min(1,extent/(placed?.04:.07))*Math.exp(-residual/.003);
  return {position:position.toArray(),quaternion:quaternion.toArray(),confidence,pixels:candidates.length,extent,residual,method};
}
function trackSpoon(points,prior) {
  const q=new Quaternion(...prior.quaternion),inverse=q.clone().invert(),position=new Vector3(...prior.position);
  const local=points.map(p=>new Vector3(...p).sub(position).applyQuaternion(inverse)).filter(p=>
    p.x>-.045&&p.x<.121&&Math.abs(p.y)<.022&&Math.abs(p.z)<.012);
  const bowl=local.filter(p=>p.x>.078&&p.x<.113&&Math.abs(p.y)<.012&&p.z<.005);
  const stem=local.filter(p=>p.x<.070&&Math.abs(p.y)<.0048&&p.z>-.0015&&p.z<.0045);
  if(stem.length<20) return null;
  const plane=linear3(stem.map(p=>[p.x,p.y,1]),stem.map(p=>p.z));
  if(!plane) return null;
  const axisX=new Vector3(1,0,plane[0]).normalize(),axisY=new Vector3(0,1,plane[1]).normalize();
  const axisZ=axisX.clone().cross(axisY).normalize();axisY.copy(axisZ).cross(axisX).normalize();
  const deltaQ=new Quaternion().setFromRotationMatrix(new Matrix4().makeBasis(axisX,axisY,axisZ));
  const refinedQ=q.clone().multiply(deltaQ),invDelta=deltaQ.clone().invert();
  const refinedBowl=bowl.map(p=>p.clone().applyQuaternion(invDelta));
  let offset;
  if(refinedBowl.length>=40) {
    const a=10,b=.004/.013**2;
    const fit=linear3(refinedBowl.map(p=>[p.x-.094,p.y,1]),refinedBowl.map(p=>p.z-a*(p.x-.094)**2-b*p.y**2));
    if(!fit) return null;
    const dx=-fit[0]/(2*a),dy=-fit[1]/(2*b),dz=fit[2]+.004-a*dx*dx-b*dy*dy;
    offset=new Vector3(dx,dy,dz);
  } else {
    // With the bowl hidden, the rounded handle tip anchors translation along
    // the stem. Never infer that endpoint from an arbitrary occluded fragment.
    const refined=stem.map(p=>p.clone().applyQuaternion(invDelta));
    const tip=percentile(refined.map(p=>p.x),.02);
    if(Math.abs(tip+.03665)>.006) return null;
    offset=new Vector3(tip+.03665,mean(refined.map(p=>p.y)),percentile(refined.map(p=>p.z),.7)-.00135);
  }
  if(offset.length()>.018) return null;
  const confidence=Math.min(1,stem.length/100)*Math.min(1,(bowl.length||stem.length)/100);
  return {position:position.add(offset.applyQuaternion(refinedQ)).toArray(),quaternion:refinedQ.toArray(),confidence,
    pixels:local.length,method:bowl.length>=40?'tracked-bowl-surface':'tracked-visible-tip',visibleBowlPixels:bowl.length,visibleStemPixels:stem.length};
}
export function detectTabletopObjects(input,options={}) {
  const began=performance.now(),frames=Array.isArray(input)?input:[input];
  const clouds=imagePointCloud(frames);
  const cup=detectCup(clouds.green,options.prior?.cup);
  const spoon=detectSpoon(clouds.metal,cup,options);
  return {source:'rgbd-pixels',timestamp:Math.min(...frames.map(frame=>frame.timestamp)),cup,spoon,
    valid:Boolean(cup&&spoon&&cup.confidence>.6&&spoon.confidence>.6),
    diagnostics:{greenPixels:clouds.green.length,metalPixels:clouds.metal.length,computeMs:performance.now()-began,
      ...(options.debug?{clusters:[clouds.green,clouds.metal].map(points=>{const c=largestCluster(points);return {count:c.length,bounds:[0,1,2].map(a=>[Math.min(...c.map(p=>p[a])),Math.max(...c.map(p=>p[a]))])};})}:{})}};
}
