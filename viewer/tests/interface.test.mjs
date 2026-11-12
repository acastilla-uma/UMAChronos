import assert from 'node:assert/strict';
import fs from 'node:fs';
import vm from 'node:vm';
import {test} from 'node:test';

// Exercise the shipped event handlers; stand in only for the browser, transport,
// and GPU. No copied application logic is used in these tests.
function boot(){
  const nodes=new Map(),calls=[],uploads=[],meshLoads=[],progressSamples=[];
  class Element{
    constructor(tag='div'){this.tagName=tag;this.children=[];this.style={};this.attrs={};this.listeners={};this._value='';this.disabled=false;this.files=[];this.clientWidth=800;this.clientHeight=600;this.open=false;this.textContent=''}
    set id(v){this._id=v;nodes.set(v,this)}get id(){return this._id}
    set value(v){this._value=String(v);if(this.id==='loadProgress')progressSamples.push(Number(v));if(this.tagName==='input'&&v==='')this.files=[]}get value(){return this._value}
    append(...children){this.children.push(...children);for(const c of children)c.parent=this;if(this.tagName==='select'&&!this.value&&children[0])this.value=children[0].value}
    prepend(...children){this.children.unshift(...children)}
    replaceChildren(...children){this.children=[];if(this.tagName==='select')this._value='';this.append(...children)}
    get options(){return this.children}get selectedOptions(){return this.options.filter(x=>x.value===this.value)}
    setAttribute(k,v){this.attrs[k]=String(v)}getAttribute(k){return this.attrs[k]??null}removeAttribute(k){delete this.attrs[k];if(k==='value')this._value=''}
    addEventListener(k,fn){this.listeners[k]=fn}getBoundingClientRect(){return {left:0,top:0,right:800,bottom:600,width:800,height:600}}
    showModal(){this.open=true}close(){this.open=false;this.listeners.close?.()}remove(){this.removed=true}click(){this.clicked=true;return this.onclick?.({})}
    getContext(){this.drawCalls??=[];return new Proxy({},{get:(_,method)=>(...args)=>this.drawCalls.push([method,...args])})}
    querySelector(){return null}closest(){return this.parent}
  }
  const html=fs.readFileSync(new URL('../index.html',import.meta.url),'utf8');
  for(const m of html.matchAll(/<(\w+)[^>]*\bid="([^"]+)"[^>]*>/g)){const el=new Element(m[1]);el.id=m[2];const v=m[0].match(/value="([^"]*)"/);if(v)el.value=v[1];el.readOnly=/readonly/.test(m[0])}
  const document={body:new Element('body'),getElementById:id=>{assert(nodes.has(id),`missing DOM element ${id}`);return nodes.get(id)},createElement:tag=>new Element(tag),querySelector:()=>null,querySelectorAll:q=>q==='dialog'?[...nodes.values()].filter(n=>n.tagName==='dialog'):[]};
  class Vec{constructor(x=0,y=0,z=0){this.set(x,y,z)}set(x,y,z){Object.assign(this,{x,y,z});return this}fromArray(a){return this.set(...a)}copy(p){return this.set(p.x,p.y,p.z)}add(p){this.x+=p.x;this.y+=p.y;this.z+=p.z;return this}normalize(){return this}multiplyScalar(){return this}length(){return Math.hypot(this.x,this.y,this.z)}}
  class Object3D{constructor(){this.children=[];this.position=new Vec;this.up=new Vec;this.rotation={};this.quaternion={set:()=>({normalize(){}})}}add(...a){this.children.push(...a)}remove(c){this.children=this.children.filter(x=>x!==c)}traverse(fn){fn(this);this.children.forEach(c=>c.traverse?.(fn))}}
  class Geometry{constructor(){this.attributes={}}getAttribute(n){return this.attributes[n]}setAttribute(n,v){this.attributes[n]=v}dispose(){}computeBoundingSphere(){this.boundingSphere={center:new Vec,radius:20}}setFromPoints(){return this}}
  class Attribute{constructor(array,size){this.array=array;this.count=array.length/size}setUsage(){}}
  class Material{dispose(){}}
  class Points extends Object3D{constructor(g,m){super();this.geometry=g;this.material=m}}
  class Camera extends Object3D{constructor(fov,aspect){super();this.fov=fov;this.aspect=aspect}updateProjectionMatrix(){}}
  class Renderer{constructor(){this.domElement=new Element('canvas')}setPixelRatio(){}setSize(){}render(){}}
  class Quaternion{constructor(){this.x=0;this.y=0;this.z=0;this.w=1}setFromEuler(){return this}}
  class Arrow extends Object3D{setDirection(v){this.direction=v}setLength(){}setColor(){}}
  const THREE={ArrowHelper:Arrow,Mesh:Points,CylinderGeometry:Geometry,Vector3:Vec,Vector2:Vec,Quaternion,Euler:class{},Scene:Object3D,Group:Object3D,PerspectiveCamera:Camera,WebGLRenderer:Renderer,GridHelper:Object3D,AxesHelper:Object3D,HemisphereLight:Object3D,DirectionalLight:Object3D,BufferGeometry:Geometry,Float32BufferAttribute:Attribute,Points,Line:Points,PointsMaterial:Material,LineBasicMaterial:Material,MeshPhongMaterial:Material,Color:class{constructor(){this.r=this.g=this.b=.5}},Raycaster:class{constructor(){this.params={Points:{}}}setFromCamera(){}intersectObject(){return this.hits||[]}},MathUtils:{radToDeg:v=>v*180/Math.PI,degToRad:v=>v*Math.PI/180},DynamicDrawUsage:1};
  const data={cloud:{sha256:'fixture',count:3,label_mapping:{}},points:[[0,0,0],[20,0,0],[0,20,0]],labels:[-1,-1,-1],center:[10,10,0],catalog:[{id:'MAN_10t',name:'MAN 10t · rígido experimental'}],model:{id:'MAN_10t'}};
  const portable={project:null,rejectPlacement:false,poll:0};
  const context=vm.createContext({document,THREE,OrbitControls:class{constructor(){this.target=new Vec}update(){}addEventListener(){}},OBJLoader:class{load(path,done){meshLoads.push(path);done(new Object3D)}},ResizeObserver:class{observe(){}},devicePixelRatio:1,requestAnimationFrame:()=>{},window:{addEventListener(){}},performance:{now:()=>0},fetch:async(url,opts)=>{calls.push({url,opts});const payload=opts?.body?JSON.parse(opts.body):{};const rejected=url==='/api/place'&&portable.rejectPlacement;let response={};
      if(url==='/api/info')response={...data,cloud:null,points:[],labels:[]};
      if(url==='/api/place')response=rejected?{error:'Apoyo no válido'}:{position:[payload.x,payload.y,1.2],orientation:[Math.cos(payload.yaw*Math.PI/360),0,0,Math.sin(payload.yaw*Math.PI/360)],roll:0,pitch:0,yaw:payload.yaw,wheel_centers:[],tire_radius:.645,tire_width:.385};
      if(url==='/api/simulation/start'){portable.poll=0;response={job_id:'job-1',status:'running'}}
      if(url.startsWith('/api/simulation/job?')){portable.poll++;const complete=portable.poll>2;response={job_id:'job-1',status:complete?'complete':'running',progress:{stage:complete?'complete':'simulation',fraction:complete?1:portable.poll===1?.25:.75,processed:complete?4:portable.poll===1?1:3,total:4},...(complete?{result:{frames:[{time:0,position:[0,0,1.2],orientation:[1,0,0,0],points:[]},{time:1,position:[1,0,1.2],orientation:[1,0,0,0],points:[]}],csv:'/api/export.csv'}}:{})}}
      return {ok:!rejected,blob:async()=>({}),json:async()=>response}},XMLHttpRequest:class{constructor(){this.upload={};this.status=200}open(method,url){this.url=url}setRequestHeader(){}send(file){uploads.push({url:this.url,file});if(file.fail){this.status=400;this.responseText=JSON.stringify({error:'PLY inválido'})}else{this.upload.onprogress?.({lengthComputable:true,loaded:file.size,total:file.size});this.upload.onload?.();this.responseText=JSON.stringify(this.url==='/api/project/import'?{project:portable.project,info:data,csv_available:true}:data)}this.onload()}},URL:{createObjectURL:()=>'',revokeObjectURL(){}},setTimeout:fn=>fn(),console});
  const source=fs.readFileSync(new URL('../app.js',import.meta.url),'utf8').replace(/^import .*;\r?$/gm,'');vm.runInContext(source,context);
  return {nodes,calls,uploads,meshLoads,progressSamples,context,portable,run:code=>vm.runInContext(code,context),data};
}
const tick=()=>new Promise(resolve=>setImmediate(resolve));

test('file selection waits for Cargar; imports full map once and reports progress',async()=>{
 const b=boot();await tick();b.nodes.get('file').files=[{size:123,name:'map.ply'}];b.nodes.get('file').onchange();
 assert.equal(b.uploads.length,0);assert.equal(b.nodes.get('loadCloud').disabled,false);
 await b.nodes.get('loadCloud').onclick();assert.equal(b.uploads.length,1);assert.equal(b.uploads[0].url,'/api/load');
 assert.equal(b.run('cloudPoints.length'),3);assert.equal(b.nodes.get('loadProgress').value,'100');assert.equal(b.nodes.get('file').files.length,0);
 assert.equal(b.nodes.get('simulate').disabled,true);assert.equal(b.run('chassis.visible'),false);
});

test('a scene click places immediately and orientation edits rotate without Apply',async()=>{
 const b=boot();await tick();await b.run('loadInfo('+JSON.stringify(b.data)+')');
 b.run('ray.hits=[{index:0}];renderer.domElement.listeners.pointerdown({clientX:100,clientY:100})');
 await b.run('renderer.domElement.listeners.click({clientX:100,clientY:100})');
 assert.equal(b.nodes.has('applyVehicle'),false);assert.equal(b.run('chassis.visible'),true);assert.equal(b.nodes.get('simulate').disabled,false);
 for(const id of ['x','y','z','roll','pitch'])assert.equal(b.nodes.get(id).readOnly,true);
 b.nodes.get('yaw').value='90';await b.nodes.get('yaw').onchange();assert.equal(b.run('appliedPose.values.yaw'),90);assert(Math.abs(b.run('headingArrow.direction.y')-1)<1e-10);assert.equal(Math.round(b.run('route[0].yaw')),90);
 assert.equal(b.calls.filter(c=>c.url==='/api/simulation/start').length,0);assert.equal(b.run('headingArrow.visible'),true);
});

test('route starts at vehicle and preserves its first point on undo and clear',async()=>{
 const b=boot();await tick();await b.run('loadInfo('+JSON.stringify(b.data)+')');
 assert.equal(b.nodes.has('routeRows'),false);assert.equal(b.nodes.has('addWaypoint'),false);
 const start=b.run("mapTransform($('routeMap')).point([0,0])");await b.nodes.get('routeMap').onclick({clientX:start[0],clientY:start[1]});
 const end=b.run("mapTransform($('routeMap')).point([20,0])");await b.nodes.get('routeMap').onclick({clientX:end[0],clientY:end[1]});assert.equal(b.run('route.length'),2);
 await b.nodes.get('routeMap').onclick({clientX:400,clientY:300});assert.equal(b.run('route.length'),2);assert.match(b.nodes.get('error').textContent,/cobertura/);
 await b.run('placeVehicle(0,20)');assert.equal(b.run('route[0].y'),20);assert.equal(b.run('route[1].x'),20);
 b.nodes.get('undoWaypoint').onclick();assert.equal(b.run('route.length'),1);
 b.nodes.get('clearRoute').onclick();assert.equal(b.run('route.length'),1);assert.equal(b.run('route[0].y'),20);
});

test('invalid placement and rotation keep accepted vehicle and route',async()=>{
 const b=boot();await tick();await b.run('loadInfo('+JSON.stringify(b.data)+')');await b.run('placeVehicle(0,0)');
 b.portable.rejectPlacement=true;await assert.rejects(b.run('placeVehicle(20,0)'),/Apoyo no válido/);
 assert.equal(b.run('appliedPose.position[0]'),0);assert.equal(b.run('route[0].x'),0);
 await assert.rejects(b.run('rotateVehicle(90)'),/Apoyo no válido/);assert.equal(b.nodes.get('heading').value,'0');
 await assert.rejects(b.run('appendWaypointAt([20,0,0])'),/Apoyo no válido/);assert.equal(b.run('route.length'),1);
});

test('failed upload preserves existing map and unlocks controls',async()=>{
 const b=boot();await tick();await b.run('loadInfo('+JSON.stringify(b.data)+')');b.nodes.get('file').files=[{size:100,fail:true}];
 await b.nodes.get('loadCloud').onclick();assert.equal(b.run('cloudPoints.length'),3);assert.equal(b.run('busy'),false);assert.equal(b.nodes.get('file').disabled,false);assert.match(b.nodes.get('error').textContent,/PLY inválido/);
});

test('simulation uses applied state, restores playback and CSV, never reloads map',async()=>{
 const b=boot();await tick();await b.run('loadInfo('+JSON.stringify(b.data)+')');await b.run('placeVehicle(0,0)');await b.nodes.get('simulate').onclick();
 const call=b.calls.find(c=>c.url==='/api/simulation/start');assert(call);const payload=JSON.parse(call.opts.body);assert.equal(payload.route.length,0);assert.equal(payload.until_destination,false);assert.equal(payload.duration,2);assert.equal(payload.dt,.01);assert(!('disable_csv' in payload));
 assert.equal(b.run('frames.length'),2);assert(b.progressSamples.includes(25));assert(b.progressSamples.includes(75));assert.equal(b.progressSamples.at(-1),100);assert.equal(b.nodes.get('play').disabled,false);assert.equal(b.nodes.get('downloadCsv').getAttribute('aria-disabled'),'false');
 b.nodes.get('timeline').value=1;b.nodes.get('timeline').oninput();assert.equal(b.run('index'),1);assert.equal(b.calls.filter(c=>c.url==='/api/view').length,0);
});

test('placement uses active terrain and recalculates support before simulating changed ground',async()=>{
 const b=boot();await tick();await b.run('loadInfo('+JSON.stringify(b.data)+')');
 b.run('terrain.classes.desconocido.stiffness=2000000');await b.run('placeVehicle(0,0,30)');
 let placements=b.calls.filter(c=>c.url==='/api/place');
 assert.equal(JSON.parse(placements[0].opts.body).terrain.classes.desconocido.stiffness,2000000);
 assert.equal(b.nodes.get('status').textContent,'Vehículo apoyado sobre el terreno');
 await b.nodes.get('simulate').onclick();
 assert.equal(b.calls.filter(c=>c.url==='/api/place').length,1);
 b.run('terrain.classes.desconocido.stiffness=3000000');await b.nodes.get('simulate').onclick();
 placements=b.calls.filter(c=>c.url==='/api/place');assert.equal(placements.length,2);
 assert.equal(JSON.parse(placements[1].opts.body).terrain.classes.desconocido.stiffness,3000000);
 const payload=JSON.parse(b.calls.filter(c=>c.url==='/api/simulation/start').at(-1).opts.body);
 assert.equal(payload.z,b.run('appliedPose.position[2]'));
 assert.equal(payload.roll,b.run('appliedPose.roll'));assert.equal(payload.pitch,b.run('appliedPose.pitch'));
 assert.equal(payload.vz,0);assert.equal(payload.route.length,0);
});

test('removed configuration cannot be reached and project retains complete results',async()=>{
 const b=boot();await tick();for(const id of ['configDialog','showVectors','disableCsv','lightweightPhysics','saveConfig','loadConfig','evaluate'])assert.equal(b.nodes.has(id),false);
 await b.run('loadInfo('+JSON.stringify(b.data)+')');await b.run('placeVehicle(0,0)');await b.nodes.get('simulate').onclick();
 const p=b.run('project()');assert.equal(p.frames.length,2);assert.equal(p.csv_available,true);assert(p.applied_pose);assert(!('config' in p));
});


test('portable file restores poses, frames, labels and CSV; saving includes complete scene',async()=>{
 const b=boot();await tick();await b.run('loadInfo('+JSON.stringify(b.data)+')');await b.run('placeVehicle(0,0)');await b.nodes.get('simulate').onclick();
 b.nodes.get('timeline').value=1;b.nodes.get('timeline').oninput();
 b.portable.project=JSON.parse(JSON.stringify(b.run('project()')));
 await b.nodes.get('saveProject').onclick();const save=b.calls.find(c=>c.url==='/api/project/export');assert(save);assert.equal(JSON.parse(save.opts.body).project.frames.length,2);
 b.nodes.get('projectFile').files=[{size:100,name:'scene.fusionforce'}];await b.nodes.get('projectFile').onchange();
 assert.equal(b.run('frames.length'),2);assert.equal(b.run('index'),1);assert.equal(b.run('chassis.visible'),true);assert.equal(b.run('playing'),false);assert.equal(b.nodes.get('play').disabled,false);assert.equal(b.nodes.get('downloadCsv').getAttribute('aria-disabled'),'false');assert.equal(b.nodes.get('projectFile').files.length,0);
});


test('route and terrain stay unchanged during a pending operation',async()=>{
 const b=boot();await tick();await b.run('loadInfo('+JSON.stringify(b.data)+')');
 await b.run('placeVehicle(0,0)');
 b.run('busy=true');await b.nodes.get('routeMap').onclick({clientX:100,clientY:100});b.nodes.get('undoWaypoint').onclick();b.nodes.get('clearRoute').onclick();
 assert.equal(b.run('route.length'),1);await b.nodes.get('addZone').onclick();assert.equal(b.run('terrain.zones.length'),0);b.run('busy=false');
});


test('a fixed destination enables simulation until arrival with the vehicle as route start',async()=>{
 const b=boot();await tick();await b.run('loadInfo('+JSON.stringify(b.data)+')');
 await b.run('placeVehicle(0,0)');await b.run('appendWaypointAt([20,0,0])');await b.nodes.get('simulate').onclick();
 const payload=JSON.parse(b.calls.find(c=>c.url==='/api/simulation/start').opts.body);
 assert.equal(payload.until_destination,true);assert.equal(payload.route.length,2);assert.equal(payload.route[0].x,0);assert.equal(payload.route.at(-1).x,20);
});


test('destination progress shows actual elapsed time and remaining distance',async()=>{
 const b=boot();await tick();b.run("showSimulationProgress({stage:'simulation',until_destination:true,fraction:.2,time:8,distance_remaining:4,time_limit:40,processed:4001,total:20001})");
 assert.match(b.nodes.get('progressLabel').textContent,/8,0 s/);assert.match(b.nodes.get('progressLabel').textContent,/4,0 m/);
 assert.match(b.nodes.get('loadProgress').title,/0,01 s/);assert.match(b.nodes.get('progressLabel').textContent,/20 %/);assert.equal(b.nodes.get('loadProgress').value,'20');
});

function stabilityFixture(){
  return [0,1,2].map((time,i)=>({time,position:[i,0,1.2],orientation:[1,0,0,0],points:[],stability:{
    ltr:{value:[0,.25,-.5][i],params:{fz_left_N:1200+i,fz_right_N:1500+i,fz_total_N:2700+2*i},reason:null},
    pltr:{value:i===0?null:[0,-1.2,8][i],params:{horizon_s:.5,ay_dot_m_s3:i?4:null},reason:i===0?'Primera muestra: derivada no disponible.':null},
    zml:{value:[i*.1,-i*.2],params:{z_ref_m:-1.2,fx_N:10,fy_N:20,fz_N:2700,mx_Nm:30,my_Nm:40,mz_Nm:50,torsion_residual_Nm:2},reason:null},
    si:{value:[1,-.4,-3][i],params:{mass_kg:7085,omega_body_x_rad_s:.2,k1:1.15,k2:2.05},reason:null}
  }}));
}

test('stability follows real seek, pause, advance and reset handlers without exposing future samples',async()=>{
  const b=boot();await tick();b.run('frames='+JSON.stringify(stabilityFixture())+';renderFrame(0)');
  assert.equal(b.nodes.get('ltrValue').textContent,'0');assert.equal(b.nodes.get('pltrValue').textContent,'No disponible');
  assert.match(b.nodes.get('pltrReason').textContent,/Primera muestra/);
  assert.equal(b.run("stabilityPlotData('ltr').length"),1);
  b.nodes.get('timeline').value=1;b.nodes.get('timeline').oninput();
  assert.equal(b.run('playing'),false);assert.equal(b.nodes.get('ltrValue').textContent,'0,25');assert.equal(b.nodes.get('siValue').textContent,'-0,4');
  assert.match(b.nodes.get('zmlValue').textContent,/X 0,1 \/ Y -0,2 m/);
  assert.equal(b.run("Math.max(...stabilityPlotData('pltr').map(p=>p[1]??-Infinity))"),-1.2);
  assert.equal(b.run("stabilityPlotData('zml',1).length"),2);
  b.nodes.get('play').onclick();b.run('animate(1001)');
  assert.equal(b.run('index'),2);assert.equal(b.nodes.get('siValue').textContent,'-3');
  b.nodes.get('timeline').value=1;b.nodes.get('timeline').oninput();b.run('animate(9000)');
  assert.equal(b.run('index'),1);assert.equal(b.nodes.get('pltrValue').textContent,'-1,2');
  b.nodes.get('reset').onclick();assert.equal(b.run('index'),0);assert.equal(b.run("stabilityPlotData('si').length"),1);
  assert.equal(b.nodes.get('pltrParams').children.length,4);
  assert.equal(b.nodes.get('pltrParams').children[0].textContent,'Δt · horizonte · s');
  assert.equal(b.nodes.get('pltrParams').children[3].textContent,'No disponible');
});

test('stability remains explicit for legacy frames and invalidates old plots when inputs change',async()=>{
  const b=boot();await tick();
  assert.equal(b.nodes.get('siValue').textContent,'No disponible');assert.match(b.nodes.get('stabilityState').textContent,/Simula/);
  b.run('frames='+JSON.stringify(stabilityFixture())+';renderFrame(2)');
  const plots=b.nodes.get('loadChart').drawCalls.filter(c=>c[0]==='clearRect').length;
  b.run('renderStability();renderStability()');
  assert.equal(b.nodes.get('loadChart').drawCalls.filter(c=>c[0]==='clearRect').length,plots);
  b.run('invalidateResults()');assert.equal(b.nodes.get('ltrValue').textContent,'No disponible');
  assert.equal(b.nodes.get('ltrParams').children.length,0);assert.match(b.nodes.get('stabilityState').textContent,/Simula/);
  b.run('frames=[{time:0,position:[0,0,1],orientation:[1,0,0,0],points:[]}];renderFrame(0)');
  assert.match(b.nodes.get('stabilityState').textContent,/no contiene índices/);assert.equal(b.nodes.get('siValue').textContent,'No disponible');
});

test('chart reduction preserves peaks, negative values and missing-data gaps',async()=>{
  const b=boot();await tick();
  const samples=JSON.parse(b.run('JSON.stringify(stabilityChartSamples([[0,0],[1,8],[2,-7],[3,null],[4,.2],[5,.3]],1))'));
  assert(samples.includes(null));assert(samples.some(p=>p?.[1]===8));assert(samples.some(p=>p?.[1]===-7));
  const gap=samples.indexOf(null);assert(samples.slice(0,gap).every(p=>p[0]<3));assert(samples.slice(gap+1).every(p=>p[0]>3));
  assert.equal(b.run('stabilityNumber(null)'),'No disponible');assert.equal(b.run('stabilityNumber(NaN)'),'No disponible');
});

test('portable replay restores stability values and the selected history prefix',async()=>{
  const b=boot();await tick();await b.run('loadInfo('+JSON.stringify(b.data)+')');await b.run('placeVehicle(0,0)');
  b.run('frames='+JSON.stringify(stabilityFixture())+';renderFrame(1)');
  b.portable.project=JSON.parse(JSON.stringify(b.run('project()')));
  b.nodes.get('projectFile').files=[{size:100,name:'indices.fusionforce'}];await b.nodes.get('projectFile').onchange();
  assert.equal(b.run('index'),1);assert.equal(b.nodes.get('siValue').textContent,'-0,4');
  assert.equal(b.run("stabilityPlotData('ltr').length"),2);assert.equal(b.run('frames[2].stability.pltr.value'),8);
  assert.equal(b.nodes.get('siParams').children[1].textContent,'7085');
});
