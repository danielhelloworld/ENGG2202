// Browser event model; no camera, motor, NPU or actual browser access.
const fs=require('fs'),path=require('path'),vm=require('vm'),assert=require('assert');
const html=fs.readFileSync(path.join(__dirname,'../src/web.html'),'utf8');
const js=html.split('<script>')[1].split('</script>')[0];
const nodes={},listeners={},requests=[],beacons=[];let focused=true,loseFocusOnConnect=false;
let state={ready:true,fault:null,armed:true,angles:[0,0],phase:'HOME_HOLD',age:.01,
 reference_valid:true,zero_confirmed:true,connected:false,home_requested:false,
 vision_state:'DETECT',selected:null,c_next_home:false,control_owner:'WEB',notice:'',can_recheck:false};
const node=id=>nodes[id]||=(id==='video'?{
 getBoundingClientRect:()=>({left:10,top:20,width:640,height:480}),
 set src(v){this.url=v;setImmediate(()=>this.onload&&this.onload());}
}:{style:{},getBoundingClientRect:()=>({left:0,top:0,width:260,height:260}),setPointerCapture(){}});
const add=(e,f)=>(listeners[e]||=[]).push(f),emit=(e,data)=>{for(const f of listeners[e]||[])f(data);};
const context=vm.createContext({document:{getElementById:node,hidden:false,hasFocus:()=>focused,addEventListener:add},
 window:{addEventListener:add},navigator:{sendBeacon:(p,b)=>beacons.push([p,b])},
 URL:{createObjectURL:()=> 'blob:test',revokeObjectURL(){}},Blob,AbortController,AbortSignal,
 setTimeout:()=>1,clearTimeout(){},console,
 fetch:async(p,opt)=>{requests.push([p,opt]);let result=state;
  if(p==='/api/connect'){result={token:'session',state};if(loseFocusOnConnect)focused=false;}
  if(p==='/api/control'){
   const b=JSON.parse(opt.body);
   if(b.action==='select')state={...state,selected:[0,1],vision_state:'TRACK',c_next_home:false};
   if(b.action==='c')state={...state,selected:null,c_next_home:!state.c_next_home};
   result=state;
  }
  return {ok:true,headers:{get:()=> '42'},blob:async()=>new Blob(['jpeg']),json:async()=>result};
 }});
vm.runInContext(js,context);
const run=s=>vm.runInContext(s,context), event=(key,code,repeat=false)=>({key,code,repeat,preventDefault(){}});
(async()=>{
 await new Promise(r=>setImmediate(r));await new Promise(r=>setImmediate(r));
 assert.equal(run('displayedFrame'),42);
 emit('blur');assert.equal(beacons.length,0);  // A viewer must not cancel local startup.
 assert.equal(node('connect').disabled,false);
 await node('connect').onclick();
 node('video').onclick({clientX:410,clientY:200});
 assert.equal(run('pending'),'select');
 await run('tick()');
 let sent=JSON.parse(requests.filter(r=>r[0]==='/api/control').at(-1)[1].body);
 assert.equal(sent.frame_id,42);assert.equal(sent.nx,.625);assert.equal(sent.ny,.375);
 emit('keydown',event('c','KeyC'));await run('tick()');
 assert.equal(node('clear').textContent,'C：回原点');
 emit('keydown',event('c','KeyC',true));assert.equal(run('pending'),'');
 emit('keydown',event('c','KeyC'));await run('tick()');
 assert.equal(node('clear').textContent,'C：清除目标');
 emit('keydown',event('ArrowRight','ArrowRight'));assert.equal(run('vector[0]'),1);
 emit('keyup',event('ArrowRight','ArrowRight'));assert.equal(run('vector[0]'),0);
 emit('keydown',{...event('ArrowRight','ArrowRight'),target:{tagName:'INPUT'}});assert.equal(run('vector[0]'),0);
 emit('keydown',{...event('c','KeyC'),ctrlKey:true});assert.equal(run('pending'),'');
 emit('keydown',event('ArrowLeft','ArrowLeft'));assert.equal(run('vector[0]'),-1);
 focused=false;emit('blur');assert.equal(run('token'),null);
 assert.equal(run('vector[0]'),0);assert.equal(run('keys.size'),0);
 assert.equal(beacons.at(-1)[0],'/api/background');
 assert.equal(JSON.parse(await beacons.at(-1)[1].text()).token,'session');
 state={...state,control_owner:'LOCAL',connected:false};focused=true;await run('tick()');
 assert.equal(run('token'),'session');assert.equal(run('resumeOnFocus'),false);
 context.document.hidden=true;emit('visibilitychange');assert.equal(run('token'),null);
 assert.equal(beacons.at(-1)[0],'/api/background');
 context.document.hidden=false;await run('tick()');assert.equal(run('token'),'session');
 context.armingState={...state,control_owner:'WEB',arm_homing:true,home_requested:true,startup_auto_home:true,max_rate:[240,180],tracking_rate:[150,120],home_rate:[108,90],limits:[[-150,150],[-100,30]],joystick_signs:[-1,-1],performance:{inference_fps_actual:25.5,capture_to_result_ms:60}};
 run("token='session';render(armingState)");
 assert(node('notice').textContent.includes('正在从当前位置'));
 assert(node('zeroState').textContent.includes('驱动已保存'));
 assert.equal(node('connect').disabled,true);
 assert(node('mountInfo').textContent.includes('上下反向'));
 assert(node('speedInfo').textContent.includes('摇杆 YAW 240°/s'));
 assert(node('speedInfo').textContent.includes('跟踪 150 / 120°/s'));
 assert(node('speedInfo').textContent.includes('HOME 108 / 90°/s'));
 assert(node('visionInfo').textContent.includes('25.5fps'));assert(node('visionInfo').textContent.includes('60ms'));
 node('video').onclick({clientX:410,clientY:200});
 assert.equal(run('pending'),'');
 assert(node('notice').textContent.includes('等待回零完成'));
 context.idleState={...context.armingState,armed:false,arm_homing:false,home_requested:false,phase:'DISABLED'};
 run('render(idleState)');
 assert.equal(node('arm').disabled,false);
 assert.equal(node('home').disabled,true);
 assert(node('armReason').textContent.includes('任意位置 ARM'));
 node('arm').onclick();await run('tick()');
 assert.equal(JSON.parse(requests.filter(r=>r[0]==='/api/control').at(-1)[1].body).action,'arm');
 node('home').onclick();await run('tick()');
 assert.equal(JSON.parse(requests.filter(r=>r[0]==='/api/control').at(-1)[1].body).action,'home');
 context.responseState={...state,armed:true,feedback_degraded:true,feedback_loss_grace_s:.1,default_saved_zero:true,zero_confirmed:true,response:{error_deg:[1,-.2],software_accel_deg_s2:[70,60],drive_scale:[1.1,.9],status:['LAG_COMPENSATING','DAMPING']}};
 run('render(responseState)');assert(node('responseInfo').textContent.includes('1.0 / -0.2'));
 assert(node('notice').textContent.includes('100ms'));assert(node('zeroState').textContent.includes('默认使用'));
 node('stop').onclick();assert.equal(beacons.at(-1)[0],'/api/stop');assert.equal(run('resumeOnFocus'),false);
 loseFocusOnConnect=true;await node('connect').onclick();
 assert.equal(run('token'),null);assert.equal(run('resumeOnFocus'),true);
 assert.equal(beacons.at(-1)[0],'/api/background');
 node('stop').onclick();assert.equal(JSON.parse(await beacons.at(-1)[1].text()).token,'session');
 console.log('PASS: displayed-frame clicks, two C presses, editing/modifier filtering, blur/hidden release, automatic reconnect, explicit DISARM');
 console.log('PASS: inverted pitch, speed display, HOME click barrier, separate ARM/HOME actions');
})().catch(e=>{console.error(e);process.exitCode=1;});
