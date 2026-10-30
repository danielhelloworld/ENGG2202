// Browser event smoke test with a fake DOM/network. No hardware or real browser.
const fs = require('fs'), vm = require('vm'), assert = require('assert');
const path = require('path');
const source = fs.readFileSync(path.join(__dirname, '../src/web.py'), 'utf8');
const js = source.split('<script>')[1].split('</script>')[0];
const nodes = {}, events = {}, beacons = [], requests = [];
const state = {phase:'DISABLED', connected:false, angles:[12,-6], age:.01, reference_valid:true,
  zero_confirmed:false, ready:true, armed:false, fault:null, home_requested:false, notice:'ready'};
const node = id => nodes[id] ||= {style:{}, getBoundingClientRect:()=>({left:0, top:0, width:260, height:260}), setPointerCapture:()=>{}};
const context = vm.createContext({document:{getElementById:node, addEventListener:(e,f)=>events[e]=f, hidden:false},
  window:{addEventListener:(e,f)=>events[e]=f}, navigator:{sendBeacon:(p,b)=>beacons.push([p,b])},
  Blob, AbortController, AbortSignal, console, setTimeout:()=>1, clearTimeout:()=>{},
  fetch:async(p, options)=>{requests.push([p,options]);return {ok:true,json:async()=>p==='/api/connect'?{token:'demo-token',state}:p==='/api/recheck'?{...state,phase:'RECHECK_PENDING',ready:false,recheck_status:'PENDING',can_recheck:false}:state};}});
vm.runInContext(js, context);
const get = expr => vm.runInContext(expr, context);
const event = (x,y,id=1)=>({clientX:x, clientY:y, pointerId:id, preventDefault(){}});
(async()=>{
  await new Promise(resolve=>setImmediate(resolve));
  await node('connect').onclick();
  assert.equal(get('token'),'demo-token');
  node('pad').onpointerdown(event(234,130));
  assert(get('vector[0]')>.99);
  node('pad').onpointerup(event(234,130));
  assert.equal(get('vector[0]'),0);
  events.keydown({key:'ArrowUp',code:'ArrowUp',preventDefault(){}});
  assert.equal(get('vector[1]'),1);
  events.keyup({key:'ArrowUp'});
  assert.equal(get('vector[1]'),0);
  node('home').onclick();
  assert.equal(get('pending'),'home');
  await get('tick()');
  const sent=JSON.parse(requests.filter(r=>r[0]==='/api/control').at(-1)[1].body);
  assert.equal(sent.action,'home');assert.equal(sent.x,0);assert.equal(sent.y,0);
  events.blur();
  assert.equal(get('token'),null);assert.equal(beacons[0][0],'/api/stop');
  assert.equal(JSON.parse(await beacons[0][1].text()).token,'demo-token');
  assert.equal(get('pending'),'');
  context.faultState={...state, fault:'PITCH ID=2 116.2 deg, allowed -100..100', ready:false,
    reference_valid:false, control_feedback_seen:false, age:null,
    mechanical_angles:[7.7,116.2], mechanical_feedback_age:[.1,.2]};
  get('render(faultState)');
  assert.equal(node('yaw').textContent,'--');assert.equal(node('pitch').textContent,'--');
  assert(node('mechanical').textContent.includes('116.2°'));
  assert(node('feedback').textContent.includes('尚无有效控制反馈'));
  assert(node('armReason').textContent.includes('116.2'));
  assert.equal(node('arm').disabled,true);
  context.mountState={...state, limits:[[-150,150],[-100,30]], joystick_signs:[-1,1],max_rate:[24,16],home_rate:[10,8]};
  get('render(mountState)');
  assert(node('mountInfo').textContent.includes('左右反向'));
  assert(node('mountInfo').textContent.includes('-100°～30°'));
  assert(node('speedInfo').textContent.includes('YAW 24°/s'));
  context.disableFault={...state,fault:'Axis moving after disable: ID=2 speed=1 RPM',ready:false};
  get('render(disableFault)');
  assert(node('armReason').textContent.includes('支撑并检查负载'));
  assert(!node('armReason').textContent.includes('角度越界'));
  context.heldState={...state, phase:'HOME_HOLD', armed:true,can_recheck:false};
  get("token='demo-token';render(heldState)");
  assert.equal(node('arm').disabled,true);
  assert.equal(node('recheck').disabled,true);
  context.idleState={...state,can_recheck:true};
  get('render(idleState)');
  assert.equal(node('recheck').disabled,false);
  await node('recheck').onclick();
  assert.equal(get('token'),null);
  assert.equal(get('pending'),'');
  assert(node('notice').textContent.includes('正在重新检查'));
  assert.equal(JSON.parse(requests.filter(r=>r[0]==='/api/recheck').at(-1)[1].body).token,'demo-token');
  node('stop').onclick();
  assert.equal(JSON.parse(await beacons.at(-1)[1].text()).token,null);
  console.log('PASS: connect, pointer move/release, arrow keys, HOME request, blur disarm');
  console.log('PASS: startup limit fault shows actual diagnostic angles, no fake zero/age, manual ARM blocked');
  console.log('PASS: HOME holding, recheck button/request, token revocation, cancel without a control session');
})().catch(e=>{console.error(e);process.exitCode=1;});
