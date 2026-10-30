"""Browser control; UART remains owned exclusively by MechanicalOnlyService."""
import json
import secrets
import socket
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

WEB_CONFIG = {"host": "0.0.0.0", "port": 8080, "lease_s": 0.6}


class ControlError(Exception):
    def __init__(self, message, status=409):
        super().__init__(message)
        self.status = status


class BrowserControl:
    def __init__(self, motor, clock=time.monotonic, lease_s=0.6):
        if not 0.2 <= lease_s < motor.c["watchdog_s"]:
            raise ValueError("Browser lease must be shorter than motor watchdog")
        self.motor, self.clock, self.lease_s = motor, clock, lease_s
        self.lock = threading.RLock()
        self.token = None
        self.deadline = 0
        self.seq = 0
        self.hold_start = None
        self.blocked = False
        self.notice = "请连接控制，确认已有机械零点后 ZERO OK → ARM"

    def _stop(self, reason):
        self.motor.request_arm(False)
        self.token = None
        self.hold_start = None
        self.blocked = False
        self.notice = reason

    def poll(self):
        with self.lock:
            if self.token and self.clock() >= self.deadline:
                self._stop("控制连接超时，已请求失能；请重新连接并手动 ARM")

    def state(self):
        with self.lock:
            self.poll()
            value = self.motor.snapshot()
            value.update(connected=bool(self.token), notice=self.notice, hold_blocked=self.blocked)
            return value

    def recheck(self, token):
        with self.lock:
            self.poll()
            if self.token and token != self.token:
                raise ControlError("另一网页正在控制，请先在该网页 DISARM", 403)
            try:
                self.motor.request_recheck()
            except RuntimeError as exc:
                raise ControlError(str(exc))
            self.token = None  # Revoke old queued ARM/movement commands.
            self.hold_start, self.blocked = None, False
            self.notice = "重新检查中，通过后连接控制 → ZERO OK → 手动使能"
            return self.state()

    def connect(self):
        with self.lock:
            self.poll()
            if self.token:
                raise ControlError("已有网页控制，请先断开或等待超时")
            state = self.motor.snapshot()
            if state["fault"] or not state["ready"] or state["armed"] or self.motor.arm_requested:
                raise ControlError(state["fault"] or "请等待电机就绪且失能")
            self.motor.request_arm(False)
            self.motor.input(0, 0)
            self.token = secrets.token_urlsafe(24)
            self.deadline, self.seq = self.clock() + self.lease_s, 0
            self.notice = "控制已连接，当前失能；检查零点后 ZERO OK → ARM"
            return {"token": self.token, "state": self.state()}

    def stop(self, token):
        with self.lock:
            if token != self.token:
                raise ControlError("控制连接已失效", 403)
            self._stop("已请求双轴失能；再次操作请重新连接")
            return self.state()

    def command(self, body):
        with self.lock:
            self.poll()
            if not self.token or body.get("token") != self.token:
                raise ControlError("控制连接已失效，请重新连接", 403)
            seq, x, y, action = (body.get(k) for k in ("seq", "x", "y", "action"))
            if type(seq) is not int or seq != self.seq + 1:
                raise ControlError("控制序号无效")
            if (any(type(v) not in (int, float) or not math.isfinite(v) or abs(v) > 1 for v in (x, y))
                    or math.hypot(x, y) > 1.001 or action not in ("", "zero", "arm", "home")):
                self._stop("无效控制输入，已请求失能")
                raise ControlError("无效控制输入", 400)
            self.seq = seq
            now = self.clock()
            if x == 0 and y == 0:
                self.hold_start, self.blocked = None, False
            else:
                self.hold_start = now if self.hold_start is None else self.hold_start
                if now-self.hold_start >= self.motor.c["max_touch_hold_s"]:
                    self.blocked = True
            if self.blocked:
                x = y = 0
            # ARM/HOME must start with released controls. Never queue movement with ARM.
            if action and (x or y):
                self._stop("按钮操作需先松开摇杆／方向键")
                raise ControlError(self.notice, 400)
            self.motor.input(x, y)
            try:
                if action == "zero":
                    self.motor.confirm_reference()
                    self.notice = "已确认现有机械零点（没有重新标定）"
                elif action == "arm":
                    self.motor.request_arm(True)
                    self.notice = "已请求在当前实测位置使能"
                elif action == "home":
                    self.motor.request_home()
                    self.notice = "正在返回机械原点；完成后保持使能"
            except RuntimeError as exc:
                self.notice = str(exc)
                self._stop(self.notice)
                raise ControlError(str(exc))
            self.deadline = now + self.lease_s
            return self.state()


def make_http_server(control, address):
    class Handler(BaseHTTPRequestHandler):
        def setup(self):
            super().setup()
            self.connection.settimeout(0.3)

        def log_message(self, *args):
            pass

        def reply(self, status, body, content_type="application/json; charset=utf-8"):
            data = body.encode("utf-8") if isinstance(body, str) else json.dumps(body, allow_nan=False).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Frame-Options", "DENY")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.end_headers()
            try:
                self.wfile.write(data)
            except (BrokenPipeError, ConnectionResetError, socket.timeout):
                pass

        def do_GET(self):
            if self.path == "/":
                self.reply(200, WEB_HTML, "text/html; charset=utf-8")
            elif self.path == "/api/state":
                self.reply(200, control.state())
            else:
                self.reply(404, {"error": "Not found"})

        def do_POST(self):
            try:
                # Same-origin JSON only; no CORS, file serving, shell or raw serial API.
                if self.headers.get("Origin") != "http://" + self.headers.get("Host", ""):
                    raise ControlError("Origin rejected", 403)
                if self.headers.get_content_type() != "application/json":
                    raise ControlError("JSON required", 415)
                if self.headers.get("Transfer-Encoding"):
                    raise ControlError("Chunked requests unsupported", 400)
                size = int(self.headers.get("Content-Length", "0"))
                if not 1 <= size <= 1024:
                    raise ControlError("Invalid body size", 400)
                body = json.loads(self.rfile.read(size))
                if not isinstance(body, dict):
                    raise ControlError("JSON object required", 400)
                if self.path == "/api/connect":
                    value = control.connect()
                elif self.path == "/api/control":
                    value = control.command(body)
                elif self.path == "/api/stop":
                    value = control.stop(body.get("token"))
                elif self.path == "/api/recheck":
                    value = control.recheck(body.get("token"))
                else:
                    raise ControlError("Not found", 404)
                self.reply(200, value)
            except ControlError as exc:
                self.reply(exc.status, {"error": str(exc)})
            except (ValueError, TypeError, socket.timeout):
                self.reply(400, {"error": "Invalid JSON request"})

    server = ThreadingHTTPServer(address, Handler)
    server.daemon_threads = True
    return server


WEB_HTML = r'''<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>F32C 电脑控制</title>
<style>
*{box-sizing:border-box}body{margin:0;background:#0b1420;color:#e9f1f8;font:16px system-ui,sans-serif}
main{max-width:900px;margin:auto;padding:28px}h1{font-size:26px;margin-bottom:4px}.muted{color:#9cacbd}
.panel{border:1px solid #2d4259;border-radius:16px;background:#131f2e;padding:22px;margin-top:18px}
.row{display:flex;gap:26px;align-items:center;flex-wrap:wrap}button{font:inherit;cursor:pointer;border:0;border-radius:9px;padding:13px 20px;background:#285467;color:white;margin:5px 6px 5px 0}
button:disabled{opacity:.4;cursor:default}.stop{background:#b23341}.big{font-size:32px;font-variant-numeric:tabular-nums}
#pad{width:260px;height:260px;position:relative;border:3px solid #526e88;border-radius:50%;touch-action:none;user-select:none;background:linear-gradient(transparent 49.7%,#304052 50%,transparent 50.3%),linear-gradient(90deg,transparent 49.7%,#304052 50%,transparent 50.3%)}
#knob{position:absolute;width:48px;height:48px;border-radius:50%;background:#28c8bf;left:103px;top:103px;pointer-events:none}
#notice{color:#ffd18a;white-space:pre-wrap;overflow-wrap:anywhere}kbd{background:#2b4056;padding:2px 7px;border-radius:4px}
</style><main><h1>F32C · 电脑控制</h1><div class="muted">MECH ONLY · 机械角度模式</div>
<p id="mountInfo" class="muted">倒装：左右反向 · PITCH −100°～+30° · 平滑位置控制</p>
<p id="speedInfo" class="muted">目标速度上限：YAW 24°/s · PITCH 16°/s</p>
<div class="panel"><div class="row"><div><span id="link">未连接控制</span> · <b id="phase">--</b></div><button id="connect">连接控制</button><button id="recheck">重新检查状态</button><button id="stop" class="stop">DISARM / 失能</button></div>
<div id="notice">加载状态…</div></div>
<div class="panel row"><div><div class="muted">↑ PITCH · YAW →</div><div id="pad" tabindex="0" aria-label="按住拖动摇杆"><div id="knob"></div></div></div><div>
<div class="muted">YAW / 水平角度</div><div class="big" id="yaw">--°</div><br>
<div class="muted">PITCH / 俯仰角度</div><div class="big" id="pitch">--°</div><p id="feedback" class="muted">--</p>
<p id="mechanical" class="muted">机械反馈：--</p>
<p id="zeroState">机械零点未确认</p><button id="zero">ZERO OK</button><button id="arm">手动使能 / ARM</button><button id="home">HOME / 回原点</button>
<p id="armReason" class="muted"></p></div></div>
<p>按住摇杆或 <kbd>↑</kbd> <kbd>↓</kbd> <kbd>←</kbd> <kbd>→</kbd> 转动；松开保持位置。<kbd>空格</kbd> 失能。</p>
<p class="muted">单次连续移动最多 8 秒，达到后松开再按。切到后台或失去焦点会请求失能。HOME 完成后保持使能，累计角度不参与控制。调整后可在失能状态点“重新检查状态”。</p></main>
<script>
'use strict';
const $=id=>document.getElementById(id), keys=new Set();
let token=null,seq=0,vector=[0,0],pointer=null,pending='',busy=false,lastState=null;
function release(){vector=[0,0];keys.clear();pointer=null;draw();}
function draw(){ $('knob').style.transform=`translate(${vector[0]*96}px,${-vector[1]*96}px)`; }
function render(s){lastState=s;$('phase').textContent=s.phase;$('link').textContent=token?'控制已连接':s.connected?'另一网页正在控制':'未连接控制';
 if(s.limits&&s.joystick_signs)$('mountInfo').textContent=`左右${s.joystick_signs[0]<0?'反向':'正常'} · PITCH ${s.limits[1][0]}°～${s.limits[1][1]}° · 平滑位置控制`;
 if(s.max_rate&&s.home_rate)$('speedInfo').textContent=`手动目标上限 YAW ${s.max_rate[0]}°/s · PITCH ${s.max_rate[1]}°/s；HOME ${s.home_rate[0]} / ${s.home_rate[1]}°/s`;
 const valid=s.reference_valid&&s.control_feedback_seen!==false;
 const degree=v=>Number.isFinite(v)?v.toFixed(1)+'°':'--';
 $('yaw').textContent=valid?degree(s.angles[0]):'--';$('pitch').textContent=valid?degree(s.angles[1]):'--';
 const age=Number.isFinite(s.age)?s.age.toFixed(2)+'s':'尚无有效控制反馈';
 $('feedback').textContent=`控制反馈：${age} · REF ${s.reference_valid?'VALID':'INVALID'}`;
 const ma=s.mechanical_angles||[], da=s.mechanical_feedback_age||[];
 const diag=i=>`${degree(ma[i])}（${Number.isFinite(da[i])?da[i].toFixed(2)+'s 前':'尚未收到'}）`;
 $('mechanical').textContent=`诊断机械反馈 YAW ${diag(0)} / PITCH ${diag(1)}`;
 $('zeroState').textContent=s.zero_confirmed?'机械零点已由用户确认':'机械零点未确认';
 const checking=['RECHECK_PENDING','RECHECKING'].includes(s.phase);
 $('notice').textContent=checking?'正在重新检查角度、限位与静止状态，请保持轴静止':s.fault|| (s.hold_blocked?'已达到连续移动上限，请松开后再按':s.phase==='HOME_HOLD'?'已回机械原点，保持使能；可继续摇杆控制，DISARM 请求失能':s.recheck_status==='PASSED'&&!token?'重新检查通过，当前失能；连接控制 → ZERO OK → 手动使能':s.notice);
 const usable=!!token&&s.ready&&!s.fault;
 $('connect').disabled=!!token||s.connected||!s.ready||!!s.fault;
 $('recheck').disabled=!s.can_recheck||(s.connected&&!token);
 $('zero').disabled=!usable||s.armed;$('arm').disabled=!usable||s.armed||!s.zero_confirmed;
 $('home').disabled=!usable||!s.armed||s.home_requested;
 $('armReason').textContent=checking?'重新检查不会自动使能，也不会改机械零点':s.fault?'使能已阻止：'+s.fault+(s.fault.includes('after disable')?'。失能后仍检测到运动／偏离零位，请支撑并检查负载，静止后点“重新检查状态”。':'。排查并移回允许范围后，点“重新检查状态”。'):
  !s.ready?'等待电机就绪':!token?'先连接控制':s.armed?'已使能；DISARM / 空格请求失能':
  !s.zero_confirmed?'确认已有机械零点正确，再点 ZERO OK':'当前失能；点击“手动使能 / ARM”后才会使能';
}
async function post(path,body){const ctl=new AbortController(),timer=setTimeout(()=>ctl.abort(),450);
 try{const r=await fetch(path,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body),signal:ctl.signal,cache:'no-store'});
 const j=await r.json();if(!r.ok)throw Error(j.error||r.status);return j;}finally{clearTimeout(timer);}}
function stop(){const old=token;token=null;pending='';release();navigator.sendBeacon('/api/stop',new Blob([JSON.stringify({token:old})],{type:'application/json'}));
 $('link').textContent='已请求失能';}
$('connect').onclick=async()=>{release();try{const j=await post('/api/connect',{});token=j.token;seq=0;render(j.state);}catch(e){$('notice').textContent=e.message;}};
$('recheck').onclick=async()=>{const old=token;release();pending='';try{const s=await post('/api/recheck',{token:old});token=null;seq=0;render(s);}catch(e){$('notice').textContent=e.message;}};
$('stop').onclick=stop;
for(const action of ['zero','arm','home'])$(action).onclick=()=>{release();pending=action;};
const pad=$('pad');
function point(e){const r=pad.getBoundingClientRect();let x=(e.clientX-r.left-r.width/2)/(r.width*.4),y=(r.top+r.height/2-e.clientY)/(r.height*.4);let d=Math.hypot(x,y);
 if(d<.15)vector=[0,0];else{const a=Math.min(1,(d-.15)/.85);vector=[x/d*a,y/d*a];}draw();}
pad.onpointerdown=e=>{if(!token||pointer!==null)return;e.preventDefault();keys.clear();pointer=e.pointerId;pad.setPointerCapture(pointer);point(e);};
pad.onpointermove=e=>{if(e.pointerId===pointer)point(e);};
pad.onpointerup=pad.onpointercancel=pad.onlostpointercapture=e=>{if(e.pointerId===pointer)release();};
function keyVector(){let x=Number(keys.has('ArrowRight'))-Number(keys.has('ArrowLeft')),y=Number(keys.has('ArrowUp'))-Number(keys.has('ArrowDown')),n=Math.max(1,Math.hypot(x,y));vector=[x/n,y/n];draw();}
window.addEventListener('keydown',e=>{if(e.code==='Space'){e.preventDefault();stop();return;}if(!e.key.startsWith('Arrow'))return;e.preventDefault();if(token&&pointer===null){keys.add(e.key);keyVector();}});
window.addEventListener('keyup',e=>{if(keys.delete(e.key))keyVector();});
window.addEventListener('blur',stop);window.addEventListener('pagehide',stop);
document.addEventListener('visibilitychange',()=>{if(document.hidden)stop();});
async function tick(){if(busy)return;busy=true;const current=token;try{if(current){const action=pending;pending='';const v=action?[0,0]:vector.slice();const s=await post('/api/control',{token:current,seq:++seq,x:v[0],y:v[1],action});if(token===current)render(s);}
 else{const r=await fetch('/api/state',{cache:'no-store',signal:AbortSignal.timeout(450)});render(await r.json());}}
 catch(e){if(current&&token===current)stop();$('notice').textContent='连接失败 / '+e.message+'；请重新连接';}finally{busy=false;setTimeout(tick,80);}}
tick();
</script></html>'''


def web_main():
    from maix import app, sys as maix_sys
    if str(maix_sys.device_id()).lower() != "maixcam2":
        raise RuntimeError("This UART3 mapping requires MaixCAM2")
    validate_config(CONFIG)
    motor = MechanicalOnlyService(dict(CONFIG))
    control = BrowserControl(motor, lease_s=WEB_CONFIG["lease_s"])
    server = None
    thread = None
    try:
        # Bind before UART startup: a port conflict must not leave hardware running.
        server = make_http_server(control, (WEB_CONFIG["host"], WEB_CONFIG["port"]))
        motor.start()
        thread = threading.Thread(target=server.serve_forever, kwargs={"poll_interval": 0.05}, daemon=True)
        thread.start()
        print("F32C WEB | MECH ONLY | UART3 B2/B3 115200 | START DISABLED", flush=True)
        print("MOTOR_MOUNT yaw_input=%d pitch_limits=%s mode=%d position_period=%.3fs" % (
            CONFIG["joystick_signs"][0], CONFIG["limits"][1], CONFIG["position_mode"],
            CONFIG["position_period_s"]), flush=True)
        addresses = []
        try:
            import psutil
            addresses = [a.address for items in psutil.net_if_addrs().values() for a in items
                         if a.family == socket.AF_INET and not a.address.startswith("127.")]
        except Exception:
            pass
        for ip in sorted(set(addresses)):
            print("COMPUTER OPEN: http://%s:%d" % (ip, WEB_CONFIG["port"]), flush=True)
        print("同一网络电脑打开 http://相机IP:%d；使用 MaixVision 连接的相机 IP。" % WEB_CONFIG["port"], flush=True)
        last_log = 0
        while not app.need_exit():
            control.poll()  # Never synthesizes browser heartbeat.
            if time.monotonic()-last_log >= 1:
                print("MOTOR_WEB " + json.dumps(control.state()), flush=True)
                last_log = time.monotonic()
            time.sleep(0.02)
    finally:
        motor.request_arm(False)
        motor.close()
        if server is not None:
            if thread is not None:
                server.shutdown()
                thread.join(1)
            server.server_close()


if __name__ == "__main__":
    web_main()
