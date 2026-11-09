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
        self.tuning_error = None
        self.notice = "请连接控制，确认已有机械零点后 ZERO OK → ARM"

    def _stop(self, reason):
        self._disable_request(reason)
        self.token = None
        self.hold_start = None
        self.blocked = False
        self.notice = reason

    def _disable_request(self, reason):
        self.motor.request_arm(False)

    def _reject_command(self, reason, status=409):
        self._stop(reason)
        raise ControlError(reason, status)

    def poll(self):
        with self.lock:
            if self.token and self.clock() >= self.deadline:
                if getattr(self.motor, "autotune_active", False):
                    self.tuning_background(self.token)
                else:
                    self._stop("控制连接超时，已请求失能；请重新连接并手动 ARM")

    def tuning_background(self, token):
        with self.lock:
            if not token or token != self.token:
                raise ControlError("控制连接已失效", 403)
            if not getattr(self.motor, "autotune_active", False):
                return self.stop(token)
            self.motor.pi_autotune.detached = True
            self.motor.input(0, 0)
            self.token = None
            self.notice = "板端继续 PI 自动整定，完成后保持使能"
            return self.state()

    def state(self):
        with self.lock:
            self.poll()
            value = self.motor.snapshot()
            value.update(connected=bool(self.token), notice=self.notice, hold_blocked=self.blocked,
                         tuning_error=self.tuning_error)
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
            tune_takeover = bool(state.get("autotune_takeover"))
            takeover = bool(state.get("startup_takeover")) or tune_takeover
            if state["fault"] or not state["ready"] or ((state["armed"] or self.motor.arm_requested) and not takeover):
                raise ControlError(state["fault"] or "请等待电机就绪且失能")
            if takeover:
                if tune_takeover:
                    self.motor.pi_autotune.detached = False
                elif not self.motor.claim_startup_hold():
                    raise ControlError("启动保持状态已改变，请刷新状态")
            else:
                self.motor.request_arm(False)
            self.motor.input(0, 0)
            self.tuning_error = None
            self.token = secrets.token_urlsafe(24)
            self.deadline, self.seq = self.clock() + self.lease_s, 0
            self.notice = "控制已连接，继续保持机械原点" if takeover else "控制已连接，当前失能；检查零点后 ZERO OK → ARM"
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
                    or math.hypot(x, y) > 1.001 or action not in ("", "zero", "arm", "home", "pi", "record_start", "record_stop", "pi_autotune", "pi_autotune_cancel", "pi_autotune_save")):
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
                self._reject_command("按钮操作需先松开摇杆／方向键", 400)
            tuning_action = action in ("pi", "record_start", "record_stop", "pi_autotune", "pi_autotune_cancel", "pi_autotune_save")
            if tuning_action:
                self.tuning_error = None
            try:
                self.motor.input(x, y)
                if action == "zero":
                    self.motor.confirm_reference()
                    self.notice = "已确认现有机械零点（没有重新标定）"
                elif action == "arm":
                    self.motor.request_arm(True)
                    self.notice = "已请求在当前实测位置使能"
                elif action == "home":
                    self.motor.request_home()
                    self.notice = "正在返回机械原点；完成后保持使能"
                elif action == "pi":
                    self.motor.request_speed_pi(body.get("axis"), body.get("kp"), body.get("ki"))
                    self.notice = "已提交临时 PI 请求；等待串口线程静止检查"
                elif action == "record_start":
                    self.motor.start_tuning_capture(body.get("note", ""))
                    self.notice = "记录已开始；此操作不发运动指令"
                elif action == "record_stop":
                    self.motor.stop_tuning_capture()
                    self.notice = "记录已结束，可以导出实测数据"
                elif action == "pi_autotune":
                    self.motor.request_pi_autotune(body.get("axis"), body.get("kp"), body.get("ki"))
                    self.notice = "已提交 PI 自动整定，基线取自填写的 Kp/Ki"
                elif action == "pi_autotune_cancel":
                    self.motor.cancel_pi_autotune()
                    self.notice = "正在回到测试起点并恢复已知 PI 基线"
                elif action == "pi_autotune_save":
                    self.notice = self.motor.pi_autotune.save()
            except (RuntimeError, ValueError) as exc:
                if tuning_action:
                    self.tuning_error = str(exc)
                    self.deadline = now+self.lease_s
                    return self.state()  # Invalid tuning input must not drop holding torque.
                self._reject_command(str(exc))
            self.deadline = now + self.lease_s
            return self.state()


def make_http_server(control, address):
    class Handler(BaseHTTPRequestHandler):
        def setup(self):
            super().setup()
            self.connection.settimeout(0.3)

        def log_message(self, *args):
            pass

        def reply(self, status, body, content_type="application/json; charset=utf-8", filename=None):
            data = body.encode("utf-8") if isinstance(body, str) else json.dumps(body, allow_nan=False).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Frame-Options", "DENY")
            self.send_header("X-Content-Type-Options", "nosniff")
            if filename:
                self.send_header("Content-Disposition", 'attachment; filename="%s"' % filename)
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
            elif self.path == "/api/tuning/export":
                try:
                    self.reply(200, control.motor.export_tuning(), filename="f32c-tuning.json")
                except ValueError as exc:
                    self.reply(409, {"error": str(exc)})
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
                elif self.path == "/api/tuning/background":
                    value = control.tuning_background(body.get("token"))
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
input,select{font:inherit;color:#e9f1f8;background:#0b1420;border:1px solid #526e88;border-radius:6px;padding:8px;margin:4px;max-width:100%}input[type=number]{width:95px}#note{width:100%}.tune-row{display:flex;gap:8px;align-items:center;flex-wrap:wrap}#trace{width:100%;height:210px;background:#0b1420;border-radius:8px;margin-top:10px}#piStatus,#recordStatus,#metrics{white-space:pre-wrap;overflow-wrap:anywhere}.download{color:#5bd8d0;display:inline-block;padding:12px}
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
<p class="muted">单次连续移动最多 8 秒，达到后松开再按。切到后台或失去焦点会请求失能。HOME 完成后保持使能，累计角度不参与控制。调整后可在失能状态点“重新检查状态”。</p>
<section class="panel" id="tuningPanel">
<h2 style="margin-top:0;font-size:21px">负载与 PI 调试</h2>
<p class="muted">启动后自动快速回已保存的机械零点并保持使能。启动、手动、HOME 的两轴驱动配置独立。失能时相机会下垂，请支撑俯仰负载。</p>
<p id="profileInfo" class="muted">启动 / 手动 / HOME 使用独立的两轴驱动配置。</p>
<div class="tune-row"><label>轴 <select id="tuneAxis"><option value="0">X / YAW</option><option value="1">Y / PITCH</option></select></label><label>Kp <input id="kp" type="number" min="0" max="256" step="1" placeholder="留空不改"></label><label>Ki <input id="ki" type="number" min="0" max="256" step="1" placeholder="留空不改"></label><button id="applyPi" disabled>原地保持时临时发送 PI</button></div>
<p id="piStatus" class="muted">PI 未读取，输入框用于填写已知基线。</p>
<p>PI 自动整定会围绕当前位置做 +1°、返回、−1°、返回的单轴测试。开始前填写已知 Kp 和 Ki 基线。</p>
<button id="piAutoStart">开始 PI 自动整定</button><button id="piAutoCancel">取消并恢复基线</button><button id="piAutoSave">保存验证配置</button><p id="piAutoStatus"></p>
<label class="muted" for="note">本轮备注（负载、姿态、参数来源）</label><input id="note" maxlength="200" placeholder="例如：带相机，PITCH -20°，原参数未知，原地保持">
<div class="tune-row"><button id="record" disabled>开始记录</button><a class="download" href="/api/tuning/export" download="f32c-tuning.json">导出本轮 JSON</a></div>
<div id="recordStatus" class="muted">先记录原参数下的保持状态，记录按钮不会自动试转。</div>
<canvas id="trace" width="900" height="240" aria-label="所选轴目标角度与实测角度曲线"></canvas>
<div class="muted">青色：实测机械角度 · 黄色：已发送目标 · 图为网页采样，导出保留串口线程的原始采样时间。</div>
<p id="metrics" class="muted"></p><p class="muted">最多记录 60 秒。角度分辨率 0.1°，采样曲线不能排除高频振动。PI 可在原地保持使能时修改，无需每轮失能；留空的参数不发送。</p>
</section></main>
<script>
'use strict';
const $=id=>document.getElementById(id), keys=new Set();
let token=null,seq=0,vector=[0,0],pointer=null,pending='',pendingTune={},busy=false,lastState=null;
const traceRows=[];
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
 $('zeroState').textContent=s.zero_confirmed?(['HOMING','HOME_HOLD'].includes(s.tuning?.startup_status)?'采用已保存的机械零点':'机械零点已确认'):'机械零点未确认';
 const checking=['RECHECK_PENDING','RECHECKING'].includes(s.phase);
 $('notice').textContent=checking?'正在重新检查角度、限位与静止状态，请保持轴静止':s.fault|| (s.hold_blocked?'已达到连续移动上限，请松开后再按':s.phase==='HOME_HOLD'?'已回机械原点，保持使能；可继续摇杆控制，DISARM 请求失能':s.recheck_status==='PASSED'&&!token?'重新检查通过，当前失能；连接控制 → ZERO OK → 手动使能':s.notice);
 const usable=!!token&&s.ready&&!s.fault;
 $('connect').disabled=!!token||s.connected||!s.ready||!!s.fault||(s.armed&&!s.startup_takeover&&!s.autotune_takeover);
 $('recheck').disabled=!s.can_recheck||(s.connected&&!token);
 $('zero').disabled=!usable||s.armed||s.arm_requested;$('arm').disabled=!usable||s.armed||s.arm_requested||!s.zero_confirmed;
 $('home').disabled=!usable||!s.armed||s.home_requested;
 $('armReason').textContent=s.phase==='ARMING_HOLD'?'启动保持中：已请求使能并保持当前姿态，请保持摇杆松开':checking?'重新检查不会自动使能，也不会改机械零点':s.fault?'使能已阻止：'+s.fault+(s.fault.includes('after disable')?'。失能后仍检测到运动／偏离零位，请支撑并检查负载，静止后点“重新检查状态”。':'。排查并移回允许范围后，点“重新检查状态”。'):
  !s.ready?'等待电机就绪':!token?'先连接控制':s.armed?'已使能；DISARM / 空格请求失能':
  !s.zero_confirmed?'确认已有机械零点正确，再点 ZERO OK':'当前失能；点击“手动使能 / ARM”后才会使能';
 renderTuning(s);
}
function renderTuning(s){const t=s.tuning;if(!t)return;
 const usable=!!token&&s.ready&&!s.fault;
 const auto=t.autotune||{active:false,state:'IDLE'};
 $('piAutoStart').disabled=!usable||!s.armed||auto.active||s.home_requested;
 $('piAutoCancel').disabled=!usable||!auto.active;
 $('piAutoSave').disabled=!usable||auto.active||auto.state!=='COMPLETE';
 $('piAutoStatus').textContent=auto.state+(auto.reason?' · '+auto.reason:'')+(auto.restore_pending?' · 基线恢复待确认':'')+(auto.saved_path?' · 已保存 '+auto.saved_path:'');
 if(auto.active)$('home').disabled=true;
 $('applyPi').disabled=!usable||auto.active||!['DISABLED','ARMED','HOME_HOLD'].includes(s.phase)||s.home_requested||t.pi_busy;
 $('record').disabled=!usable;$('record').textContent=t.recording?'停止记录':'开始记录';
 const pair=p=>p?`Kp ${p[0]??'未知'} / Ki ${p[1]??'未知'}`:'未知';
 $('piStatus').textContent=`${s.tuning_error||t.pi_status}\n最近发送：YAW ${pair(t.pi_last_sent[0])}；PITCH ${pair(t.pi_last_sent[1])}（无实机回读）`;
 const p=t.profiles||{}, describe=n=>p[n]?`${n}: RPM ${p[n].rpm.join('/')} · 目标速度 ${p[n].rate.join('/')}°/s`:n;
 $('profileInfo').textContent=`启动 ${t.startup_status} · 当前配置 ${t.drive_profile||'尚未使能'}\n${['startup','manual','home'].map(describe).join('；')}`;
 $('recordStatus').textContent=`${t.recording?'记录中':'记录已停止'} · ${t.count} 个串口样本${t.record_reason?' · '+t.record_reason:''}`;
 const f=v=>Number.isFinite(v)?v.toFixed(3):'--';
 $('metrics').textContent=t.result&&t.result.axes? t.result.axes.map(a=>`${a.axis}：跟踪 RMSE ${f(a.tracking_rmse_deg)}°；末段固定目标峰峰值 ${f(a.holding_peak_to_peak_deg)}°；平均误差 ${f(a.holding_mean_error_deg)}°`).join('\n'):'';
 if(t.latest&&s.reference_valid&&Number.isFinite(s.age)&&s.age<.5){const r=t.latest,last=traceRows[traceRows.length-1];
  if(last&&r.seq<last.seq)traceRows.length=0;
  if(!last||last.seq!==r.seq){traceRows.push(r);if(traceRows.length>260)traceRows.shift();}}
 drawTrace();
}
function drawTrace(){const c=$('trace'),g=c.getContext('2d');if(!g)return;const w=c.width,h=c.height,axis=Number($('tuneAxis').value);g.clearRect(0,0,w,h);if(!traceRows.length)return;
 const end=traceRows[traceRows.length-1].time_s,rows=traceRows.filter(r=>r.time_s>=end-12),values=rows.flatMap(r=>[r.angle_deg[axis],r.target_deg?.[axis]]).filter(Number.isFinite);
 if(!values.length)return;const lo=Math.min(...values)-.5,hi=Math.max(lo+2,...values.map(v=>v+.5)),x=t=>55+(t-(end-12))/12*(w-75),y=v=>h-25-(v-lo)/(hi-lo)*(h-45);
 g.font='15px system-ui';g.fillStyle='#9cacbd';g.strokeStyle='#2d4259';for(let i=0;i<4;i++){const v=lo+(hi-lo)*i/3;g.beginPath();g.moveTo(55,y(v));g.lineTo(w-20,y(v));g.stroke();g.fillText(v.toFixed(1)+'°',3,y(v)+5);}g.fillText('-12s',55,h-5);g.fillText('现在',w-55,h-5);
 for(const [key,color] of [['angle_deg','#28c8bf'],['target_deg','#ffd18a']]){g.strokeStyle=color;g.lineWidth=2;g.beginPath();let previous=null;for(const r of rows){const v=r[key]?.[axis];if(!Number.isFinite(v)){previous=null;continue;}if(previous===null||r.time_s-previous>.3)g.moveTo(x(r.time_s),y(v));else g.lineTo(x(r.time_s),y(v));previous=r.time_s;}g.stroke();}}
function queueTuning(action,fields={}){release();pendingTune=fields;pending=action;}
$('applyPi').onclick=()=>{const raw=[$('kp').value.trim(),$('ki').value.trim()],nums=raw.map(v=>v===''?null:Number(v));if(nums.every(v=>v===null)||nums.some(v=>v!==null&&(!Number.isInteger(v)||v<0||v>256))){$('piStatus').textContent='填写至少一个 0～256 的整数；留空不改，没有读取到当前参数。';return;}queueTuning('pi',{axis:Number($('tuneAxis').value),kp:nums[0],ki:nums[1]});};
$('record').onclick=()=>queueTuning(lastState?.tuning?.recording?'record_stop':'record_start',{note:$('note').value});
$('piAutoStart').onclick=()=>{
 const raw=[$('kp').value.trim(),$('ki').value.trim()];
 const values=raw.map(Number);
 if(raw.some(v=>v==='')||values.some(v=>!Number.isInteger(v)||v<0||v>256)){
  $('piAutoStatus').textContent='先填写已知的 Kp 和 Ki 基线，范围为 0～256 整数';
  return;
 }
 queueTuning('pi_autotune',{axis:Number($('tuneAxis').value),kp:values[0],ki:values[1]});
};
$('piAutoCancel').onclick=()=>queueTuning('pi_autotune_cancel');
$('piAutoSave').onclick=()=>queueTuning('pi_autotune_save');
$('tuneAxis').onchange=drawTrace;
async function post(path,body){const ctl=new AbortController(),timer=setTimeout(()=>ctl.abort(),450);
 try{const r=await fetch(path,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body),signal:ctl.signal,cache:'no-store'});
 const j=await r.json();if(!r.ok)throw Error(j.error||r.status);return j;}finally{clearTimeout(timer);}}
function stop(){const old=token;token=null;pending='';pendingTune={};release();navigator.sendBeacon('/api/stop',new Blob([JSON.stringify({token:old})],{type:'application/json'}));
 $('link').textContent='已请求失能';}
$('connect').onclick=async()=>{release();try{const j=await post('/api/connect',{});token=j.token;seq=0;render(j.state);}catch(e){$('notice').textContent=e.message;}};
$('recheck').onclick=async()=>{const old=token;release();pending='';try{const s=await post('/api/recheck',{token:old});token=null;seq=0;render(s);}catch(e){$('notice').textContent=e.message;}};
$('stop').onclick=stop;
for(const action of ['zero','arm','home'])$(action).onclick=()=>{release();pending=action;};
const pad=$('pad');
function point(e){const r=pad.getBoundingClientRect();let x=(e.clientX-r.left-r.width/2)/(r.width*.4),y=(r.top+r.height/2-e.clientY)/(r.height*.4);let d=Math.hypot(x,y);
 if(d<.15)vector=[0,0];else{const a=Math.min(1,(d-.15)/.85);vector=[x/d*a,y/d*a];}draw();}
pad.onpointerdown=e=>{if(!token||pointer!==null||lastState?.phase==='ARMING_HOLD'||lastState?.tuning?.autotune?.active)return;e.preventDefault();keys.clear();pointer=e.pointerId;pad.setPointerCapture(pointer);point(e);};
pad.onpointermove=e=>{if(e.pointerId===pointer)point(e);};
pad.onpointerup=pad.onpointercancel=pad.onlostpointercapture=e=>{if(e.pointerId===pointer)release();};
function keyVector(){let x=Number(keys.has('ArrowRight'))-Number(keys.has('ArrowLeft')),y=Number(keys.has('ArrowUp'))-Number(keys.has('ArrowDown')),n=Math.max(1,Math.hypot(x,y));vector=[x/n,y/n];draw();}
window.addEventListener('keydown',e=>{if(e.code==='Space'){e.preventDefault();stop();return;}if(['INPUT','SELECT','TEXTAREA'].includes(e.target?.tagName))return;if(!e.key.startsWith('Arrow'))return;e.preventDefault();if(token&&pointer===null&&lastState?.phase!=='ARMING_HOLD'&&!lastState?.tuning?.autotune?.active){keys.add(e.key);keyVector();}});
window.addEventListener('keyup',e=>{if(keys.delete(e.key))keyVector();});
function stopIfOwned(){
 if(!token)return;
 if(lastState?.tuning?.autotune?.active){
  const old=token;token=null;pending='';release();
  navigator.sendBeacon('/api/tuning/background',new Blob([JSON.stringify({token:old})],{type:'application/json'}));
 }else stop();
}
window.addEventListener('blur',stopIfOwned);window.addEventListener('pagehide',stopIfOwned);
document.addEventListener('visibilitychange',()=>{if(document.hidden)stopIfOwned();});
async function tick(){if(busy)return;busy=true;const current=token;try{if(current){const action=pending,fields=pendingTune;pending='';pendingTune={};const v=action?[0,0]:vector.slice();const s=await post('/api/control',{...fields,token:current,seq:++seq,x:v[0],y:v[1],action});if(token===current)render(s);}
 else{const r=await fetch('/api/state',{cache:'no-store',signal:AbortSignal.timeout(450)});render(await r.json());}}
 catch(e){if(current&&token===current)stop();$('notice').textContent='连接失败 / '+e.message+'；请重新连接';}finally{busy=false;setTimeout(tick,80);}}
tick();
</script></html>'''


def web_main():
    from maix import app, sys as maix_sys
    if str(maix_sys.device_id()).lower() != "maixcam2":
        raise RuntimeError("This UART3 mapping requires MaixCAM2")
    validate_config(CONFIG)
    motor = TuningMotorService(dict(CONFIG))
    control = BrowserControl(motor, lease_s=WEB_CONFIG["lease_s"])
    server = None
    thread = None
    try:
        # Bind before UART startup: a port conflict must not leave hardware running.
        server = make_http_server(control, (WEB_CONFIG["host"], WEB_CONFIG["port"]))
        print("F32C WEB | MECH ONLY | AUTO STARTUP HOME=%s | saved mechanical zero" % TUNING_CONFIG["startup_auto_home"], flush=True)
        motor.start()
        thread = threading.Thread(target=server.serve_forever, kwargs={"poll_interval": 0.05}, daemon=True)
        thread.start()
        print("F32C WEB | MECH ONLY | UART3 B2/B3 115200 | startup/manual/home profiles", flush=True)
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
