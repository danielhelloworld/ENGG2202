const CATEGORY_DEFS = [
  { id: "vision", label: "视觉识别", short: "VISION", icon: "◎", description: "摄像头、目标类别、检测框" },
  { id: "tracking", label: "目标跟踪", short: "TRACKING", icon: "⌖", description: "状态机、方位角、运动速度" },
  { id: "gimbal", label: "云台控制", short: "GIMBAL", icon: "⌁", description: "双轴角度、反馈与控制" },
  { id: "system", label: "系统运行", short: "SYSTEM", icon: "⌘", description: "进程、模型、设备服务" },
];

const METRICS = {
  vision: [
    ["person_count", "Person detections", "当前帧检测到的人数", "count"],
    ["selected_label", "Selected target", "当前关联的单一目标类别", "class"],
    ["confidence", "Confidence", "选中目标的检测置信度", "0–1"],
    ["center_x", "Bounding-box center X", "检测框中心横坐标", "px"],
    ["center_y", "Bounding-box center Y", "检测框中心纵坐标", "px"],
    ["width", "Input width", "模型输入图像宽度", "px"],
    ["height", "Input height", "模型输入图像高度", "px"],
    ["fps", "Detector throughput", "检测循环吞吐率（若已上报）", "FPS"],
  ],
  tracking: [
    ["mode", "Tracking mode", "BOOT / ACQUIRE / TRACK / COAST / SEARCH / FAULT", "state"],
    ["azimuth_deg", "Azimuth", "目标水平世界角度", "deg"],
    ["elevation_deg", "Elevation", "目标垂直世界角度", "deg"],
    ["azimuth_rate_deg_s", "Azimuth rate", "水平角速度估计", "deg/s"],
    ["elevation_rate_deg_s", "Elevation rate", "垂直角速度估计", "deg/s"],
    ["azimuth_sigma_deg", "Azimuth uncertainty", "水平角估计标准差", "deg"],
    ["elevation_sigma_deg", "Elevation uncertainty", "垂直角估计标准差", "deg"],
  ],
  gimbal: [
    ["enabled", "Controller enabled", "云台控制是否启用", "bool"],
    ["pan_deg", "Pan feedback", "水平轴当前反馈角度", "deg"],
    ["tilt_deg", "Tilt feedback", "俯仰轴当前反馈角度", "deg"],
    ["pan_valid", "Pan feedback valid", "水平轴反馈有效性", "bool"],
    ["tilt_valid", "Tilt feedback valid", "俯仰轴反馈有效性", "bool"],
    ["command_pan_deg", "Pan command", "水平轴目标指令", "deg"],
    ["command_tilt_deg", "Tilt command", "俯仰轴目标指令", "deg"],
  ],
  system: [
    ["running", "Process running", "追踪脚本运行状态", "bool"],
    ["script", "Running file", "当前设备端入口文件", "file"],
    ["pid", "Process ID", "设备端进程标识", "pid"],
    ["uptime_seconds", "Process uptime", "进程运行时间", "s"],
    ["model_path", "Model file", "当前加载的模型路径", "path"],
    ["camera_available", "Camera available", "摄像头初始化状态", "bool"],
    ["npu_ready", "NPU ready", "NPU 模型执行状态", "bool"],
  ],
};

const CATEGORY_OBJECT = Object.fromEntries(CATEGORY_DEFS.map((item) => [item.id, item]));
const pageNames = { overview: "总览", camera: "实时画面", data: "数据面板", runtime: "运行状态", settings: "接入设置" };
let currentState = null;
let selectedCategory = "vision";
let events = [];
let lastRevision = -1;
let cameraRetryAt = 0;
let fallbackPoll = null;

const $ = (id) => document.getElementById(id);

function escapeHtml(value) {
  return String(value).replace(/[&<>"']/g, (character) => ({
    "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;",
  })[character]);
}

function displayValue(value, unit = "") {
  if (value === undefined || value === null || value === "") return "—";
  if (typeof value === "boolean") return value ? "是" : "否";
  if (typeof value === "number") return `${Number.isInteger(value) ? value : value.toFixed(2)}${unit ? ` ${unit}` : ""}`;
  if (typeof value === "object") return JSON.stringify(value);
  return String(value);
}

function formatDuration(seconds) {
  if (!Number.isFinite(seconds)) return "—";
  const total = Math.max(0, Math.floor(seconds));
  const h = Math.floor(total / 3600);
  const m = Math.floor((total % 3600) / 60);
  const s = total % 60;
  return h ? `${h}h ${m}m ${s}s` : `${m}m ${s}s`;
}

function timeLabel(timestamp = Date.now()) {
  return new Date(timestamp).toLocaleTimeString([], { hour12: false });
}

function recordEvent(type, text, timestamp = Date.now()) {
  events.unshift({ type, text, timestamp });
  events = events.slice(0, 9);
  renderEvents();
}

function setText(id, value) {
  const node = $(id);
  if (node) node.textContent = value;
}

function setOnline(element, online) {
  if (element) element.classList.toggle("online", Boolean(online));
}

function setBadge(id, online, yesText = "ONLINE", noText = "OFFLINE") {
  const node = $(id);
  if (!node) return;
  node.classList.toggle("badge-online", Boolean(online));
  node.classList.toggle("badge-offline", !online);
  node.innerHTML = `<i></i> ${online ? yesText : noText}`;
}

function showCamera(available, connected) {
  const images = [$("camera-image"), $("camera-image-large")];
  const placeholders = [$("camera-placeholder"), $("camera-placeholder-large")];
  const now = Date.now();
  if (available && connected) {
    for (const image of images) {
      if (!image) continue;
      image.hidden = false;
      if (!image.dataset.started || (cameraRetryAt && now >= cameraRetryAt)) {
        image.dataset.started = "1";
        image.src = `/api/camera/stream?session=${now}`;
        cameraRetryAt = 0;
      }
      image.onerror = () => {
        image.hidden = true;
        cameraRetryAt = Date.now() + 2500;
        for (const placeholder of placeholders) if (placeholder) placeholder.hidden = false;
      };
    }
    for (const placeholder of placeholders) if (placeholder) placeholder.hidden = true;
  } else {
    for (const image of images) {
      if (!image) continue;
      image.hidden = true;
      if (image.dataset.started) image.removeAttribute("src");
      image.dataset.started = "";
    }
    for (const placeholder of placeholders) if (placeholder) placeholder.hidden = false;
  }
  setBadge("camera-badge", available && connected, "LIVE", "OFFLINE");
  setBadge("camera-page-badge", available && connected, "LIVE", "WAITING");
  for (const id of ["camera-led", "camera-led-large"]) setOnline($(id), available && connected);
  setText("camera-footer-state", available ? "摄像头服务在线" : "相机未连接");
  setText("camera-footer-state-large", available ? "实时视频流已接入" : "等待设备心跳");
  const size = available ? `${currentState.camera.width || "—"} × ${currentState.camera.height || "—"}` : "— × —";
  setText("camera-resolution", size);
  setText("camera-resolution-large", size);
  const fps = displayValue(currentState.camera.fps, "FPS");
  setText("camera-fps", fps === "—" ? "— FPS" : fps);
  setText("camera-fps-large", currentState.camera.fps == null ? "—" : displayValue(currentState.camera.fps));
}

function updateOverview(state) {
  const connected = state.connected;
  const vision = state.vision || {};
  const tracking = state.tracking || {};
  const gimbal = state.gimbal || {};
  const runtime = state.runtime || {};
  const camera = state.camera || {};
  setText("device-address", state.device?.ip || "—");
  setText("overview-person-count", connected ? displayValue(vision.person_count, "") : "—");
  setText("vision-state", connected ? (vision.person_count > 0 ? "检测到目标" : "画面中无目标") : "等待数据");
  $("vision-state").className = `metric-state ${connected && vision.person_count > 0 ? "ok" : ""}`;
  setText("overview-mode", connected ? (tracking.mode || "—") : "—");
  setText("tracking-state", connected ? (tracking.mode === "TRACK" ? "目标跟踪中" : "状态机") : "等待数据");
  $("tracking-state").className = `metric-state ${tracking.mode === "TRACK" ? "ok" : ""}`;
  setText("overview-gimbal", connected ? `${displayValue(gimbal.pan_deg)} / ${displayValue(gimbal.tilt_deg)}°` : "—");
  setText("gimbal-state", connected ? (gimbal.enabled ? "控制已启用" : "视觉模式 · 电机关闭") : "等待数据");
  $("gimbal-state").className = `metric-state ${gimbal.enabled ? "warn" : ""}`;
  setText("overview-file", connected && runtime.script ? runtime.script : "—");
  setText("runtime-state", connected ? (runtime.running ? "运行中" : "已停止") : "未运行");
  $("runtime-state").className = `metric-state ${runtime.running ? "ok" : ""}`;

  setText("target-azimuth", connected ? displayValue(tracking.azimuth_deg) : "—");
  setText("target-elevation", connected ? displayValue(tracking.elevation_deg) : "—");
  setText("target-confidence", connected ? displayValue(vision.confidence) : "—");
  setText("target-rate", connected ? `${displayValue(tracking.azimuth_rate_deg_s)} / ${displayValue(tracking.elevation_rate_deg_s)}°/s` : "—");
  const target = $("radar-target");
  const hasBearing = connected && tracking.azimuth_deg != null && tracking.elevation_deg != null;
  target.classList.toggle("visible", Boolean(hasBearing));
  if (hasBearing) {
    const x = Math.max(12, Math.min(88, 50 + Number(tracking.azimuth_deg) * 0.85));
    const y = Math.max(12, Math.min(88, 50 - Number(tracking.elevation_deg) * 0.85));
    target.style.left = `${x}%`;
    target.style.top = `${y}%`;
  }
  $("tracking-chip").textContent = connected ? (tracking.mode || "UNKNOWN") : "NO DATA";
  $("tracking-chip").classList.toggle("active", connected && tracking.mode === "TRACK");

  const online = Boolean(connected);
  setOnline($("sidebar-dot"), online);
  setOnline($("top-live-dot"), online);
  setOnline($("health-pulse"), online);
  setOnline($("foot-led"), online);
  setText("top-live-label", online ? "DEVICE CONNECTED" : "DEVICE OFFLINE");
  setText("sidebar-link-state", online ? "設備已連接" : "等待設備接入");
  setText("sidebar-link-hint", online ? `最後心跳 ${state.age_seconds ?? 0}s 前` : "啟動追蹤程序後將自動接收狀態");
  setText("connection-title", online ? "MaixCAM 2 已連接" : "等待 MaixCAM 2 接入");
  setText("connection-description", online ? `設備心跳正常 · ${runtime.running ? "追蹤程序正在運行" : "追蹤程序已停止"}` : "啟動設備追蹤程序後，數據和攝像頭畫面將在此顯示。");
  $("connection-icon").textContent = online ? "✓" : "↗";
  setBadge("runtime-page-badge", online, "CONNECTED", "DISCONNECTED");
  const camAvailable = online && Boolean(camera.available);
  showCamera(camAvailable, online);
  renderCategories(state);
  renderDataRows(state);
  renderRuntime(state);
}

function renderCategories(state) {
  const root = $("overview-categories");
  root.innerHTML = CATEGORY_DEFS.map((item) => {
    const data = state[item.id] || {};
    const populated = Object.values(data).filter((value) => value !== null && value !== undefined).length;
    return `<div class="category-card" data-category="${item.id}"><div class="category-card-top"><span class="category-icon">${item.icon}</span><strong>${item.label}</strong></div><p>${CATEGORY_OBJECT[item.id].short} · ${state.connected ? `${populated} 指標` : "等待數據"}</p></div>`;
  }).join("");
  root.querySelectorAll(".category-card").forEach((card) => card.addEventListener("click", () => {
    selectedCategory = card.dataset.category;
    navigate("data");
  }));
}

function formatMetric(category, key, value, unit) {
  if (value == null) return "—";
  if (key === "running" || key.endsWith("_valid") || key === "enabled" || key === "camera_available" || key === "npu_ready") return value ? "是 / ONLINE" : "否 / OFFLINE";
  if (key === "uptime_seconds") return formatDuration(Number(value));
  return displayValue(value, unit);
}

function renderDataTabs(state) {
  const extras = Object.keys(state.extensions || {});
  const defs = [...CATEGORY_DEFS, ...extras.map((id) => ({ id: `ext:${id}`, label: id, short: "EXTENSION", icon: "＋", description: "擴展模組數據" }))];
  const root = $("data-tabs");
  if (!defs.some((item) => item.id === selectedCategory)) selectedCategory = defs[0].id;
  root.innerHTML = defs.map((item) => `<button class="data-tab ${item.id === selectedCategory ? "active" : ""}" data-category="${escapeHtml(item.id)}">${escapeHtml(item.label)}</button>`).join("");
  root.querySelectorAll(".data-tab").forEach((button) => button.addEventListener("click", () => {
    selectedCategory = button.dataset.category;
    renderDataTabs(currentState);
    renderDataRows(currentState);
  }));
}

function renderDataRows(state) {
  renderDataTabs(state);
  const extension = selectedCategory.startsWith("ext:");
  const category = extension ? selectedCategory.slice(4) : selectedCategory;
  const values = extension ? (state.extensions?.[category] || {}) : (state[category] || {});
  const rows = extension
    ? Object.entries(values).map(([key, value]) => [key, key.replaceAll("_", " "), "Extension metric", "—", value])
    : (METRICS[category] || []).map(([key, label, description, unit]) => [key, label, description, unit, values[key]]);
  const root = $("data-rows");
  const populated = rows.filter(([key, , , , value]) => value !== undefined || (category === "system" && key === "camera_available"));
  root.innerHTML = populated.length ? populated.map(([key, label, description, unit, value]) => `<div class="data-row"><span class="data-name">${escapeHtml(label)}</span><span class="data-value">${escapeHtml(formatMetric(category, key, value, unit))}</span><span class="data-description">${escapeHtml(description)}${unit ? ` · ${escapeHtml(unit)}` : ""}</span><span class="data-source">${escapeHtml(extension ? "EXTENSION" : CATEGORY_OBJECT[category]?.short || "SYSTEM")}</span></div>`).join("") : '<div class="empty-data">此分類暫無已接入數據。</div>';
}

function renderRuntime(state) {
  const runtime = state.runtime || {};
  const online = Boolean(state.connected);
  const running = online && Boolean(runtime.running);
  setText("runtime-script", running ? (runtime.script || "unknown.py") : (online ? "無活動程序" : "設備未連接"));
  setText("runtime-running", online ? (runtime.running ? "RUNNING" : "STOPPED") : "DISCONNECTED");
  setText("runtime-pid", runtime.pid ?? "—");
  setText("runtime-uptime", runtime.running ? formatDuration(runtime.uptime_seconds) : "—");
  setText("runtime-model", runtime.model_path || "—");
  setText("runtime-heartbeat", state.received_at ? timeLabel(state.received_at * 1000) : "—");
  setOnline($("process-indicator"), running);
  const camera = state.camera || {};
  setService("service-camera", online && camera.available, online ? (camera.available ? "ONLINE" : "ERROR") : "UNKNOWN");
  setText("service-camera-detail", online ? `${camera.width || "—"} × ${camera.height || "—"} · ${camera.available ? "capture active" : "not initialized"}` : "等待設備狀態");
  setService("service-npu", online && Boolean(state.system?.npu_ready), online ? (state.system?.npu_ready ? "ONLINE" : "UNKNOWN") : "UNKNOWN");
  setText("service-npu-detail", online ? (runtime.model_path || "model state unavailable") : "等待設備狀態");
  setService("service-stream", online && camera.available, online && camera.available ? "ONLINE" : "OFFLINE");
  const gimbal = state.gimbal || {};
  setService("service-gimbal", Boolean(gimbal.enabled), gimbal.enabled ? "ENABLED" : "DISABLED");
  setText("service-gimbal-detail", gimbal.enabled ? "UART control active" : "Vision-only mode · motors disabled");
}

function setService(id, online, label) {
  const node = $(id);
  node.textContent = label;
  node.className = `service-status ${online ? "online" : ""}`;
}

function renderEvents() {
  setText("event-count", `${events.length} events`);
  const root = $("event-list");
  root.innerHTML = events.length ? events.map((event) => `<div class="event-item"><span class="event-time">${timeLabel(event.timestamp)}</span><span class="event-dot"></span><span>${escapeHtml(event.text)}</span><span class="event-type">${escapeHtml(event.type)}</span></div>`).join("") : '<div class="empty-state">設備連接後，運行狀態變化將顯示在這裡。</div>';
}

function applyState(state, source = "poll") {
  if (!state || typeof state !== "object") return;
  const previous = currentState;
  currentState = state;
  updateOverview(state);
  if (state.revision !== lastRevision) {
    setText("data-updated", state.received_at ? timeLabel(state.received_at * 1000) : "—");
    if (previous && previous.connected !== state.connected) recordEvent("LINK", state.connected ? "MaixCAM 2 telemetry connected" : "MaixCAM 2 telemetry timed out");
    if (previous && previous.tracking?.mode !== state.tracking?.mode && state.tracking?.mode) recordEvent("TRACK", `Tracking mode → ${state.tracking.mode}`);
    if (previous && previous.runtime?.script !== state.runtime?.script && state.runtime?.script) recordEvent("PROCESS", `Running file → ${state.runtime.script}`);
    lastRevision = state.revision;
  }
}

async function refreshState() {
  try {
    const response = await fetch("/api/state", { cache: "no-store" });
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    applyState(await response.json());
  } catch (error) {
    setText("connection-title", "工作台服務無法連接");
    setText("connection-description", "請確認 workbench/server.py 正在執行，然後重新載入頁面。");
  }
}

async function loadConfig() {
  try {
    const response = await fetch("/api/config", { cache: "no-store" });
    if (!response.ok) return;
    const config = await response.json();
    $("device-ip-input").value = config.device_ip;
    $("video-port-input").value = config.video_port;
    $("ingest-endpoint").textContent = config.ingest_url;
    setText("device-address", config.device_ip);
  } catch {
    // The dashboard still renders if the gateway config endpoint is unavailable.
  }
}

function navigate(view) {
  if (!pageNames[view]) return;
  document.querySelectorAll(".view").forEach((item) => item.classList.toggle("active", item.id === `view-${view}`));
  document.querySelectorAll(".nav-item[data-view]").forEach((item) => item.classList.toggle("active", item.dataset.view === view));
  setText("page-crumb", pageNames[view]);
  document.title = `${pageNames[view]} · MaixCAM Workbench`;
  if (view === "camera" && currentState) showCamera(Boolean(currentState.camera?.available), Boolean(currentState.connected));
  if (view === "data" && currentState) renderDataRows(currentState);
}

document.querySelectorAll(".nav-item[data-view]").forEach((button) => button.addEventListener("click", () => navigate(button.dataset.view)));
document.querySelectorAll("[data-open-view]").forEach((button) => button.addEventListener("click", () => navigate(button.dataset.openView)));
$("refresh-button").addEventListener("click", refreshState);

$("settings-form").addEventListener("submit", async (event) => {
  event.preventDefault();
  const feedback = $("settings-feedback");
  try {
    const response = await fetch("/api/config", {
      method: "POST", headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ device_ip: $("device-ip-input").value.trim(), video_port: Number($("video-port-input").value) }),
    });
    const result = await response.json();
    if (!response.ok) throw new Error(result.error || `HTTP ${response.status}`);
    feedback.textContent = "連接設定已更新";
    setTimeout(() => { feedback.textContent = ""; }, 2500);
    await refreshState();
  } catch (error) {
    feedback.textContent = error.message;
  }
});

$("copy-endpoint").addEventListener("click", async () => {
  try {
    await navigator.clipboard.writeText($("ingest-endpoint").textContent);
    $("copy-endpoint").textContent = "已複製";
    setTimeout(() => { $("copy-endpoint").textContent = "複製"; }, 1600);
  } catch {
    $("copy-endpoint").textContent = "請手動複製";
  }
});

setText("session-time", new Date().toLocaleDateString([], { month: "short", day: "2-digit" }));
setText("footer-clock", `LOCAL SESSION · ${timeLabel()}`);
setInterval(() => setText("footer-clock", `LOCAL SESSION · ${timeLabel()}`), 1000);
refreshState();
loadConfig();
const telemetryEvents = new EventSource("/api/events");
telemetryEvents.onmessage = (event) => {
  try { applyState(JSON.parse(event.data), "event"); } catch { /* Ignore malformed extension messages. */ }
};
telemetryEvents.onopen = () => {
  if (fallbackPoll) clearInterval(fallbackPoll);
  fallbackPoll = null;
};
telemetryEvents.onerror = () => {
  if (!fallbackPoll) fallbackPoll = setInterval(refreshState, 5000);
};
