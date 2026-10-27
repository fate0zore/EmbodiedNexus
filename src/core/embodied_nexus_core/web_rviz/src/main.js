import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { ColladaLoader } from "three/addons/loaders/ColladaLoader.js";
import { STLLoader } from "three/addons/loaders/STLLoader.js";
import URDFLoader from "urdf-loader";
import { Decoder } from "cbor-x";
import { decodeCborFrame } from "./cbor-codec.js";
import { buildTfPoses, imagePixels, pointCloudBuffers } from "./ros-data.js";
import { updatePointLayer } from "./point-layer.js";
import "./style.css";

/**
 * 按元素 ID 获取 DOM 元素。
 * @param {*} id - 页面元素或设备的标识符。
 * @returns {*} 按元素 ID 获取 DOM 元素得到的数据或处理结果。
 */
const $ = (id) => document.getElementById(id);
/**
 * 在页面中显示运行错误信息。
 * @param {*} error - 运行期间捕获的错误对象。
 * @returns {*} 在页面中显示运行错误信息得到的数据或处理结果。
 */
function showRuntimeError(error) {
  const message = `页面脚本错误：${String(error?.message || error).slice(0, 220)}`;
  const label = $("connection-label");
  const sceneMessage = $("scene-message");
  const emptyState = $("empty-state");
  if (label) label.textContent = "页面运行错误";
  if (sceneMessage) sceneMessage.textContent = message;
  if (emptyState) { emptyState.textContent = message; emptyState.hidden = false; }
  $("viewer-status")?.classList.remove("is-online");
}
window.addEventListener("error", (event) => { if (event.error) showRuntimeError(event.error); });
window.addEventListener("unhandledrejection", (event) => showRuntimeError(event.reason));
const TOKEN_KEY = "embodiedNexusCoreToken";
const apiUrl = "/api/v1/core/visualization";
const token = localStorage.getItem(TOKEN_KEY) || "";
const topicSelectors = {
  description: $("description-topic"), cloud: $("cloud-topic"), cameraCloud: $("camera-cloud-topic"), scan: $("scan-topic"),
  rgb: $("rgb-topic"), depth: $("depth-topic"),
};
const typeFor = { description: "std_msgs/msg/String", cloud: "sensor_msgs/msg/PointCloud2", cameraCloud: "sensor_msgs/msg/PointCloud2",
  scan: "sensor_msgs/msg/LaserScan", rgb: "sensor_msgs/msg/Image", depth: "sensor_msgs/msg/Image" };
const CAMERA_CLOUD_COLOR = 0xff66cc;
const selection = new Map();
const manualTopics = new Set();
const latest = new Map();
const topicStats = new Map();
const fallbackDecoder = new Decoder({ useRecords: false, mapsAsObjects: true });
let decoderWorker = null;
try { decoderWorker = new Worker(new URL("./cbor-worker.js", import.meta.url), { type: "module" }); }
catch { /* decode sensor data on the main thread if workers are unavailable */ }
let topics = [];
let resourcePackages = [];
let explicitQos = false;
let presetTouched = false;
let ws;
let sensorWs;
let pageLeaving = false;
let wsConnectTimer;
let reconnectTimer;
let reconnectDelay = 1000;
let currentModel = null;
let modelRootLink = "";
let modelDescription = "";
let maxPoints = 100000;
let pointSize = 0.035;
let depthRange = 6;
let lastClock = 0;
const staticTf = new Map();
const dynamicTf = new Map();
const observedSensorFrames = new Set();
const transformCache = new Map();
const preparedData = new Map();
const pendingSensors = new Set();
let frameSignature = "";
let tfDirty = false;
let renderFrameId = 0;
let lastRenderTime = Number.NEGATIVE_INFINITY;
const renderIntervalMs = 1000 / 30;

/**
 * 请求渲染下一帧三维场景。
 * @returns {void} 无返回值；函数通过更新界面或发送请求完成操作。
 */
function requestRender() {
  if (!renderFrameId && !document.hidden && !pageLeaving) renderFrameId = requestAnimationFrame(animate);
}

const scene = new THREE.Scene();
scene.background = new THREE.Color("#111916");
const camera = new THREE.PerspectiveCamera(54, 1, 0.03, 300);
camera.position.set(3.8, -4.5, 3.0);
camera.up.set(0, 0, 1);
let renderer = null;
let controls = null;
let rendererError = "";
try {
  renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false, powerPreference: "high-performance" });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 1.25));
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  renderer.domElement.setAttribute("aria-label", "ROS 3D 可视化场景");
  $("three-canvas").prepend(renderer.domElement);
  controls = new OrbitControls(camera, renderer.domElement);
  controls.target.set(0, 0, 0.45);
  controls.enableDamping = true;
  controls.addEventListener("change", requestRender);
} catch (error) {
  rendererError = `3D 渲染不可用：${error.message || error}`;
  renderer?.dispose();
  renderer?.domElement.remove();
  renderer = null;
  controls = null;
  $("scene-message").textContent = rendererError;
  $("empty-state").textContent = `${rendererError}；仍会连接 ROS 并显示图像与数据状态。`;
}
scene.add(new THREE.HemisphereLight(0xc8e3d4, 0x28332d, 2.2));
const keyLight = new THREE.DirectionalLight(0xffffff, 2.1); keyLight.position.set(4, -3, 8); scene.add(keyLight);
const grid = new THREE.GridHelper(12, 24, 0x466351, 0x293b31);
grid.rotation.x = Math.PI / 2;
scene.add(grid);
const axes = new THREE.AxesHelper(0.45);
scene.add(axes);
const modelGroup = new THREE.Group(); scene.add(modelGroup);
const frameGroup = new THREE.Group(); scene.add(frameGroup);
const geometryObjects = new Map();
const rootAxes = new Map();
const pausedLayers = new Set();

const resizeObserver = new ResizeObserver(() => {
  const bounds = $("three-canvas").getBoundingClientRect();
  if (!bounds.width || !bounds.height) return;
  camera.aspect = bounds.width / bounds.height;
  camera.updateProjectionMatrix();
  renderer?.setSize(bounds.width, bounds.height, false);
  requestRender();
});
resizeObserver.observe($("three-canvas"));

/**
 * 更新连接状态和对应显示文本。
 * @param {*} online - 表示服务连接是否在线。
 * @param {*} label - 连接状态对应的显示文本。
 * @returns {void} 无返回值；函数通过更新界面或发送请求完成操作。
 */
function setConnection(online, label) {
  const status = $("viewer-status");
  status.classList.toggle("is-online", online);
  $("connection-label").textContent = label;
}

/**
 * 构造访问 Core API 所需的授权请求头。
 * @returns {*} 构造访问 Core API 所需的授权请求头得到的数据或处理结果。
 */
function apiHeaders() { return { Authorization: `Bearer ${token}` }; }

/**
 * 读取 Core 与机器人模型信息。
 * @returns {Promise<*>} 异步读取 Core 与机器人模型信息的结果。
 */
async function loadInfo() {
  const response = await fetch(apiUrl, { headers: apiHeaders(), cache: "no-store" });
  if (response.status === 401) { location.assign("/"); return null; }
  if (!response.ok) throw new Error(`Core visualization API: HTTP ${response.status}`);
  return response.json();
}

/**
 * 更新选择框选项并按需保留当前值。
 * @param {*} select - 需要更新的 HTML 选择框元素。
 * @param {*} options - fetch 请求选项。
 * @param {*} emptyLabel - 没有候选项时显示的占位文本。
 * @param {*} keepValue - 是否尽可能保留原有选中值。
 * @returns {*} 更新选择框选项并按需保留当前值得到的数据或处理结果。
 */
function replaceOptions(select, options, emptyLabel = "未选择", keepValue = true) {
  const previous = keepValue ? select.value : "";
  const signature = JSON.stringify([emptyLabel, options]);
  if (select.dataset.options === signature) { if (!keepValue) select.value = ""; return; }
  select.dataset.options = signature;
  select.replaceChildren(new Option(emptyLabel, ""));
  for (const item of options) select.add(new Option(`${item.name}  ·  ${item.type.split("/").at(-1)}`, item.name));
  if (previous && options.some((item) => item.name === previous)) select.value = previous;
}

/**
 * 按条件从话题候选项中选择话题。
 * @param {*} key - 话题配置项或字段的键。
 * @param {*} predicate - 用于判断候选话题是否符合要求的函数。
 * @returns {*} 按条件从话题候选项中选择话题得到的数据或处理结果。
 */
function chooseTopic(key, predicate) {
  if (manualTopics.has(key)) return;
  const found = topics.find((topic) => predicate(topic.name, topic.type));
  if (found) topicSelectors[key].value = found.name;
}

/**
 * 根据显示预设选择固定坐标系。
 * @param {*} preset - 当前可视化预设。
 * @returns {*} 根据显示预设选择固定坐标系得到的数据或处理结果。
 */
function fixedFrameFor(preset) {
  return preset?.id === "gazebo_arachne" ? "base_link" : preset?.fixed_frame || "base_link";
}

/**
 * 加载可用话题并更新配置选项。
 * @param {*} preset - 当前可视化预设。
 * @param {*} applyPreset - 是否将预设值应用到选择器。
 * @returns {void} 无返回值；函数通过更新界面或发送请求完成操作。
 */
function populateTopics(preset, applyPreset = false) {
  if (applyPreset) manualTopics.clear();
  for (const [key, select] of Object.entries(topicSelectors)) {
    const options = topics.filter((topic) => topic.type === typeFor[key]);
    replaceOptions(select, options, key === "description" ? "等待 robot_description" : "未选择", !applyPreset);
  }
  const description = preset?.robot_description;
  if (description && !manualTopics.has("description") && (applyPreset || !topicSelectors.description.value)) topicSelectors.description.value = description;
  if (!manualTopics.has("description") && !topics.some((topic) => topic.name === topicSelectors.description.value)) {
    const availableDescription = topics.find((topic) => topic.name.endsWith("robot_description") && topic.type === typeFor.description);
    if (availableDescription) topicSelectors.description.value = availableDescription.name;
  }
  const root = preset?.id === "gazebo_arachne" ? "/sim/arachne" : "/embodied_nexus";
  const joints = topics.find((topic) => topic.type === "sensor_msgs/msg/JointState" &&
    (preset?.id === "gazebo_arachne" ? topic.name === "/sim/arachne/joint_states" : topic.name.endsWith("/arms/arm_main/joint_states")));
  latest.set("selected-joints", joints?.name || "");
  if (applyPreset || !topicSelectors.cloud.value) chooseTopic("cloud", (name) => name === `${root}/lidar/points` || name.endsWith("/lidar_main/points"));
  if (applyPreset || !topicSelectors.cameraCloud.value) chooseTopic("cameraCloud", (name) =>
    name.endsWith("/camera_front/points") || (preset?.id === "gazebo_arachne" && name === `${root}/camera/points`));
  if (applyPreset || !topicSelectors.scan.value) chooseTopic("scan", (name) => name.endsWith("/lidars/lidar_main/scan"));
  if (applyPreset || !topicSelectors.rgb.value) chooseTopic("rgb", (name) => name === `${root}/camera/image_raw` || name.endsWith("/cameras/camera_front/color/image_raw"));
  if (applyPreset || !topicSelectors.depth.value) chooseTopic("depth", (name) => name === `${root}/camera/depth_image_raw` || name.endsWith("/cameras/camera_front/depth/image_raw"));
}

/**
 * 返回指定话题支持的可视化选项。
 * @param {*} topic - ROS 话题对象或名称。
 * @returns {*} 返回指定话题支持的可视化选项得到的数据或处理结果。
 */
function optionsFor(topic) {
  const base = { op: "subscribe", topic: topic.name, type: topic.type, compression: "cbor", queue_length: 1 };
  const options = topic.type === "tf2_msgs/msg/TFMessage"
    ? { ...base, queue_length: 10, throttle_rate: topic.name === "/tf_static" ? 0 : 100 }
    : { ...base, throttle_rate: topic.type === "std_msgs/msg/String" ? 0 : 200 };
  if (explicitQos) {
    if (topic.type === "tf2_msgs/msg/TFMessage") {
      options.qos = topic.name === "/tf_static"
        ? { history: "keep_last", depth: 100, reliability: "reliable", durability: "transient_local" }
        : { history: "keep_last", depth: 100, reliability: "best_effort", durability: "volatile" };
    } else if (topic.type === "std_msgs/msg/String") {
      options.qos = { history: "keep_last", depth: 1, reliability: "reliable", durability: "transient_local" };
    } else {
      options.qos = { history: "keep_last", depth: 1, reliability: "best_effort", durability: "volatile" };
    }
  }
  return options;
}

/**
 * 判断话题是否属于传感器数据。
 * @param {*} name - 设备、状态或字段名称。
 * @returns {*} 判断话题是否属于传感器数据得到的数据或处理结果。
 */
function isSensorTopic(name) {
  return ["sensor_msgs/msg/PointCloud2", "sensor_msgs/msg/LaserScan", "sensor_msgs/msg/Image"].includes(topics.find((topic) => topic.name === name)?.type);
}

/**
 * 通过 WebSocket 连接发送消息。
 * @param {*} message - 要发送或处理的消息内容。
 * @returns {*} 通过 WebSocket 连接发送消息得到的数据或处理结果。
 */
function send(message) {
  const socket = isSensorTopic(message.topic) ? sensorWs : ws;
  if (socket?.readyState === WebSocket.OPEN) socket.send(JSON.stringify(message));
}

/**
 * 订阅指定 ROS 话题。
 * @param {*} topicName - ROS 话题名称。
 * @returns {void} 无返回值；函数通过更新界面或发送请求完成操作。
 */
function subscribe(topicName) {
  const topic = topics.find((item) => item.name === topicName);
  const socket = isSensorTopic(topicName) ? sensorWs : ws;
  if (!topic || socket?.readyState !== WebSocket.OPEN) return;
  if (selection.has(topicName)) return;
  const randomId = globalThis.crypto?.randomUUID?.() ??
    `${Date.now().toString(36)}-${Math.random().toString(36).slice(2, 10)}`;
  const id = `viewer-${randomId}`;
  selection.set(topicName, id);
  send({ ...optionsFor(topic), id });
}

/**
 * 取消指定 ROS 话题的订阅。
 * @param {*} topicName - ROS 话题名称。
 * @returns {void} 无返回值；函数通过更新界面或发送请求完成操作。
 */
function unsubscribe(topicName) {
  const id = selection.get(topicName);
  if (!id) return;
  send({ op: "unsubscribe", topic: topicName, id });
  selection.delete(topicName);
  latest.delete(topicName); preparedData.delete(topicName); pendingSensors.delete(topicName);
}

/**
 * 列出当前启用的 ROS 话题名称。
 * @returns {*} 列出当前启用的 ROS 话题名称得到的数据或处理结果。
 */
function activeTopicNames() {
  return [topicSelectors.description.value, $("lidar-toggle").checked ? topicSelectors.cloud.value : "",
    $("camera-cloud-toggle").checked ? topicSelectors.cameraCloud.value : "", $("lidar-toggle").checked ? topicSelectors.scan.value : "",
    topicSelectors.rgb.value, topicSelectors.depth.value, $("model-toggle").checked ? latest.get("selected-joints") : "", "/tf", "/tf_static"].filter(Boolean);
}

/**
 * 根据当前配置同步 ROS 话题订阅。
 * @returns {void} 无返回值；函数通过更新界面或发送请求完成操作。
 */
function syncSubscriptions() {
  const active = new Set(activeTopicNames());
  for (const name of selection.keys()) if (!active.has(name)) unsubscribe(name);
  for (const name of active) subscribe(name);
}

/**
 * 保存指定类别当前选择的话题。
 * @param {*} key - 话题配置项或字段的键。
 * @param {*} value - 需要转换或显示的输入值。
 * @returns {void} 无返回值；函数通过更新界面或发送请求完成操作。
 */
function setSelectedTopic(key, value) {
  const previous = latest.get(`selected-${key}`);
  if (previous === value) return;
  if (previous) unsubscribe(previous);
  if (previous && previous !== value) {
    if (key === "cloud") removeGeometry("lidar");
    if (key === "cameraCloud") removeGeometry("camera");
    if (key === "scan") removeGeometry("scan");
  }
  if (key === "description" && previous !== value) clearModel();
  latest.set(`selected-${key}`, value);
  syncSubscriptions();
  updateStatusPanel();
}

/**
 * 断开 WebSocket 并清理连接状态。
 * @returns {void} 无返回值；函数通过更新界面或发送请求完成操作。
 */
function disconnect() {
  clearTimeout(reconnectTimer); clearTimeout(wsConnectTimer);
  reconnectTimer = null;
  for (const socket of [ws, sensorWs]) if (socket) {
    socket.onopen = socket.onmessage = socket.onclose = socket.onerror = null;
    socket.close();
  }
  ws = sensorWs = null;
  selection.clear();
  pendingSensors.clear();
}

/**
 * 处理 WebSocket 关闭事件并更新界面状态。
 * @param {*} event - 浏览器派发的连接或输入事件。
 * @returns {*} 处理 WebSocket 关闭事件并更新界面状态得到的数据或处理结果。
 */
function connectionClosed(event = {}) {
  disconnect();
  setConnection(false, event.code === 4401 ? "Core token 无效" : "ROS 连接中断");
  updateStatusPanel();
  if (event.code === 4401 || document.hidden || pageLeaving) return;
  reconnectTimer = window.setTimeout(connect, reconnectDelay);
  reconnectDelay = Math.min(10000, reconnectDelay * 1.7);
}

/**
 * 建立经过令牌授权的 WebSocket 连接。
 * @returns {*} 建立经过令牌授权的 WebSocket 连接得到的数据或处理结果。
 */
function connect() {
  disconnect();
  if (document.hidden || pageLeaving) return;
  const scheme = location.protocol === "https:" ? "wss:" : "ws:";
  /**
 * 处理open。
 * @returns {*} 处理open得到的数据或处理结果。
 */
  const open = () => new WebSocket(`${scheme}//${location.host}${apiUrl}/ws`, ["en-core-viz-v1", `bearer.${token}`]);
  ws = open(); sensorWs = open();
  wsConnectTimer = window.setTimeout(() => {
    if ([ws, sensorWs].some((socket) => socket?.readyState !== WebSocket.OPEN)) connectionClosed();
  }, 8000);
  staticTf.clear(); dynamicTf.clear(); transformCache.clear(); lastClock = 0; tfDirty = true;
  for (const kind of ["lidar", "camera"]) setPointLayerStale(geometryObjects.get(kind), true);
  removeGeometry("scan");
  for (const [key, select] of Object.entries(topicSelectors)) latest.set(`selected-${key}`, select.value);
  latest.set("subscribed-joints", latest.get("selected-joints") || "");
  for (const socket of [ws, sensorWs]) {
    socket.binaryType = "arraybuffer";
    socket.onopen = () => {
      reconnectDelay = 1000;
      if (ws.readyState === WebSocket.OPEN && sensorWs.readyState === WebSocket.OPEN) {
        clearTimeout(wsConnectTimer);
        setConnection(true, rendererError ? "ROS 已连接 · 3D 不可用" : "ROS 已连接");
      }
      syncSubscriptions();
      $("scene-message").textContent = rendererError || "订阅只读 ROS 话题";
      requestRender();
    };
    socket.onmessage = ({ data }) => {
      if (document.hidden || pageLeaving) return;
      if (typeof data === "string") {
        try { handleRosMessage(JSON.parse(data)); } catch (error) { showRuntimeError(error); }
      } else if (socket === sensorWs && decoderWorker) decoderWorker.postMessage(data, [data]);
      else {
        try { handleRosMessage(decodeCborFrame(fallbackDecoder, data)); }
        catch (error) { showRuntimeError(error); }
      }
    };
    socket.onclose = connectionClosed;
    socket.onerror = () => setConnection(false, "ROS 连接异常");
  }
}

if (decoderWorker) {
  decoderWorker.onmessage = ({ data }) => {
    if (document.hidden || pageLeaving) return;
    if (data.ok) handleRosMessage(data.message, data.prepared);
    else showRuntimeError(`CBOR 解码失败：${data.error}`);
  };
  decoderWorker.onerror = () => {
    decoderWorker.terminate();
    decoderWorker = null;
    $("scene-message").textContent = "CBOR Worker 不可用，已切换主线程解码";
  };
}

/**
 * 向数据解码工作线程发送当前配置。
 * @returns {void} 无返回值；函数通过更新界面或发送请求完成操作。
 */
function configureWorker() { decoderWorker?.postMessage({ op: "configure", maxPoints, depthRange }); }

/**
 * 将 ROS 时间戳转换为秒数。
 * @param {*} stamp - 供本函数将 ROS 时间戳转换为秒数使用的输入值。
 * @returns {*} 将 ROS 时间戳转换为秒数得到的数据或处理结果。
 */
function stampSeconds(stamp = {}) { return Number(stamp.sec || 0) + Number(stamp.nanosec || 0) / 1e9; }

/**
 * 将动态或静态 TF 消息写入变换缓存。
 * @param {*} message - 要发送或处理的消息内容。
 * @param {*} isStatic - 供本函数将动态或静态 TF 消息写入变换缓存使用的输入值。
 * @returns {void} 无返回值；函数通过更新界面或发送请求完成操作。
 */
function updateTf(message, isStatic) {
  const transforms = message.transforms || [];
  for (const entry of transforms) {
    const parent = String(entry.header?.frame_id || "").replace(/^\//, "");
    const child = String(entry.child_frame_id || "").replace(/^\//, "");
    if (!parent || !child) continue;
    const stamp = stampSeconds(entry.header?.stamp);
    if (!isStatic && stamp && lastClock && stamp < lastClock - 1) {
      dynamicTf.clear();
      lastClock = stamp;
    } else if (!isStatic && stamp) lastClock = Math.max(lastClock, stamp);
    const item = { parent, child, stamp, static: isStatic,
      translation: entry.transform?.translation || { x: 0, y: 0, z: 0 },
      rotation: entry.transform?.rotation || { x: 0, y: 0, z: 0, w: 1 } };
    const key = `${parent}\u0000${child}`;
    if (isStatic) staticTf.set(key, item);
    else {
      const list = dynamicTf.get(key) || [];
      list.push(item);
      if (list.length > 1 && list.at(-2).stamp > stamp) list.sort((a, b) => a.stamp - b.stamp);
      if (list.length > 120) list.splice(0, list.length - 120);
      dynamicTf.set(key, list);
    }
  }
  transformCache.clear();
  tfDirty = true;
  requestRender();
}

/**
 * 刷新暂停状态下保留的可视化图层。
 * @returns {void} 无返回值；函数通过更新界面或发送请求完成操作。
 */
function refreshPausedLayers() {
  for (const layer of [...pausedLayers]) {
    if (layer === "scan") {
      const scan = latest.get(topicSelectors.scan.value);
      if (scan && lookupTransform($("fixed-frame").value, scan.header?.frame_id || "", stampFromMessage(scan))) handleScan(scan);
    } else {
      const selector = layer === "camera" ? topicSelectors.cameraCloud : topicSelectors.cloud;
      const cloud = latest.get(selector.value);
      if (cloud && lookupTransform($("fixed-frame").value, cloud.header?.frame_id || "", stampFromMessage(cloud))) {
        handleCloud(cloud, layer);
      }
    }
  }
}

/**
 * 筛选指定时间可用的 TF 变换边。
 * @param {*} stampSec - 数据对应的 ROS 时间戳（秒）。
 * @returns {*} 筛选指定时间可用的 TF 变换边得到的数据或处理结果。
 */
function transformEdges(stampSec) {
  const result = [...staticTf.values()];
  for (const values of dynamicTf.values()) {
    for (let index = values.length - 1; index >= 0; index--) {
      const entry = values[index];
      if (entry.stamp > stampSec) continue;
      if (stampSec === Number.POSITIVE_INFINITY || stampSec - entry.stamp <= 1.25) result.push(entry);
      break;
    }
  }
  return result;
}

/**
 * 查找两个坐标系之间的时序变换。
 * @param {*} targetFrame - 查询或显示使用的目标坐标系。
 * @param {*} sourceFrame - 查询变换时使用的源坐标系。
 * @param {*} stampSec - 数据对应的 ROS 时间戳（秒）。
 * @returns {*} 查找两个坐标系之间的时序变换得到的数据或处理结果。
 */
function lookupTransform(targetFrame, sourceFrame, stampSec = lastClock || Number.POSITIVE_INFINITY) {
  const key = `${targetFrame}\u0000${stampSec}`;
  if (!transformCache.has(key)) {
    if (transformCache.size >= 16) transformCache.delete(transformCache.keys().next().value);
    transformCache.set(key, buildTfPoses(transformEdges(stampSec), targetFrame));
  }
  return transformCache.get(key).get(sourceFrame.replace(/^\//, "")) || null;
}

/**
 * 根据当前 TF 缓存更新坐标系选项。
 * @returns {*} 根据当前 TF 缓存更新坐标系选项得到的数据或处理结果。
 */
function refreshFrameOptions() {
  const current = $("fixed-frame").value;
  const frames = new Set(observedSensorFrames);
  for (const edge of [...staticTf.values(), ...[...dynamicTf.values()].flatMap((list) => list.slice(-1))]) {
    frames.add(edge.parent); frames.add(edge.child);
  }
  if (!frames.size) return;
  const signature = [...frames].sort().join("\u0000");
  if (signature !== frameSignature) {
    frameSignature = signature;
    $("fixed-frame").replaceChildren(...[...frames].sort().map((frame) => new Option(frame, frame)));
  }
  $("fixed-frame").value = frames.has(current) ? current : frames.has("odom") ? "odom" : frames.has("base_link") ? "base_link" : [...frames][0];
  $("frame-label").textContent = `固定坐标系：${$("fixed-frame").value}`;
}

/**
 * 记录传感器消息使用的坐标系。
 * @param {*} message - 要发送或处理的消息内容。
 * @returns {void} 无返回值；函数通过更新界面或发送请求完成操作。
 */
function rememberSensorFrame(message) {
  const frame = String(message.header?.frame_id || "").replace(/^\//, "");
  if (frame && !observedSensorFrames.has(frame)) {
    observedSensorFrames.add(frame);
    refreshFrameOptions();
  }
  return frame;
}

/**
 * 获取或创建坐标系的轴向辅助对象。
 * @param {*} frame - 坐标系名称或对应消息字段。
 * @returns {*} 获取或创建坐标系的轴向辅助对象得到的数据或处理结果。
 */
function frameAxis(frame) {
  if (!rootAxes.has(frame)) {
    const helper = new THREE.AxesHelper(0.15);
    rootAxes.set(frame, helper); frameGroup.add(helper);
  }
  return rootAxes.get(frame);
}

/**
 * 更新场景中的坐标系轴向标记。
 * @returns {void} 无返回值；函数通过更新界面或发送请求完成操作。
 */
function updateFrameAxes() {
  if (!$('tf-toggle').checked) { frameGroup.visible = false; return; }
  frameGroup.visible = true;
  const fixed = $("fixed-frame").value;
  const edges = transformEdges(lastClock || Number.POSITIVE_INFINITY);
  const frames = new Set(edges.flatMap((edge) => [edge.parent, edge.child]));
  for (const frame of frames) {
    const pose = lookupTransform(fixed, frame);
    if (!pose) { if (rootAxes.has(frame)) rootAxes.get(frame).visible = false; continue; }
    const helper = frameAxis(frame);
    helper.visible = true; helper.matrixAutoUpdate = false; helper.matrix.copy(pose);
  }
  for (const [frame, helper] of rootAxes) if (!frames.has(frame)) helper.visible = false;
}

/**
 * 根据 TF 数据更新机器人模型位姿。
 * @returns {void} 无返回值；函数通过更新界面或发送请求完成操作。
 */
function updateModelPose() {
  if (!currentModel || !modelRootLink) return;
  const pose = lookupTransform($("fixed-frame").value, modelRootLink);
  if (!pose) {
    currentModel.visible = false;
    $("model-state").textContent = `等待 ${modelRootLink} 到固定坐标系的 TF。`;
    return;
  }
  pose.decompose(currentModel.position, currentModel.quaternion, currentModel.scale);
  currentModel.visible = $("model-toggle").checked;
  $("model-state").textContent = `模型已加载 · 根链接 ${modelRootLink}`;
}

/**
 * 解析 URDF 并加载机器人模型。
 * @param {*} xml - URDF XML 文本。
 * @returns {void} 无返回值；函数通过更新界面或发送请求完成操作。
 */
function loadRobotDescription(xml) {
  if (xml === modelDescription) return;
  try {
    const loader = new URDFLoader(new THREE.LoadingManager(requestRender));
    loader.packages = Object.fromEntries(resourcePackages.map((name) => [name,
      `${location.origin}${apiUrl}/resources/${encodeURIComponent(name)}`]));
    loader.loadMeshCb = loadAuthorizedMesh;
    const robot = loader.parse(xml);
    const urdfDocument = new DOMParser().parseFromString(xml, "application/xml");
    if (urdfDocument.querySelector("parsererror")) throw new Error("URDF XML 格式无效");
    const childLinks = new Set([...urdfDocument.querySelectorAll("joint > child")].map((child) => child.getAttribute("link")));
    clearModel();
    modelRootLink = Object.keys(robot.links).find((name) => !childLinks.has(name)) || Object.keys(robot.links)[0] || "";
    modelGroup.add(robot);
    currentModel = robot;
    modelDescription = xml;
    robot.visible = $("model-toggle").checked;
    $("model-state").textContent = "URDF 已解析，等待 TF";
    controls?.target.set(0, 0, 0.45);
    updateModelPose();
    requestRender();
  } catch (error) {
    $("model-state").textContent = `模型解析失败：${error.message}`;
  }
}

/**
 * 清除当前机器人模型和相关资源。
 * @returns {void} 无返回值；函数通过更新界面或发送请求完成操作。
 */
function clearModel() {
  if (currentModel) currentModel.traverse((object) => {
    object.geometry?.dispose();
    const materials = Array.isArray(object.material) ? object.material : [object.material];
    for (const material of materials) material?.dispose();
  });
  modelGroup.clear();
  currentModel = null;
  modelRootLink = "";
  modelDescription = "";
  $("model-state").textContent = "选择已发布的 URDF 模型话题。";
  requestRender();
}

/**
 * 将二进制 Blob 转换为 data URL。
 * @param {*} blob - 需要转换为 URL 的二进制 Blob。
 * @returns {*} 将二进制 Blob 转换为 data URL得到的数据或处理结果。
 */
function blobDataUrl(blob) {
  return new Promise((resolve, reject) => {
    const reader = new FileReader();
    reader.onload = () => resolve(reader.result);
    reader.onerror = () => reject(reader.error || new Error("模型纹理读取失败"));
    reader.readAsDataURL(blob);
  });
}

/**
 * 通过授权接口加载机器人网格资源。
 * @param {*} path - 接口请求路径。
 * @param {*} manager - 用于跟踪网格资源的加载管理器。
 * @param {*} done - 网格资源加载完成后的回调函数。
 * @returns {Promise<*>} 异步通过授权接口加载机器人网格资源的结果。
 */
async function loadAuthorizedMesh(path, manager, done) {
  try {
    const url = new URL(path, location.href);
    const resourcePrefix = `${apiUrl}/resources/`;
    if (url.origin !== location.origin || !url.pathname.startsWith(resourcePrefix)) {
      throw new Error("URDF 网格资源必须来自 Core 许可的 ROS 包");
    }
    const response = await fetch(url, { headers: apiHeaders(), cache: "force-cache" });
    if (!response.ok) throw new Error(`网格资源请求失败：HTTP ${response.status}`);
    if (/\.stl$/i.test(url.pathname)) {
      const geometry = new STLLoader(manager).parse(await response.arrayBuffer());
      done(new THREE.Mesh(geometry, new THREE.MeshPhongMaterial()));
      requestRender();
      return;
    }
    if (!/\.dae$/i.test(url.pathname)) throw new Error("仅支持 DAE 和 STL 网格");
    const xml = new DOMParser().parseFromString(await response.text(), "application/xml");
    if (xml.querySelector("parsererror")) throw new Error("DAE 网格 XML 格式无效");
    // Legacy FBX COLLADA exports (including Aubo) use a bare zero to mean
    // no transparency. Three.js assumes A_ONE when <transparent> is absent,
    // which instead makes these materials invisible. Keep explicit alpha modes.
    for (const shader of xml.querySelectorAll("profile_COMMON technique > phong, profile_COMMON technique > blinn, profile_COMMON technique > lambert, profile_COMMON technique > constant")) {
      const transparency = shader.querySelector("transparency");
      const value = transparency?.querySelector("float")?.textContent.trim();
      if (!shader.querySelector("transparent") && value && Number(value) === 0) transparency.remove();
    }
    for (const source of xml.querySelectorAll("library_images image init_from")) {
      const reference = source.textContent.trim();
      if (!reference || reference.startsWith("data:")) continue;
      const textureUrl = new URL(reference, url);
      if (textureUrl.origin !== location.origin || !textureUrl.pathname.startsWith(resourcePrefix)) {
        throw new Error("DAE 纹理必须来自 Core 许可的 ROS 包");
      }
      const textureResponse = await fetch(textureUrl, { headers: apiHeaders(), cache: "force-cache" });
      if (!textureResponse.ok) throw new Error(`网格纹理请求失败：HTTP ${textureResponse.status}`);
      source.textContent = await blobDataUrl(await textureResponse.blob());
    }
    const colladaLoader = new ColladaLoader(manager);
    colladaLoader.resourcePath = "";
    const collada = colladaLoader.parse(new XMLSerializer().serializeToString(xml), "");
    done(collada.scene);
    requestRender();
  } catch (error) {
    $("model-state").textContent = `模型资源加载失败：${error.message}`;
    done(null, error);
  }
}

/**
 * 根据关节状态消息更新机器人模型。
 * @param {*} message - 要发送或处理的消息内容。
 * @returns {void} 无返回值；函数通过更新界面或发送请求完成操作。
 */
function updateJoints(message) {
  if (!currentModel) return;
  const positions = Object.fromEntries((message.name || []).map((name, index) => [name, message.position?.[index]]));
  let changed = false;
  for (const [name, position] of Object.entries(positions)) if (Number.isFinite(position)) changed = currentModel.setJointValue(name, position) || changed;
  if (changed) requestRender();
}

/**
 * 从 ROS 消息中读取时间戳（秒）。
 * @param {*} message - 要发送或处理的消息内容。
 * @returns {*} 从 ROS 消息中读取时间戳（秒）得到的数据或处理结果。
 */
function stampFromMessage(message) { return stampSeconds(message.header?.stamp) || lastClock || Number.POSITIVE_INFINITY; }

/**
 * 替换指定类别的点云图层内容。
 * @param {*} buffers - 点云顶点及其数量缓冲区。
 * @param {*} frame - 坐标系名称或对应消息字段。
 * @param {*} stampSec - 数据对应的 ROS 时间戳（秒）。
 * @param {*} kind - 图层或传感器数据类别。
 * @returns {void} 无返回值；函数通过更新界面或发送请求完成操作。
 */
function replaceCloud(buffers, frame, stampSec, kind) {
  if (!buffers || !buffers.count) {
    removeGeometry(kind);
    return;
  }
  const pose = lookupTransform($("fixed-frame").value, frame, stampSec);
  if (!pose) {
    pausedLayers.add(kind);
    const previous = geometryObjects.get(kind);
    if (previous) {
      setPointLayerStale(previous, true);
      previous.visible = kind === "lidar" ? $("lidar-toggle").checked : $("camera-cloud-toggle").checked;
      $("scene-message").textContent = `等待 ${frame} → ${$("fixed-frame").value} 的 TF，暂时保留上一帧`;
    } else {
      $("scene-message").textContent = `等待 ${frame} → ${$("fixed-frame").value} 的 TF`;
    }
    return;
  }
  pausedLayers.delete(kind);
  const pointsObject = updatePointLayer(geometryObjects.get(kind), buffers, pose, pointSize,
    kind === "camera" ? CAMERA_CLOUD_COLOR : null);
  setPointLayerStale(pointsObject, false);
  geometryObjects.set(kind, pointsObject); if (!pointsObject.parent) scene.add(pointsObject);
  pointsObject.visible = kind === "lidar" ? $("lidar-toggle").checked : $("camera-cloud-toggle").checked;
  $("scene-message").textContent = `已渲染 ${buffers.count.toLocaleString()} 个点 · ${frame}`;
  requestRender();
}

/**
 * 处理点云消息并更新对应可视化图层。
 * @param {*} message - 要发送或处理的消息内容。
 * @param {*} kind - 图层或传感器数据类别。
 * @param {*} buffers - 点云顶点及其数量缓冲区。
 * @returns {void} 无返回值；函数通过更新界面或发送请求完成操作。
 */
function handleCloud(message, kind, buffers = pointCloudBuffers(message, maxPoints)) {
  rememberSensorFrame(message);
  replaceCloud(buffers, message.header?.frame_id || "", stampFromMessage(message), kind);
}

/**
 * 处理 LaserScan 消息并更新扫描图层。
 * @param {*} message - 要发送或处理的消息内容。
 * @returns {void} 无返回值；函数通过更新界面或发送请求完成操作。
 */
function handleScan(message) {
  rememberSensorFrame(message);
  const fixed = $("fixed-frame").value, stamp = stampFromMessage(message);
  const pose = lookupTransform(fixed, message.header?.frame_id || "", stamp);
  if (!pose) {
    pausedLayers.add("scan");
    const previous = geometryObjects.get("scan");
    if (previous) previous.visible = false;
    $("scene-message").textContent = `等待雷达坐标变换 ${message.header?.frame_id || "?"} → ${fixed}`;
    return;
  }
  pausedLayers.delete("scan");
  const stride = Math.max(1, Math.ceil(message.ranges.length / maxPoints));
  const points = new Float32Array(Math.ceil(message.ranges.length / stride) * 3);
  let count = 0;
  for (let i = 0; i < message.ranges.length; i += stride) {
    const range = message.ranges[i];
    if (Number.isFinite(range) && range >= message.range_min && range <= message.range_max) {
      const angle = message.angle_min + i * message.angle_increment;
      points[count * 3] = range * Math.cos(angle); points[count * 3 + 1] = range * Math.sin(angle); count++;
    }
  }
  const object = updatePointLayer(geometryObjects.get("scan"), { positions: points, count }, pose, pointSize, 0xffac62);
  geometryObjects.set("scan", object); if (!object.parent) scene.add(object); object.visible = $("lidar-toggle").checked;
  requestRender();
}

/**
 * 将图像消息绘制到指定画布。
 * @param {*} canvas - 用于显示图像的 HTML 画布。
 * @param {*} message - 要发送或处理的消息内容。
 * @param {*} stateId - 要更新的图像或状态类别标识符。
 * @param {*} converted - 预先转换好的像素数据。
 * @returns {void} 无返回值；函数通过更新界面或发送请求完成操作。
 */
function showImage(canvas, message, stateId, converted = imagePixels(message, depthRange)) {
  if (!converted) { $(stateId).textContent = `不支持 ${message.encoding || "未知编码"}`; return; }
  if (canvas.width !== converted.width) canvas.width = converted.width;
  if (canvas.height !== converted.height) canvas.height = converted.height;
  canvas.getContext("2d", { alpha: false }).putImageData(new ImageData(converted.pixels, converted.width, converted.height), 0, 0);
  $(stateId).textContent = `${converted.width} × ${converted.height}`;
}

/**
 * 根据 ROS 消息类型分发传感器数据处理。
 * @param {*} message - 要发送或处理的消息内容。
 * @param {*} prepared - 工作线程预先处理好的消息数据。
 * @returns {void} 无返回值；函数通过更新界面或发送请求完成操作。
 */
function handleRosMessage(message, prepared = null) {
  if (message.op === "core_error") { $("scene-message").textContent = message.error || "ROS subscription error"; return; }
  if (message.op !== "publish") return;
  const topic = message.topic, payload = message.msg || {};
  if (!selection.has(topic)) return;
  const stat = topicStats.get(topic) || { count: 0, last: 0 };
  stat.count++; stat.last = Date.now(); topicStats.set(topic, stat);
  latest.set(topic, payload);
  if (prepared) preparedData.set(topic, prepared);
  else preparedData.delete(topic);
  if (topic === "/tf") updateTf(payload, false);
  else if (topic === "/tf_static") updateTf(payload, true);
  else if (topic === topicSelectors.description.value) loadRobotDescription(payload.data || "");
  else if (topic === latest.get("selected-joints") && payload.name) updateJoints(payload);
  else if (isSensorTopic(topic)) { pendingSensors.add(topic); requestRender(); }
  if (topic !== "/tf" && topic !== "/tf_static") $("empty-state").hidden = true;
}

/**
 * 刷新已缓存的传感器消息。
 * @returns {void} 无返回值；函数通过更新界面或发送请求完成操作。
 */
function flushSensors() {
  for (const topic of pendingSensors) {
    const message = latest.get(topic), prepared = preparedData.get(topic) || undefined;
    if (!message) continue;
    if (topic === topicSelectors.cloud.value) handleCloud(message, "lidar", prepared);
    if (topic === topicSelectors.cameraCloud.value) handleCloud(message, "camera", prepared);
    if (topic === topicSelectors.scan.value) handleScan(message);
    if (topic === topicSelectors.rgb.value) showImage($("rgb-image"), message, "rgb-state", prepared);
    if (topic === topicSelectors.depth.value) showImage($("depth-image"), message, "depth-state", prepared);
  }
  pendingSensors.clear();
}

/**
 * 更新机器人状态面板内容。
 * @returns {void} 无返回值；函数通过更新界面或发送请求完成操作。
 */
function updateStatusPanel() {
  const active = [...selection.keys()];
  const rows = active.map((name) => {
    const stat = topicStats.get(name);
    const age = stat ? `${Math.max(0, Date.now() - stat.last)} ms` : "等待第一帧";
    return `<p><span>${escapeHtml(name.split("/").filter(Boolean).at(-1) || name)}</span><b>${stat ? `${stat.count} 帧 · ${age}` : age}</b></p>`;
  });
  if ([ws, sensorWs].some((socket) => socket?.readyState !== WebSocket.OPEN)) rows.unshift("<p class=\"status-error\">ROS bridge 未连接</p>");
  if (!currentModel) rows.unshift("<p><span>机器人模型</span><b>等待 URDF</b></p>");
  $("data-status").innerHTML = rows.join("") || "<p>请选择需要查看的 ROS 话题。</p>";
}

/**
 * 刷新传感器数据图层。
 * @returns {void} 无返回值；函数通过更新界面或发送请求完成操作。
 */
function refreshSensorLayers() {
  const lidarCloud = latest.get(topicSelectors.cloud.value);
  const cameraCloud = latest.get(topicSelectors.cameraCloud.value);
  const scan = latest.get(topicSelectors.scan.value);
  if (lidarCloud) handleCloud(lidarCloud, "lidar");
  if (cameraCloud) handleCloud(cameraCloud, "camera");
  if (scan) handleScan(scan);
}

/**
 * 转义文本中的 HTML 特殊字符。
 * @param {*} value - 需要转换或显示的输入值。
 * @returns {*} 转义文本中的 HTML 特殊字符得到的数据或处理结果。
 */
function escapeHtml(value) { return value.replace(/[&<>"']/g, (char) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" })[char]); }

/**
 * 设置场景图层的显示状态。
 * @param {*} kind - 图层或传感器数据类别。
 * @param {*} visible - 图层是否显示。
 * @returns {void} 无返回值；函数通过更新界面或发送请求完成操作。
 */
function setVisible(kind, visible) {
  const object = geometryObjects.get(kind);
  if (object) object.visible = visible;
  requestRender();
}

/**
 * 从场景中移除指定几何对象。
 * @param {*} kind - 图层或传感器数据类别。
 * @returns {void} 无返回值；函数通过更新界面或发送请求完成操作。
 */
function removeGeometry(kind) {
  const object = geometryObjects.get(kind);
  if (object) {
    scene.remove(object);
    object.geometry.dispose();
    object.material.dispose();
    geometryObjects.delete(kind);
  }
  pausedLayers.delete(kind);
  requestRender();
}

/**
 * 设置点云图层的数据过期状态。
 * @param {*} object - 需要更新的三维场景对象。
 * @param {*} stale - 点云数据是否已过期。
 * @returns {void} 无返回值；函数通过更新界面或发送请求完成操作。
 */
function setPointLayerStale(object, stale) {
  if (!object || object.userData.stale === stale) return;
  object.userData.stale = stale;
  object.material.transparent = stale;
  object.material.opacity = stale ? 0.45 : 1;
  object.material.depthWrite = !stale;
  object.material.needsUpdate = true;
  requestRender();
}

/**
 * 更新动画状态并渲染当前帧。
 * @param {*} timestamp - 浏览器动画帧的时间戳。
 * @returns {void} 无返回值；函数通过更新界面或发送请求完成操作。
 */
function animate(timestamp) {
  renderFrameId = 0;
  if (document.hidden || pageLeaving) return;
  if (timestamp - lastRenderTime < renderIntervalMs) { requestRender(); return; }
  lastRenderTime = timestamp;
  if (tfDirty) {
    tfDirty = false;
    refreshFrameOptions(); updateModelPose(); updateFrameAxes(); refreshPausedLayers();
  }
  flushSensors();
  controls?.update();
  renderer?.render(scene, camera);
}

/**
 * 读取最新状态并刷新三维场景。
 * @returns {Promise<*>} 异步读取最新状态并刷新三维场景的结果。
 */
async function refresh() {
  if (document.hidden || pageLeaving) return;
  try {
    const info = await loadInfo();
    if (!info) return;
    resourcePackages = info.resource_packages || [];
    explicitQos = Boolean(info.explicit_qos);
    topics = info.topics || [];
    latest.set("presets", info.presets || []);
    if (!presetTouched && $("preset-select").value !== "gazebo_arachne" && topics.some((topic) => topic.name.startsWith("/sim/arachne/"))) {
      $("preset-select").value = "gazebo_arachne";
      /**
 * 处理simulationPreset。
 * @param {*} info.presets || [] - 供本函数处理simulationPreset使用的输入值。
 * @returns {*} 处理simulationPreset得到的数据或处理结果。
 */
      const simulationPreset = (info.presets || []).find((item) => item.id === "gazebo_arachne");
      $("fixed-frame").value = fixedFrameFor(simulationPreset);
      $("frame-label").textContent = `固定坐标系：${$("fixed-frame").value}`;
    }
    if (!info.available) {
      $("empty-state").textContent = info.error || "rosbridge 不可用";
      $("scene-message").textContent = info.error || "rosbridge 不可用";
      setConnection(false, "可视化不可用");
    }
    /**
 * 处理preset。
 * @param {*} info.presets || [] - 供本函数处理preset使用的输入值。
 * @returns {*} 处理preset得到的数据或处理结果。
 */
    const preset = (info.presets || []).find((item) => item.id === $("preset-select").value);
    populateTopics(preset);
    if (!$("fixed-frame").value) $("fixed-frame").value = fixedFrameFor(preset);
    if (info.available && !reconnectTimer && (!ws || ws.readyState === WebSocket.CLOSED)) connect();
    else if (ws?.readyState === WebSocket.OPEN) {
      for (const [key, select] of Object.entries(topicSelectors)) {
        const current = latest.get(`selected-${key}`) || "";
        if (current !== select.value) setSelectedTopic(key, select.value);
      }
      const previousJoint = latest.get("subscribed-joints") || "";
      const selectedJoint = latest.get("selected-joints") || "";
      if (previousJoint !== selectedJoint) {
        if (previousJoint) unsubscribe(previousJoint);
        if (selectedJoint) subscribe(selectedJoint);
        latest.set("subscribed-joints", selectedJoint);
      }
    }
    $("empty-state").hidden = renderer ? Boolean(currentModel || geometryObjects.size) : false;
  } catch (error) {
    $("empty-state").textContent = `Core 可视化接口不可用：${error.message}`;
    setConnection(false, "Core 不可达");
  }
}

for (const [key, select] of Object.entries(topicSelectors)) {
  select.addEventListener("change", () => { manualTopics.add(key); setSelectedTopic(key, select.value); });
}
$("preset-select").addEventListener("change", async () => {
  presetTouched = true;
  const preset = latest.get("presets")?.find((item) => item.id === $("preset-select").value);
  populateTopics(preset, true); $("fixed-frame").value = fixedFrameFor(preset);
  for (const [key, select] of Object.entries(topicSelectors)) setSelectedTopic(key, select.value);
  await refresh();
});
$("refresh-topics").addEventListener("click", () => refresh());
$("reset-view").addEventListener("click", () => {
  camera.position.set(3.8, -4.5, 3);
  if (controls) { controls.target.set(0, 0, 0.45); controls.update(); }
  requestRender();
});
$("fixed-frame").addEventListener("change", () => {
  $("frame-label").textContent = `固定坐标系：${$("fixed-frame").value}`;
  for (const kind of ["lidar", "camera", "scan"]) removeGeometry(kind);
  updateModelPose(); updateFrameAxes(); refreshSensorLayers();
  requestRender();
});
$("model-toggle").addEventListener("change", () => { if (currentModel) currentModel.visible = $("model-toggle").checked; syncSubscriptions(); requestRender(); });
$("tf-toggle").addEventListener("change", () => { updateFrameAxes(); requestRender(); });
$("lidar-toggle").addEventListener("change", (event) => { setVisible("lidar", event.target.checked); setVisible("scan", event.target.checked); syncSubscriptions(); });
$("camera-cloud-toggle").addEventListener("change", (event) => { setVisible("camera", event.target.checked); syncSubscriptions(); });
$("point-size").addEventListener("input", (event) => {
  pointSize = Number(event.target.value); $("point-size-value").value = pointSize.toFixed(3);
  for (const object of geometryObjects.values()) object.material.size = pointSize;
  requestRender();
});
$("max-points").addEventListener("input", (event) => {
  maxPoints = Number(event.target.value); $("max-points-value").value = maxPoints.toLocaleString(); configureWorker();
  for (const select of [topicSelectors.cloud, topicSelectors.cameraCloud]) if (select.value) {
    preparedData.delete(select.value);
    if (latest.has(select.value)) pendingSensors.add(select.value);
  }
  requestRender();
});
$("depth-range").addEventListener("input", (event) => {
  depthRange = Number(event.target.value); $("depth-range-value").value = `0.5–${depthRange} m`;
  configureWorker(); preparedData.delete(topicSelectors.depth.value);
  if (latest.has(topicSelectors.depth.value)) { pendingSensors.add(topicSelectors.depth.value); requestRender(); }
});

document.addEventListener("visibilitychange", () => {
  if (document.hidden) {
    disconnect();
    cancelAnimationFrame(renderFrameId); renderFrameId = 0;
    setConnection(false, "页面已隐藏，订阅暂停");
  } else { refresh(); requestRender(); }
});
window.addEventListener("pagehide", () => { pageLeaving = true; disconnect(); });
window.addEventListener("pageshow", () => { pageLeaving = false; if (token) refresh(); requestRender(); });

$("viewer-status").classList.remove("is-online");
$("empty-state").textContent = rendererError || (token ? "正在连接 ROS graph…" : "请先回到 Core 页面登录。");
window.__webRvizBooted = true;
clearTimeout(window.__webRvizBootTimer);
if (!token) setConnection(false, "需要 Core token");
else refresh();
window.setInterval(() => { if (token) refresh(); }, 5000);
window.setInterval(() => { if (!document.hidden) updateStatusPanel(); }, 500);
requestRender();
