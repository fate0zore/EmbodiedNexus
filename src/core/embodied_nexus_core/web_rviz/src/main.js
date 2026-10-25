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

const $ = (id) => document.getElementById(id);
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
const transformCache = new Map();
const preparedData = new Map();
const pendingSensors = new Set();
let frameSignature = "";
let tfDirty = false;
let renderFrameId = 0;
let lastRenderTime = Number.NEGATIVE_INFINITY;
const renderIntervalMs = 1000 / 30;

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

function setConnection(online, label) {
  const status = $("viewer-status");
  status.classList.toggle("is-online", online);
  $("connection-label").textContent = label;
}

function apiHeaders() { return { Authorization: `Bearer ${token}` }; }

async function loadInfo() {
  const response = await fetch(apiUrl, { headers: apiHeaders(), cache: "no-store" });
  if (response.status === 401) { location.assign("/"); return null; }
  if (!response.ok) throw new Error(`Core visualization API: HTTP ${response.status}`);
  return response.json();
}

function replaceOptions(select, options, emptyLabel = "未选择", keepValue = true) {
  const previous = keepValue ? select.value : "";
  const signature = JSON.stringify([emptyLabel, options]);
  if (select.dataset.options === signature) { if (!keepValue) select.value = ""; return; }
  select.dataset.options = signature;
  select.replaceChildren(new Option(emptyLabel, ""));
  for (const item of options) select.add(new Option(`${item.name}  ·  ${item.type.split("/").at(-1)}`, item.name));
  if (previous && options.some((item) => item.name === previous)) select.value = previous;
}

function chooseTopic(key, predicate) {
  if (manualTopics.has(key)) return;
  const found = topics.find((topic) => predicate(topic.name, topic.type));
  if (found) topicSelectors[key].value = found.name;
}

function fixedFrameFor(preset) {
  return preset?.id === "gazebo_arachne" ? "base_link" : preset?.fixed_frame || "base_link";
}

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

function isSensorTopic(name) {
  return ["sensor_msgs/msg/PointCloud2", "sensor_msgs/msg/LaserScan", "sensor_msgs/msg/Image"].includes(topics.find((topic) => topic.name === name)?.type);
}

function send(message) {
  const socket = isSensorTopic(message.topic) ? sensorWs : ws;
  if (socket?.readyState === WebSocket.OPEN) socket.send(JSON.stringify(message));
}

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

function unsubscribe(topicName) {
  const id = selection.get(topicName);
  if (!id) return;
  send({ op: "unsubscribe", topic: topicName, id });
  selection.delete(topicName);
  latest.delete(topicName); preparedData.delete(topicName); pendingSensors.delete(topicName);
}

function activeTopicNames() {
  return [topicSelectors.description.value, $("lidar-toggle").checked ? topicSelectors.cloud.value : "",
    $("camera-cloud-toggle").checked ? topicSelectors.cameraCloud.value : "", $("lidar-toggle").checked ? topicSelectors.scan.value : "",
    topicSelectors.rgb.value, topicSelectors.depth.value, $("model-toggle").checked ? latest.get("selected-joints") : "", "/tf", "/tf_static"].filter(Boolean);
}

function syncSubscriptions() {
  const active = new Set(activeTopicNames());
  for (const name of selection.keys()) if (!active.has(name)) unsubscribe(name);
  for (const name of active) subscribe(name);
}

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

function connectionClosed(event = {}) {
  disconnect();
  setConnection(false, event.code === 4401 ? "Core token 无效" : "ROS 连接中断");
  updateStatusPanel();
  if (event.code === 4401 || document.hidden || pageLeaving) return;
  reconnectTimer = window.setTimeout(connect, reconnectDelay);
  reconnectDelay = Math.min(10000, reconnectDelay * 1.7);
}

function connect() {
  disconnect();
  if (document.hidden || pageLeaving) return;
  const scheme = location.protocol === "https:" ? "wss:" : "ws:";
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

function configureWorker() { decoderWorker?.postMessage({ op: "configure", maxPoints, depthRange }); }

function stampSeconds(stamp = {}) { return Number(stamp.sec || 0) + Number(stamp.nanosec || 0) / 1e9; }

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

function lookupTransform(targetFrame, sourceFrame, stampSec = lastClock || Number.POSITIVE_INFINITY) {
  const key = `${targetFrame}\u0000${stampSec}`;
  if (!transformCache.has(key)) {
    if (transformCache.size >= 16) transformCache.delete(transformCache.keys().next().value);
    transformCache.set(key, buildTfPoses(transformEdges(stampSec), targetFrame));
  }
  return transformCache.get(key).get(sourceFrame.replace(/^\//, "")) || null;
}

function refreshFrameOptions() {
  const current = $("fixed-frame").value;
  const frames = new Set();
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

function frameAxis(frame) {
  if (!rootAxes.has(frame)) {
    const helper = new THREE.AxesHelper(0.15);
    rootAxes.set(frame, helper); frameGroup.add(helper);
  }
  return rootAxes.get(frame);
}

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

function blobDataUrl(blob) {
  return new Promise((resolve, reject) => {
    const reader = new FileReader();
    reader.onload = () => resolve(reader.result);
    reader.onerror = () => reject(reader.error || new Error("模型纹理读取失败"));
    reader.readAsDataURL(blob);
  });
}

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

function updateJoints(message) {
  if (!currentModel) return;
  const positions = Object.fromEntries((message.name || []).map((name, index) => [name, message.position?.[index]]));
  let changed = false;
  for (const [name, position] of Object.entries(positions)) if (Number.isFinite(position)) changed = currentModel.setJointValue(name, position) || changed;
  if (changed) requestRender();
}

function stampFromMessage(message) { return stampSeconds(message.header?.stamp) || lastClock || Number.POSITIVE_INFINITY; }

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

function handleCloud(message, kind, buffers = pointCloudBuffers(message, maxPoints)) {
  replaceCloud(buffers, message.header?.frame_id || "", stampFromMessage(message), kind);
}

function handleScan(message) {
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

function showImage(canvas, message, stateId, converted = imagePixels(message, depthRange)) {
  if (!converted) { $(stateId).textContent = `不支持 ${message.encoding || "未知编码"}`; return; }
  if (canvas.width !== converted.width) canvas.width = converted.width;
  if (canvas.height !== converted.height) canvas.height = converted.height;
  canvas.getContext("2d", { alpha: false }).putImageData(new ImageData(converted.pixels, converted.width, converted.height), 0, 0);
  $(stateId).textContent = `${converted.width} × ${converted.height}`;
}

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

function refreshSensorLayers() {
  const lidarCloud = latest.get(topicSelectors.cloud.value);
  const cameraCloud = latest.get(topicSelectors.cameraCloud.value);
  const scan = latest.get(topicSelectors.scan.value);
  if (lidarCloud) handleCloud(lidarCloud, "lidar");
  if (cameraCloud) handleCloud(cameraCloud, "camera");
  if (scan) handleScan(scan);
}

function escapeHtml(value) { return value.replace(/[&<>"']/g, (char) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" })[char]); }

function setVisible(kind, visible) {
  const object = geometryObjects.get(kind);
  if (object) object.visible = visible;
  requestRender();
}

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

function setPointLayerStale(object, stale) {
  if (!object || object.userData.stale === stale) return;
  object.userData.stale = stale;
  object.material.transparent = stale;
  object.material.opacity = stale ? 0.45 : 1;
  object.material.depthWrite = !stale;
  object.material.needsUpdate = true;
  requestRender();
}

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
      const simulationPreset = (info.presets || []).find((item) => item.id === "gazebo_arachne");
      $("fixed-frame").value = fixedFrameFor(simulationPreset);
      $("frame-label").textContent = `固定坐标系：${$("fixed-frame").value}`;
    }
    if (!info.available) {
      $("empty-state").textContent = info.error || "rosbridge 不可用";
      $("scene-message").textContent = info.error || "rosbridge 不可用";
      setConnection(false, "可视化不可用");
    }
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
