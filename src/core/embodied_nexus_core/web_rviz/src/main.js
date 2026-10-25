import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { ColladaLoader } from "three/addons/loaders/ColladaLoader.js";
import { STLLoader } from "three/addons/loaders/STLLoader.js";
import URDFLoader from "urdf-loader";
import { imagePixels, lookupTfMatrix, pointCloudBuffers } from "./ros-data.js";
import "./style.css";

const $ = (id) => document.getElementById(id);
const TOKEN_KEY = "embodiedNexusCoreToken";
const apiUrl = "/api/v1/core/visualization";
const token = localStorage.getItem(TOKEN_KEY) || "";
const topicSelectors = {
  description: $("description-topic"), cloud: $("cloud-topic"), scan: $("scan-topic"),
  rgb: $("rgb-topic"), depth: $("depth-topic"),
};
const typeFor = { description: "std_msgs/msg/String", cloud: "sensor_msgs/msg/PointCloud2",
  scan: "sensor_msgs/msg/LaserScan", rgb: "sensor_msgs/msg/Image", depth: "sensor_msgs/msg/Image" };
const selection = new Map();
const latest = new Map();
const topicStats = new Map();
const decoderWorker = new Worker(new URL("./cbor-worker.js", import.meta.url), { type: "module" });
let topics = [];
let resourcePackages = [];
let explicitQos = false;
let ws;
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

const scene = new THREE.Scene();
scene.background = new THREE.Color("#111916");
const camera = new THREE.PerspectiveCamera(54, 1, 0.03, 300);
camera.position.set(3.8, -4.5, 3.0);
camera.up.set(0, 0, 1);
const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false, powerPreference: "high-performance" });
renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 1.75));
renderer.outputColorSpace = THREE.SRGBColorSpace;
renderer.domElement.setAttribute("aria-label", "ROS 3D 可视化场景");
$("three-canvas").prepend(renderer.domElement);
const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0, 0, 0.45);
controls.enableDamping = true;
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
  renderer.setSize(bounds.width, bounds.height, false);
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
  select.replaceChildren(new Option(emptyLabel, ""));
  for (const item of options) select.add(new Option(`${item.name}  ·  ${item.type.split("/").at(-1)}`, item.name));
  if (previous && options.some((item) => item.name === previous)) select.value = previous;
}

function chooseTopic(key, predicate) {
  const found = topics.find((topic) => predicate(topic.name, topic.type));
  if (found) topicSelectors[key].value = found.name;
}

function populateTopics(preset) {
  for (const [key, select] of Object.entries(topicSelectors)) {
    const options = topics.filter((topic) => topic.type === typeFor[key]);
    replaceOptions(select, options, key === "description" ? "等待 robot_description" : "未选择");
  }
  const description = preset?.robot_description;
  if (description) topicSelectors.description.value = description;
  if (!topics.some((topic) => topic.name === topicSelectors.description.value)) {
    const availableDescription = topics.find((topic) => topic.name.endsWith("robot_description") && topic.type === typeFor.description);
    if (availableDescription) topicSelectors.description.value = availableDescription.name;
  }
  const root = preset?.id === "gazebo_arachne" ? "/sim/arachne" : "/embodied_nexus";
  const joints = topics.find((topic) => topic.type === "sensor_msgs/msg/JointState" &&
    (preset?.id === "gazebo_arachne" ? topic.name === "/sim/arachne/joint_states" : topic.name.endsWith("/arms/arm_main/joint_states")));
  latest.set("selected-joints", joints?.name || "");
  chooseTopic("cloud", (name) => name === `${root}/lidar/points` || name.endsWith("/lidar_main/points"));
  if (!topicSelectors.cloud.value) chooseTopic("cloud", (name) => name.endsWith("/camera_front/points"));
  chooseTopic("scan", (name) => name.endsWith("/lidars/lidar_main/scan"));
  chooseTopic("rgb", (name) => name === `${root}/camera/image_raw` || name.endsWith("/cameras/camera_front/color/image_raw"));
  chooseTopic("depth", (name) => name === `${root}/camera/depth_image_raw` || name.endsWith("/cameras/camera_front/depth/image_raw"));
}

function optionsFor(topic) {
  const base = { op: "subscribe", topic: topic.name, type: topic.type, compression: "cbor", queue_length: 1 };
  const options = topic.type === "tf2_msgs/msg/TFMessage"
    ? { ...base, queue_length: 10, throttle_rate: 0 }
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

function send(message) {
  if (ws?.readyState === WebSocket.OPEN) ws.send(JSON.stringify(message));
}

function subscribe(topicName) {
  const topic = topics.find((item) => item.name === topicName);
  if (!topic || !ws || ws.readyState !== WebSocket.OPEN) return;
  if (selection.has(topicName)) return;
  const id = `viewer-${crypto.randomUUID()}`;
  selection.set(topicName, id);
  send({ ...optionsFor(topic), id });
}

function unsubscribe(topicName) {
  const id = selection.get(topicName);
  if (!id) return;
  send({ op: "unsubscribe", topic: topicName, id });
  selection.delete(topicName);
}

function activeTopicNames() {
  return [topicSelectors.description.value, topicSelectors.cloud.value, topicSelectors.scan.value,
    topicSelectors.rgb.value, topicSelectors.depth.value, latest.get("selected-joints"), "/tf", "/tf_static"].filter(Boolean);
}

function setSelectedTopic(key, value) {
  const previous = latest.get(`selected-${key}`);
  if (previous) unsubscribe(previous);
  if (previous && previous !== value) {
    if (key === "cloud") removeGeometry(previous.includes("camera") ? "camera" : "lidar");
    if (key === "scan") removeGeometry("scan");
  }
  if (key === "description" && previous !== value) clearModel();
  latest.set(`selected-${key}`, value);
  if (value) subscribe(value);
  updateStatusPanel();
}

function connect() {
  clearTimeout(reconnectTimer);
  selection.clear();
  const scheme = location.protocol === "https:" ? "wss:" : "ws:";
  ws = new WebSocket(`${scheme}//${location.host}${apiUrl}/ws`, ["en-core-viz-v1", `bearer.${token}`]);
  ws.binaryType = "arraybuffer";
  ws.onopen = () => {
    reconnectDelay = 1000;
    setConnection(true, "ROS 已连接");
    staticTf.clear(); dynamicTf.clear(); lastClock = 0;
    for (const kind of ["lidar", "camera", "scan"]) removeGeometry(kind);
    updateModelPose(); updateFrameAxes();
    for (const [key, select] of Object.entries(topicSelectors)) latest.set(`selected-${key}`, select.value);
    latest.set("subscribed-joints", latest.get("selected-joints") || "");
    for (const name of activeTopicNames()) subscribe(name);
    $("scene-message").textContent = "订阅只读 ROS 话题";
  };
  ws.onmessage = ({ data }) => {
    if (typeof data === "string") {
      try { handleRosMessage(JSON.parse(data)); } catch { /* malformed rosbridge feedback */ }
    } else decoderWorker.postMessage(data, [data]);
  };
  ws.onclose = (event) => {
    setConnection(false, event.code === 4401 ? "Core token 无效" : "ROS 连接中断");
    updateStatusPanel();
    if (event.code === 4401) return;
    reconnectTimer = window.setTimeout(connect, reconnectDelay);
    reconnectDelay = Math.min(10000, reconnectDelay * 1.7);
  };
  ws.onerror = () => { setConnection(false, "ROS 连接异常"); updateStatusPanel(); };
}

decoderWorker.onmessage = ({ data }) => {
  if (data.ok) handleRosMessage(data.message);
  else $("scene-message").textContent = `CBOR 解码失败：${data.error}`;
};

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
      list.sort((a, b) => a.stamp - b.stamp);
      dynamicTf.set(key, list.slice(-120));
    }
  }
  refreshFrameOptions();
  updateModelPose();
  updateFrameAxes();
  for (const layer of [...pausedLayers]) {
    if (layer === "scan") {
      const scan = latest.get(topicSelectors.scan.value);
      if (scan && lookupTransform($("fixed-frame").value, scan.header?.frame_id || "", stampFromMessage(scan))) handleScan(scan);
    } else {
      const cloud = latest.get(topicSelectors.cloud.value);
      if (cloud && lookupTransform($("fixed-frame").value, cloud.header?.frame_id || "", stampFromMessage(cloud))) {
        handleCloud(cloud, layer);
      }
    }
  }
}

function transformEdges(stampSec) {
  const result = [...staticTf.values()];
  for (const values of dynamicTf.values()) {
    const eligible = values.filter((entry) => entry.stamp <= stampSec &&
      (stampSec === Number.POSITIVE_INFINITY || stampSec - entry.stamp <= 1.25));
    if (eligible.length) result.push(eligible.at(-1));
  }
  return result;
}

function lookupTransform(targetFrame, sourceFrame, stampSec = lastClock || Number.POSITIVE_INFINITY) {
  return lookupTfMatrix(transformEdges(stampSec), targetFrame, sourceFrame);
}

function refreshFrameOptions() {
  const current = $("fixed-frame").value;
  const frames = new Set();
  for (const edge of [...staticTf.values(), ...[...dynamicTf.values()].flatMap((list) => list.slice(-1))]) {
    frames.add(edge.parent); frames.add(edge.child);
  }
  if (!frames.size) return;
  $("fixed-frame").replaceChildren(...[...frames].sort().map((frame) => new Option(frame, frame)));
  $("fixed-frame").value = frames.has(current) ? current : frames.has("odom") ? "odom" : [...frames][0];
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
    if (!pose) continue;
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
    const loader = new URDFLoader();
    loader.packages = Object.fromEntries(resourcePackages.map((name) => [name,
      `${location.origin}${apiUrl}/resources/${encodeURIComponent(name)}/`]));
    loader.loadMeshCb = loadAuthorizedMesh;
    const robot = loader.parse(xml);
    const urdfDocument = new DOMParser().parseFromString(xml, "application/xml");
    if (urdfDocument.querySelector("parsererror")) throw new Error("URDF XML 格式无效");
    const childLinks = new Set([...urdfDocument.querySelectorAll("joint > child")].map((child) => child.getAttribute("link")));
    modelRootLink = Object.keys(robot.links).find((name) => !childLinks.has(name)) || Object.keys(robot.links)[0] || "";
    modelGroup.clear();
    modelGroup.add(robot);
    currentModel = robot;
    modelDescription = xml;
    robot.visible = $("model-toggle").checked;
    $("model-state").textContent = "URDF 已解析，等待 TF";
    controls.target.set(0, 0, 0.45);
    updateModelPose();
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
      return;
    }
    if (!/\.dae$/i.test(url.pathname)) throw new Error("仅支持 DAE 和 STL 网格");
    const xml = new DOMParser().parseFromString(await response.text(), "application/xml");
    if (xml.querySelector("parsererror")) throw new Error("DAE 网格 XML 格式无效");
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
  } catch (error) {
    $("model-state").textContent = `模型资源加载失败：${error.message}`;
    done(null, error);
  }
}

function updateJoints(message) {
  if (!currentModel) return;
  const positions = Object.fromEntries((message.name || []).map((name, index) => [name, message.position?.[index]]));
  for (const [name, position] of Object.entries(positions)) if (Number.isFinite(position)) currentModel.setJointValue(name, position);
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
    if (previous) previous.visible = false;
    $("scene-message").textContent = `沒有 ${frame} → ${$("fixed-frame").value} 的 TF，已暂停该图层`;
    return;
  }
  pausedLayers.delete(kind);
  const position = new THREE.BufferAttribute(buffers.positions, 3);
  const color = new THREE.BufferAttribute(buffers.colors, 3);
  const geometry = new THREE.BufferGeometry(); geometry.setAttribute("position", position); geometry.setAttribute("color", color);
  geometry.applyMatrix4(pose); geometry.computeBoundingSphere();
  const material = new THREE.PointsMaterial({ size: pointSize, vertexColors: true, sizeAttenuation: true });
  const pointsObject = new THREE.Points(geometry, material);
  removeGeometry(kind);
  geometryObjects.set(kind, pointsObject); scene.add(pointsObject);
  pointsObject.visible = kind === "lidar" ? $("lidar-toggle").checked : $("camera-cloud-toggle").checked;
  $("scene-message").textContent = `已渲染 ${buffers.count.toLocaleString()} 个点 · ${frame}`;
}

function handleCloud(message, kind) {
  const buffers = pointCloudBuffers(message, maxPoints);
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
  const points = [];
  const stride = Math.max(1, Math.ceil(message.ranges.length / maxPoints));
  for (let i = 0; i < message.ranges.length; i += stride) {
    const range = message.ranges[i];
    if (Number.isFinite(range) && range >= message.range_min && range <= message.range_max) {
      const angle = message.angle_min + i * message.angle_increment;
      const p = new THREE.Vector3(range * Math.cos(angle), range * Math.sin(angle), 0).applyMatrix4(pose);
      points.push(p.x, p.y, p.z);
    }
  }
  const geometry = new THREE.BufferGeometry(); geometry.setAttribute("position", new THREE.Float32BufferAttribute(points, 3));
  const object = new THREE.Points(geometry, new THREE.PointsMaterial({ color: 0xffac62, size: pointSize }));
  removeGeometry("scan");
  geometryObjects.set("scan", object); scene.add(object); object.visible = $("lidar-toggle").checked;
}

function showImage(canvas, message, stateId, depth = false) {
  const converted = imagePixels(message, depthRange);
  if (!converted) { $(stateId).textContent = `不支持 ${message.encoding || "未知编码"}`; return; }
  canvas.width = converted.width; canvas.height = converted.height;
  canvas.getContext("2d", { alpha: false }).putImageData(new ImageData(converted.pixels, converted.width, converted.height), 0, 0);
  $(stateId).textContent = `${converted.width} × ${converted.height}`;
}

function handleRosMessage(message) {
  if (message.op === "core_error") { $("scene-message").textContent = message.error || "ROS subscription error"; return; }
  if (message.op !== "publish") return;
  const topic = message.topic, payload = message.msg || {};
  const stat = topicStats.get(topic) || { count: 0, last: 0 };
  stat.count++; stat.last = Date.now(); topicStats.set(topic, stat);
  latest.set(topic, payload);
  if (topic === "/tf") updateTf(payload, false);
  else if (topic === "/tf_static") updateTf(payload, true);
  else if (topic === topicSelectors.description.value) loadRobotDescription(payload.data || "");
  else if (topic === latest.get("selected-joints") && payload.name) updateJoints(payload);
  else if (topic === topicSelectors.cloud.value) handleCloud(payload, topic.includes("camera") ? "camera" : "lidar");
  else if (topic === topicSelectors.scan.value) handleScan(payload);
  else if (topic === topicSelectors.rgb.value) showImage($("rgb-image"), payload, "rgb-state");
  else if (topic === topicSelectors.depth.value) showImage($("depth-image"), payload, "depth-state", true);
  if (topic !== "/tf" && topic !== "/tf_static") $("empty-state").hidden = true;
  updateStatusPanel();
}

function updateStatusPanel() {
  const active = [...selection.keys()];
  const rows = active.map((name) => {
    const stat = topicStats.get(name);
    const age = stat ? `${Math.max(0, Date.now() - stat.last)} ms` : "等待第一帧";
    return `<p><span>${escapeHtml(name.split("/").filter(Boolean).at(-1) || name)}</span><b>${stat ? `${stat.count} 帧 · ${age}` : age}</b></p>`;
  });
  if (!ws || ws.readyState !== WebSocket.OPEN) rows.unshift("<p class=\"status-error\">ROS bridge 未连接</p>");
  if (!currentModel) rows.unshift("<p><span>机器人模型</span><b>等待 URDF</b></p>");
  $("data-status").innerHTML = rows.join("") || "<p>请选择需要查看的 ROS 话题。</p>";
}

function refreshSensorLayers() {
  const cloud = latest.get(topicSelectors.cloud.value);
  const scan = latest.get(topicSelectors.scan.value);
  if (cloud) handleCloud(cloud, topicSelectors.cloud.value.includes("camera") ? "camera" : "lidar");
  if (scan) handleScan(scan);
}

function escapeHtml(value) { return value.replace(/[&<>"']/g, (char) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" })[char]); }

function setVisible(kind, visible) {
  const object = geometryObjects.get(kind);
  if (object) object.visible = visible;
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
}

function animate() {
  requestAnimationFrame(animate);
  controls.update();
  const visibleTime = lastClock || Number.POSITIVE_INFINITY;
  if (visibleTime !== Number.POSITIVE_INFINITY) updateModelPose();
  renderer.render(scene, camera);
}

async function refresh() {
  try {
    const info = await loadInfo();
    if (!info) return;
    resourcePackages = info.resource_packages || [];
    explicitQos = Boolean(info.explicit_qos);
    topics = info.topics || [];
    latest.set("presets", info.presets || []);
    if (!info.available) {
      $("empty-state").textContent = info.error || "rosbridge 不可用";
      $("scene-message").textContent = info.error || "rosbridge 不可用";
      setConnection(false, "可视化不可用");
    }
    const preset = (info.presets || []).find((item) => item.id === $("preset-select").value);
    populateTopics(preset);
    if (!$("fixed-frame").value) $("fixed-frame").value = preset?.fixed_frame || "base_link";
    if (info.available && (!ws || ws.readyState === WebSocket.CLOSED)) connect();
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
    $("empty-state").hidden = Boolean(currentModel || geometryObjects.size);
  } catch (error) {
    $("empty-state").textContent = `Core 可视化接口不可用：${error.message}`;
    setConnection(false, "Core 不可达");
  }
}

for (const [key, select] of Object.entries(topicSelectors)) {
  select.addEventListener("change", () => setSelectedTopic(key, select.value));
}
$("preset-select").addEventListener("change", async () => {
  const preset = latest.get("presets")?.find((item) => item.id === $("preset-select").value);
  populateTopics(preset); $("fixed-frame").value = preset?.fixed_frame || "base_link";
  for (const [key, select] of Object.entries(topicSelectors)) setSelectedTopic(key, select.value);
  await refresh();
});
$("refresh-topics").addEventListener("click", () => refresh());
$("reset-view").addEventListener("click", () => { camera.position.set(3.8, -4.5, 3); controls.target.set(0, 0, 0.45); controls.update(); });
$("fixed-frame").addEventListener("change", () => {
  $("frame-label").textContent = `固定坐标系：${$("fixed-frame").value}`;
  updateModelPose(); updateFrameAxes(); refreshSensorLayers();
});
$("model-toggle").addEventListener("change", () => { if (currentModel) currentModel.visible = $("model-toggle").checked; });
$("tf-toggle").addEventListener("change", updateFrameAxes);
$("lidar-toggle").addEventListener("change", (event) => { setVisible("lidar", event.target.checked); setVisible("scan", event.target.checked); });
$("camera-cloud-toggle").addEventListener("change", (event) => setVisible("camera", event.target.checked));
$("point-size").addEventListener("input", (event) => {
  pointSize = Number(event.target.value); $("point-size-value").value = pointSize.toFixed(3);
  for (const object of geometryObjects.values()) object.material.size = pointSize;
});
$("max-points").addEventListener("input", (event) => { maxPoints = Number(event.target.value); $("max-points-value").value = maxPoints.toLocaleString(); });
$("depth-range").addEventListener("input", (event) => {
  depthRange = Number(event.target.value); $("depth-range-value").value = `0.5–${depthRange} m`;
  const message = latest.get(topicSelectors.depth.value); if (message) showImage($("depth-image"), message, "depth-state", true);
});

$("viewer-status").classList.remove("is-online");
$("empty-state").textContent = token ? "正在连接 ROS graph…" : "请先回到 Core 页面登录。";
if (!token) setConnection(false, "需要 Core token");
else refresh();
window.setInterval(() => { if (token) refresh(); }, 5000);
animate();
