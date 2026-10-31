import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { TransformControls } from "three/addons/controls/TransformControls.js";
import { STLLoader } from "three/addons/loaders/STLLoader.js";
import { OBJLoader } from "three/addons/loaders/OBJLoader.js";
import { ColladaLoader } from "three/addons/loaders/ColladaLoader.js";
import URDFLoader from "urdf-loader";
import "./scene-editor.css";

const TOKEN_KEY = "embodiedNexusCoreToken";
const API = "/api/v1/core/scenes";
const token = localStorage.getItem(TOKEN_KEY) || "";
/**
 * Look up one editor element by its DOM ID.
 * @param {string} id - Element identifier.
 * @returns {HTMLElement} The matching editor element.
 */
const byId = (id) => document.getElementById(id);
const viewport = byId("viewport");
const statusLabel = byId("editor-status");
const sceneSelect = byId("scene-select");
const scene = new THREE.Scene();
scene.background = new THREE.Color("#0f1512");
const camera = new THREE.PerspectiveCamera(52, 1, 0.03, 250);
camera.up.set(0, 0, 1);
camera.position.set(4.2, -5.2, 3.7);
const renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: "default" });
renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 1.5));
renderer.outputColorSpace = THREE.SRGBColorSpace;
viewport.append(renderer.domElement);
const orbit = new OrbitControls(camera, renderer.domElement);
orbit.target.set(0, 0, 0.5);
orbit.enableDamping = true;
orbit.addEventListener("change", renderFrame);
const transform = new TransformControls(camera, renderer.domElement);
scene.add(transform.getHelper());
transform.addEventListener("dragging-changed", (event) => { orbit.enabled = !event.value; });
transform.addEventListener("objectChange", updateFromGizmo);
scene.add(new THREE.HemisphereLight(0xdceee0, 0x26322b, 2.0));
const keyLight = new THREE.DirectionalLight(0xffffff, 2.2);
keyLight.position.set(4, -3, 8);
scene.add(keyLight);
const grid = new THREE.GridHelper(12, 24, 0x54725c, 0x2b3b31);
grid.rotation.x = Math.PI / 2;
scene.add(grid);
scene.add(new THREE.AxesHelper(0.5));
const workcell = new THREE.Group();
scene.add(workcell);
const robotGroup = new THREE.Group();
scene.add(robotGroup);
let robotModel = null;
let currentId = "";
let currentScene = null;
let selectedId = "";
let savedSceneIds = new Set();
let renderPending = false;

/**
 * Update the editor status message.
 * @param {string} message - Status text to display.
 * @param {string} [kind] - Optional visual state such as error or success.
 * @returns {void} No return value.
 */
function setStatus(message, kind = "") {
  statusLabel.textContent = message;
  statusLabel.className = kind;
}

/**
 * Send an authenticated request to the Core scene API.
 * @param {string} path - API path relative to the scenes endpoint.
 * @param {RequestInit} [options] - Fetch options and optional JSON body.
 * @returns {Promise<any>} The decoded JSON response or an empty object.
 */
async function request(path, options = {}) {
  const headers = new Headers(options.headers || {});
  headers.set("Authorization", `Bearer ${token}`);
  let body = options.body;
  if (body !== undefined && typeof body !== "string" && !(body instanceof Blob)) {
    headers.set("Content-Type", "application/json");
    body = JSON.stringify(body);
  }
  const response = await fetch(`${API}${path}`, { ...options, headers, body, cache: "no-store" });
  const text = response.status === 204 ? "" : await response.text();
  let payload = {};
  if (text) {
    if (response.headers.get("content-type")?.includes("application/json")) {
      try { payload = JSON.parse(text); } catch { payload = { error: text }; }
    } else payload = text;
  }
  if (response.status === 401) { location.assign("/"); throw new Error("Core token 无效，请返回主页重新连接。"); }
  if (!response.ok) {
    const error = new Error(payload.error || `HTTP ${response.status}`);
    error.payload = payload;
    error.status = response.status;
    throw error;
  }
  return payload;
}

/**
 * Return a deep editable copy of a scene definition.
 * @param {Object} source - Parsed scene mapping.
 * @returns {Object} A JSON-compatible independent scene copy.
 */
function cloneScene(source) { return JSON.parse(JSON.stringify(source)); }

/**
 * Schedule one WebGL render after scene or camera changes.
 * @returns {void} No return value.
 */
function renderFrame() {
  if (renderPending) return;
  renderPending = true;
  requestAnimationFrame(() => {
    renderPending = false;
    renderer.render(scene, camera);
  });
}

/**
 * Resize the WebGL renderer to fit the editor viewport.
 * @returns {void} No return value.
 */
function resizeViewport() {
  const bounds = viewport.getBoundingClientRect();
  if (!bounds.width || !bounds.height) return;
  camera.aspect = bounds.width / bounds.height;
  camera.updateProjectionMatrix();
  renderer.setSize(bounds.width, bounds.height, false);
  renderFrame();
}

/**
 * Convert an RGBA array into a CSS color input value.
 * @param {number[]} rgba - Four normalized color components.
 * @returns {string} A six-digit hexadecimal CSS color.
 */
function colorHex(rgba = [0.6, 0.6, 0.6, 1]) {
  return `#${rgba.slice(0, 3).map((value) => Math.round(value * 255).toString(16).padStart(2, "0")).join("")}`;
}

/**
 * Convert a CSS color input value into normalized RGBA components.
 * @param {string} hex - Six-digit hexadecimal color.
 * @param {number} [alpha] - Alpha value to preserve.
 * @returns {number[]} Four normalized color components.
 */
function rgbaFromHex(hex, alpha = 1) {
  return [1, 3, 5].map((offset) => parseInt(hex.slice(offset, offset + 2), 16) / 255).concat(alpha);
}

/**
 * Resolve the currently selected scene object.
 * @returns {Object|null} The selected scene record, if it exists.
 */
function selectedObject() { return currentScene?.objects?.find((object) => object.id === selectedId) || null; }

/**
 * Dispose geometry and materials below a Three.js object.
 * @param {THREE.Object3D} root - Object tree to release.
 * @returns {void} No return value.
 */
function disposeTree(root) {
  root.traverse((object) => {
    object.geometry?.dispose();
    const materials = Array.isArray(object.material) ? object.material : [object.material];
    materials.forEach((material) => material?.dispose());
  });
}

/**
 * Load a scene mesh from the authenticated asset endpoint.
 * @param {Object} object - Scene object with mesh path and scale fields.
 * @returns {Promise<THREE.Object3D>} Parsed mesh group for preview.
 */
async function loadSceneMesh(object) {
  const parts = object.mesh.split("/");
  const filename = parts.at(-1);
  const response = await fetch(`${API}/${encodeURIComponent(currentId)}/assets/${encodeURIComponent(filename)}`,
    { headers: { Authorization: `Bearer ${token}` }, cache: "no-store" });
  if (!response.ok) throw new Error(`网格读取失败：HTTP ${response.status}`);
  const bytes = await response.arrayBuffer();
  const extension = filename.toLowerCase().split(".").at(-1);
  let root;
  if (extension === "stl") {
    const geometry = new STLLoader().parse(bytes);
    root = new THREE.Mesh(geometry, new THREE.MeshStandardMaterial({ color: colorHex(object.rgba), roughness: 0.8 }));
  } else {
    root = new OBJLoader().parse(new TextDecoder().decode(bytes));
    root.traverse((child) => {
      if (child.isMesh) child.material = new THREE.MeshStandardMaterial({ color: colorHex(object.rgba), roughness: 0.8 });
    });
  }
  root.scale.fromArray(object.scale || [1, 1, 1]);
  return root;
}

/**
 * Create a Three.js preview for a saved or draft scene object.
 * @param {Object} object - Scene object record.
 * @returns {Promise<THREE.Group>} A positioned Three.js object group.
 */
async function makeObjectGroup(object) {
  const group = new THREE.Group();
  let mesh;
  const material = new THREE.MeshStandardMaterial({ color: colorHex(object.rgba), roughness: 0.82 });
  if (object.shape === "box") {
    mesh = new THREE.Mesh(new THREE.BoxGeometry(...object.size), material);
  } else if (object.shape === "sphere") {
    mesh = new THREE.Mesh(new THREE.SphereGeometry(object.radius, 28, 20), material);
  } else if (object.shape === "cylinder") {
    mesh = new THREE.Mesh(new THREE.CylinderGeometry(object.radius, object.radius, object.height, 28), material);
    mesh.rotation.x = Math.PI / 2;
  } else if (object.shape === "mesh") {
    try { mesh = await loadSceneMesh(object); }
    catch (error) {
      setStatus(`${object.id} 网格预览失败：${error.message}`, "error");
      mesh = new THREE.Mesh(new THREE.BoxGeometry(0.3, 0.3, 0.3), material);
    }
  }
  if (!mesh) return group;
  mesh.name = object.id;
  group.add(mesh);
  group.position.fromArray(object.position || [0, 0, 0]);
  group.rotation.order = "XYZ";
  group.rotation.set(...(object.rotation || [0, 0, 0]));
  group.userData.sceneObjectId = object.id;
  return group;
}

/**
 * Rebuild preview objects, ground, and selectable object rows from the current scene.
 * @returns {Promise<void>} Resolves after all scene object previews are ready.
 */
async function renderWorkcell() {
  transform.detach();
  for (const child of [...workcell.children]) { disposeTree(child); workcell.remove(child); }
  const ground = currentScene?.ground || { size: [20, 20, 0.1], rgba: [0.75, 0.75, 0.75, 1] };
  const floor = new THREE.Mesh(new THREE.PlaneGeometry(ground.size[0], ground.size[1]),
    new THREE.MeshStandardMaterial({ color: colorHex(ground.rgba), roughness: 1 }));
  floor.position.z = 0;
  floor.name = "ground-preview";
  workcell.add(floor);
  const rows = [];
  for (const object of currentScene?.objects || []) {
    const group = await makeObjectGroup(object);
    workcell.add(group);
    rows.push(group);
  }
  renderObjectList();
  const active = rows.find((row) => row.userData.sceneObjectId === selectedId);
  if (active) transform.attach(active);
  renderFrame();
}

/**
 * Render object names and color swatches in the scene outline.
 * @returns {void} No return value.
 */
function renderObjectList() {
  const list = byId("object-list");
  const objects = currentScene?.objects || [];
  byId("object-count").textContent = String(objects.length);
  list.replaceChildren();
  for (const object of objects) {
    const row = document.createElement("button");
    row.type = "button";
    row.className = `object-row${object.id === selectedId ? " active" : ""}`;
    const swatch = document.createElement("span");
    swatch.className = "swatch";
    swatch.style.background = colorHex(object.rgba);
    const label = document.createElement("span");
    label.textContent = `${object.id} · ${object.shape}`;
    row.append(swatch, label);
    row.addEventListener("click", () => selectObject(object.id));
    list.append(row);
  }
}

/**
 * Populate the selected object's property controls.
 * @returns {void} No return value.
 */
function renderProperties() {
  const object = selectedObject();
  byId("empty-selection").hidden = Boolean(object);
  byId("properties").hidden = !object;
  if (!object) { renderObjectList(); return; }
  byId("object-id").value = object.id;
  byId("object-shape").value = object.shape;
  ["x", "y", "z"].forEach((axis, index) => { byId(`pos-${axis}`).value = Number(object.position[index]).toFixed(3); });
  ["x", "y", "z"].forEach((axis, index) => { byId(`rot-${axis}`).value = (object.rotation[index] * 180 / Math.PI).toFixed(1); });
  byId("box-fields").hidden = object.shape !== "box";
  byId("round-fields").hidden = !["sphere", "cylinder"].includes(object.shape);
  byId("height-field").hidden = object.shape !== "cylinder";
  byId("mesh-fields").hidden = object.shape !== "mesh";
  if (object.shape === "box") ["x", "y", "z"].forEach((axis, index) => { byId(`size-${axis}`).value = Number(object.size[index]).toFixed(3); });
  if (["sphere", "cylinder"].includes(object.shape)) byId("radius").value = Number(object.radius).toFixed(3);
  if (object.shape === "cylinder") byId("height").value = Number(object.height).toFixed(3);
  if (object.shape === "mesh") {
    ["x", "y", "z"].forEach((axis, index) => { byId(`scale-${axis}`).value = Number(object.scale?.[index] || 1).toFixed(3); });
    byId("mesh-path").textContent = object.mesh || "尚未上传网格";
  }
  byId("object-color").value = colorHex(object.rgba);
  byId("object-motion").value = object.motion || "static";
  byId("mass-field").hidden = object.motion !== "dynamic";
  byId("object-mass").value = object.mass_kg ?? 1;
  renderObjectList();
}

/**
 * Select one scene object and attach the transform gizmo to it.
 * @param {string} objectId - Scene object identifier.
 * @returns {void} No return value.
 */
function selectObject(objectId) {
  selectedId = objectId;
  renderProperties();
  const group = workcell.children.find((child) => child.userData.sceneObjectId === objectId);
  if (group) transform.attach(group); else transform.detach();
  renderFrame();
}

/**
 * Copy gizmo changes back into the selected scene object and its controls.
 * @returns {void} No return value.
 */
function updateFromGizmo() {
  const object = selectedObject();
  const group = transform.object;
  if (!object || !group) return;
  object.position = group.position.toArray();
  object.rotation = group.rotation.toArray().slice(0, 3);
  renderProperties();
}

/**
 * Update the selected object from its property panel and rebuild the preview.
 * @returns {Promise<void>} Resolves after the preview has been refreshed.
 */
async function applyProperties() {
  const object = selectedObject();
  if (!object) return;
  const nextId = byId("object-id").value.trim();
  if (!/^[A-Za-z][A-Za-z0-9_-]{0,47}$/.test(nextId) ||
      currentScene.objects.some((candidate) => candidate.id === nextId && candidate !== object)) {
    setStatus("物体 ID 格式无效或重复。", "error");
    byId("object-id").value = object.id;
    return;
  }
  const previousShape = object.shape;
  const nextShape = byId("object-shape").value;
  if (nextShape === "mesh" && previousShape !== "mesh") {
    byId("object-shape").value = previousShape;
    setStatus("请通过“导入 STL / OBJ”创建网格物体。", "error");
    return;
  }
  object.id = nextId;
  selectedId = nextId;
  object.shape = nextShape;
  object.position = ["x", "y", "z"].map((axis) => Number(byId(`pos-${axis}`).value));
  object.rotation = ["x", "y", "z"].map((axis) => Number(byId(`rot-${axis}`).value) * Math.PI / 180);
  object.rgba = rgbaFromHex(byId("object-color").value, object.rgba?.[3] ?? 1);
  object.motion = byId("object-motion").value;
  if (previousShape !== nextShape) {
    delete object.size;
    delete object.radius;
    delete object.height;
    delete object.mesh;
    delete object.scale;
    if (nextShape === "box") object.size = [0.5, 0.5, 0.5];
    if (nextShape === "sphere") object.radius = 0.25;
    if (nextShape === "cylinder") { object.radius = 0.2; object.height = 0.6; }
    if (object.motion === "dynamic") object.mass_kg = Number(byId("object-mass").value || 1);
    else delete object.mass_kg;
    await renderWorkcell();
    renderProperties();
    setStatus("有未保存的场景修改。", "");
    return;
  }
  if (object.shape === "box") object.size = ["x", "y", "z"].map((axis) => Number(byId(`size-${axis}`).value));
  if (["sphere", "cylinder"].includes(object.shape)) object.radius = Number(byId("radius").value);
  if (object.shape === "cylinder") object.height = Number(byId("height").value);
  if (object.shape === "mesh") object.scale = ["x", "y", "z"].map((axis) => Number(byId(`scale-${axis}`).value));
  if (object.motion === "dynamic") object.mass_kg = Number(byId("object-mass").value);
  else delete object.mass_kg;
  await renderWorkcell();
  renderProperties();
  setStatus("有未保存的场景修改。", "");
}

/**
 * Refresh the named scene selector from the scene library API.
 * @returns {Promise<void>} Resolves after the selector has been updated.
 */
async function refreshSceneList() {
  const result = await request("");
  const previous = sceneSelect.value;
  sceneSelect.replaceChildren(new Option("选择场景…", ""));
  savedSceneIds = new Set();
  for (const item of result.scenes || []) {
    savedSceneIds.add(item.id);
    sceneSelect.add(new Option(`${item.name} · ${item.id}`, item.id));
  }
  if (currentId && savedSceneIds.has(currentId)) sceneSelect.value = currentId;
  else if (previous && savedSceneIds.has(previous)) sceneSelect.value = previous;
}

/**
 * Open a saved scene by identifier and update the editor panels.
 * @param {string} sceneId - Saved scene identifier.
 * @returns {Promise<void>} Resolves after the scene has loaded.
 */
async function openScene(sceneId) {
  if (!sceneId) return;
  setStatus(`正在打开 ${sceneId}…`);
  const payload = await request(`/${encodeURIComponent(sceneId)}`);
  currentId = sceneId;
  currentScene = cloneScene(payload);
  currentScene.ground ||= { size: [20, 20, 0.1], rgba: [0.75, 0.75, 0.75, 1] };
  currentScene.objects ||= [];
  selectedId = currentScene.objects[0]?.id || "";
  byId("scene-name").value = currentScene.name;
  sceneSelect.value = sceneId;
  await renderWorkcell();
  renderProperties();
  setStatus(`已打开：${currentScene.name}`);
}

/**
 * Create an in-memory scene draft with a new stable identifier.
 * @returns {void} No return value.
 */
function newScene() {
  const name = prompt("新场景名称", "新场景");
  if (!name) return;
  currentId = `scene-${Date.now()}`;
  currentScene = { name: name.trim(), ground: { size: [20, 20, 0.1], rgba: [0.75, 0.75, 0.75, 1] }, objects: [] };
  selectedId = "";
  sceneSelect.value = "";
  byId("scene-name").value = currentScene.name;
  renderWorkcell();
  renderProperties();
  setStatus("新场景草稿；保存后才会写入 YAML。", "success");
}

/**
 * Validate and save the current scene as YAML through the Core API.
 * @returns {Promise<void>} Resolves after the scene has been validated and saved.
 */
async function saveScene() {
  if (!currentScene || !currentId) return false;
  currentScene.name = byId("scene-name").value.trim();
  setStatus("正在校验 MuJoCo 模型…");
  try {
    const validation = await request(`/${encodeURIComponent(currentId)}/validate`, { method: "POST", body: currentScene });
    await request(`/${encodeURIComponent(currentId)}`, { method: "PUT", body: currentScene });
    await refreshSceneList();
    sceneSelect.value = currentId;
    setStatus(`已保存 · ${validation.body_count} bodies / ${validation.geom_count} geoms`, "success");
    return true;
  } catch (error) { setStatus(`保存失败：${error.message}`, "error"); return false; }
}

/**
 * Validate, save, and hot-apply the current scene draft to the running MuJoCo node.
 * @returns {Promise<void>} Resolves after the apply request has completed.
 */
async function applyScene() {
  if (!currentScene || !currentId) return;
  currentScene.name = byId("scene-name").value.trim();
  const button = byId("apply-scene");
  button.disabled = true;
  setStatus("正在校验、保存并应用场景…");
  try {
    const result = await request(`/${encodeURIComponent(currentId)}/apply`, {
      method: "POST", body: currentScene,
    });
    setStatus(`${result.message || "场景已应用"} · ${result.body_count} bodies / ${result.geom_count} geoms`, "success");
    try {
      await refreshSceneList();
      sceneSelect.value = currentId;
    } catch {
      setStatus(`${result.message || "场景已应用"} · 场景列表刷新失败`, "success");
    }
  } catch (error) {
    if (error.payload?.saved) {
      setStatus(`场景已保存，但尚未应用：${error.message}`, "error");
    } else {
      setStatus(`场景校验或保存失败，运行场景未改变：${error.message}`, "error");
    }
  } finally {
    button.disabled = false;
  }
}

/**
 * Copy the current scene to a new identifier and persist the copy.
 * @returns {Promise<void>} Resolves after the new scene has been saved.
 */
async function saveAsScene() {
  if (!currentScene) return;
  const requested = prompt("另存为的新场景 ID（仅字母、数字、下划线和短横线）", `scene-${Date.now()}`);
  if (!requested) return;
  if (!/^[A-Za-z][A-Za-z0-9_-]{0,63}$/.test(requested)) { setStatus("场景 ID 格式无效。", "error"); return; }
  const oldId = currentId;
  if (requested === oldId || savedSceneIds.has(requested)) { setStatus("另存为需要一个尚未使用的场景 ID。", "error"); return; }
  const oldScene = currentScene;
  const copy = cloneScene(currentScene);
  try {
    for (const object of copy.objects.filter((item) => item.shape === "mesh")) {
      const filename = object.mesh.split("/").at(-1);
      const response = await fetch(`${API}/${encodeURIComponent(oldId)}/assets/${encodeURIComponent(filename)}`,
        { headers: { Authorization: `Bearer ${token}` }, cache: "no-store" });
      if (!response.ok) throw new Error(`无法复制 ${filename}：HTTP ${response.status}`);
      const file = await response.blob();
      const uploaded = await request(`/${encodeURIComponent(requested)}/assets/${encodeURIComponent(filename)}`,
        { method: "PUT", body: file });
      object.mesh = uploaded.path;
    }
  } catch (error) { setStatus(`另存为失败：${error.message}`, "error"); return; }
  copy.name = `${copy.name} 副本`;
  currentId = requested;
  currentScene = copy;
  const saved = await saveScene();
  if (!saved) { currentId = oldId; currentScene = oldScene; renderProperties(); }
}

/**
 * Rename the current scene's display name without changing its stable file ID.
 * @returns {Promise<void>} Resolves after the renamed scene is saved.
 */
async function renameScene() {
  if (!currentScene) return;
  const name = prompt("场景名称", currentScene.name);
  if (!name?.trim()) return;
  currentScene.name = name.trim();
  byId("scene-name").value = currentScene.name;
  await saveScene();
}

/**
 * Delete the selected saved scene and open the default scene.
 * @returns {Promise<void>} Resolves after deletion and fallback loading.
 */
async function deleteScene() {
  if (!currentId || currentId === "default" || !savedSceneIds.has(currentId)) return;
  if (!confirm(`删除场景“${currentScene.name}”及其上传网格？`)) return;
  try {
    await request(`/${encodeURIComponent(currentId)}`, { method: "DELETE" });
    await refreshSceneList();
    await openScene("default");
  } catch (error) { setStatus(`删除失败：${error.message}`, "error"); }
}

/**
 * Add a primitive object to the current scene draft.
 * @param {string} shape - Primitive shape name.
 * @returns {Promise<void>} Resolves after the new object is previewed.
 */
async function addPrimitive(shape) {
  if (!currentScene) newScene();
  if (!currentScene) return;
  const base = `${shape}-${currentScene.objects.length + 1}`;
  let id = base;
  let suffix = 2;
  while (currentScene.objects.some((object) => object.id === id)) id = `${base}-${suffix++}`;
  const object = { id, shape, position: [0, 0, 0.5], rotation: [0, 0, 0], rgba: [0.28, 0.72, 0.42, 1], motion: "static" };
  if (shape === "box") object.size = [0.5, 0.5, 0.5];
  if (shape === "sphere") object.radius = 0.25;
  if (shape === "cylinder") { object.radius = 0.2; object.height = 0.6; }
  currentScene.objects.push(object);
  selectedId = id;
  await renderWorkcell();
  renderProperties();
  setStatus(`已添加 ${id}；保存后写入场景配置。`);
}

/**
 * Upload a selected STL or OBJ file and add it as a mesh object.
 * @param {File} file - Browser-selected STL or OBJ mesh file.
 * @returns {Promise<void>} Resolves after upload and preview creation.
 */
async function uploadMesh(file) {
  if (!file) return;
  if (!currentScene) newScene();
  if (!currentScene) return;
  const sceneId = currentId;
  setStatus(`正在上传 ${file.name}…`);
  try {
    const result = await request(`/${encodeURIComponent(sceneId)}/assets/${encodeURIComponent(file.name)}`,
      { method: "PUT", body: file });
    let id = file.name.replace(/\.[^.]+$/, "").replace(/[^A-Za-z0-9_-]+/g, "-");
    if (!/^[A-Za-z]/.test(id)) id = `mesh-${id}`;
    id = id.slice(0, 48) || `mesh-${Date.now()}`;
    let uniqueId = id;
    let suffix = 2;
    while (currentScene.objects.some((object) => object.id === uniqueId)) uniqueId = `${id}-${suffix++}`;
    currentScene.objects.push({ id: uniqueId, shape: "mesh", mesh: result.path, scale: [1, 1, 1],
      position: [0, 0, 0.5], rotation: [0, 0, 0], rgba: [0.68, 0.72, 0.76, 1], motion: "static" });
    selectedId = uniqueId;
    await renderWorkcell();
    renderProperties();
    setStatus(`网格已上传：${file.name}；保存场景后生效。`, "success");
  } catch (error) { setStatus(`网格上传失败：${error.message}`, "error"); }
}

/**
 * Load the shared Arachne URDF with authenticated package mesh resources.
 * @returns {Promise<void>} Resolves after the robot preview has been parsed.
 */
async function loadRobot() {
  try {
    const response = await request("/robot-description");
    const manager = new THREE.LoadingManager(renderFrame);
    const loader = new URDFLoader(manager);
    const resourceBase = `${location.origin}/api/v1/core/visualization/resources/embodied_nexus_sim_description`;
    loader.packages = { embodied_nexus_sim_description: resourceBase };
    loader.loadMeshCb = async (path, loadingManager, done) => {
      try {
        const url = new URL(path, location.href);
        if (url.origin !== location.origin || !url.pathname.startsWith("/api/v1/core/visualization/resources/")) {
          throw new Error("URDF mesh path is outside the allowed package resources");
        }
        const meshResponse = await fetch(url, { headers: { Authorization: `Bearer ${token}` } });
        if (!meshResponse.ok) throw new Error(`mesh HTTP ${meshResponse.status}`);
        const extension = url.pathname.toLowerCase().split(".").at(-1);
        let mesh;
        if (extension === "stl") {
          mesh = new THREE.Mesh(new STLLoader(loadingManager).parse(await meshResponse.arrayBuffer()),
            new THREE.MeshPhongMaterial());
        } else if (extension === "dae") {
          mesh = new ColladaLoader(loadingManager).parse(await meshResponse.text(), "").scene;
        } else throw new Error(`unsupported robot mesh .${extension}`);
        done(mesh);
        renderFrame();
      } catch (error) { done(null, error); }
    };
    robotModel = loader.parse(response);
    robotModel.position.z = 0.236;
    robotGroup.add(robotModel);
    renderFrame();
  } catch (error) { setStatus(`机器人模型加载失败：${error.message}`, "error"); }
}

/**
 * Bind editor controls and load the initial named scene.
 * @returns {Promise<void>} Resolves after initial scene data has loaded.
 */
async function initializeEditor() {
  if (!token) { location.assign("/"); return; }
  byId("open-scene").addEventListener("click", () => openScene(sceneSelect.value).catch((error) => setStatus(error.message, "error")));
  byId("new-scene").addEventListener("click", newScene);
  byId("save-scene").addEventListener("click", saveScene);
  byId("apply-scene").addEventListener("click", applyScene);
  byId("save-as").addEventListener("click", saveAsScene);
  byId("rename-scene").addEventListener("click", renameScene);
  byId("delete-scene").addEventListener("click", deleteScene);
  byId("remove-object").addEventListener("click", async () => {
    if (!selectedObject()) return;
    currentScene.objects = currentScene.objects.filter((object) => object.id !== selectedId);
    selectedId = currentScene.objects[0]?.id || "";
    await renderWorkcell();
    renderProperties();
    setStatus("物体已从草稿移除；保存场景后写入配置。");
  });
  document.querySelectorAll("[data-shape]").forEach((button) => button.addEventListener("click", () => addPrimitive(button.dataset.shape)));
  byId("mesh-file").addEventListener("change", (event) => {
    const file = event.target.files?.[0];
    event.target.value = "";
    if (file) uploadMesh(file);
  });
  byId("scene-name").addEventListener("change", () => {
    if (currentScene) { currentScene.name = byId("scene-name").value.trim(); setStatus("有未保存的场景修改。"); }
  });
  ["object-id", "object-shape", "pos-x", "pos-y", "pos-z", "rot-x", "rot-y", "rot-z", "size-x", "size-y", "size-z", "radius", "height", "scale-x", "scale-y", "scale-z", "object-color", "object-motion", "object-mass"]
    .forEach((id) => byId(id).addEventListener("change", () => applyProperties().catch((error) => setStatus(error.message, "error"))));
  byId("translate-mode").addEventListener("click", () => {
    transform.setMode("translate"); byId("translate-mode").classList.add("active"); byId("rotate-mode").classList.remove("active");
  });
  byId("rotate-mode").addEventListener("click", () => {
    transform.setMode("rotate"); byId("rotate-mode").classList.add("active"); byId("translate-mode").classList.remove("active");
  });
  window.addEventListener("resize", resizeViewport);
  new ResizeObserver(resizeViewport).observe(viewport);
  resizeViewport();
  try {
    await refreshSceneList();
    await loadRobot();
    await openScene("default");
    setStatus("Core 在线；预览和配置已就绪。", "success");
  } catch (error) { setStatus(`连接失败：${error.message}`, "error"); }
}

initializeEditor();
