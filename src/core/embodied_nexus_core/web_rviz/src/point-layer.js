import * as THREE from "three";

/**
 * 在点精灵片元着色器中裁剪圆形外的像素。
 * @param {{ fragmentShader: string }} shader - Three.js 点材质的着色器程序。
 * @returns {void} 无返回值；直接更新着色器源码。
 */
function applyCircularPointMask(shader) {
  const mainDeclaration = "void main() {";
  const circularMask = `${mainDeclaration}
  vec2 circularPoint = gl_PointCoord - vec2(0.5);
  if (dot(circularPoint, circularPoint) > 0.25) discard;`;
  shader.fragmentShader = shader.fragmentShader.replace(mainDeclaration, circularMask);
}

// Keep GPU objects alive between frames; only grow storage when needed.
/**
 * 更新三维场景中的点云图层。
 * @param {*} object - 需要更新的三维场景对象。
 * @param {*} buffers - 点云顶点及其数量缓冲区。
 * @param {*} pose - 点云或模型在目标坐标系中的位姿。
 * @param {*} size - 绘制点的尺寸。
 * @param {*} color - 可选的点云颜色设置。
 * @returns {*} 更新三维场景中的点云图层得到的数据或处理结果。
 */
export function updatePointLayer(object, buffers, pose, size, color = null) {
  if (!object) {
    const material = new THREE.PointsMaterial({
      size, vertexColors: color === null, color: color ?? 0xffffff, sizeAttenuation: true,
    });
    material.onBeforeCompile = applyCircularPointMask;
    object = new THREE.Points(new THREE.BufferGeometry(), material);
    object.matrixAutoUpdate = false;
    object.frustumCulled = false;
  }
  const required = buffers.count * 3;
  if (!object.geometry.getAttribute("position") || object.geometry.getAttribute("position").array.length < required) {
    object.geometry.dispose();
    object.geometry = new THREE.BufferGeometry();
    const capacity = Math.max(3, 2 ** Math.ceil(Math.log2(Math.max(1, buffers.count)))) * 3;
    object.geometry.setAttribute("position", new THREE.BufferAttribute(new Float32Array(capacity), 3).setUsage(THREE.DynamicDrawUsage));
    if (color === null) object.geometry.setAttribute("color", new THREE.BufferAttribute(new Float32Array(capacity), 3).setUsage(THREE.DynamicDrawUsage));
  }
  for (const [name, data] of [["position", buffers.positions], ["color", buffers.colors]]) {
    const attribute = object.geometry.getAttribute(name);
    if (!attribute || !data) continue;
    attribute.array.set(data.subarray(0, required));
    attribute.clearUpdateRanges(); attribute.addUpdateRange(0, required); attribute.needsUpdate = true;
  }
  object.geometry.setDrawRange(0, buffers.count);
  object.matrix.copy(pose);
  object.matrixWorldNeedsUpdate = true;
  object.material.size = size;
  return object;
}
