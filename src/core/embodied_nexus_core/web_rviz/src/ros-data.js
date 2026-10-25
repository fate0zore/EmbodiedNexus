import * as THREE from "three";

const FIELD_SIZE = { 1: 1, 2: 1, 3: 2, 4: 2, 5: 4, 6: 4, 7: 4, 8: 8 };

function readField(view, offset, datatype, littleEndian) {
  switch (datatype) {
    case 1: return view.getInt8(offset);
    case 2: return view.getUint8(offset);
    case 3: return view.getInt16(offset, littleEndian);
    case 4: return view.getUint16(offset, littleEndian);
    case 5: return view.getInt32(offset, littleEndian);
    case 6: return view.getUint32(offset, littleEndian);
    case 7: return view.getFloat32(offset, littleEndian);
    case 8: return view.getFloat64(offset, littleEndian);
    default: return Number.NaN;
  }
}

export function pointCloudBuffers(message, maximum = 100000) {
  const fields = Object.fromEntries((message.fields || []).map((field) => [field.name, field]));
  if (!["x", "y", "z"].every((name) => fields[name] && FIELD_SIZE[fields[name].datatype])) return null;
  const count = Math.max(0, Number(message.width) * Number(message.height));
  const stride = Math.max(1, Math.ceil(count / Math.max(1, maximum)));
  const sampled = Math.ceil(count / stride);
  const positions = new Float32Array(sampled * 3);
  const colors = new Float32Array(sampled * 3);
  const view = new DataView(message.data.buffer, message.data.byteOffset, message.data.byteLength);
  const little = !message.is_bigendian;
  const rgb = fields.rgb || fields.rgba;
  const intensity = fields.intensity;
  let output = 0;
  for (let index = 0; index < count; index += stride) {
    const offset = Math.floor(index / message.width) * message.row_step + (index % message.width) * message.point_step;
    const x = readField(view, offset + fields.x.offset, fields.x.datatype, little);
    const y = readField(view, offset + fields.y.offset, fields.y.datatype, little);
    const z = readField(view, offset + fields.z.offset, fields.z.datatype, little);
    if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) continue;
    const dst = output * 3;
    positions[dst] = x; positions[dst + 1] = y; positions[dst + 2] = z;
    if (rgb) {
      const packed = view.getUint32(offset + rgb.offset, little) >>> 0;
      colors[dst] = ((packed >> 16) & 255) / 255;
      colors[dst + 1] = ((packed >> 8) & 255) / 255; colors[dst + 2] = (packed & 255) / 255;
    } else if (intensity) {
      const value = Math.max(0, Math.min(1, readField(view, offset + intensity.offset, intensity.datatype, little)));
      colors[dst] = value; colors[dst + 1] = 0.35 + value * 0.55; colors[dst + 2] = 1 - value * 0.8;
    } else { colors[dst] = 0.32; colors[dst + 1] = 0.88; colors[dst + 2] = 0.63; }
    output++;
  }
  return { positions: positions.subarray(0, output * 3), colors: colors.subarray(0, output * 3), count: output };
}

function imageChannel(data, index, encoding) {
  if (encoding === "bgr8") return data[index + 2];
  if (encoding === "bgra8") return data[index + 2];
  return data[index];
}

export function imagePixels(message, depthMax = 6) {
  const width = Number(message.width), height = Number(message.height);
  const encoding = String(message.encoding || "").toLowerCase();
  const data = message.data;
  if (!(data instanceof Uint8Array) || width <= 0 || height <= 0) return null;
  const rgba = new Uint8ClampedArray(width * height * 4);
  if (["rgb8", "bgr8", "rgba8", "bgra8"].includes(encoding)) {
    const channels = encoding.endsWith("a8") ? 4 : 3;
    for (let row = 0; row < height; row++) for (let col = 0; col < width; col++) {
      const src = row * message.step + col * channels, dst = (row * width + col) * 4;
      rgba[dst] = imageChannel(data, src, encoding); rgba[dst + 1] = data[src + 1];
      rgba[dst + 2] = encoding === "bgr8" || encoding === "bgra8" ? data[src] : data[src + 2];
      rgba[dst + 3] = channels === 4 ? data[src + 3] : 255;
    }
  } else if (encoding === "mono8") {
    for (let row = 0; row < height; row++) for (let col = 0; col < width; col++) {
      const value = data[row * message.step + col], dst = (row * width + col) * 4;
      rgba[dst] = value; rgba[dst + 1] = value; rgba[dst + 2] = value; rgba[dst + 3] = 255;
    }
  } else if (["16uc1", "mono16", "32fc1"].includes(encoding)) {
    const view = new DataView(data.buffer, data.byteOffset, data.byteLength);
    const isFloat = encoding === "32fc1", bytes = isFloat ? 4 : 2;
    const minimum = 0.5, maximum = Math.max(minimum + 0.01, depthMax), span = maximum - minimum;
    for (let row = 0; row < height; row++) for (let col = 0; col < width; col++) {
      const src = row * message.step + col * bytes;
      let value = isFloat ? view.getFloat32(src, !message.is_bigendian) : view.getUint16(src, !message.is_bigendian) / 1000;
      if (!Number.isFinite(value) || value < minimum || value > maximum) value = 0;
      const t = value ? 1 - (value - minimum) / span : 0;
      const dst = (row * width + col) * 4;
      rgba[dst] = Math.round(255 * Math.min(1, 1.5 * t));
      rgba[dst + 1] = Math.round(255 * Math.max(0, 1 - Math.abs(t - 0.5) * 2));
      rgba[dst + 2] = Math.round(255 * Math.max(0, 1 - 1.5 * t)); rgba[dst + 3] = value ? 255 : 0;
    }
  } else return null;
  return { width, height, pixels: rgba };
}

export function selectTransform(transforms, parent, child, stampSec, maxAge = 1) {
  const matches = transforms.filter((item) => item.parent === parent && item.child === child && item.static);
  if (matches.length) return matches.at(-1);
  const dynamic = transforms.filter((item) => item.parent === parent && item.child === child && item.stamp <= stampSec);
  dynamic.sort((a, b) => b.stamp - a.stamp);
  return dynamic[0] && stampSec - dynamic[0].stamp <= maxAge ? dynamic[0] : null;
}

export function buildTfPoses(edges, targetFrame) {
  const target = targetFrame.replace(/^\//, "");
  const graph = new Map();
  for (const edge of edges) {
    const matrix = new THREE.Matrix4().compose(
      new THREE.Vector3(edge.translation.x, edge.translation.y, edge.translation.z),
      new THREE.Quaternion(edge.rotation.x, edge.rotation.y, edge.rotation.z, edge.rotation.w),
      new THREE.Vector3(1, 1, 1),
    );
    const inverse = matrix.clone().invert();
    (graph.get(edge.parent) || graph.set(edge.parent, []).get(edge.parent)).push([edge.child, matrix]);
    (graph.get(edge.child) || graph.set(edge.child, []).get(edge.child)).push([edge.parent, inverse]);
  }
  const poses = new Map([[target, new THREE.Matrix4()]]), queue = [target];
  for (let index = 0; index < queue.length; index++) {
    const frame = queue[index], pose = poses.get(frame);
    for (const [next, relative] of graph.get(frame) || []) {
      if (poses.has(next)) continue;
      const combined = pose.clone().multiply(relative);
      poses.set(next, combined);
      queue.push(next);
    }
  }
  return poses;
}

export function lookupTfMatrix(edges, targetFrame, sourceFrame) {
  return buildTfPoses(edges, targetFrame).get(sourceFrame.replace(/^\//, "")) || null;
}

export const FIELD_SIZE_BYTES = FIELD_SIZE;
