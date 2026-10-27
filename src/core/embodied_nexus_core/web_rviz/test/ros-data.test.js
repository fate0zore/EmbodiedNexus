import test from "node:test";
import assert from "node:assert/strict";
import * as THREE from "three";
import { buildTfPoses, imagePixels, lookupTfMatrix, pointCloudBuffers, selectTransform } from "../src/ros-data.js";

test("PointCloud2 honors point fields, row padding, endianness and RGB", () => {
  const data = new Uint8Array(40);
  const view = new DataView(data.buffer);
  view.setFloat32(0, 1, true); view.setFloat32(4, 2, true); view.setFloat32(8, 3, true);
  view.setUint32(12, 0x00ff8040, true);
  view.setFloat32(16, Number.NaN, true); view.setFloat32(20, 0, true); view.setFloat32(24, 0, true);
  return assert.deepEqual(pointCloudBuffers({
    width: 2, height: 1, point_step: 16, row_step: 40, is_bigendian: false, data,
    fields: ["x", "y", "z"].map((name, i) => ({ name, offset: i * 4, datatype: 7 }))
      .concat({ name: "rgb", offset: 12, datatype: 6 }),
  }), { positions: new Float32Array([1, 2, 3]), colors: new Float32Array([1, 128 / 255, 64 / 255]), count: 1 });
});

test("PointCloud2 reads reordered big-endian fields and intensity", () => {
  const data = new Uint8Array(16);
  const view = new DataView(data.buffer);
  view.setFloat32(0, 1, false); view.setFloat32(4, 2, false);
  view.setFloat32(8, 3, false); view.setFloat32(12, 0.5, false);
  const cloud = pointCloudBuffers({
    width: 1, height: 1, point_step: 16, row_step: 16, is_bigendian: true, data,
    fields: [
      { name: "y", offset: 0, datatype: 7 }, { name: "x", offset: 4, datatype: 7 },
      { name: "z", offset: 8, datatype: 7 }, { name: "intensity", offset: 12, datatype: 7 },
    ],
  });
  assert.deepEqual(cloud.positions, new Float32Array([2, 1, 3]));
  assert.deepEqual(cloud.colors, new Float32Array([0.5, 0.625, 0.6]));
});

test("image conversion handles BGR channels and 16-bit depth", () => {
  const rgb = imagePixels({ width: 1, height: 1, step: 3, encoding: "bgr8", data: new Uint8Array([3, 2, 1]) });
  assert.deepEqual(Array.from(rgb.pixels), [1, 2, 3, 255]);
  const mono = imagePixels({ width: 1, height: 1, step: 1, encoding: "mono8", data: new Uint8Array([64]) });
  assert.deepEqual(Array.from(mono.pixels), [64, 64, 64, 255]);
  const bytes = new Uint8Array(2); new DataView(bytes.buffer).setUint16(0, 1250, true);
  const depth = imagePixels({ width: 1, height: 1, step: 2, encoding: "16UC1", data: bytes }, 2);
  assert.equal(depth.pixels.length, 4);
  assert.equal(depth.pixels[3], 255);
  const floatDepthBytes = new Uint8Array(4);
  new DataView(floatDepthBytes.buffer).setFloat32(0, 1.25, false);
  const floatDepth = imagePixels({ width: 1, height: 1, step: 4, encoding: "32FC1", is_bigendian: true,
    data: floatDepthBytes }, 2);
  assert.equal(floatDepth.pixels[3], 255);
});

test("transform selection uses static edges and nearest non-future dynamic transform", () => {
  const transforms = [
    { parent: "a", child: "b", stamp: 5, static: false },
    { parent: "a", child: "b", stamp: 7, static: false },
    { parent: "a", child: "c", stamp: 0, static: true },
  ];
  assert.equal(selectTransform(transforms, "a", "b", 8).stamp, 7);
  assert.equal(selectTransform(transforms, "a", "b", 6).stamp, 5);
  assert.equal(selectTransform(transforms, "a", "b", 4), null);
  assert.equal(selectTransform(transforms, "a", "c", 8).static, true);
});

test("TF graph composes link transforms and traverses inverse edges", () => {
  const half = Math.SQRT1_2;
  const edges = [
    { parent: "map", child: "base", translation: { x: 1, y: 0, z: 0 }, rotation: { x: 0, y: 0, z: half, w: half } },
    { parent: "base", child: "sensor", translation: { x: 1, y: 0, z: 0 }, rotation: { x: 0, y: 0, z: 0, w: 1 } },
  ];
  const mapFromSensor = lookupTfMatrix(edges, "map", "sensor");
  const sensorOriginInMap = new THREE.Vector3().applyMatrix4(mapFromSensor);
  assert.ok(sensorOriginInMap.distanceTo(new THREE.Vector3(1, 1, 0)) < 1e-6);
  const sensorFromMap = lookupTfMatrix(edges, "sensor", "map");
  const mapOriginInSensor = new THREE.Vector3().applyMatrix4(sensorFromMap);
  assert.ok(mapOriginInSensor.distanceTo(new THREE.Vector3(-1, 1, 0)) < 1e-6);
});

test("one TF traversal resolves all connected frames and excludes disconnected frames", () => {
  /**
 * 创建测试使用的 TF 变换边对象。
 * @param {*} parent - 要追加信息的父 DOM 元素。
 * @param {*} child - 变换的子坐标系名称。
 * @param {*} x - TF 边的 X 方向平移值。
 * @returns {*} 创建测试使用的 TF 变换边对象得到的数据或处理结果。
 */
  const edge = (parent, child, x) => ({ parent, child, translation: { x, y: 0, z: 0 }, rotation: { x: 0, y: 0, z: 0, w: 1 } });
  const poses = buildTfPoses([edge("map", "base", 2), edge("base", "sensor", 3), edge("other", "lost", 4)], "base");
  assert.deepEqual([...poses.keys()].sort(), ["base", "map", "sensor"]);
  assert.equal(poses.get("map").elements[12], -2);
  assert.equal(poses.get("sensor").elements[12], 3);
});
