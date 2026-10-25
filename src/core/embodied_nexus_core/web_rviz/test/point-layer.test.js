import test from "node:test";
import assert from "node:assert/strict";
import * as THREE from "three";
import { updatePointLayer } from "../src/point-layer.js";

test("point layers reuse GPU storage and keep coordinates in their ROS frame", () => {
  const frame = new THREE.Matrix4().makeTranslation(10, 20, 30);
  const buffers = { positions: new Float32Array([1, 2, 3, 4, 5, 6]), colors: new Float32Array(6).fill(1), count: 2 };
  const layer = updatePointLayer(null, buffers, frame, 0.03);
  const geometry = layer.geometry, material = layer.material, position = geometry.getAttribute("position");
  updatePointLayer(layer, { ...buffers, count: 1 }, frame, 0.05);
  assert.equal(layer.geometry, geometry);
  assert.equal(layer.material, material);
  assert.equal(geometry.getAttribute("position"), position);
  assert.deepEqual([...position.array.slice(0, 3)], [1, 2, 3]);
  assert.equal(geometry.drawRange.count, 1);
  assert.equal(layer.matrix.elements[12], 10);
  assert.equal(layer.material.size, 0.05);
});

test("point storage grows only when required and releases the old geometry", () => {
  const buffers = (count) => ({ positions: new Float32Array(count * 3), count });
  const layer = updatePointLayer(null, buffers(1), new THREE.Matrix4(), 0.03, 0xffac62);
  const old = layer.geometry;
  let disposed = false; old.addEventListener("dispose", () => { disposed = true; });
  updatePointLayer(layer, buffers(10), new THREE.Matrix4(), 0.03, 0xffac62);
  assert.ok(disposed);
  assert.notEqual(layer.geometry, old);
  assert.equal(layer.geometry.getAttribute("color"), undefined);
  assert.equal(layer.geometry.drawRange.count, 10);
});
