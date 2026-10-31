import { Decoder } from "cbor-x";
import { decodeCborFrame } from "./cbor-codec.js";
import { imagePixels, pointCloudBuffers } from "./ros-data.js";

const decoder = new Decoder({ useRecords: false, mapsAsObjects: true });
let maxPoints = 100000;
let depthRange = 6;
let activeGeneration = 0;
const pendingByTopic = new Map();
let drainScheduled = false;

/**
 * Schedule sensor conversion without building a second queue of stale frames.
 * @returns {void} No return value; schedules the next pending sensor frame.
 */
function scheduleDrain() {
  if (drainScheduled || !pendingByTopic.size) return;
  drainScheduled = true;
  setTimeout(processNextFrame, 0);
}

/**
 * Convert and forward the newest waiting frame for one sensor topic.
 * @returns {void} No return value; forwards prepared sensor data to the main thread.
 */
function processNextFrame() {
  drainScheduled = false;
  const next = pendingByTopic.entries().next();
  if (next.done) return;
  const [topic, frame] = next.value;
  pendingByTopic.delete(topic);
  if (frame.generation !== activeGeneration) {
    scheduleDrain();
    return;
  }
  const { message, generation, receivedAt } = frame;
  try {
    const payload = message.msg;
    const prepared = payload?.fields ? pointCloudBuffers(payload, maxPoints)
      : payload?.encoding ? imagePixels(payload, depthRange) : null;
    const buffers = new Set();
    for (const array of [payload?.data, prepared?.positions, prepared?.colors, prepared?.pixels]) {
      if (array?.buffer instanceof ArrayBuffer) buffers.add(array.buffer);
    }
    self.postMessage({ ok: true, message, prepared, generation, receivedAt }, [...buffers]);
  } catch (error) {
    self.postMessage({ ok: false, error: String(error), generation, receivedAt });
  }
  scheduleDrain();
}

self.onmessage = ({ data }) => {
  if (data.op === "configure") {
    maxPoints = data.maxPoints; depthRange = data.depthRange;
    return;
  }
  if (data.op === "reset") {
    activeGeneration = data.generation;
    pendingByTopic.clear();
    return;
  }
  if (data.op !== "frame" || data.generation !== activeGeneration) return;
  try {
    const message = decodeCborFrame(decoder, data.frame);
    pendingByTopic.set(message.topic || "__untagged__", {
      message, generation: data.generation, receivedAt: data.receivedAt,
    });
    scheduleDrain();
  } catch (error) {
    self.postMessage({
      ok: false, error: String(error), generation: data.generation, receivedAt: data.receivedAt,
    });
  }
};
