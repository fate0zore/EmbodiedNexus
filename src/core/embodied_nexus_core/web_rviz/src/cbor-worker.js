import { Decoder } from "cbor-x";
import { decodeCborFrame } from "./cbor-codec.js";
import { imagePixels, pointCloudBuffers } from "./ros-data.js";

const decoder = new Decoder({ useRecords: false, mapsAsObjects: true });
let maxPoints = 100000;
let depthRange = 6;
self.onmessage = ({ data }) => {
  if (data.op === "configure") {
    maxPoints = data.maxPoints; depthRange = data.depthRange;
    return;
  }
  try {
    const message = decodeCborFrame(decoder, data);
    const payload = message.msg;
    const prepared = payload?.fields ? pointCloudBuffers(payload, maxPoints)
      : payload?.encoding ? imagePixels(payload, depthRange) : null;
    const buffers = new Set();
    for (const array of [payload?.data, prepared?.positions, prepared?.colors, prepared?.pixels]) {
      if (array?.buffer instanceof ArrayBuffer) buffers.add(array.buffer);
    }
    self.postMessage({ ok: true, message, prepared }, [...buffers]);
  } catch (error) {
    self.postMessage({ ok: false, error: String(error) });
  }
};
