import assert from "node:assert/strict";
import test from "node:test";
import { Decoder, Encoder } from "cbor-x";
import { decodeCborFrame } from "../src/cbor-codec.js";

test("CBOR decoder accepts the ArrayBuffer delivered by browser WebSocket", () => {
  const expected = { op: "publish", topic: "/camera/image", msg: { width: 640, height: 480 } };
  const encoded = new Encoder().encode(expected);
  const arrayBuffer = encoded.buffer.slice(encoded.byteOffset, encoded.byteOffset + encoded.byteLength);
  const decoder = new Decoder({ useRecords: false, mapsAsObjects: true });

  assert.deepEqual(decodeCborFrame(decoder, arrayBuffer), expected);
});
