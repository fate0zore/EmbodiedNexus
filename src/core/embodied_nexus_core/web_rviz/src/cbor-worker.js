import { Decoder } from "cbor-x";

const decoder = new Decoder({ useRecords: false, mapsAsObjects: true });
self.onmessage = ({ data }) => {
  try {
    const message = decoder.decode(data);
    self.postMessage({ ok: true, message });
  } catch (error) {
    self.postMessage({ ok: false, error: String(error) });
  }
};
