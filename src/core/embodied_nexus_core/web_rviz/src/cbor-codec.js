export function decodeCborFrame(decoder, data) {
  return decoder.decode(data instanceof ArrayBuffer ? new Uint8Array(data) : data);
}
