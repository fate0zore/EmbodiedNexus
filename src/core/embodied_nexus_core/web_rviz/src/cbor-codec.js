/**
 * 解码 CBOR 格式的消息帧。
 * @param {*} decoder - 供本函数解码 CBOR 格式的消息帧使用的输入值。
 * @param {*} data - 待解析、转换或发送的数据。
 * @returns {*} 解码 CBOR 格式的消息帧得到的数据或处理结果。
 */
export function decodeCborFrame(decoder, data) {
  return decoder.decode(data instanceof ArrayBuffer ? new Uint8Array(data) : data);
}
