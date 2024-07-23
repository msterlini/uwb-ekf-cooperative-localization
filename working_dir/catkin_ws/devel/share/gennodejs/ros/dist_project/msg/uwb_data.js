// Auto-generated. Do not edit!

// (in-package dist_project.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class uwb_data {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.destination_id = null;
      this.distance = null;
    }
    else {
      if (initObj.hasOwnProperty('destination_id')) {
        this.destination_id = initObj.destination_id
      }
      else {
        this.destination_id = [];
      }
      if (initObj.hasOwnProperty('distance')) {
        this.distance = initObj.distance
      }
      else {
        this.distance = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type uwb_data
    // Serialize message field [destination_id]
    bufferOffset = _arraySerializer.int64(obj.destination_id, buffer, bufferOffset, null);
    // Serialize message field [distance]
    bufferOffset = _arraySerializer.float64(obj.distance, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type uwb_data
    let len;
    let data = new uwb_data(null);
    // Deserialize message field [destination_id]
    data.destination_id = _arrayDeserializer.int64(buffer, bufferOffset, null)
    // Deserialize message field [distance]
    data.distance = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.destination_id.length;
    length += 8 * object.distance.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'dist_project/uwb_data';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f96d6cbe0eaf6e7544b86045e6091f3e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int64[] destination_id
    float64[] distance
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new uwb_data(null);
    if (msg.destination_id !== undefined) {
      resolved.destination_id = msg.destination_id;
    }
    else {
      resolved.destination_id = []
    }

    if (msg.distance !== undefined) {
      resolved.distance = msg.distance;
    }
    else {
      resolved.distance = []
    }

    return resolved;
    }
};

module.exports = uwb_data;
