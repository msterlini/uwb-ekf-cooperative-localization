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

class robot_data {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.robot_id = null;
      this.target_height = null;
      this.x = null;
      this.y = null;
      this.theta = null;
      this.sigma_x = null;
      this.sigma_y = null;
      this.sigma_theta = null;
    }
    else {
      if (initObj.hasOwnProperty('robot_id')) {
        this.robot_id = initObj.robot_id
      }
      else {
        this.robot_id = '';
      }
      if (initObj.hasOwnProperty('target_height')) {
        this.target_height = initObj.target_height
      }
      else {
        this.target_height = 0.0;
      }
      if (initObj.hasOwnProperty('x')) {
        this.x = initObj.x
      }
      else {
        this.x = 0.0;
      }
      if (initObj.hasOwnProperty('y')) {
        this.y = initObj.y
      }
      else {
        this.y = 0.0;
      }
      if (initObj.hasOwnProperty('theta')) {
        this.theta = initObj.theta
      }
      else {
        this.theta = 0.0;
      }
      if (initObj.hasOwnProperty('sigma_x')) {
        this.sigma_x = initObj.sigma_x
      }
      else {
        this.sigma_x = 0.0;
      }
      if (initObj.hasOwnProperty('sigma_y')) {
        this.sigma_y = initObj.sigma_y
      }
      else {
        this.sigma_y = 0.0;
      }
      if (initObj.hasOwnProperty('sigma_theta')) {
        this.sigma_theta = initObj.sigma_theta
      }
      else {
        this.sigma_theta = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type robot_data
    // Serialize message field [robot_id]
    bufferOffset = _serializer.string(obj.robot_id, buffer, bufferOffset);
    // Serialize message field [target_height]
    bufferOffset = _serializer.float64(obj.target_height, buffer, bufferOffset);
    // Serialize message field [x]
    bufferOffset = _serializer.float64(obj.x, buffer, bufferOffset);
    // Serialize message field [y]
    bufferOffset = _serializer.float64(obj.y, buffer, bufferOffset);
    // Serialize message field [theta]
    bufferOffset = _serializer.float64(obj.theta, buffer, bufferOffset);
    // Serialize message field [sigma_x]
    bufferOffset = _serializer.float64(obj.sigma_x, buffer, bufferOffset);
    // Serialize message field [sigma_y]
    bufferOffset = _serializer.float64(obj.sigma_y, buffer, bufferOffset);
    // Serialize message field [sigma_theta]
    bufferOffset = _serializer.float64(obj.sigma_theta, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type robot_data
    let len;
    let data = new robot_data(null);
    // Deserialize message field [robot_id]
    data.robot_id = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [target_height]
    data.target_height = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [x]
    data.x = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [y]
    data.y = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [theta]
    data.theta = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [sigma_x]
    data.sigma_x = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [sigma_y]
    data.sigma_y = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [sigma_theta]
    data.sigma_theta = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.robot_id);
    return length + 60;
  }

  static datatype() {
    // Returns string type for a message object
    return 'dist_project/robot_data';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'de74b7f6fa08eefd9305c86ead89bd1b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string robot_id
    float64 target_height
    float64 x
    float64 y
    float64 theta
    float64 sigma_x
    float64 sigma_y
    float64 sigma_theta
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new robot_data(null);
    if (msg.robot_id !== undefined) {
      resolved.robot_id = msg.robot_id;
    }
    else {
      resolved.robot_id = ''
    }

    if (msg.target_height !== undefined) {
      resolved.target_height = msg.target_height;
    }
    else {
      resolved.target_height = 0.0
    }

    if (msg.x !== undefined) {
      resolved.x = msg.x;
    }
    else {
      resolved.x = 0.0
    }

    if (msg.y !== undefined) {
      resolved.y = msg.y;
    }
    else {
      resolved.y = 0.0
    }

    if (msg.theta !== undefined) {
      resolved.theta = msg.theta;
    }
    else {
      resolved.theta = 0.0
    }

    if (msg.sigma_x !== undefined) {
      resolved.sigma_x = msg.sigma_x;
    }
    else {
      resolved.sigma_x = 0.0
    }

    if (msg.sigma_y !== undefined) {
      resolved.sigma_y = msg.sigma_y;
    }
    else {
      resolved.sigma_y = 0.0
    }

    if (msg.sigma_theta !== undefined) {
      resolved.sigma_theta = msg.sigma_theta;
    }
    else {
      resolved.sigma_theta = 0.0
    }

    return resolved;
    }
};

module.exports = robot_data;
