// Auto-generated. Do not edit!

// (in-package intera_motion_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let WaypointSimple = require('./WaypointSimple.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class InterpolatedPath {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.label = null;
      this.joint_names = null;
      this.interpolated_path = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('label')) {
        this.label = initObj.label
      }
      else {
        this.label = '';
      }
      if (initObj.hasOwnProperty('joint_names')) {
        this.joint_names = initObj.joint_names
      }
      else {
        this.joint_names = [];
      }
      if (initObj.hasOwnProperty('interpolated_path')) {
        this.interpolated_path = initObj.interpolated_path
      }
      else {
        this.interpolated_path = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type InterpolatedPath
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [label]
    bufferOffset = _serializer.string(obj.label, buffer, bufferOffset);
    // Serialize message field [joint_names]
    bufferOffset = _arraySerializer.string(obj.joint_names, buffer, bufferOffset, null);
    // Serialize message field [interpolated_path]
    // Serialize the length for message field [interpolated_path]
    bufferOffset = _serializer.uint32(obj.interpolated_path.length, buffer, bufferOffset);
    obj.interpolated_path.forEach((val) => {
      bufferOffset = WaypointSimple.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type InterpolatedPath
    let len;
    let data = new InterpolatedPath(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [label]
    data.label = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [joint_names]
    data.joint_names = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [interpolated_path]
    // Deserialize array length for message field [interpolated_path]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.interpolated_path = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.interpolated_path[i] = WaypointSimple.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += object.label.length;
    object.joint_names.forEach((val) => {
      length += 4 + val.length;
    });
    object.interpolated_path.forEach((val) => {
      length += WaypointSimple.getMessageSize(val);
    });
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'intera_motion_msgs/InterpolatedPath';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4e078bdc2ed88b86420f5b19cbd78219';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Inpteroplation of a path generated by the motion controller
    
    std_msgs/Header header
    
    # optional label
    string label
    
    # Array of joint names that correspond to the waypoint joint_positions
    string[] joint_names
    
    # Array of waypoints interpolated from the generated path
    WaypointSimple[] interpolated_path
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    ================================================================================
    MSG: intera_motion_msgs/WaypointSimple
    # Representation of a waypoint returned during path interpolation
    # Does not include extra options
    
    # Desired joint positions
    float64[] joint_positions
    
    # Name of the endpoint that is currently active
    string active_endpoint
    
    # Cartesian pose
    geometry_msgs/Pose pose
    
    int32 segment_index
    float64 time
    
    
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new InterpolatedPath(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.label !== undefined) {
      resolved.label = msg.label;
    }
    else {
      resolved.label = ''
    }

    if (msg.joint_names !== undefined) {
      resolved.joint_names = msg.joint_names;
    }
    else {
      resolved.joint_names = []
    }

    if (msg.interpolated_path !== undefined) {
      resolved.interpolated_path = new Array(msg.interpolated_path.length);
      for (let i = 0; i < resolved.interpolated_path.length; ++i) {
        resolved.interpolated_path[i] = WaypointSimple.Resolve(msg.interpolated_path[i]);
      }
    }
    else {
      resolved.interpolated_path = []
    }

    return resolved;
    }
};

module.exports = InterpolatedPath;
