// Auto-generated. Do not edit!

// (in-package intera_motion_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Waypoint = require('./Waypoint.js');
let TrajectoryOptions = require('./TrajectoryOptions.js');

//-----------------------------------------------------------

class Trajectory {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.label = null;
      this.joint_names = null;
      this.waypoints = null;
      this.trajectory_options = null;
    }
    else {
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
      if (initObj.hasOwnProperty('waypoints')) {
        this.waypoints = initObj.waypoints
      }
      else {
        this.waypoints = [];
      }
      if (initObj.hasOwnProperty('trajectory_options')) {
        this.trajectory_options = initObj.trajectory_options
      }
      else {
        this.trajectory_options = new TrajectoryOptions();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Trajectory
    // Serialize message field [label]
    bufferOffset = _serializer.string(obj.label, buffer, bufferOffset);
    // Serialize message field [joint_names]
    bufferOffset = _arraySerializer.string(obj.joint_names, buffer, bufferOffset, null);
    // Serialize message field [waypoints]
    // Serialize the length for message field [waypoints]
    bufferOffset = _serializer.uint32(obj.waypoints.length, buffer, bufferOffset);
    obj.waypoints.forEach((val) => {
      bufferOffset = Waypoint.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [trajectory_options]
    bufferOffset = TrajectoryOptions.serialize(obj.trajectory_options, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Trajectory
    let len;
    let data = new Trajectory(null);
    // Deserialize message field [label]
    data.label = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [joint_names]
    data.joint_names = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [waypoints]
    // Deserialize array length for message field [waypoints]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.waypoints = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.waypoints[i] = Waypoint.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [trajectory_options]
    data.trajectory_options = TrajectoryOptions.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.label.length;
    object.joint_names.forEach((val) => {
      length += 4 + val.length;
    });
    object.waypoints.forEach((val) => {
      length += Waypoint.getMessageSize(val);
    });
    length += TrajectoryOptions.getMessageSize(object.trajectory_options);
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'intera_motion_msgs/Trajectory';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9ab7e6d17ba67f0a6b00ab5f35f6d93e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Representation of a trajectory used by the engine and motion controller.
    
    # optional label
    string label
    
    # Array of joint names that correspond to the waypoint joint_positions
    string[] joint_names
    
    # Array of waypoints that comprise the trajectory
    Waypoint[] waypoints
    
    # Trajectory level options
    TrajectoryOptions trajectory_options
    ================================================================================
    MSG: intera_motion_msgs/Waypoint
    # Representation of a waypoint used by the motion controller
    
    # Desired joint positions
    # For Cartesian segments, the joint positions are used as nullspace biases
    float64[] joint_positions
    
    # Name of the endpoint that is currently active
    string active_endpoint
    
    # Cartesian pose
    # This is not used in trajectories using joint interpolation
    geometry_msgs/PoseStamped pose
    
    # Waypoint specific options
    # Default values will be used if not set
    # All waypoint options are applied to the segment moving to that waypoint
    WaypointOptions options
    
    ================================================================================
    MSG: geometry_msgs/PoseStamped
    # A Pose with reference coordinate frame and timestamp
    Header header
    Pose pose
    
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
    
    ================================================================================
    MSG: intera_motion_msgs/WaypointOptions
    # Optional waypoint label
    string label
    
    # Ratio of max allowed joint speed : max planned joint speed (from 0.0 to 1.0)
    float64 max_joint_speed_ratio
    
    # Slowdown heuristic is triggered if tracking error exceeds tolerances - radians
    float64[] joint_tolerances
    
    # Maximum accelerations for each joint (only for joint paths) - rad/s^2.
    float64[] max_joint_accel
    
    
    ###########################################################
    # The remaining parameters only apply to Cartesian paths
    
    # Maximum linear speed of endpoint - m/s
    float64 max_linear_speed
    
    # Maximum linear acceleration of endpoint - m/s^2
    float64 max_linear_accel
    
    # Maximum rotational speed of endpoint - rad/s
    float64 max_rotational_speed
    
    # Maximum rotational acceleration of endpoint - rad/s^2
    float64 max_rotational_accel
    
    # Used for smoothing corners for continuous motion - m
    # The distance from the waypoint to where the curve starts while blending from
    # one straight line segment to the next.
    # Larger distance:  trajectory passes farther from the waypoint at a higher speed
    # Smaller distance:  trajectory passes closer to the waypoint at a lower speed
    # Zero distance:  trajectory passes through the waypoint at zero speed
    float64 corner_distance
    
    ================================================================================
    MSG: intera_motion_msgs/TrajectoryOptions
    # Trajectory interpolation type
    string CARTESIAN=CARTESIAN
    string JOINT=JOINT
    string interpolation_type
    
    # True if the trajectory uses interaction control, false for position control.
    bool interaction_control
    
    # Interaction control parameters
    intera_core_msgs/InteractionControlCommand interaction_params
    
    # Allow small joint adjustments at the beginning of Cartesian trajectories.
    # Set to false for 'small' motions.
    bool nso_start_offset_allowed
    
    # Check the offset at the end of a Cartesian trajectory from the final waypoint nullspace goal.
    bool nso_check_end_offset
    
    # Options for the tracking controller:
    TrackingOptions tracking_options
    
    # Desired trajectory end time, ROS timestamp
    time end_time
    
    # The rate in seconds that the path is interpolated and returned back to the user
    # No interpolation will happen if set to zero
    float64 path_interpolation_step
    
    ================================================================================
    MSG: intera_core_msgs/InteractionControlCommand
    # Message sets the interaction (impedance/force) control on or off
    # It also contains desired cartesian stiffness K, damping D, and force values
    
    Header header
    bool      interaction_control_active
    
    ## Cartesian Impedance Control Parameters
    # Stiffness units are (N/m) for first 3 and (Nm/rad) for second 3 values
    float64[] K_impedance
    # Force certain directions to have maximum possible impedance for a given pose
    bool[] max_impedance
    # Damping units are (Ns/m) for first 3 and (Nms/rad) for the second 3 values
    float64[] D_impedance
    # Joint Nullspace stiffness units are in (Nm/rad) (length == number of joints)
    float64[] K_nullspace
    
    ## Parameters for force control or impedance control with force limit
    # If in force mode, this is the vector of desired forces/torques
    # to be regulated in (N) and (Nm)
    # If in impedance with force limit mode, this vector specifies the
    # magnitude of forces/torques (N and Nm) that the command will not exceed.
    float64[] force_command
    
    ## Desired frame
    geometry_msgs/Pose interaction_frame
    string endpoint_name
    # True if impedance and force commands are defined in endpoint frame
    bool in_endpoint_frame
    
    # Set to true to disable damping during force control. Damping is used
    # to slow down robot motion during force control in free space.
    # Option included for SDK users to disable damping in force control
    bool disable_damping_in_force_control
    
    # Set to true to disable reference resetting. Reference resetting is
    # used when interaction parameters change, in order to avoid jumps/jerks.
    # Option included for SDK users to disable reference resetting if the
    # intention is to change interaction parameters.
    bool disable_reference_resetting
    
    ## Mode Selection Parameters
    # The possible interaction control modes are:
    # Impedance mode: implements desired endpoint stiffness and damping.
    uint8 IMPEDANCE_MODE=1
    # Force mode: applies force/torque in the specified dimensions.
    uint8 FORCE_MODE=2
    # Impedance with force limit: impedance control while ensuring the commanded
    # forces/torques do not exceed force_command.
    uint8 IMPEDANCE_WITH_FORCE_LIMIT_MODE=3
    # Force with motion bounds: force control while ensuring the current
    # pose/velocities do not exceed forceMotionThreshold (currenetly defined in yaml)
    uint8 FORCE_WITH_MOTION_LIMIT_MODE=4
    
    # Specifies the interaction control mode for each Cartesian dimension (6)
    uint8[] interaction_control_mode
    
    # All 6 values in force and impedance parameter vectors have to be filled,
    # If a control mode is not used in a Cartesian dimension,
    # the corresponding parameters will be ignored.
    
    ## Parameters for Constrained Zero-G Behaviors
    # Allow for arbitrary rotational displacements from the current orientation
    # for constrained zero-G. Setting 'rotations_for_constrained_zeroG = True'
    # will disable the rotational stiffness field which limits rotational
    # displacements to +/- 82.5 degree.
    # NOTE: it will be only enabled for a stationary reference orientation
    bool rotations_for_constrained_zeroG
    
    ================================================================================
    MSG: intera_motion_msgs/TrackingOptions
    # Minimum trajectory tracking time rate:  (default = less than one)
    bool     use_min_time_rate
    float64  min_time_rate
    
    # Maximum trajectory tracking time rate:  (1.0 = real-time = default)
    bool     use_max_time_rate
    float64  max_time_rate
    
    # Angular error tolerance at final point on trajectory (rad)
    float64[] goal_joint_tolerance
    
    # Time for the controller to settle within joint tolerances at the goal (sec)
    bool     use_settling_time_at_goal
    float64  settling_time_at_goal
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Trajectory(null);
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

    if (msg.waypoints !== undefined) {
      resolved.waypoints = new Array(msg.waypoints.length);
      for (let i = 0; i < resolved.waypoints.length; ++i) {
        resolved.waypoints[i] = Waypoint.Resolve(msg.waypoints[i]);
      }
    }
    else {
      resolved.waypoints = []
    }

    if (msg.trajectory_options !== undefined) {
      resolved.trajectory_options = TrajectoryOptions.Resolve(msg.trajectory_options)
    }
    else {
      resolved.trajectory_options = new TrajectoryOptions()
    }

    return resolved;
    }
};

module.exports = Trajectory;
