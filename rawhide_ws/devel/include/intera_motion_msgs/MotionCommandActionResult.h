// Generated by gencpp from file intera_motion_msgs/MotionCommandActionResult.msg
// DO NOT EDIT!


#ifndef INTERA_MOTION_MSGS_MESSAGE_MOTIONCOMMANDACTIONRESULT_H
#define INTERA_MOTION_MSGS_MESSAGE_MOTIONCOMMANDACTIONRESULT_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <actionlib_msgs/GoalStatus.h>
#include <intera_motion_msgs/MotionCommandResult.h>

namespace intera_motion_msgs
{
template <class ContainerAllocator>
struct MotionCommandActionResult_
{
  typedef MotionCommandActionResult_<ContainerAllocator> Type;

  MotionCommandActionResult_()
    : header()
    , status()
    , result()  {
    }
  MotionCommandActionResult_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , status(_alloc)
    , result(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::actionlib_msgs::GoalStatus_<ContainerAllocator>  _status_type;
  _status_type status;

   typedef  ::intera_motion_msgs::MotionCommandResult_<ContainerAllocator>  _result_type;
  _result_type result;





  typedef boost::shared_ptr< ::intera_motion_msgs::MotionCommandActionResult_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::intera_motion_msgs::MotionCommandActionResult_<ContainerAllocator> const> ConstPtr;

}; // struct MotionCommandActionResult_

typedef ::intera_motion_msgs::MotionCommandActionResult_<std::allocator<void> > MotionCommandActionResult;

typedef boost::shared_ptr< ::intera_motion_msgs::MotionCommandActionResult > MotionCommandActionResultPtr;
typedef boost::shared_ptr< ::intera_motion_msgs::MotionCommandActionResult const> MotionCommandActionResultConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::intera_motion_msgs::MotionCommandActionResult_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::intera_motion_msgs::MotionCommandActionResult_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace intera_motion_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'intera_core_msgs': ['/home/rachel/rawhide/rawhide_ws/src/intera_common/intera_core_msgs/msg', '/home/rachel/rawhide/rawhide_ws/devel/share/intera_core_msgs/msg'], 'sensor_msgs': ['/opt/ros/melodic/share/sensor_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/melodic/share/actionlib_msgs/cmake/../msg'], 'intera_motion_msgs': ['/home/rachel/rawhide/rawhide_ws/src/intera_common/intera_motion_msgs/msg', '/home/rachel/rawhide/rawhide_ws/devel/share/intera_motion_msgs/msg'], 'std_msgs': ['/opt/ros/melodic/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/melodic/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::intera_motion_msgs::MotionCommandActionResult_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::intera_motion_msgs::MotionCommandActionResult_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::intera_motion_msgs::MotionCommandActionResult_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::intera_motion_msgs::MotionCommandActionResult_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::intera_motion_msgs::MotionCommandActionResult_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::intera_motion_msgs::MotionCommandActionResult_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::intera_motion_msgs::MotionCommandActionResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "78437d84884cb2db646d41b73dbf98db";
  }

  static const char* value(const ::intera_motion_msgs::MotionCommandActionResult_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x78437d84884cb2dbULL;
  static const uint64_t static_value2 = 0x646d41b73dbf98dbULL;
};

template<class ContainerAllocator>
struct DataType< ::intera_motion_msgs::MotionCommandActionResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "intera_motion_msgs/MotionCommandActionResult";
  }

  static const char* value(const ::intera_motion_msgs::MotionCommandActionResult_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::intera_motion_msgs::MotionCommandActionResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"\n"
"Header header\n"
"actionlib_msgs/GoalStatus status\n"
"MotionCommandResult result\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
"\n"
"================================================================================\n"
"MSG: actionlib_msgs/GoalStatus\n"
"GoalID goal_id\n"
"uint8 status\n"
"uint8 PENDING         = 0   # The goal has yet to be processed by the action server\n"
"uint8 ACTIVE          = 1   # The goal is currently being processed by the action server\n"
"uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing\n"
"                            #   and has since completed its execution (Terminal State)\n"
"uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)\n"
"uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due\n"
"                            #    to some failure (Terminal State)\n"
"uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,\n"
"                            #    because the goal was unattainable or invalid (Terminal State)\n"
"uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing\n"
"                            #    and has not yet completed execution\n"
"uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,\n"
"                            #    but the action server has not yet confirmed that the goal is canceled\n"
"uint8 RECALLED        = 8   # The goal received a cancel request before it started executing\n"
"                            #    and was successfully cancelled (Terminal State)\n"
"uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be\n"
"                            #    sent over the wire by an action server\n"
"\n"
"#Allow for the user to associate a string with GoalStatus for debugging\n"
"string text\n"
"\n"
"\n"
"================================================================================\n"
"MSG: actionlib_msgs/GoalID\n"
"# The stamp should store the time at which this goal was requested.\n"
"# It is used by an action server when it tries to preempt all\n"
"# goals that were requested before a certain time\n"
"time stamp\n"
"\n"
"# The id provides a way to associate feedback and\n"
"# result message with specific goal requests. The id\n"
"# specified must be unique.\n"
"string id\n"
"\n"
"\n"
"================================================================================\n"
"MSG: intera_motion_msgs/MotionCommandResult\n"
"# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"# result\n"
"bool result\n"
"\n"
"string errorId\n"
"string FAILED_TO_PARAMETERIZE=FAILED_TO_PARAMETERIZE\n"
"string PLANNED_MOTION_COLLISION=PLANNED_MOTION_COLLISION\n"
"string INVALID_TRAJECTORY_MESSAGE=INVALID_TRAJECTORY_MESSAGE\n"
"string ENDPOINT_DOES_NOT_EXIST=ENDPOINT_DOES_NOT_EXIST\n"
"string CARTESIAN_INTERPOLATION_FAILED=CARTESIAN_INTERPOLATION_FAILED\n"
"string FINAL_POSE_NOT_WITHIN_TOLERANCE=FINAL_POSE_NOT_WITHIN_TOLERANCE\n"
"string CONTROLLER_NOT_FOLLOWING=CONTROLLER_NOT_FOLLOWING\n"
"string ZERO_G_ACTIVATED_DURING_TRAJECTORY=ZERO_G_ACTIVATED_DURING_TRAJECTORY\n"
"string PLANNED_JOINT_ACCEL_LIMIT=PLANNED_JOINT_ACCEL_LIMIT\n"
"\n"
"TrajectoryAnalysis trajectory_analysis\n"
"\n"
"int32 last_successful_waypoint\n"
"int32 HAVE_NOT_REACHED_FIRST_WAYPOINT=-1\n"
"int32 GENERAL_TRAJECTORY_FAILURE=-2\n"
"\n"
"\n"
"================================================================================\n"
"MSG: intera_motion_msgs/TrajectoryAnalysis\n"
"# The duration of the reference trajectory, as originally planned\n"
"float64 planned_duration\n"
"\n"
"# The measured duration of the trajectory, as executed\n"
"float64 measured_duration\n"
"\n"
"# Minimum commanded angle during trajectory for each joint\n"
"float64[] min_angle_command\n"
"\n"
"# Maximum commanded angle during trajectory for each joint\n"
"float64[] max_angle_command\n"
"\n"
"# Peak speed command = max(abs(reference velocity)) for each joint\n"
"float64[] peak_speed_command\n"
"\n"
"# Peak accel command = max(abs(reference acceleration)) for each joint\n"
"float64[] peak_accel_command\n"
"\n"
"# Peak jerk command = max(abs(reference jerk)) for each joint\n"
"float64[] peak_jerk_command\n"
"\n"
"# Minimum trajectory time rate observed\n"
"float64 min_time_rate\n"
"\n"
"# Maximium trajectory time rate observed\n"
"float64 max_time_rate\n"
"\n"
"# Max joint position error = max(abs(position error)) for each joint\n"
"float64[] max_position_error\n"
"\n"
"# Max joint velocity error = max(abs(velocity error)) for each joint\n"
"float64[] max_velocity_error\n"
;
  }

  static const char* value(const ::intera_motion_msgs::MotionCommandActionResult_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::intera_motion_msgs::MotionCommandActionResult_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.status);
      stream.next(m.result);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct MotionCommandActionResult_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::intera_motion_msgs::MotionCommandActionResult_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::intera_motion_msgs::MotionCommandActionResult_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "status: ";
    s << std::endl;
    Printer< ::actionlib_msgs::GoalStatus_<ContainerAllocator> >::stream(s, indent + "  ", v.status);
    s << indent << "result: ";
    s << std::endl;
    Printer< ::intera_motion_msgs::MotionCommandResult_<ContainerAllocator> >::stream(s, indent + "  ", v.result);
  }
};

} // namespace message_operations
} // namespace ros

#endif // INTERA_MOTION_MSGS_MESSAGE_MOTIONCOMMANDACTIONRESULT_H
