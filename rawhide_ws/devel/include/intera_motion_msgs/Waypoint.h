// Generated by gencpp from file intera_motion_msgs/Waypoint.msg
// DO NOT EDIT!


#ifndef INTERA_MOTION_MSGS_MESSAGE_WAYPOINT_H
#define INTERA_MOTION_MSGS_MESSAGE_WAYPOINT_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/PoseStamped.h>
#include <intera_motion_msgs/WaypointOptions.h>

namespace intera_motion_msgs
{
template <class ContainerAllocator>
struct Waypoint_
{
  typedef Waypoint_<ContainerAllocator> Type;

  Waypoint_()
    : joint_positions()
    , active_endpoint()
    , pose()
    , options()  {
    }
  Waypoint_(const ContainerAllocator& _alloc)
    : joint_positions(_alloc)
    , active_endpoint(_alloc)
    , pose(_alloc)
    , options(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _joint_positions_type;
  _joint_positions_type joint_positions;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _active_endpoint_type;
  _active_endpoint_type active_endpoint;

   typedef  ::geometry_msgs::PoseStamped_<ContainerAllocator>  _pose_type;
  _pose_type pose;

   typedef  ::intera_motion_msgs::WaypointOptions_<ContainerAllocator>  _options_type;
  _options_type options;





  typedef boost::shared_ptr< ::intera_motion_msgs::Waypoint_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::intera_motion_msgs::Waypoint_<ContainerAllocator> const> ConstPtr;

}; // struct Waypoint_

typedef ::intera_motion_msgs::Waypoint_<std::allocator<void> > Waypoint;

typedef boost::shared_ptr< ::intera_motion_msgs::Waypoint > WaypointPtr;
typedef boost::shared_ptr< ::intera_motion_msgs::Waypoint const> WaypointConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::intera_motion_msgs::Waypoint_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::intera_motion_msgs::Waypoint_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace intera_motion_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'intera_core_msgs': ['/home/rachel/rawhide/rawhide_ws/src/intera_common/intera_core_msgs/msg', '/home/rachel/rawhide/rawhide_ws/devel/share/intera_core_msgs/msg'], 'sensor_msgs': ['/opt/ros/melodic/share/sensor_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/melodic/share/actionlib_msgs/cmake/../msg'], 'intera_motion_msgs': ['/home/rachel/rawhide/rawhide_ws/src/intera_common/intera_motion_msgs/msg', '/home/rachel/rawhide/rawhide_ws/devel/share/intera_motion_msgs/msg'], 'std_msgs': ['/opt/ros/melodic/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/melodic/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::intera_motion_msgs::Waypoint_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::intera_motion_msgs::Waypoint_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::intera_motion_msgs::Waypoint_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::intera_motion_msgs::Waypoint_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::intera_motion_msgs::Waypoint_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::intera_motion_msgs::Waypoint_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::intera_motion_msgs::Waypoint_<ContainerAllocator> >
{
  static const char* value()
  {
    return "8284b290b22204acc5e4d8000467b033";
  }

  static const char* value(const ::intera_motion_msgs::Waypoint_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x8284b290b22204acULL;
  static const uint64_t static_value2 = 0xc5e4d8000467b033ULL;
};

template<class ContainerAllocator>
struct DataType< ::intera_motion_msgs::Waypoint_<ContainerAllocator> >
{
  static const char* value()
  {
    return "intera_motion_msgs/Waypoint";
  }

  static const char* value(const ::intera_motion_msgs::Waypoint_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::intera_motion_msgs::Waypoint_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Representation of a waypoint used by the motion controller\n"
"\n"
"# Desired joint positions\n"
"# For Cartesian segments, the joint positions are used as nullspace biases\n"
"float64[] joint_positions\n"
"\n"
"# Name of the endpoint that is currently active\n"
"string active_endpoint\n"
"\n"
"# Cartesian pose\n"
"# This is not used in trajectories using joint interpolation\n"
"geometry_msgs/PoseStamped pose\n"
"\n"
"# Waypoint specific options\n"
"# Default values will be used if not set\n"
"# All waypoint options are applied to the segment moving to that waypoint\n"
"WaypointOptions options\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/PoseStamped\n"
"# A Pose with reference coordinate frame and timestamp\n"
"Header header\n"
"Pose pose\n"
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
"MSG: geometry_msgs/Pose\n"
"# A representation of pose in free space, composed of position and orientation. \n"
"Point position\n"
"Quaternion orientation\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Quaternion\n"
"# This represents an orientation in free space in quaternion form.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"float64 w\n"
"\n"
"================================================================================\n"
"MSG: intera_motion_msgs/WaypointOptions\n"
"# Optional waypoint label\n"
"string label\n"
"\n"
"# Ratio of max allowed joint speed : max planned joint speed (from 0.0 to 1.0)\n"
"float64 max_joint_speed_ratio\n"
"\n"
"# Slowdown heuristic is triggered if tracking error exceeds tolerances - radians\n"
"float64[] joint_tolerances\n"
"\n"
"# Maximum accelerations for each joint (only for joint paths) - rad/s^2.\n"
"float64[] max_joint_accel\n"
"\n"
"\n"
"###########################################################\n"
"# The remaining parameters only apply to Cartesian paths\n"
"\n"
"# Maximum linear speed of endpoint - m/s\n"
"float64 max_linear_speed\n"
"\n"
"# Maximum linear acceleration of endpoint - m/s^2\n"
"float64 max_linear_accel\n"
"\n"
"# Maximum rotational speed of endpoint - rad/s\n"
"float64 max_rotational_speed\n"
"\n"
"# Maximum rotational acceleration of endpoint - rad/s^2\n"
"float64 max_rotational_accel\n"
"\n"
"# Used for smoothing corners for continuous motion - m\n"
"# The distance from the waypoint to where the curve starts while blending from\n"
"# one straight line segment to the next.\n"
"# Larger distance:  trajectory passes farther from the waypoint at a higher speed\n"
"# Smaller distance:  trajectory passes closer to the waypoint at a lower speed\n"
"# Zero distance:  trajectory passes through the waypoint at zero speed\n"
"float64 corner_distance\n"
;
  }

  static const char* value(const ::intera_motion_msgs::Waypoint_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::intera_motion_msgs::Waypoint_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.joint_positions);
      stream.next(m.active_endpoint);
      stream.next(m.pose);
      stream.next(m.options);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Waypoint_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::intera_motion_msgs::Waypoint_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::intera_motion_msgs::Waypoint_<ContainerAllocator>& v)
  {
    s << indent << "joint_positions[]" << std::endl;
    for (size_t i = 0; i < v.joint_positions.size(); ++i)
    {
      s << indent << "  joint_positions[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.joint_positions[i]);
    }
    s << indent << "active_endpoint: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.active_endpoint);
    s << indent << "pose: ";
    s << std::endl;
    Printer< ::geometry_msgs::PoseStamped_<ContainerAllocator> >::stream(s, indent + "  ", v.pose);
    s << indent << "options: ";
    s << std::endl;
    Printer< ::intera_motion_msgs::WaypointOptions_<ContainerAllocator> >::stream(s, indent + "  ", v.options);
  }
};

} // namespace message_operations
} // namespace ros

#endif // INTERA_MOTION_MSGS_MESSAGE_WAYPOINT_H
