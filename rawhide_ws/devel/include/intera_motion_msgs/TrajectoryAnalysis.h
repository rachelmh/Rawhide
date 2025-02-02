// Generated by gencpp from file intera_motion_msgs/TrajectoryAnalysis.msg
// DO NOT EDIT!


#ifndef INTERA_MOTION_MSGS_MESSAGE_TRAJECTORYANALYSIS_H
#define INTERA_MOTION_MSGS_MESSAGE_TRAJECTORYANALYSIS_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace intera_motion_msgs
{
template <class ContainerAllocator>
struct TrajectoryAnalysis_
{
  typedef TrajectoryAnalysis_<ContainerAllocator> Type;

  TrajectoryAnalysis_()
    : planned_duration(0.0)
    , measured_duration(0.0)
    , min_angle_command()
    , max_angle_command()
    , peak_speed_command()
    , peak_accel_command()
    , peak_jerk_command()
    , min_time_rate(0.0)
    , max_time_rate(0.0)
    , max_position_error()
    , max_velocity_error()  {
    }
  TrajectoryAnalysis_(const ContainerAllocator& _alloc)
    : planned_duration(0.0)
    , measured_duration(0.0)
    , min_angle_command(_alloc)
    , max_angle_command(_alloc)
    , peak_speed_command(_alloc)
    , peak_accel_command(_alloc)
    , peak_jerk_command(_alloc)
    , min_time_rate(0.0)
    , max_time_rate(0.0)
    , max_position_error(_alloc)
    , max_velocity_error(_alloc)  {
  (void)_alloc;
    }



   typedef double _planned_duration_type;
  _planned_duration_type planned_duration;

   typedef double _measured_duration_type;
  _measured_duration_type measured_duration;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _min_angle_command_type;
  _min_angle_command_type min_angle_command;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _max_angle_command_type;
  _max_angle_command_type max_angle_command;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _peak_speed_command_type;
  _peak_speed_command_type peak_speed_command;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _peak_accel_command_type;
  _peak_accel_command_type peak_accel_command;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _peak_jerk_command_type;
  _peak_jerk_command_type peak_jerk_command;

   typedef double _min_time_rate_type;
  _min_time_rate_type min_time_rate;

   typedef double _max_time_rate_type;
  _max_time_rate_type max_time_rate;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _max_position_error_type;
  _max_position_error_type max_position_error;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _max_velocity_error_type;
  _max_velocity_error_type max_velocity_error;





  typedef boost::shared_ptr< ::intera_motion_msgs::TrajectoryAnalysis_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::intera_motion_msgs::TrajectoryAnalysis_<ContainerAllocator> const> ConstPtr;

}; // struct TrajectoryAnalysis_

typedef ::intera_motion_msgs::TrajectoryAnalysis_<std::allocator<void> > TrajectoryAnalysis;

typedef boost::shared_ptr< ::intera_motion_msgs::TrajectoryAnalysis > TrajectoryAnalysisPtr;
typedef boost::shared_ptr< ::intera_motion_msgs::TrajectoryAnalysis const> TrajectoryAnalysisConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::intera_motion_msgs::TrajectoryAnalysis_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::intera_motion_msgs::TrajectoryAnalysis_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::intera_motion_msgs::TrajectoryAnalysis_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::intera_motion_msgs::TrajectoryAnalysis_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::intera_motion_msgs::TrajectoryAnalysis_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::intera_motion_msgs::TrajectoryAnalysis_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::intera_motion_msgs::TrajectoryAnalysis_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::intera_motion_msgs::TrajectoryAnalysis_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::intera_motion_msgs::TrajectoryAnalysis_<ContainerAllocator> >
{
  static const char* value()
  {
    return "f30ec541413b4eb2cecc0d0af7d30ad4";
  }

  static const char* value(const ::intera_motion_msgs::TrajectoryAnalysis_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf30ec541413b4eb2ULL;
  static const uint64_t static_value2 = 0xcecc0d0af7d30ad4ULL;
};

template<class ContainerAllocator>
struct DataType< ::intera_motion_msgs::TrajectoryAnalysis_<ContainerAllocator> >
{
  static const char* value()
  {
    return "intera_motion_msgs/TrajectoryAnalysis";
  }

  static const char* value(const ::intera_motion_msgs::TrajectoryAnalysis_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::intera_motion_msgs::TrajectoryAnalysis_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# The duration of the reference trajectory, as originally planned\n"
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

  static const char* value(const ::intera_motion_msgs::TrajectoryAnalysis_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::intera_motion_msgs::TrajectoryAnalysis_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.planned_duration);
      stream.next(m.measured_duration);
      stream.next(m.min_angle_command);
      stream.next(m.max_angle_command);
      stream.next(m.peak_speed_command);
      stream.next(m.peak_accel_command);
      stream.next(m.peak_jerk_command);
      stream.next(m.min_time_rate);
      stream.next(m.max_time_rate);
      stream.next(m.max_position_error);
      stream.next(m.max_velocity_error);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct TrajectoryAnalysis_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::intera_motion_msgs::TrajectoryAnalysis_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::intera_motion_msgs::TrajectoryAnalysis_<ContainerAllocator>& v)
  {
    s << indent << "planned_duration: ";
    Printer<double>::stream(s, indent + "  ", v.planned_duration);
    s << indent << "measured_duration: ";
    Printer<double>::stream(s, indent + "  ", v.measured_duration);
    s << indent << "min_angle_command[]" << std::endl;
    for (size_t i = 0; i < v.min_angle_command.size(); ++i)
    {
      s << indent << "  min_angle_command[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.min_angle_command[i]);
    }
    s << indent << "max_angle_command[]" << std::endl;
    for (size_t i = 0; i < v.max_angle_command.size(); ++i)
    {
      s << indent << "  max_angle_command[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.max_angle_command[i]);
    }
    s << indent << "peak_speed_command[]" << std::endl;
    for (size_t i = 0; i < v.peak_speed_command.size(); ++i)
    {
      s << indent << "  peak_speed_command[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.peak_speed_command[i]);
    }
    s << indent << "peak_accel_command[]" << std::endl;
    for (size_t i = 0; i < v.peak_accel_command.size(); ++i)
    {
      s << indent << "  peak_accel_command[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.peak_accel_command[i]);
    }
    s << indent << "peak_jerk_command[]" << std::endl;
    for (size_t i = 0; i < v.peak_jerk_command.size(); ++i)
    {
      s << indent << "  peak_jerk_command[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.peak_jerk_command[i]);
    }
    s << indent << "min_time_rate: ";
    Printer<double>::stream(s, indent + "  ", v.min_time_rate);
    s << indent << "max_time_rate: ";
    Printer<double>::stream(s, indent + "  ", v.max_time_rate);
    s << indent << "max_position_error[]" << std::endl;
    for (size_t i = 0; i < v.max_position_error.size(); ++i)
    {
      s << indent << "  max_position_error[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.max_position_error[i]);
    }
    s << indent << "max_velocity_error[]" << std::endl;
    for (size_t i = 0; i < v.max_velocity_error.size(); ++i)
    {
      s << indent << "  max_velocity_error[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.max_velocity_error[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // INTERA_MOTION_MSGS_MESSAGE_TRAJECTORYANALYSIS_H
