// Generated by gencpp from file intera_core_msgs/IOComponentCommand.msg
// DO NOT EDIT!


#ifndef INTERA_CORE_MSGS_MESSAGE_IOCOMPONENTCOMMAND_H
#define INTERA_CORE_MSGS_MESSAGE_IOCOMPONENTCOMMAND_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace intera_core_msgs
{
template <class ContainerAllocator>
struct IOComponentCommand_
{
  typedef IOComponentCommand_<ContainerAllocator> Type;

  IOComponentCommand_()
    : time()
    , op()
    , args()  {
    }
  IOComponentCommand_(const ContainerAllocator& _alloc)
    : time()
    , op(_alloc)
    , args(_alloc)  {
  (void)_alloc;
    }



   typedef ros::Time _time_type;
  _time_type time;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _op_type;
  _op_type op;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _args_type;
  _args_type args;





  typedef boost::shared_ptr< ::intera_core_msgs::IOComponentCommand_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::intera_core_msgs::IOComponentCommand_<ContainerAllocator> const> ConstPtr;

}; // struct IOComponentCommand_

typedef ::intera_core_msgs::IOComponentCommand_<std::allocator<void> > IOComponentCommand;

typedef boost::shared_ptr< ::intera_core_msgs::IOComponentCommand > IOComponentCommandPtr;
typedef boost::shared_ptr< ::intera_core_msgs::IOComponentCommand const> IOComponentCommandConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::intera_core_msgs::IOComponentCommand_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::intera_core_msgs::IOComponentCommand_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace intera_core_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'intera_core_msgs': ['/home/rachel/rawhide/rawhide_ws/src/intera_common/intera_core_msgs/msg', '/home/rachel/rawhide/rawhide_ws/devel/share/intera_core_msgs/msg'], 'geometry_msgs': ['/opt/ros/melodic/share/geometry_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/melodic/share/actionlib_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/melodic/share/std_msgs/cmake/../msg'], 'sensor_msgs': ['/opt/ros/melodic/share/sensor_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::intera_core_msgs::IOComponentCommand_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::intera_core_msgs::IOComponentCommand_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::intera_core_msgs::IOComponentCommand_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::intera_core_msgs::IOComponentCommand_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::intera_core_msgs::IOComponentCommand_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::intera_core_msgs::IOComponentCommand_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::intera_core_msgs::IOComponentCommand_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ede95ba2953dc221dc82cac20f697530";
  }

  static const char* value(const ::intera_core_msgs::IOComponentCommand_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xede95ba2953dc221ULL;
  static const uint64_t static_value2 = 0xdc82cac20f697530ULL;
};

template<class ContainerAllocator>
struct DataType< ::intera_core_msgs::IOComponentCommand_<ContainerAllocator> >
{
  static const char* value()
  {
    return "intera_core_msgs/IOComponentCommand";
  }

  static const char* value(const ::intera_core_msgs::IOComponentCommand_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::intera_core_msgs::IOComponentCommand_<ContainerAllocator> >
{
  static const char* value()
  {
    return "## IO Component Command\n"
"time time      # time the message was created, serves as a sequence number\n"
"string op      # operation to perform\n"
"string args    # JSON arguments\n"
;
  }

  static const char* value(const ::intera_core_msgs::IOComponentCommand_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::intera_core_msgs::IOComponentCommand_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.time);
      stream.next(m.op);
      stream.next(m.args);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct IOComponentCommand_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::intera_core_msgs::IOComponentCommand_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::intera_core_msgs::IOComponentCommand_<ContainerAllocator>& v)
  {
    s << indent << "time: ";
    Printer<ros::Time>::stream(s, indent + "  ", v.time);
    s << indent << "op: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.op);
    s << indent << "args: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.args);
  }
};

} // namespace message_operations
} // namespace ros

#endif // INTERA_CORE_MSGS_MESSAGE_IOCOMPONENTCOMMAND_H
