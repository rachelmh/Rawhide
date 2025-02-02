// Generated by gencpp from file intera_core_msgs/AnalogOutputCommand.msg
// DO NOT EDIT!


#ifndef INTERA_CORE_MSGS_MESSAGE_ANALOGOUTPUTCOMMAND_H
#define INTERA_CORE_MSGS_MESSAGE_ANALOGOUTPUTCOMMAND_H


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
struct AnalogOutputCommand_
{
  typedef AnalogOutputCommand_<ContainerAllocator> Type;

  AnalogOutputCommand_()
    : name()
    , value(0)  {
    }
  AnalogOutputCommand_(const ContainerAllocator& _alloc)
    : name(_alloc)
    , value(0)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _name_type;
  _name_type name;

   typedef uint16_t _value_type;
  _value_type value;





  typedef boost::shared_ptr< ::intera_core_msgs::AnalogOutputCommand_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::intera_core_msgs::AnalogOutputCommand_<ContainerAllocator> const> ConstPtr;

}; // struct AnalogOutputCommand_

typedef ::intera_core_msgs::AnalogOutputCommand_<std::allocator<void> > AnalogOutputCommand;

typedef boost::shared_ptr< ::intera_core_msgs::AnalogOutputCommand > AnalogOutputCommandPtr;
typedef boost::shared_ptr< ::intera_core_msgs::AnalogOutputCommand const> AnalogOutputCommandConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::intera_core_msgs::AnalogOutputCommand_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::intera_core_msgs::AnalogOutputCommand_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::intera_core_msgs::AnalogOutputCommand_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::intera_core_msgs::AnalogOutputCommand_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::intera_core_msgs::AnalogOutputCommand_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::intera_core_msgs::AnalogOutputCommand_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::intera_core_msgs::AnalogOutputCommand_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::intera_core_msgs::AnalogOutputCommand_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::intera_core_msgs::AnalogOutputCommand_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a7b945129a083ca4095d48aa94841d85";
  }

  static const char* value(const ::intera_core_msgs::AnalogOutputCommand_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa7b945129a083ca4ULL;
  static const uint64_t static_value2 = 0x095d48aa94841d85ULL;
};

template<class ContainerAllocator>
struct DataType< ::intera_core_msgs::AnalogOutputCommand_<ContainerAllocator> >
{
  static const char* value()
  {
    return "intera_core_msgs/AnalogOutputCommand";
  }

  static const char* value(const ::intera_core_msgs::AnalogOutputCommand_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::intera_core_msgs::AnalogOutputCommand_<ContainerAllocator> >
{
  static const char* value()
  {
    return "##the name of the output\n"
"string name  \n"
"##the value to set output \n"
"uint16 value   \n"
;
  }

  static const char* value(const ::intera_core_msgs::AnalogOutputCommand_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::intera_core_msgs::AnalogOutputCommand_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.name);
      stream.next(m.value);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct AnalogOutputCommand_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::intera_core_msgs::AnalogOutputCommand_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::intera_core_msgs::AnalogOutputCommand_<ContainerAllocator>& v)
  {
    s << indent << "name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.name);
    s << indent << "value: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.value);
  }
};

} // namespace message_operations
} // namespace ros

#endif // INTERA_CORE_MSGS_MESSAGE_ANALOGOUTPUTCOMMAND_H
