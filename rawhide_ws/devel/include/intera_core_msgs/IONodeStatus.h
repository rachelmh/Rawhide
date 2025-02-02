// Generated by gencpp from file intera_core_msgs/IONodeStatus.msg
// DO NOT EDIT!


#ifndef INTERA_CORE_MSGS_MESSAGE_IONODESTATUS_H
#define INTERA_CORE_MSGS_MESSAGE_IONODESTATUS_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <intera_core_msgs/IOComponentStatus.h>
#include <intera_core_msgs/IOComponentStatus.h>

namespace intera_core_msgs
{
template <class ContainerAllocator>
struct IONodeStatus_
{
  typedef IONodeStatus_<ContainerAllocator> Type;

  IONodeStatus_()
    : time()
    , node()
    , devices()
    , commands()  {
    }
  IONodeStatus_(const ContainerAllocator& _alloc)
    : time()
    , node(_alloc)
    , devices(_alloc)
    , commands(_alloc)  {
  (void)_alloc;
    }



   typedef ros::Time _time_type;
  _time_type time;

   typedef  ::intera_core_msgs::IOComponentStatus_<ContainerAllocator>  _node_type;
  _node_type node;

   typedef std::vector< ::intera_core_msgs::IOComponentStatus_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::intera_core_msgs::IOComponentStatus_<ContainerAllocator> >::other >  _devices_type;
  _devices_type devices;

   typedef std::vector<ros::Time, typename ContainerAllocator::template rebind<ros::Time>::other >  _commands_type;
  _commands_type commands;





  typedef boost::shared_ptr< ::intera_core_msgs::IONodeStatus_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::intera_core_msgs::IONodeStatus_<ContainerAllocator> const> ConstPtr;

}; // struct IONodeStatus_

typedef ::intera_core_msgs::IONodeStatus_<std::allocator<void> > IONodeStatus;

typedef boost::shared_ptr< ::intera_core_msgs::IONodeStatus > IONodeStatusPtr;
typedef boost::shared_ptr< ::intera_core_msgs::IONodeStatus const> IONodeStatusConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::intera_core_msgs::IONodeStatus_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::intera_core_msgs::IONodeStatus_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::intera_core_msgs::IONodeStatus_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::intera_core_msgs::IONodeStatus_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::intera_core_msgs::IONodeStatus_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::intera_core_msgs::IONodeStatus_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::intera_core_msgs::IONodeStatus_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::intera_core_msgs::IONodeStatus_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::intera_core_msgs::IONodeStatus_<ContainerAllocator> >
{
  static const char* value()
  {
    return "260fce3c02f43bd977c92642b3c09c1d";
  }

  static const char* value(const ::intera_core_msgs::IONodeStatus_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x260fce3c02f43bd9ULL;
  static const uint64_t static_value2 = 0x77c92642b3c09c1dULL;
};

template<class ContainerAllocator>
struct DataType< ::intera_core_msgs::IONodeStatus_<ContainerAllocator> >
{
  static const char* value()
  {
    return "intera_core_msgs/IONodeStatus";
  }

  static const char* value(const ::intera_core_msgs::IONodeStatus_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::intera_core_msgs::IONodeStatus_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# IO Node Status\n"
"time time                    # time the message was created\n"
"IOComponentStatus node       # IO Node status\n"
"IOComponentStatus[] devices  # status of IO Devices in this node\n"
"time[] commands              # recent command timestamps, for syncing\n"
"================================================================================\n"
"MSG: intera_core_msgs/IOComponentStatus\n"
"## IO Component status data\n"
"string name            # component name\n"
"IOStatus status        # component status\n"
"#\n"
"\n"
"\n"
"\n"
"================================================================================\n"
"MSG: intera_core_msgs/IOStatus\n"
"## IO status data\n"
"#\n"
"string tag             # one of the values listed below\n"
"#   down     Inoperative, not fully instantiated\n"
"#   ready    OK, fully operational\n"
"#   busy     OK, not ready to output data; input data value may be stale\n"
"#   unready  OK, not operational; data is invalid\n"
"#   error    Error, not operational\n"
"string DOWN      = down\n"
"string READY     = ready\n"
"string BUSY      = busy\n"
"string UNREADY   = unready\n"
"string ERROR     = error\n"
"#\n"
"string id             # message id, for internationalization\n"
"#\n"
"string detail         # optional additional status detail\n"
"#\n"
;
  }

  static const char* value(const ::intera_core_msgs::IONodeStatus_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::intera_core_msgs::IONodeStatus_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.time);
      stream.next(m.node);
      stream.next(m.devices);
      stream.next(m.commands);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct IONodeStatus_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::intera_core_msgs::IONodeStatus_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::intera_core_msgs::IONodeStatus_<ContainerAllocator>& v)
  {
    s << indent << "time: ";
    Printer<ros::Time>::stream(s, indent + "  ", v.time);
    s << indent << "node: ";
    s << std::endl;
    Printer< ::intera_core_msgs::IOComponentStatus_<ContainerAllocator> >::stream(s, indent + "  ", v.node);
    s << indent << "devices[]" << std::endl;
    for (size_t i = 0; i < v.devices.size(); ++i)
    {
      s << indent << "  devices[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::intera_core_msgs::IOComponentStatus_<ContainerAllocator> >::stream(s, indent + "    ", v.devices[i]);
    }
    s << indent << "commands[]" << std::endl;
    for (size_t i = 0; i < v.commands.size(); ++i)
    {
      s << indent << "  commands[" << i << "]: ";
      Printer<ros::Time>::stream(s, indent + "  ", v.commands[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // INTERA_CORE_MSGS_MESSAGE_IONODESTATUS_H
