// Generated by gencpp from file wiimote/TimedSwitch.msg
// DO NOT EDIT!


#ifndef WIIMOTE_MESSAGE_TIMEDSWITCH_H
#define WIIMOTE_MESSAGE_TIMEDSWITCH_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace wiimote
{
template <class ContainerAllocator>
struct TimedSwitch_
{
  typedef TimedSwitch_<ContainerAllocator> Type;

  TimedSwitch_()
    : switch_mode(0)
    , num_cycles(0)
    , pulse_pattern()  {
    }
  TimedSwitch_(const ContainerAllocator& _alloc)
    : switch_mode(0)
    , num_cycles(0)
    , pulse_pattern(_alloc)  {
  (void)_alloc;
    }



   typedef int8_t _switch_mode_type;
  _switch_mode_type switch_mode;

   typedef int32_t _num_cycles_type;
  _num_cycles_type num_cycles;

   typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _pulse_pattern_type;
  _pulse_pattern_type pulse_pattern;



  enum {
    ON = 1,
    OFF = 0,
    NO_CHANGE = -2,
    REPEAT = -1,
    FOREVER = -1,
  };


  typedef boost::shared_ptr< ::wiimote::TimedSwitch_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::wiimote::TimedSwitch_<ContainerAllocator> const> ConstPtr;

}; // struct TimedSwitch_

typedef ::wiimote::TimedSwitch_<std::allocator<void> > TimedSwitch;

typedef boost::shared_ptr< ::wiimote::TimedSwitch > TimedSwitchPtr;
typedef boost::shared_ptr< ::wiimote::TimedSwitch const> TimedSwitchConstPtr;

// constants requiring out of line definition

   

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::wiimote::TimedSwitch_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::wiimote::TimedSwitch_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace wiimote

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'sensor_msgs': ['/opt/ros/kinetic/share/sensor_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'wiimote': ['/home/rachel/Documents/School/Spring_2019/ME_102B/ros_workspace/src/joystick_drivers/wiimote/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::wiimote::TimedSwitch_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::wiimote::TimedSwitch_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::wiimote::TimedSwitch_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::wiimote::TimedSwitch_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::wiimote::TimedSwitch_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::wiimote::TimedSwitch_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::wiimote::TimedSwitch_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e4c8d9327409cef6066fa6c368032c1e";
  }

  static const char* value(const ::wiimote::TimedSwitch_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe4c8d9327409cef6ULL;
  static const uint64_t static_value2 = 0x066fa6c368032c1eULL;
};

template<class ContainerAllocator>
struct DataType< ::wiimote::TimedSwitch_<ContainerAllocator> >
{
  static const char* value()
  {
    return "wiimote/TimedSwitch";
  }

  static const char* value(const ::wiimote::TimedSwitch_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::wiimote::TimedSwitch_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# TimedSwitch allows sender to:\n\
#    o turn a switch on,\n\
#    o turn a switch off, and\n\
#    o repeat an on/off pattern forever or for a\n\
#          given number of times.\n\
# Fields (refer to definitions of constants in the definition body):\n\
#     o switch_mode:\n\
#         ON: turn on  (num_cycles and pulse_pattern fields are ignored)\n\
#        OFF: turn off (num_cycles and pulse_pattern fields are ignored)\n\
#  NO_CHANGE: leave LED in its current state\n\
#     REPEAT: repeat an on/off pattern for as long\n\
#             as is indicated in the num_cycles field. The\n\
#             pattern is defined in the pulse_pattern field.\n\
#\n\
#     o num_cycles:\n\
#          n>=0: run the pattern that is defined in pulse_pattern\n\
#                n times.\n\
#          n==FOREVER: run the pattern that is defined in pulse_pattern\n\
#                       until a new TimedSwitch message is sent.              \n\
#\n\
#     o pulse_pattern:\n\
#          A series of time durations in fractions of a second. The\n\
#          first number is the duration for having the switch on.\n\
#          The second number is the duration for which the switch\n\
#          is off. The third is an 'on' period again, etc.\n\
#          A pattern is terminated with the end of the array.\n\
#           \n\
#          Example: [1,1] specifies an on-off sequence of 1 second.               \n\
\n\
int8 ON        =  1\n\
int8 OFF       =  0\n\
int8 NO_CHANGE = -2\n\
int8 REPEAT    = -1\n\
int8 FOREVER   = -1\n\
\n\
int8 switch_mode\n\
int32 num_cycles\n\
float32[] pulse_pattern\n\
";
  }

  static const char* value(const ::wiimote::TimedSwitch_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::wiimote::TimedSwitch_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.switch_mode);
      stream.next(m.num_cycles);
      stream.next(m.pulse_pattern);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct TimedSwitch_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::wiimote::TimedSwitch_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::wiimote::TimedSwitch_<ContainerAllocator>& v)
  {
    s << indent << "switch_mode: ";
    Printer<int8_t>::stream(s, indent + "  ", v.switch_mode);
    s << indent << "num_cycles: ";
    Printer<int32_t>::stream(s, indent + "  ", v.num_cycles);
    s << indent << "pulse_pattern[]" << std::endl;
    for (size_t i = 0; i < v.pulse_pattern.size(); ++i)
    {
      s << indent << "  pulse_pattern[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.pulse_pattern[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // WIIMOTE_MESSAGE_TIMEDSWITCH_H
