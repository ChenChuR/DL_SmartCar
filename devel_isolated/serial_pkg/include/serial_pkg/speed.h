// Generated by gencpp from file serial_pkg/speed.msg
// DO NOT EDIT!


#ifndef SERIAL_PKG_MESSAGE_SPEED_H
#define SERIAL_PKG_MESSAGE_SPEED_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace serial_pkg
{
template <class ContainerAllocator>
struct speed_
{
  typedef speed_<ContainerAllocator> Type;

  speed_()
    : vx(0.0)
    , angle(0.0)
    , signage(0)  {
    }
  speed_(const ContainerAllocator& _alloc)
    : vx(0.0)
    , angle(0.0)
    , signage(0)  {
  (void)_alloc;
    }



   typedef float _vx_type;
  _vx_type vx;

   typedef float _angle_type;
  _angle_type angle;

   typedef uint8_t _signage_type;
  _signage_type signage;





  typedef boost::shared_ptr< ::serial_pkg::speed_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::serial_pkg::speed_<ContainerAllocator> const> ConstPtr;

}; // struct speed_

typedef ::serial_pkg::speed_<std::allocator<void> > speed;

typedef boost::shared_ptr< ::serial_pkg::speed > speedPtr;
typedef boost::shared_ptr< ::serial_pkg::speed const> speedConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::serial_pkg::speed_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::serial_pkg::speed_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::serial_pkg::speed_<ContainerAllocator1> & lhs, const ::serial_pkg::speed_<ContainerAllocator2> & rhs)
{
  return lhs.vx == rhs.vx &&
    lhs.angle == rhs.angle &&
    lhs.signage == rhs.signage;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::serial_pkg::speed_<ContainerAllocator1> & lhs, const ::serial_pkg::speed_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace serial_pkg

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::serial_pkg::speed_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::serial_pkg::speed_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::serial_pkg::speed_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::serial_pkg::speed_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::serial_pkg::speed_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::serial_pkg::speed_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::serial_pkg::speed_<ContainerAllocator> >
{
  static const char* value()
  {
    return "628d09100e1102fef4349bb734fbb743";
  }

  static const char* value(const ::serial_pkg::speed_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x628d09100e1102feULL;
  static const uint64_t static_value2 = 0xf4349bb734fbb743ULL;
};

template<class ContainerAllocator>
struct DataType< ::serial_pkg::speed_<ContainerAllocator> >
{
  static const char* value()
  {
    return "serial_pkg/speed";
  }

  static const char* value(const ::serial_pkg::speed_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::serial_pkg::speed_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 vx\n"
"float32 angle\n"
"uint8 signage\n"
;
  }

  static const char* value(const ::serial_pkg::speed_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::serial_pkg::speed_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.vx);
      stream.next(m.angle);
      stream.next(m.signage);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct speed_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::serial_pkg::speed_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::serial_pkg::speed_<ContainerAllocator>& v)
  {
    s << indent << "vx: ";
    Printer<float>::stream(s, indent + "  ", v.vx);
    s << indent << "angle: ";
    Printer<float>::stream(s, indent + "  ", v.angle);
    s << indent << "signage: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.signage);
  }
};

} // namespace message_operations
} // namespace ros

#endif // SERIAL_PKG_MESSAGE_SPEED_H
