// Generated by gencpp from file kortex_driver/GetGPIOStateRequest.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_GETGPIOSTATEREQUEST_H
#define KORTEX_DRIVER_MESSAGE_GETGPIOSTATEREQUEST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <kortex_driver/GPIOIdentification.h>

namespace kortex_driver
{
template <class ContainerAllocator>
struct GetGPIOStateRequest_
{
  typedef GetGPIOStateRequest_<ContainerAllocator> Type;

  GetGPIOStateRequest_()
    : input()  {
    }
  GetGPIOStateRequest_(const ContainerAllocator& _alloc)
    : input(_alloc)  {
  (void)_alloc;
    }



   typedef  ::kortex_driver::GPIOIdentification_<ContainerAllocator>  _input_type;
  _input_type input;





  typedef boost::shared_ptr< ::kortex_driver::GetGPIOStateRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::GetGPIOStateRequest_<ContainerAllocator> const> ConstPtr;

}; // struct GetGPIOStateRequest_

typedef ::kortex_driver::GetGPIOStateRequest_<std::allocator<void> > GetGPIOStateRequest;

typedef boost::shared_ptr< ::kortex_driver::GetGPIOStateRequest > GetGPIOStateRequestPtr;
typedef boost::shared_ptr< ::kortex_driver::GetGPIOStateRequest const> GetGPIOStateRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::GetGPIOStateRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::GetGPIOStateRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_driver::GetGPIOStateRequest_<ContainerAllocator1> & lhs, const ::kortex_driver::GetGPIOStateRequest_<ContainerAllocator2> & rhs)
{
  return lhs.input == rhs.input;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_driver::GetGPIOStateRequest_<ContainerAllocator1> & lhs, const ::kortex_driver::GetGPIOStateRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::GetGPIOStateRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::GetGPIOStateRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::GetGPIOStateRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::GetGPIOStateRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::GetGPIOStateRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::GetGPIOStateRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::GetGPIOStateRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "28fc5544fa28c41b8590a221a4f4db30";
  }

  static const char* value(const ::kortex_driver::GetGPIOStateRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x28fc5544fa28c41bULL;
  static const uint64_t static_value2 = 0x8590a221a4f4db30ULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::GetGPIOStateRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/GetGPIOStateRequest";
  }

  static const char* value(const ::kortex_driver::GetGPIOStateRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::GetGPIOStateRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "GPIOIdentification input\n"
"\n"
"================================================================================\n"
"MSG: kortex_driver/GPIOIdentification\n"
"\n"
"uint32 identifier\n"
;
  }

  static const char* value(const ::kortex_driver::GetGPIOStateRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::GetGPIOStateRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.input);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GetGPIOStateRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::GetGPIOStateRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_driver::GetGPIOStateRequest_<ContainerAllocator>& v)
  {
    s << indent << "input: ";
    s << std::endl;
    Printer< ::kortex_driver::GPIOIdentification_<ContainerAllocator> >::stream(s, indent + "  ", v.input);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_GETGPIOSTATEREQUEST_H
