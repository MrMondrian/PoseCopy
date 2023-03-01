// Generated by gencpp from file kortex_driver/I2CReadRegisterParameter.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_I2CREADREGISTERPARAMETER_H
#define KORTEX_DRIVER_MESSAGE_I2CREADREGISTERPARAMETER_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace kortex_driver
{
template <class ContainerAllocator>
struct I2CReadRegisterParameter_
{
  typedef I2CReadRegisterParameter_<ContainerAllocator> Type;

  I2CReadRegisterParameter_()
    : device(0)
    , device_address(0)
    , register_address(0)
    , register_address_size(0)
    , size(0)
    , timeout(0)  {
    }
  I2CReadRegisterParameter_(const ContainerAllocator& _alloc)
    : device(0)
    , device_address(0)
    , register_address(0)
    , register_address_size(0)
    , size(0)
    , timeout(0)  {
  (void)_alloc;
    }



   typedef uint32_t _device_type;
  _device_type device;

   typedef uint32_t _device_address_type;
  _device_address_type device_address;

   typedef uint32_t _register_address_type;
  _register_address_type register_address;

   typedef uint32_t _register_address_size_type;
  _register_address_size_type register_address_size;

   typedef uint32_t _size_type;
  _size_type size;

   typedef uint32_t _timeout_type;
  _timeout_type timeout;





  typedef boost::shared_ptr< ::kortex_driver::I2CReadRegisterParameter_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::I2CReadRegisterParameter_<ContainerAllocator> const> ConstPtr;

}; // struct I2CReadRegisterParameter_

typedef ::kortex_driver::I2CReadRegisterParameter_<std::allocator<void> > I2CReadRegisterParameter;

typedef boost::shared_ptr< ::kortex_driver::I2CReadRegisterParameter > I2CReadRegisterParameterPtr;
typedef boost::shared_ptr< ::kortex_driver::I2CReadRegisterParameter const> I2CReadRegisterParameterConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::I2CReadRegisterParameter_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::I2CReadRegisterParameter_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_driver::I2CReadRegisterParameter_<ContainerAllocator1> & lhs, const ::kortex_driver::I2CReadRegisterParameter_<ContainerAllocator2> & rhs)
{
  return lhs.device == rhs.device &&
    lhs.device_address == rhs.device_address &&
    lhs.register_address == rhs.register_address &&
    lhs.register_address_size == rhs.register_address_size &&
    lhs.size == rhs.size &&
    lhs.timeout == rhs.timeout;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_driver::I2CReadRegisterParameter_<ContainerAllocator1> & lhs, const ::kortex_driver::I2CReadRegisterParameter_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::I2CReadRegisterParameter_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::I2CReadRegisterParameter_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::I2CReadRegisterParameter_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::I2CReadRegisterParameter_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::I2CReadRegisterParameter_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::I2CReadRegisterParameter_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::I2CReadRegisterParameter_<ContainerAllocator> >
{
  static const char* value()
  {
    return "5d3209e38cc377eccf27593ef8027f34";
  }

  static const char* value(const ::kortex_driver::I2CReadRegisterParameter_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x5d3209e38cc377ecULL;
  static const uint64_t static_value2 = 0xcf27593ef8027f34ULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::I2CReadRegisterParameter_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/I2CReadRegisterParameter";
  }

  static const char* value(const ::kortex_driver::I2CReadRegisterParameter_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::I2CReadRegisterParameter_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"uint32 device\n"
"uint32 device_address\n"
"uint32 register_address\n"
"uint32 register_address_size\n"
"uint32 size\n"
"uint32 timeout\n"
;
  }

  static const char* value(const ::kortex_driver::I2CReadRegisterParameter_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::I2CReadRegisterParameter_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.device);
      stream.next(m.device_address);
      stream.next(m.register_address);
      stream.next(m.register_address_size);
      stream.next(m.size);
      stream.next(m.timeout);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct I2CReadRegisterParameter_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::I2CReadRegisterParameter_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_driver::I2CReadRegisterParameter_<ContainerAllocator>& v)
  {
    s << indent << "device: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.device);
    s << indent << "device_address: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.device_address);
    s << indent << "register_address: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.register_address);
    s << indent << "register_address_size: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.register_address_size);
    s << indent << "size: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.size);
    s << indent << "timeout: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.timeout);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_I2CREADREGISTERPARAMETER_H
