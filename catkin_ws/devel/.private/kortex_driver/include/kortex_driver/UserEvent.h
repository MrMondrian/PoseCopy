// Generated by gencpp from file kortex_driver/UserEvent.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_USEREVENT_H
#define KORTEX_DRIVER_MESSAGE_USEREVENT_H


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
struct UserEvent_
{
  typedef UserEvent_<ContainerAllocator> Type;

  UserEvent_()
    {
    }
  UserEvent_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }





// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(UNSPECIFIED_USER_EVENT)
  #undef UNSPECIFIED_USER_EVENT
#endif
#if defined(_WIN32) && defined(LOGGED_OUT)
  #undef LOGGED_OUT
#endif
#if defined(_WIN32) && defined(LOGGED_IN)
  #undef LOGGED_IN
#endif

  enum {
    UNSPECIFIED_USER_EVENT = 0u,
    LOGGED_OUT = 1u,
    LOGGED_IN = 2u,
  };


  typedef boost::shared_ptr< ::kortex_driver::UserEvent_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::UserEvent_<ContainerAllocator> const> ConstPtr;

}; // struct UserEvent_

typedef ::kortex_driver::UserEvent_<std::allocator<void> > UserEvent;

typedef boost::shared_ptr< ::kortex_driver::UserEvent > UserEventPtr;
typedef boost::shared_ptr< ::kortex_driver::UserEvent const> UserEventConstPtr;

// constants requiring out of line definition

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::UserEvent_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::UserEvent_<ContainerAllocator> >::stream(s, "", v);
return s;
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::UserEvent_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::UserEvent_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::UserEvent_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::UserEvent_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::UserEvent_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::UserEvent_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::UserEvent_<ContainerAllocator> >
{
  static const char* value()
  {
    return "6b6b55b08fd45b34a43cf1d05e2fdbdc";
  }

  static const char* value(const ::kortex_driver::UserEvent_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x6b6b55b08fd45b34ULL;
  static const uint64_t static_value2 = 0xa43cf1d05e2fdbdcULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::UserEvent_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/UserEvent";
  }

  static const char* value(const ::kortex_driver::UserEvent_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::UserEvent_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"uint32 UNSPECIFIED_USER_EVENT = 0\n"
"\n"
"uint32 LOGGED_OUT = 1\n"
"\n"
"uint32 LOGGED_IN = 2\n"
;
  }

  static const char* value(const ::kortex_driver::UserEvent_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::UserEvent_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct UserEvent_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::UserEvent_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::kortex_driver::UserEvent_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_USEREVENT_H
