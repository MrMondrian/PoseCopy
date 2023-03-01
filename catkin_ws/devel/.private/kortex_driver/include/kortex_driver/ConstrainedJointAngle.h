// Generated by gencpp from file kortex_driver/ConstrainedJointAngle.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_CONSTRAINEDJOINTANGLE_H
#define KORTEX_DRIVER_MESSAGE_CONSTRAINEDJOINTANGLE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <kortex_driver/JointTrajectoryConstraint.h>

namespace kortex_driver
{
template <class ContainerAllocator>
struct ConstrainedJointAngle_
{
  typedef ConstrainedJointAngle_<ContainerAllocator> Type;

  ConstrainedJointAngle_()
    : joint_identifier(0)
    , value(0.0)
    , constraint()  {
    }
  ConstrainedJointAngle_(const ContainerAllocator& _alloc)
    : joint_identifier(0)
    , value(0.0)
    , constraint(_alloc)  {
  (void)_alloc;
    }



   typedef uint32_t _joint_identifier_type;
  _joint_identifier_type joint_identifier;

   typedef float _value_type;
  _value_type value;

   typedef  ::kortex_driver::JointTrajectoryConstraint_<ContainerAllocator>  _constraint_type;
  _constraint_type constraint;





  typedef boost::shared_ptr< ::kortex_driver::ConstrainedJointAngle_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::ConstrainedJointAngle_<ContainerAllocator> const> ConstPtr;

}; // struct ConstrainedJointAngle_

typedef ::kortex_driver::ConstrainedJointAngle_<std::allocator<void> > ConstrainedJointAngle;

typedef boost::shared_ptr< ::kortex_driver::ConstrainedJointAngle > ConstrainedJointAnglePtr;
typedef boost::shared_ptr< ::kortex_driver::ConstrainedJointAngle const> ConstrainedJointAngleConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::ConstrainedJointAngle_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::ConstrainedJointAngle_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_driver::ConstrainedJointAngle_<ContainerAllocator1> & lhs, const ::kortex_driver::ConstrainedJointAngle_<ContainerAllocator2> & rhs)
{
  return lhs.joint_identifier == rhs.joint_identifier &&
    lhs.value == rhs.value &&
    lhs.constraint == rhs.constraint;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_driver::ConstrainedJointAngle_<ContainerAllocator1> & lhs, const ::kortex_driver::ConstrainedJointAngle_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::ConstrainedJointAngle_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::ConstrainedJointAngle_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::ConstrainedJointAngle_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::ConstrainedJointAngle_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::ConstrainedJointAngle_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::ConstrainedJointAngle_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::ConstrainedJointAngle_<ContainerAllocator> >
{
  static const char* value()
  {
    return "6fb07b64148f47061948eb6c6ef38e00";
  }

  static const char* value(const ::kortex_driver::ConstrainedJointAngle_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x6fb07b64148f4706ULL;
  static const uint64_t static_value2 = 0x1948eb6c6ef38e00ULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::ConstrainedJointAngle_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/ConstrainedJointAngle";
  }

  static const char* value(const ::kortex_driver::ConstrainedJointAngle_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::ConstrainedJointAngle_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"uint32 joint_identifier\n"
"float32 value\n"
"JointTrajectoryConstraint constraint\n"
"================================================================================\n"
"MSG: kortex_driver/JointTrajectoryConstraint\n"
"\n"
"uint32 type\n"
"float32 value\n"
;
  }

  static const char* value(const ::kortex_driver::ConstrainedJointAngle_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::ConstrainedJointAngle_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.joint_identifier);
      stream.next(m.value);
      stream.next(m.constraint);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ConstrainedJointAngle_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::ConstrainedJointAngle_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_driver::ConstrainedJointAngle_<ContainerAllocator>& v)
  {
    s << indent << "joint_identifier: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.joint_identifier);
    s << indent << "value: ";
    Printer<float>::stream(s, indent + "  ", v.value);
    s << indent << "constraint: ";
    s << std::endl;
    Printer< ::kortex_driver::JointTrajectoryConstraint_<ContainerAllocator> >::stream(s, indent + "  ", v.constraint);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_CONSTRAINEDJOINTANGLE_H
