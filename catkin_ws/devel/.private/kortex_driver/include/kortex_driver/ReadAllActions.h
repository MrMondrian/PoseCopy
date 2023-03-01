// Generated by gencpp from file kortex_driver/ReadAllActions.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_READALLACTIONS_H
#define KORTEX_DRIVER_MESSAGE_READALLACTIONS_H

#include <ros/service_traits.h>


#include <kortex_driver/ReadAllActionsRequest.h>
#include <kortex_driver/ReadAllActionsResponse.h>


namespace kortex_driver
{

struct ReadAllActions
{

typedef ReadAllActionsRequest Request;
typedef ReadAllActionsResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct ReadAllActions
} // namespace kortex_driver


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::kortex_driver::ReadAllActions > {
  static const char* value()
  {
    return "19862e60bc6b5a5249992cb7e602ec0f";
  }

  static const char* value(const ::kortex_driver::ReadAllActions&) { return value(); }
};

template<>
struct DataType< ::kortex_driver::ReadAllActions > {
  static const char* value()
  {
    return "kortex_driver/ReadAllActions";
  }

  static const char* value(const ::kortex_driver::ReadAllActions&) { return value(); }
};


// service_traits::MD5Sum< ::kortex_driver::ReadAllActionsRequest> should match
// service_traits::MD5Sum< ::kortex_driver::ReadAllActions >
template<>
struct MD5Sum< ::kortex_driver::ReadAllActionsRequest>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::ReadAllActions >::value();
  }
  static const char* value(const ::kortex_driver::ReadAllActionsRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::ReadAllActionsRequest> should match
// service_traits::DataType< ::kortex_driver::ReadAllActions >
template<>
struct DataType< ::kortex_driver::ReadAllActionsRequest>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::ReadAllActions >::value();
  }
  static const char* value(const ::kortex_driver::ReadAllActionsRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::kortex_driver::ReadAllActionsResponse> should match
// service_traits::MD5Sum< ::kortex_driver::ReadAllActions >
template<>
struct MD5Sum< ::kortex_driver::ReadAllActionsResponse>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::ReadAllActions >::value();
  }
  static const char* value(const ::kortex_driver::ReadAllActionsResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::ReadAllActionsResponse> should match
// service_traits::DataType< ::kortex_driver::ReadAllActions >
template<>
struct DataType< ::kortex_driver::ReadAllActionsResponse>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::ReadAllActions >::value();
  }
  static const char* value(const ::kortex_driver::ReadAllActionsResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_READALLACTIONS_H
