// Generated by gencpp from file kortex_driver/OnNotificationOperatingModeTopic.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_ONNOTIFICATIONOPERATINGMODETOPIC_H
#define KORTEX_DRIVER_MESSAGE_ONNOTIFICATIONOPERATINGMODETOPIC_H

#include <ros/service_traits.h>


#include <kortex_driver/OnNotificationOperatingModeTopicRequest.h>
#include <kortex_driver/OnNotificationOperatingModeTopicResponse.h>


namespace kortex_driver
{

struct OnNotificationOperatingModeTopic
{

typedef OnNotificationOperatingModeTopicRequest Request;
typedef OnNotificationOperatingModeTopicResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct OnNotificationOperatingModeTopic
} // namespace kortex_driver


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::kortex_driver::OnNotificationOperatingModeTopic > {
  static const char* value()
  {
    return "6fefdd07c6cb63a94f7b48e7e07e815b";
  }

  static const char* value(const ::kortex_driver::OnNotificationOperatingModeTopic&) { return value(); }
};

template<>
struct DataType< ::kortex_driver::OnNotificationOperatingModeTopic > {
  static const char* value()
  {
    return "kortex_driver/OnNotificationOperatingModeTopic";
  }

  static const char* value(const ::kortex_driver::OnNotificationOperatingModeTopic&) { return value(); }
};


// service_traits::MD5Sum< ::kortex_driver::OnNotificationOperatingModeTopicRequest> should match
// service_traits::MD5Sum< ::kortex_driver::OnNotificationOperatingModeTopic >
template<>
struct MD5Sum< ::kortex_driver::OnNotificationOperatingModeTopicRequest>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::OnNotificationOperatingModeTopic >::value();
  }
  static const char* value(const ::kortex_driver::OnNotificationOperatingModeTopicRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::OnNotificationOperatingModeTopicRequest> should match
// service_traits::DataType< ::kortex_driver::OnNotificationOperatingModeTopic >
template<>
struct DataType< ::kortex_driver::OnNotificationOperatingModeTopicRequest>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::OnNotificationOperatingModeTopic >::value();
  }
  static const char* value(const ::kortex_driver::OnNotificationOperatingModeTopicRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::kortex_driver::OnNotificationOperatingModeTopicResponse> should match
// service_traits::MD5Sum< ::kortex_driver::OnNotificationOperatingModeTopic >
template<>
struct MD5Sum< ::kortex_driver::OnNotificationOperatingModeTopicResponse>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::OnNotificationOperatingModeTopic >::value();
  }
  static const char* value(const ::kortex_driver::OnNotificationOperatingModeTopicResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::OnNotificationOperatingModeTopicResponse> should match
// service_traits::DataType< ::kortex_driver::OnNotificationOperatingModeTopic >
template<>
struct DataType< ::kortex_driver::OnNotificationOperatingModeTopicResponse>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::OnNotificationOperatingModeTopic >::value();
  }
  static const char* value(const ::kortex_driver::OnNotificationOperatingModeTopicResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_ONNOTIFICATIONOPERATINGMODETOPIC_H
