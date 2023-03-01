// Generated by gencpp from file kortex_driver/ResetPayloadInformation.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_RESETPAYLOADINFORMATION_H
#define KORTEX_DRIVER_MESSAGE_RESETPAYLOADINFORMATION_H

#include <ros/service_traits.h>


#include <kortex_driver/ResetPayloadInformationRequest.h>
#include <kortex_driver/ResetPayloadInformationResponse.h>


namespace kortex_driver
{

struct ResetPayloadInformation
{

typedef ResetPayloadInformationRequest Request;
typedef ResetPayloadInformationResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct ResetPayloadInformation
} // namespace kortex_driver


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::kortex_driver::ResetPayloadInformation > {
  static const char* value()
  {
    return "38a744b19ddbb71fb4d7e8724de570f6";
  }

  static const char* value(const ::kortex_driver::ResetPayloadInformation&) { return value(); }
};

template<>
struct DataType< ::kortex_driver::ResetPayloadInformation > {
  static const char* value()
  {
    return "kortex_driver/ResetPayloadInformation";
  }

  static const char* value(const ::kortex_driver::ResetPayloadInformation&) { return value(); }
};


// service_traits::MD5Sum< ::kortex_driver::ResetPayloadInformationRequest> should match
// service_traits::MD5Sum< ::kortex_driver::ResetPayloadInformation >
template<>
struct MD5Sum< ::kortex_driver::ResetPayloadInformationRequest>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::ResetPayloadInformation >::value();
  }
  static const char* value(const ::kortex_driver::ResetPayloadInformationRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::ResetPayloadInformationRequest> should match
// service_traits::DataType< ::kortex_driver::ResetPayloadInformation >
template<>
struct DataType< ::kortex_driver::ResetPayloadInformationRequest>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::ResetPayloadInformation >::value();
  }
  static const char* value(const ::kortex_driver::ResetPayloadInformationRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::kortex_driver::ResetPayloadInformationResponse> should match
// service_traits::MD5Sum< ::kortex_driver::ResetPayloadInformation >
template<>
struct MD5Sum< ::kortex_driver::ResetPayloadInformationResponse>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::ResetPayloadInformation >::value();
  }
  static const char* value(const ::kortex_driver::ResetPayloadInformationResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::ResetPayloadInformationResponse> should match
// service_traits::DataType< ::kortex_driver::ResetPayloadInformation >
template<>
struct DataType< ::kortex_driver::ResetPayloadInformationResponse>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::ResetPayloadInformation >::value();
  }
  static const char* value(const ::kortex_driver::ResetPayloadInformationResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_RESETPAYLOADINFORMATION_H
