// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from interfaces:srv/Decoder.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__SRV__DETAIL__DECODER__TRAITS_HPP_
#define INTERFACES__SRV__DETAIL__DECODER__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "interfaces/srv/detail/decoder__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const Decoder_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: req
  {
    out << "req: ";
    rosidl_generator_traits::value_to_yaml(msg.req, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Decoder_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: req
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "req: ";
    rosidl_generator_traits::value_to_yaml(msg.req, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Decoder_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace interfaces

namespace rosidl_generator_traits
{

[[deprecated("use interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const interfaces::srv::Decoder_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const interfaces::srv::Decoder_Request & msg)
{
  return interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<interfaces::srv::Decoder_Request>()
{
  return "interfaces::srv::Decoder_Request";
}

template<>
inline const char * name<interfaces::srv::Decoder_Request>()
{
  return "interfaces/srv/Decoder_Request";
}

template<>
struct has_fixed_size<interfaces::srv::Decoder_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<interfaces::srv::Decoder_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<interfaces::srv::Decoder_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const Decoder_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: res
  {
    if (msg.res.size() == 0) {
      out << "res: []";
    } else {
      out << "res: [";
      size_t pending_items = msg.res.size();
      for (auto item : msg.res) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Decoder_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: res
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.res.size() == 0) {
      out << "res: []\n";
    } else {
      out << "res:\n";
      for (auto item : msg.res) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Decoder_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace interfaces

namespace rosidl_generator_traits
{

[[deprecated("use interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const interfaces::srv::Decoder_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const interfaces::srv::Decoder_Response & msg)
{
  return interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<interfaces::srv::Decoder_Response>()
{
  return "interfaces::srv::Decoder_Response";
}

template<>
inline const char * name<interfaces::srv::Decoder_Response>()
{
  return "interfaces/srv/Decoder_Response";
}

template<>
struct has_fixed_size<interfaces::srv::Decoder_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<interfaces::srv::Decoder_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<interfaces::srv::Decoder_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<interfaces::srv::Decoder>()
{
  return "interfaces::srv::Decoder";
}

template<>
inline const char * name<interfaces::srv::Decoder>()
{
  return "interfaces/srv/Decoder";
}

template<>
struct has_fixed_size<interfaces::srv::Decoder>
  : std::integral_constant<
    bool,
    has_fixed_size<interfaces::srv::Decoder_Request>::value &&
    has_fixed_size<interfaces::srv::Decoder_Response>::value
  >
{
};

template<>
struct has_bounded_size<interfaces::srv::Decoder>
  : std::integral_constant<
    bool,
    has_bounded_size<interfaces::srv::Decoder_Request>::value &&
    has_bounded_size<interfaces::srv::Decoder_Response>::value
  >
{
};

template<>
struct is_service<interfaces::srv::Decoder>
  : std::true_type
{
};

template<>
struct is_service_request<interfaces::srv::Decoder_Request>
  : std::true_type
{
};

template<>
struct is_service_response<interfaces::srv::Decoder_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // INTERFACES__SRV__DETAIL__DECODER__TRAITS_HPP_
