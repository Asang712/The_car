
#ifndef NAV2_UTIL__SERVICE_SERVER_HPP_
#define NAV2_UTIL__SERVICE_SERVER_HPP_

#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace nav2_util
{

/**
 * @class nav2_util::ServiceServer
 * @brief A simple wrapper on ROS2 services server
 */
template<class ServiceT, typename NodeT = rclcpp::Node::SharedPtr>
class ServiceServer
{
public:
  using RequestType = typename ServiceT::Request;
  using ResponseType = typename ServiceT::Response;
  using CallbackType = std::function<void(const std::shared_ptr<rmw_request_id_t>,
      const std::shared_ptr<RequestType>, std::shared_ptr<ResponseType>)>;
  using SharedPtr = std::shared_ptr<ServiceServer<ServiceT, NodeT>>;

  explicit ServiceServer(
    const std::string & service_name,
    const NodeT & node,
    CallbackType callback,
    const rclcpp::QoS & qos = rclcpp::ServicesQoS(),
    rclcpp::CallbackGroup::SharedPtr callback_group = nullptr)
  : service_name_(service_name), callback_(callback)
  {
    server_ = node->template create_service<ServiceT>(
      service_name,
      [this](const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<RequestType> request, std::shared_ptr<ResponseType> response) {
        this->callback_(request_header, request, response);
      },
      qos,
      callback_group);
  }

protected:
  std::string service_name_;
  CallbackType callback_;
  typename rclcpp::Service<ServiceT>::SharedPtr server_;
};

}  // namespace nav2_util


#endif  // NAV2_UTIL__SERVICE_SERVER_HPP_
