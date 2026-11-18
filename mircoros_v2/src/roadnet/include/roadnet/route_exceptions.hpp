
#ifndef NAV2_CORE__ROUTE_EXCEPTIONS_HPP_
#define NAV2_CORE__ROUTE_EXCEPTIONS_HPP_

#include <stdexcept>
#include <string>
#include <memory>

namespace nav2_core
{

class RouteException : public std::runtime_error
{
public:
  explicit RouteException(const std::string & description)
  : std::runtime_error(description) {}
};

class OperationFailed : public RouteException
{
public:
  explicit OperationFailed(const std::string & description)
  : RouteException(description) {}
};

class NoValidRouteCouldBeFound : public RouteException
{
public:
  explicit NoValidRouteCouldBeFound(const std::string & description)
  : RouteException(description) {}
};

class TimedOut : public RouteException
{
public:
  explicit TimedOut(const std::string & description)
  : RouteException(description) {}
};

class RouteTFError : public RouteException
{
public:
  explicit RouteTFError(const std::string & description)
  : RouteException(description) {}
};

class NoValidGraph : public RouteException
{
public:
  explicit NoValidGraph(const std::string & description)
  : RouteException(description) {}
};

class IndeterminantNodesOnGraph : public RouteException
{
public:
  explicit IndeterminantNodesOnGraph(const std::string & description)
  : RouteException(description) {}
};

class InvalidEdgeScorerUse : public RouteException
{
public:
  explicit InvalidEdgeScorerUse(const std::string & description)
  : RouteException(description) {}
};

}  // namespace nav2_core

#endif  // NAV2_CORE__ROUTE_EXCEPTIONS_HPP_
