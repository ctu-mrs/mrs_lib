#ifndef SERVICE_CLIENT_HANDLER_HPP
#define SERVICE_CLIENT_HANDLER_HPP

namespace mrs_lib
{

// --------------------------------------------------------------
// |                    ServiceClientHandler                    |
// --------------------------------------------------------------

/* initialize(ros::NodeHandle& nh, const std::string& address) //{ */

template <class ServiceType>
void ServiceClientHandler::initialize(ros::NodeHandle& nh, const std::string& address)
{
  service_client_ = nh.serviceClient<ServiceType>(address);
}

//}

/* call(ServiceType& srv) //{ */

template <class ServiceType>
bool ServiceClientHandler::call(ServiceType& srv)
{
  return service_client_.call(srv);
}

//}

}  // namespace mrs_lib

#endif  // SERVICE_CLIENT_HANDLER_HPP
