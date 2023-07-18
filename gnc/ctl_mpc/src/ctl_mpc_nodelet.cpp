#include "../include/ctl_acados_mpc/ctl_mpc.h"

#include <ff_common/init.h>
#include <ff_util/ff_nodelet.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <memory>

// TODO: REMOVE THIS AND PUT IT IN ff_util/ff_names.h

#ifndef NODE_CTL_MPC
#define NODE_CTL_MPC                        "ctl_mpc"
#endif //NODE_CTL_MPC

namespace ctl_mpc {

class Ctl_MPCNodelet : public ff_util::FreeFlyerNodelet {
 public:
  Ctl_MPCNodelet() : ff_util::FreeFlyerNodelet(NODE_CTL_MPC, true) {}
  ~Ctl_MPCNodelet() {}
  // This is called when the nodelet is loaded into the nodelet manager
  void Initialize(ros::NodeHandle *nh) {
    // Bootstrap our environment
    ff_common::InitFreeFlyerApplication(getMyArgv(), false);
    ctl_mpc_.reset(new Ctl_MPC(this->GetPlatformHandle(true), getName()));
  }

 private:
  std::shared_ptr<ctl_mpc::Ctl_MPC> ctl_mpc_;
};

}  // end namespace ctl_mpc

// Declare the plugin
PLUGINLIB_EXPORT_CLASS(ctl_mpc::Ctl_MPCNodelet, nodelet::Nodelet);