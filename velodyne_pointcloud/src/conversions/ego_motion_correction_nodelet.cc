#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "velodyne_pointcloud/ego_motion_correction.h"

namespace velodyne_pointcloud
{
class EgoMotionCorrectionNodelet : public nodelet::Nodelet
{
  private:
    void onInit() override;
    boost::shared_ptr<EgoMotionCorrection> emc_;
};

void EgoMotionCorrectionNodelet::onInit()
{
    emc_.reset(new EgoMotionCorrection(getNodeHandle(), getPrivateNodeHandle()));
}

} // namespace velodyne_pointcloud

PLUGINLIB_EXPORT_CLASS(velodyne_pointcloud::EgoMotionCorrectionNodelet, nodelet::Nodelet)
