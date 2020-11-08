/*
 * @Descripttion: test
 * @vision: 
 * @Author: suyunzzz
 * @Date: 2020-10-18 16:29:08
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2020-10-24 19:26:52
 */
// icp接口

#ifndef LIDAR_LOCALIZATION_MODELS_REGISTRATION_ICP_REGISTRATION_HPP_
#define LIDAR_LOCALIZATION_MODELS_REGISTRATION_ICP_REGISTRATION_HPP_

#include <pcl/registration/icp.h>
#include "lidar_localization/models/registration/registration_interface.hpp"
namespace lidar_localization {
class ICPRegistration: public RegistrationInterface {
  public:
    ICPRegistration(const YAML::Node& node);
    ICPRegistration(float max_distance, float fit_eps, float trans_eps, int max_iter);

    bool SetInputTarget(const CloudData::CLOUD_PTR& input_target) override;
    bool ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                   const Eigen::Matrix4f& predict_pose, 
                   CloudData::CLOUD_PTR& result_cloud_ptr,
                   Eigen::Matrix4f& result_pose) override;
    bool firstFrame=true;
  private:
    bool SetRegistrationParam(float max_distance, float fit_eps, float trans_eps, int max_iter);
    bool PrintRegistrationParam();      // 打印参数

  private:
    pcl::IterativeClosestPoint<CloudData::POINT, CloudData::POINT>::Ptr icp_ptr_;
};
}

#endif