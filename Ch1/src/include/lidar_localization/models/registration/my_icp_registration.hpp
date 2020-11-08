/*
 * @Description: MY_ICP
 * @Author: Ren Qian
 * @Date: 2020-02-08 21:46:57
 */

#ifndef LIDAR_LOCALIZATION_MODELS_REGISTRATION_MY_ICP_REGISTRATION_HPP_
#define LIDAR_LOCALIZATION_MODELS_REGISTRATION_MY_ICP_REGISTRATION_HPP_

#include "lidar_localization/models/registration/registration_interface.hpp"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>

#include "sophus/se3.h"
#include "sophus/so3.h"

namespace lidar_localization {
class MY_ICPRegistration: public RegistrationInterface {
  public:
    MY_ICPRegistration(const YAML::Node& node);
    MY_ICPRegistration(float max_distance, size_t max_iter);

    bool SetInputTarget(const CloudData::CLOUD_PTR& input_target) override;
    bool ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                   const Eigen::Matrix4f& predict_pose, 
                   CloudData::CLOUD_PTR& result_cloud_ptr,
                   Eigen::Matrix4f& result_pose) override;
    bool PrintRegistrationParam();
  
  private:
    bool SetRegistrationParam(float dis, size_t max_iter);
    void calculateTrans(const CloudData::CLOUD_PTR   &input_cloud );       // 计算旋转矩阵

  private:
      CloudData::CLOUD_PTR target_cloud_;        
      pcl::KdTreeFLANN<CloudData::POINT>::Ptr  kdtree_ptr_;     // 创建一个指针，不能为空，需要在构造函数中初始化
      float max_correspond_distance_;     // 阈值
      size_t max_iterator_;                                     //最大迭代次数

      Eigen::Matrix3f  rotation_matrix_;       //旋转矩阵
      Eigen::Vector3f  translation_;                 //平移矩阵
      Eigen::Matrix4f  transformation_;       // 转换矩阵     

};
}

#endif