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
    void calculateTrans(const CloudData::CLOUD_PTR   &input_cloud );       // ������ת����

  private:
      CloudData::CLOUD_PTR target_cloud_;        
      pcl::KdTreeFLANN<CloudData::POINT>::Ptr  kdtree_ptr_;     // ����һ��ָ�룬����Ϊ�գ���Ҫ�ڹ��캯���г�ʼ��
      float max_correspond_distance_;     // ��ֵ
      size_t max_iterator_;                                     //����������

      Eigen::Matrix3f  rotation_matrix_;       //��ת����
      Eigen::Vector3f  translation_;                 //ƽ�ƾ���
      Eigen::Matrix4f  transformation_;       // ת������     

};
}

#endif