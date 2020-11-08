/*
 * @Descripttion: test
 * @vision: 
 * @Author: suyunzzz
 * @Date: 2020-10-18 16:31:04
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2020-11-08 11:23:19
 */

#include "lidar_localization/models/registration/icp_registration.hpp"

#include "glog/logging.h"
namespace lidar_localization {

ICPRegistration::ICPRegistration(const YAML::Node& node)
    :icp_ptr_(new pcl::IterativeClosestPoint<CloudData::POINT, CloudData::POINT>()) {
    
    float max_distance = node["max_distance"].as<float>();
    float fit_eps = node["fit_eps"].as<float>();
    float trans_eps = node["trans_eps"].as<float>();
    int max_iter = node["max_iter"].as<int>();

    SetRegistrationParam(max_distance, fit_eps, trans_eps, max_iter);
    // PrintRegistrationParam();       // ʹ��Ĭ�ϲ���
}

ICPRegistration::ICPRegistration(float max_distance, float fit_eps, float trans_eps, int max_iter)
    :icp_ptr_(new pcl::IterativeClosestPoint<CloudData::POINT, CloudData::POINT>()) {
    
    // PrintRegistrationParam();
    SetRegistrationParam(max_distance, fit_eps, trans_eps, max_iter);
}

bool ICPRegistration::SetRegistrationParam(float max_distance, float fit_eps, float trans_eps, int max_iter) {
    icp_ptr_->setMaxCorrespondenceDistance(max_distance);
    icp_ptr_->setEuclideanFitnessEpsilon(fit_eps);
    icp_ptr_->setTransformationEpsilon(trans_eps);      // setTransformationEpsilon �������α任����֮��Ĳ�ֵ��һ������Ϊ1e-10��
    icp_ptr_->setMaximumIterations(max_iter);
  

    LOG(INFO) << "ICP 的匹配参数为：" << std::endl
              << "max_distance: " << icp_ptr_->getMaxCorrespondenceDistance() << ", "
              << "fit_eps: " << fit_eps << ", "
              << "trans_eps: " << trans_eps << ", "
              << "max_iter: " << max_iter 
              << std::endl << std::endl;

    return true;
}

bool ICPRegistration::PrintRegistrationParam()
{
    LOG(INFO) << "ICP 的匹配参数为：" << std::endl
            << "max_distance: " << icp_ptr_->getMaxCorrespondenceDistance() << ", "
            << "fit_eps: " << icp_ptr_->getEuclideanFitnessEpsilon() << ", "
            << "trans_eps: " << icp_ptr_->getTransformationEpsilon() << ", "
            << "max_iter: " << icp_ptr_->getMaximumIterations() 
            << std::endl << std::endl;

    return true;
}

bool ICPRegistration::SetInputTarget(const CloudData::CLOUD_PTR& input_target) {
    icp_ptr_->setInputTarget(input_target);

    return true;
}

bool ICPRegistration::ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                                const Eigen::Matrix4f& predict_pose, 
                                CloudData::CLOUD_PTR& result_cloud_ptr,
                                Eigen::Matrix4f& result_pose) {
    icp_ptr_->setInputSource(input_source);

    
    // if(firstFrame==true)
    // {
    //     icp_ptr_->align(*result_cloud_ptr, predict_pose);       // param1����׼���source���ƣ�param2��Ԥ���pose
    //     firstFrame=false;
    // }
    // else
    // {
    //     icp_ptr_->align(*result_cloud_ptr);       // param1����׼���source���ƣ�param2��Ԥ���pose
    // }
    
    icp_ptr_->align(*result_cloud_ptr, predict_pose);       // param1����׼���source���ƣ�param2��Ԥ���pose

    result_pose = icp_ptr_->getFinalTransformation();       // ��ȡ���յ�pose

    return true;
}
}
