/*
 * @Descripttion: test
 * @vision: 
 * @Author: suyunzzz
 * @Date: 2020-10-18 16:31:04
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2020-11-08 16:14:00
 */

#include "lidar_localization/models/registration/my_icp_registration.hpp"

#include "glog/logging.h"
namespace lidar_localization
{

    MY_ICPRegistration::MY_ICPRegistration(const YAML::Node &node):kdtree_ptr_(new pcl::KdTreeFLANN<CloudData::POINT>)
    {

        float max_distance = node["max_distance"].as<float>();
        // float fit_eps = node["fit_eps"].as<float>();
        // float trans_eps = node["trans_eps"].as<float>();
        int max_iter = node["max_iter"].as<size_t>();

        SetRegistrationParam(max_distance, max_iter);
        // PrintRegistrationParam();       // 打印参数
    }

    MY_ICPRegistration::MY_ICPRegistration(float max_distance, size_t max_iter):kdtree_ptr_(new pcl::KdTreeFLANN<CloudData::POINT>)
    {

        // PrintRegistrationParam();
        SetRegistrationParam(max_distance, max_iter);
    }

    bool MY_ICPRegistration::SetRegistrationParam(float max_distance, size_t max_iter)
    {
        max_correspond_distance_ = max_distance;
        max_iterator_ = max_iter;

        LOG(INFO) << "MY_ICP 的匹配参数为：" << std::endl
                  << "max_distance: " << max_correspond_distance_ << ", "
                  << "max_iter: " << max_iterator_
                  << std::endl
                  << std::endl;

        return true;
    }

    bool MY_ICPRegistration::PrintRegistrationParam()
    {
        LOG(INFO) << "MY_ICP 的匹配参数为：" << std::endl
                  << "max_distance: " << max_correspond_distance_ << ", "
                  << "max_iter: " << max_iterator_
                  << std::endl
                  << std::endl;

        return true;
    }

    bool MY_ICPRegistration::SetInputTarget(const CloudData::CLOUD_PTR &input_target)
    {
        target_cloud_.reset(new CloudData::CLOUD);
        target_cloud_ = input_target;
        kdtree_ptr_->setInputCloud(input_target);
        return true;
    }

    bool MY_ICPRegistration::ScanMatch(const CloudData::CLOUD_PTR &input_source,
                                       const Eigen::Matrix4f &predict_pose,
                                       CloudData::CLOUD_PTR &result_cloud_ptr,
                                       Eigen::Matrix4f &result_pose)
    {


        LOG(INFO) << "predict_pose \n"
                  << predict_pose;

        transformation_ = predict_pose;
        rotation_matrix_ = transformation_.block<3, 3>(0, 0); //ȡ��ת����
        translation_ = transformation_.block<3, 1>(0, 3);

        calculateTrans(input_source); // 计算变换，更新transformation_

        pcl::transformPointCloud<pcl::PointXYZ>(*input_source, *result_cloud_ptr, transformation_); // 得到变换后的点云
        result_pose = transformation_;

        LOG(INFO) << "result_pose \n"
                  << result_pose;

        return true;
    }


void MY_ICPRegistration::calculateTrans(const CloudData::CLOUD_PTR   &input_source){
    CloudData::CLOUD_PTR  transformed_cloud(new CloudData::CLOUD);
    int knn = 1;     // 定义最近邻
    size_t iterator_num = 0;
    while(iterator_num < max_iterator_)
    {
        pcl::transformPointCloud<pcl::PointXYZ>(*input_source,*transformed_cloud,transformation_);    // 将source变换到target坐标系下
        Eigen::Matrix<float,6,6> Hessian;
        Eigen::Matrix<float,6,1>B;
        Hessian.setZero();
        B.setZero();     // ����

        for(size_t i =0; i < transformed_cloud->size();  ++i)
        {
            auto ori_point = input_source->at(i);
            if(!pcl::isFinite(ori_point))
                continue;
            auto transformed_point = transformed_cloud->at(i);
            std::vector<float> distances;
            std::vector<int>indexs;     
            kdtree_ptr_->nearestKSearch(transformed_point,knn,indexs,distances);      // knn����
            if(distances[0] > max_correspond_distance_)
            {
                continue;
            }
            Eigen::Vector3f closet_point  = Eigen::Vector3f(target_cloud_->at(indexs[0]).x,   target_cloud_->at(indexs[0]).y ,
                                                                target_cloud_->at(indexs[0]).z );

            // 计算残差函数
            Eigen::Vector3f err_dis = 
                Eigen::Vector3f(transformed_point.x,transformed_point.y,transformed_point.z) - closet_point;

            Eigen::Matrix<float,3,6> Jacobian(Eigen::Matrix<float,3,6>::Zero());
            Jacobian.leftCols<3>() = Eigen::Matrix3f::Identity();

            // 右扰动
            // Jacobian.rightCols<3>() = 
            //         -rotation_matrix_* (Sophus::SO3::hat(Eigen::Vector3d(ori_point.x,ori_point.y,ori_point.z))).cast<float>() ;

            // 左扰动
            Jacobian.rightCols<3>()=
                        -Sophus::SO3::hat((rotation_matrix_*Eigen::Vector3f(ori_point.x,ori_point.y,ori_point.z)).cast<double>()).cast<float>();
            
                    Hessian  +=  Jacobian.transpose()* Jacobian; 
                    B += -Jacobian.transpose()*err_dis;
        }
        iterator_num++;
        if(Hessian.determinant() == 0)
        {
                continue;
        }
        Eigen::Matrix<float,6,1> delta_x =  Hessian.inverse()*B;

        translation_ += delta_x.head<3>();
        auto  delta_rotation = Sophus::SO3::exp(delta_x.tail<3>().cast<double>());

        // 右扰动，右乘
        // rotation_matrix_ *= delta_rotation.matrix().cast<float>();          

        // 左扰动，左乘
        rotation_matrix_=delta_rotation.matrix().cast<float>()*rotation_matrix_;
        
        transformation_.block<3,3>(0,0) = rotation_matrix_;
        transformation_.block<3,1>(0,3) = translation_;

    }

}

} // namespace lidar_localization
