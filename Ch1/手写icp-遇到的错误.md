> 参考博客：https://blog.csdn.net/weixin_41281151/article/details/109439049

### 手写icp遇到的一些问题

**错误1：**

```bash
/home/s/ws_multi_sensor/src/src/models/registration/my_icp_registration.cpp:79:9: error: ‘transformPointCloud’ is not a member of ‘pcl’
         pcl::transformPointCloud<pcl::PointXYZ>(*input_source, *result_cloud_ptr, transformation_); // �Ե��ƽ��б任
         ^
/home/s/ws_multi_sensor/src/src/models/registration/my_icp_registration.cpp:79:47: error: expected primary-expression before ‘>’ token
         pcl::transformPointCloud<pcl::PointXYZ>(*input_source, *result_cloud_ptr, transformation_); // �Ե��ƽ��б任
                                               ^
/home/s/ws_multi_sensor/src/src/models/registration/my_icp_registration.cpp: In member function ‘void lidar_localization::MY_ICPRegistration::calculateTrans(const CLOUD_PTR&)’:
/home/s/ws_multi_sensor/src/src/models/registration/my_icp_registration.cpp:95:9: error: ‘transformPointCloud’ is not a member of ‘pcl’
         pcl::transformPointCloud<pcl::PointXYZ>(*input_source,*transformed_cloud,transformation_);    // �Ե��ƽ��б任
         ^
/home/s/ws_multi_sensor/src/src/models/registration/my_icp_registration.cpp:95:47: error: expected primary-expression before ‘>’ token
         pcl::transformPointCloud<pcl::PointXYZ>(*input_source,*transformed_cloud,transformation_);    // �Ե��ƽ��б任
                                               ^

```

> 看起来是因为transformPointCloud函数的问题，可能是没找到头文件。
>
> 添加\#include <pcl/common/transforms.h>即可

> 乱码问题：
>
> 和原始的文件对比，发现编码方式果然不一样，将编码方式统一改为原始文件的UTF-8即可

**错误2：**

```bash
front_end_node: /usr/include/boost/smart_ptr/shared_ptr.hpp:648: typename boost::detail::sp_member_access<T>::type boost::shared_ptr<T>::operator->() const [with T = pcl::KdTreeFLANN<pcl::PointXYZ>; typename boost::detail::sp_member_access<T>::type = pcl::KdTreeFLANN<pcl::PointXYZ>*]: Assertion `px != 0' failed.
[front_end_node-3] process has died [pid 15573, exit code -6, cmd /home/s/ws_multi_sensor/devel/lib/lidar_localization/front_end_node __name:=front_end_node __log:=/home/s/.ros/log/9df1beac-216f-11eb-9556-485f9946ea55/front_end_node-3.log].
log file: /home/s/.ros/log/9df1beac-216f-11eb-9556-485f9946ea55/front_end_node-3*.log

```

> 看起来是智能指针的问题，一般是因为指针没有在构造函数中赋值