#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <cmath>

//function to compute the acceleration norm
double computeAccelerationNorm(const sensor_msgs::Imu::ConstPtr& msg) {
    double acc_x = msg->linear_acceleration.x;
    double acc_y = msg->linear_acceleration.y;
    double acc_z = msg->linear_acceleration.z;

    return sqrt(acc_x * acc_x + acc_y * acc_y + acc_z * acc_z);
}

void IMUCallback(const sensor_msgs::Imu::ConstPtr& imu_data){
    //Define the threshold
    double threshold = 15;

    //Compute the acceleration norm
    double acc_norm = computeAccelerationNorm(imu_data);

    //Check if the norm exceeds the threshold
    if (acc_norm >threshold){
        ROS_INFO("Contact detected, acceleration norm exceeded threshold %f", acc_norm);
    }

    else{
        ROS_INFO_THROTTLE(10.0, "No contact detected, acceleration norm is %f", acc_norm);
    }
}

int main(int argc, char** argv){

    ros::init(argc, argv, "imu_listener");

    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/uav1/hw_api/imu", 10, IMUCallback);

    ros::spin();

    return 0;

}