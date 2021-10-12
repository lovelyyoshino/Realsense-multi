#include<librealsense2/rs.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Header.h>
#include <ros/ros.h>
#include<opencv2/core.hpp>
#include<opencv2/highgui.hpp>

#include <iostream>
#include <algorithm>

sensor_msgs::Imu Get_imu_data(rs2::frameset frames, int index)
{
      std_msgs::Header header;
      sensor_msgs::Imu imu_data;
      header.frame_id = "imu_data" + std::to_string(index);
      header.stamp.nsec = ros::Time::now().toSec();
      imu_data.header = header;

      if (rs2::motion_frame accel_frame = frames.first_or_default(RS2_STREAM_ACCEL))
      {
          rs2_vector accel_sample = accel_frame.get_motion_data();
           imu_data.linear_acceleration.x = accel_sample.x;
           imu_data.linear_acceleration.y = accel_sample.y;
           imu_data.linear_acceleration.z = accel_sample.z;
          // std::cout << "Accel:" << accel_sample.x << ", " << accel_sample.y << ", " << accel_sample.z << std::endl;
      }
      if (rs2::motion_frame gyro_frame = frames.first_or_default(RS2_STREAM_GYRO))
      {
           rs2_vector gyro_sample = gyro_frame.get_motion_data();
           imu_data.angular_velocity.x = gyro_sample.x;
           imu_data.angular_velocity.y = gyro_sample.y;
           imu_data.angular_velocity.z = gyro_sample.z;
          // std::cout << "Gyro:" << gyro_sample.x << ", " << gyro_sample.y << ", " << gyro_sample.z << std::endl;
      }
      return imu_data;

}


int main(int argc, char * argv[])
{
  ros::init(argc, argv, "realsense_multi_cam_t265");
  ros::NodeHandle nh;

  int32_t skip_frg;
  int32_t rgb_width, rgb_height, depth_width, depth_height;
  
  //配置启始信息
  ros::param::param<int32_t>("~rgb_width", rgb_width, 848);
  ros::param::param<int32_t>("~rgb_height", rgb_height, 800);

  rs2::context ctx;
  if(ctx.query_devices().size() == 0) throw std::runtime_error("No device detected. Is it plugged in?");

  // 轮询所有设备
  std::vector<rs2::device > devices;
  std::vector<rs2::pipeline> pipes(ctx.query_devices().size());
  std::vector<rs2::config> cfgs(ctx.query_devices().size());
  std::vector<rs2::pipeline_profile> pips_profile(ctx.query_devices().size());

  std::vector<ros::Publisher> fisheye1_pubs(ctx.query_devices().size());
  std::vector<ros::Publisher> fisheye2_pubs(ctx.query_devices().size());
    std::vector<ros::Publisher> imu_pubs(ctx.query_devices().size());


      for(int i = 0; i <ctx.query_devices().size(); ++i)
      {
      fisheye1_pubs[i] = nh.advertise<sensor_msgs::Image>("camera" +
        std::to_string(i) + "/fisheye1/image_raw", 1, true);
      fisheye2_pubs[i] = nh.advertise<sensor_msgs::Image>("camera" +
        std::to_string(i) + "/fisheye2/image_raw", 1, true);
      imu_pubs[i] = nh.advertise<sensor_msgs::Imu>("imu" +
        std::to_string(i), 1, true);
      cfgs[i].enable_stream(RS2_STREAM_FISHEYE, 1, RS2_FORMAT_Y8);
      cfgs[i].enable_stream(RS2_STREAM_FISHEYE, 2, RS2_FORMAT_Y8);
      cfgs[i].enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
      cfgs[i].enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
      pips_profile[i] = pipes[i].start(cfgs[i]);
      ROS_INFO_STREAM("Device with name T265 was found.");
      }


  while (ros::ok())
  {
		rs2::frameset frames;
    cv_bridge::CvImage cv_img;
    std_msgs::Header header;
      for(int i = 0; i <ctx.query_devices().size(); ++i)
      {
        frames= pipes[i].wait_for_frames();

        // rs2::video_frame color_frame = frames.get_color_frame();
        // rs2::video_frame depth_frame = frames.get_depth_frame();
        rs2::video_frame ir_frame_left = frames.get_fisheye_frame(1);
		    rs2::video_frame ir_frame_right = frames.get_fisheye_frame(2);
        const int width = ir_frame_right.get_width();
        const int height = ir_frame_right.get_height();
        sensor_msgs::Imu imu_data = Get_imu_data(frames, i);

        // cv::Mat color_img(cv::Size(rgb_width, rgb_height), CV_8UC3, (void*)color_frame.get_data(),cv:: Mat::AUTO_STEP);
        // cv::Mat depth_img(cv::Size(depth_width, depth_height), CV_16U, (void*)depth_frame.get_data(),cv:: Mat::AUTO_STEP);
        cv::Mat fisheye1_img(cv::Size(rgb_width, rgb_height), CV_8UC1, (void*)ir_frame_left.get_data());
        cv::Mat fisheye2_img(cv::Size(rgb_width, rgb_height), CV_8UC1, (void*)ir_frame_right.get_data());

        header.frame_id = "fisheye1_frame" + std::to_string(i);
        header.stamp = ros::Time::now();
        cv_img.header = header;
        cv_img.image = fisheye1_img;
        cv_img.encoding = "mono8";
        fisheye1_pubs[i].publish(cv_img.toImageMsg());


        header.frame_id = "fisheye2_frame" + std::to_string(i);
        header.stamp = ros::Time::now();
        cv_img.header = header;
        cv_img.image = fisheye2_img;
        cv_img.encoding = "mono8";
        fisheye2_pubs[i].publish(cv_img.toImageMsg());

        imu_pubs[i].publish(imu_data);
      }

  }


}