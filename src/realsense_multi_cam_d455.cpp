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

//参考：https://github.com/ljxwy/RStest_c/blob/master/main.cpp
void Enable_emitter(rs2::pipeline_profile selection) 
{
  rs2::device selected_device = selection.get_device();
  auto depth_sensor = selected_device.first<rs2::depth_sensor>();
  if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED))
  {
//        depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1.f); // Enable emitter //实践证明这并不能开启
    depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f); // Disable emitter // 关闭倒是有用，下面的关闭也有用
  }
  if (depth_sensor.supports(RS2_OPTION_LASER_POWER))
  {
    // Query min and max values:
    auto range = depth_sensor.get_option_range(RS2_OPTION_LASER_POWER);
    //  depth_sensor.set_option(RS2_OPTION_LASER_POWER, range.max); // Set max power //这个才能开启，开启之后设备会记得这个设定，关闭同理。
    depth_sensor.set_option(RS2_OPTION_LASER_POWER, 0.f); // Disable laser
  }

}

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
  ros::init(argc, argv, "realsense_multi_cam_d455");
  ros::NodeHandle nh;

  int32_t lr_threshold;
  int32_t skip_frg;
  int32_t rgb_width, rgb_height, depth_width, depth_height;
  
  //配置启始信息
  ros::param::param<int32_t>("~rgb_width", rgb_width, 640);
  ros::param::param<int32_t>("~rgb_height", rgb_height, 480);
  ros::param::param<int32_t>("~depth_width", depth_width, 640);
  ros::param::param<int32_t>("~depth_height", depth_height, 480);

  bool lr_auto_exposure;
  ros::param::param<bool>("~lr_auto_exposure", lr_auto_exposure, true);
  rs2::context ctx;
  if(ctx.query_devices().size() == 0) throw std::runtime_error("No device detected. Is it plugged in?");

  // 轮询所有设备
  std::vector<rs2::device > devices;
  std::vector<rs2::pipeline> pipes(ctx.query_devices().size());
  std::vector<rs2::config> cfgs(ctx.query_devices().size());
  std::vector<rs2::pipeline_profile> pips_profile(ctx.query_devices().size());

  // std::vector<ros::Publisher> depth_pubs(ctx.query_devices().size());
  // std::vector<ros::Publisher> rgb_pubs(ctx.query_devices().size());
  std::vector<ros::Publisher> ir1_pubs(ctx.query_devices().size());
  std::vector<ros::Publisher> ir2_pubs(ctx.query_devices().size());
    std::vector<ros::Publisher> imu_pubs(ctx.query_devices().size());
  // std::vector<ros::Publisher> point_pubs(ctx.query_devices().size());
  // std::vector<sensor_msgs::CameraInfo> depth_infos(ctx.query_devices().size());
  // std::vector<sensor_msgs::CameraInfo> rgb_infos(ctx.query_all_sensors().size());


  for(int i = 0; i <ctx.query_devices().size(); ++i)
  {
    devices.push_back(ctx.query_devices()[i]);
    // depth_pubs[i] = nh.advertise<sensor_msgs::Image>("camera" + std::to_string(i) +
    //     "/depth/image_raw", 1, true);
    // rgb_pubs[i] = nh.advertise<sensor_msgs::Image>("camera" + std::to_string(i) +
    //     "/rgb/image_raw", 1, true);
    ir1_pubs[i] = nh.advertise<sensor_msgs::Image>("camera" +
        std::to_string(i) + "/ir1/image_raw", 1, true);
    ir2_pubs[i] = nh.advertise<sensor_msgs::Image>("camera" +
        std::to_string(i) + "/ir2/image_raw", 1, true);
    imu_pubs[i] = nh.advertise<sensor_msgs::Imu>("imu" +
        std::to_string(i), 1, true);
      //point_pubs[i] = nh.advertise<sensor_msgs::PointCloud>("camera" +
      //std::to_string(i) + "/depth/points", 1, true);   
      // cfgs[i].enable_stream(RS2_STREAM_COLOR, rgb_width, rgb_height, RS2_FORMAT_BGR8, 30);
      // cfgs[i].enable_stream(RS2_STREAM_DEPTH, depth_width, depth_height, RS2_FORMAT_Z16,30);
      cfgs[i].enable_stream(RS2_STREAM_INFRARED, 1, rgb_width, rgb_height, RS2_FORMAT_Y8, 30);
      cfgs[i].enable_stream(RS2_STREAM_INFRARED, 2, rgb_width, rgb_height, RS2_FORMAT_Y8, 30);
      cfgs[i].enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
      cfgs[i].enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);

    pips_profile[i] = pipes[i].start(cfgs[i]);
    Enable_emitter(pips_profile[i]);
  }

    // 启动设备信息
  for(auto &&dev :devices)
  {
    auto sn = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
    ROS_INFO_STREAM("Device with serial number " << sn << " was found."<<std::endl);
    std::string pn = dev.get_info(RS2_CAMERA_INFO_PHYSICAL_PORT);
		std::string name = dev.get_info(RS2_CAMERA_INFO_NAME);
		ROS_INFO_STREAM("Device with physical ID " << pn << " was found.");
		std::vector<std::string> results;
		ROS_INFO_STREAM("Device with name " << name << " was found.");

  }

  while (ros::ok())
  {
		rs2::frameset frames;
    cv_bridge::CvImage cv_img;
    std_msgs::Header header;
      for(int i = 0; i <ctx.query_devices().size(); ++i)
      {
        frames= pipes[i].wait_for_frames();
        rs2::align align_to_color(RS2_STREAM_COLOR);
        frames = align_to_color.process(frames);

        // rs2::video_frame color_frame = frames.get_color_frame();
        // rs2::video_frame depth_frame = frames.get_depth_frame();
        rs2::video_frame ir_frame_left = frames.get_infrared_frame(1);
		    rs2::video_frame ir_frame_right = frames.get_infrared_frame(2);

        sensor_msgs::Imu imu_data = Get_imu_data(frames, i);

        // cv::Mat color_img(cv::Size(rgb_width, rgb_height), CV_8UC3, (void*)color_frame.get_data(),cv:: Mat::AUTO_STEP);
        // cv::Mat depth_img(cv::Size(depth_width, depth_height), CV_16U, (void*)depth_frame.get_data(),cv:: Mat::AUTO_STEP);
        cv::Mat ir1_img(cv::Size(rgb_width, rgb_height), CV_8UC1, (void*)ir_frame_left.get_data());
        cv::Mat ir2_img(cv::Size(rgb_width, rgb_height), CV_8UC1, (void*)ir_frame_right.get_data());


		  // cv::imshow("Display Image", color_img);
      // cv::waitKey(1);

      //   header.frame_id = "depth_frame" + std::to_string(i);
      //   cv_img.header.stamp.nsec = ros::Time::now().toSec();
      //   cv_img.header = header;
      //   cv_img.image = depth_img;
      //   cv_img.encoding = "mono16";
      //   depth_pubs[i].publish(cv_img.toImageMsg());

      //   header.frame_id = "color_frame" + std::to_string(i);
      //  cv_img.header.stamp.nsec = ros::Time::now().toSec();
      //   cv_img.header = header;
      //   cv_img.image = color_img;
      //   cv_img.encoding = "bgr8";
      //   rgb_pubs[i].publish(cv_img.toImageMsg());

        header.frame_id = "ir1_frame" + std::to_string(i);
        header.stamp = ros::Time::now();
        cv_img.header = header;
        cv_img.image = ir1_img;
        cv_img.encoding = "mono8";
        ir1_pubs[i].publish(cv_img.toImageMsg());

        header.frame_id = "ir2_frame" + std::to_string(i);
        header.stamp = ros::Time::now();
        cv_img.header = header;
        cv_img.image = ir2_img;
        cv_img.encoding = "mono8";
        ir2_pubs[i].publish(cv_img.toImageMsg());

        imu_pubs[i].publish(imu_data);
      }

  }


}