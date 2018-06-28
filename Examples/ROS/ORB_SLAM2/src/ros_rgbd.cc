/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <std_msgs/Bool.h>

#include<opencv2/core/core.hpp>


#include"../../../include/System.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    ORB_SLAM2::System* mpSLAM;
    cv::Mat pose;
};

std::unique_ptr<tf::TransformBroadcaster> br;
ros::Publisher valid_tracking_pub;
bool valid_tracking;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "camera/depth_registered/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));

    valid_tracking_pub = nh.advertise<std_msgs::Bool>("orbslam2/tracking_active", 5, true);

    bool on_robot;
    nh.getParam("on_robot", on_robot);

    br = std::make_unique<tf::TransformBroadcaster>();
    tf::TransformListener listener;
    tf::StampedTransform slam_zero_world;

    if (on_robot) {
      tf::StampedTransform transform;
      try {
	listener.lookupTransform("/head_xtion", "/map", ros::Time(0), transform);
      } catch (tf::TransformException ex) {
	ROS_ERROR("%s", ex.what());
	ros::Duration(1.0).sleep();
      }
    }

    // transform from the initial position of the slam camera to the world
    // coordinates
    ros::Rate loop(100);
    while (ros::ok()) {

      if (on_robot) {
	br->sendTransform(tf::StampedTransform(slam_zero_world, ros::Time::now(), "map", "orbslam_base"));
      }

      ros::spinOnce();
      loop.sleep();
    }

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    pose = mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());

    // to create correct transform, need to create a static frame for the starting position of the sensor (taken from the initial position of head_xtion), and then have another transform which gives the transform from the camera localised position to that frame
    
    if (!pose.empty()) {
      if (!valid_tracking) {
	valid_tracking = true;
	valid_tracking_pub.publish(valid_tracking);
      }

      std::cout << "rotation: \n" << pose.rowRange(0,3).colRange(0,3) << std::endl;
      // this is for some reason in 10s of metres? e.g. 0.1 in any direction is 1m
      std::cout << "translation: \n" << pose.rowRange(0,3).col(3) << std::endl;


      cv::Mat r = pose.rowRange(0,3).colRange(0,3).t(); //rotation
      cv::Mat t = -r*pose.rowRange(0,3).col(3); // translation
      
      tf::Vector3 tr(t.at<float>(0)*10, t.at<float>(1)*10, t.at<float>(2)*10);
      tf::Matrix3x3 rot(r.at<float>(0,0), r.at<float>(0,1), r.at<float>(0,2),
			r.at<float>(1,0), r.at<float>(1,1), r.at<float>(1,2),
			r.at<float>(2,0), r.at<float>(2,1),r.at<float>(2,2));

      tf::Transform orb_base_to_pose(rot, tr);


      tfScalar roll, pitch, yaw;

      // For whatever reason the rotation matrix from orbslam gives odd ordering
      // for RPY
      rot.getRPY(pitch, yaw, roll);

      ROS_INFO("roll: %f, pitch: %f, yaw: %f", roll, pitch, yaw);
      tf::Quaternion rq;
      rq.setRPY(roll, pitch, yaw);
      //orb_base_to_pose.setRotation(rq);
      
      br->sendTransform(tf::StampedTransform(orb_base_to_pose, ros::Time::now(), "orbslam_base", "orbslam_pose"));
    } else {
      if (valid_tracking) {
	valid_tracking = false;
	valid_tracking_pub.publish(valid_tracking);
      }
    }
}
