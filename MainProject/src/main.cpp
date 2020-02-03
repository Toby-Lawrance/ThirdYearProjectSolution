#include <iostream>
#include <chrono>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>     // Basic OpenCV structures (cv::Mat, Scalar)
#include <opencv2/highgui.hpp>  // OpenCV window I/O

#include <ObjectDetection.h>
#include <Camera.h>
#include <Statistics.h>
#include "Map.h"

#include"rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/odometry.hpp>

using namespace std;
using namespace cv;
using namespace chrono;

 class ProcessorNavigator : public rclcpp::Node
 {
  public:
	 int graph = 1; //Set to 1 for graph drawing on the videoOutput

	 rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr movePub;
	 rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub;

	 geometry_msgs::msg::Twist::SharedPtr movementState;
	 nav_msgs::msg::Odometry::SharedPtr odometryState;

	 Pose currentRobotPose;
	 std::unique_ptr<ObjectDetector> od;

	 VideoCapture cap;
	 VideoWriter videoWriter;
	 VideoWriter mapWriter;

	 ProcessorNavigator() : Node("processor_navigator")
	 {
 		cap = VideoCapture(0);
		 Mat baseFrame;
		 if (!Initialise_Camera(cap, videoWriter, baseFrame))
		 {
			 cin.get();
			 return;
		 }
		 od =  std::make_unique<ObjectDetector>(baseFrame);
		mapWriter = VideoWriter("map.avi",VideoWriter::fourcc('M', 'J', 'P', 'G'),5,Size(1000,1000),true);
		movePub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
		odomSub = this->create_subscription<nav_msgs::msg::Odometry>("odom",10, bind(&ProcessorNavigator::odom_callback, this, placeholders::_1));
	 }

	 ~ProcessorNavigator()
	 {
		 imwrite("Map.bmp", od->detectionMap.map);
	 }

	 void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
	 {
		auto point = msg->pose.pose.position;
		auto quat = msg->pose.pose.orientation;
		currentRobotPose.x = (int)(point.x*100.0);
		currentRobotPose.y = (int)(point.y*100.0);
		currentRobotPose.heading = QuatToRadYaw(quat.x, quat.y, quat.z, quat.w);
	 }

	 void processImg()
	 {
		 Mat frame;
		 bool readSuccess = cap.read(frame);

		 if (!readSuccess)
		 {
			 cout << "Stream ended" << endl;
			 exit(0);
		 }

		 bool draw = graph == 1;

		 od->processFrame(frame,currentRobotPose,draw);

		 if(draw)
		 {
			 frame += od->infoGraph.drawing;
		 }

		 videoWriter.write(frame);
		 float scaling = 1000.0 / od->detectionMap.map.rows;
		 Mat displayMap = od->detectionMap.getDisplayMap(currentRobotPose,scaling);
		 mapWriter.write(displayMap);
	 }
 };

int main(int argc, char* argv[])
{
	rclcpp::init(argc,argv);
	cout << "Init OpenCV: " << CV_VERSION << endl;
	auto node = make_shared<ProcessorNavigator>();
	while(rclcpp::ok())
	{
		node->processImg();
		rclcpp::spin_some(node);
	}
	rclcpp::shutdown();
	return 0;
}
