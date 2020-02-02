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
using std::placeholders::_1;
using namespace cv;
using namespace chrono;

 class ProcessorNavigator : public rclcpp::Node
 {
  public:
	 int graph = 0; //Set to 1 for graph drawing on the videoOutput

	 rclcpp::TimerBase::SharedPtr timer;
	 rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr movePub;
	 rclcpp::Subscriber<nav_msgs::msg::Odometry>::SharedPtr odomSub;

	 geometry_msgs::msg::Twist movementState;
	 nav_msgs::msg::Odometry odometryState;

	 Pose currentRobotPose;
	 ObjectDetector od;

	 VideoCapture cap;
	 VideoWriter videoWriter;
	 VideoWriter mapWriter;

	 ProcessorNavigator() : Node("processor_navigator")
	 {
 		cap = VideoCapture(CAP_DSHOW);
		 Mat base;
		 if (!Initialise_Camera(cap, videoWriter, base))
		 {
			 cin.get();
			 return -1;
		 }
		 od = ObjectDetector(base);
		mapWriter = VideoWriter("map.avi",VideoWriter::fourcc('M', 'J', 'P', 'G'),videoWriter.get(CAP_PROP_FPS),Size(1000,1000),true);
		odomSub = this->create_subscription<nav_msgs::msg::Odometry>("odom",10,bind(&ProcessorNavigator::odom_callback, this, 1))

		 timer = this->create_wall_timer(50ms, bind(&ProcessorNavigator::processImg,this));
	 }

	 ~ProcessorNavigator()
	 {
		 imwrite("Map.bmp", od.detectionMap.map);
	 }

	 void odom_callback(const nav_msgs::msgs::Odometry::SharedPtr msg) const
	 {
		auto point = msg->pose->pose->position;
		auto quat = msg->pose->pose->orientation;

		currentRobotPose.x = (int)(point.x/100.0);
		currentRobotPose.y = (int)(point.y/100.0);
		currentRobotPose.heading = quat.z;
	 }

	 void processImg()
	 {
		 Mat frame;
		 bool readSuccess = cap.read(frame);

		 if (!readSuccess)
		 {
			 cout << "Stream ended" << endl;
			 break;
		 }

		 if (waitKey(1) == 27) break;
		 bool draw = graph == 1;

		 od.processFrame(frame,currentRobotPose,draw);

		 if(draw)
		 {
			 frame += od.infoGraph.drawing;
		 }

		 videoWriter.write(frame);
		 float scaling = 1000.0 / od.detectionMap.map.rows;
		 Mat displayMap = od.detectionMap.getDisplayMap(defaultPose,scaling);
		 mapWriter.write(displayMap);
	 }
 };

int main(int argc, char* argv[])
{
	rclcpp::init(argc,argv);
	rclcpp::spin(make_shared<ProcessorNavigator>());
	rclcpp::shutdown();
	return 0;
}
