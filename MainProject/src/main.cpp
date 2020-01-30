#include <iostream>
#include <chrono>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>     // Basic OpenCV structures (cv::Mat, Scalar)
#include <opencv2/highgui.hpp>  // OpenCV window I/O

#include <ObjectDetection.h>
#include <Camera.h>
#include <Statistics.h>
#include "Map.h"

using namespace std;
using namespace cv;
using namespace chrono;


int main(int argc, char* argv[])
{
	VideoCapture cap(CAP_DSHOW);
	VideoWriter videoWriter;
	Mat base;
	if (!Initialise_Camera(cap, videoWriter, base))
	{
		cin.get();
		return -1;
	}

	/*CONTROLS*/
	namedWindow("Control", WINDOW_AUTOSIZE);

	int graph = 0;
	
	int x = 50 , y = 50;
	int poseAngle = 0;

	createTrackbar("Graph", "Control", &graph, 1);
	createTrackbar("X", "Control", &x, 100);
	createTrackbar("Y", "Control", &y, 100);
	createTrackbar("Angle", "Control", &poseAngle, 360);
	/*END CONTROLS*/

	ObjectDetector od(base);
	Pose defaultPose;
	
	while (true)
	{
		defaultPose.x = x - 50;
		defaultPose.y = y - 50;
		defaultPose.heading = degToRad(poseAngle);
		
		Mat frame;
		bool readSuccess = cap.read(frame);

		if (!readSuccess)
		{
			cout << "Stream ended" << endl;
			break;
		}
		
		if (waitKey(1) == 27) break;
		bool draw = graph == 1;
	
		od.processFrame(frame,defaultPose,draw);

		if(draw)
		{
			frame += od.infoGraph.drawing;
		}
		imshow("Source image", frame);
		float scaling = 1000.0 / od.detectionMap.map.rows;
		Mat displayMap = od.detectionMap.getDisplayMap(defaultPose,scaling);
		imshow("Map", displayMap);
	}

	imwrite("Map.bmp", od.detectionMap.map);
	return 0;
}
