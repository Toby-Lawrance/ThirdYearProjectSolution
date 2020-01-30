#pragma once
#include <string>
#include <opencv2/opencv.hpp>
#include <Statistics.h>

using namespace cv;
using namespace std;


const float HorizontalFOV = degToRad(70.42);
const float VerticalFOV = degToRad(43.3);
const float focalLength = 0.367; //cm

bool Initialise_Camera(VideoCapture& cap, VideoWriter& videoWriter, Mat& baseFrame, int width = 640, int height = 360, int fps = 30, std::string fileName = "tracking.avi");
