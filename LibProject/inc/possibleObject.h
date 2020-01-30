#pragma once

#include <opencv2/core.hpp>     // Basic OpenCV structures (cv::Mat, Scalar)

using namespace cv;
class possibleObject
{	
public:
	Rect bounds;
	float avgVal;
	Size2f estimatedSize;
	float estimatedDistance;
	Size2f maxMinAngles;
	
	static const float maxRange;

	possibleObject(Rect _b) : bounds(_b), avgVal(0),estimatedDistance(0),estimatedSize(0,0),maxMinAngles(0,0) {}

	float computeAvgVal(Mat img);
	float computeDistance(Size2f objSize, Size2f borderSize, Size2f FOVrad, float focalLength = 0.00367); //Units in metres

	bool overlap(const possibleObject& other) const;

	bool operator<(const possibleObject& other) const;
	bool operator==(const possibleObject& other) const;
};