#include "possibleObject.h"

#include <opencv2/core.hpp>     // Basic OpenCV structures (cv::Mat, Scalar)
#include <opencv2/imgproc.hpp>  // Gaussian Blur
#include <opencv2/highgui.hpp>  // OpenCV window I/O
#include <opencv2/features2d.hpp>

const float possibleObject::maxRange = 50.0f;

float possibleObject::computeAvgVal(Mat img)
{
	Mat mask = Mat(img.rows, img.cols, CV_8UC1, Scalar(0, 0, 0));
	rectangle(mask, bounds, Scalar::all(255), FILLED);
	auto avg = mean(img, mask);
	return avgVal = avg[0];
}

float possibleObject::computeDistance(Size2f objSize, Size2f borderSize, Size2f FOVrad,float focalLength)
{
	const int xMax = max(bounds.tl().x, bounds.br().x);
	const int yMax = max(bounds.tl().y, bounds.br().y);

	const int xMin = min(bounds.tl().x, bounds.br().x);
	const int yMin = min(bounds.tl().y, bounds.br().y);

	const float xLength = (xMax - xMin) > (yMax - yMin) ? objSize.height : objSize.width;
	const float yLength = (yMax - yMin) > (xMax - xMin) ? objSize.height : objSize.width;

	const float xMaxRelative = ((xMax - (borderSize.width / 2)) / (borderSize.width)) * FOVrad.width;
	const float xMinRelative = ((xMin - (borderSize.width / 2)) / (borderSize.width)) * FOVrad.width;

	maxMinAngles = Size2f(xMaxRelative, xMinRelative);
	const float xMaxAngle = abs(xMaxRelative);
	const float xMinAngle = abs(xMinRelative);
	
	const float yMaxAngle = (abs(yMax - (borderSize.height / 2)) / borderSize.height) * FOVrad.height;
	const float yMinAngle = (abs(yMin - (borderSize.height / 2)) / borderSize.height) * FOVrad.height;

	const float x1 = (xMinAngle / (xMinAngle + xMaxAngle)) * xLength;
	const float xTanMinAngle = tanf(xMinAngle);

	const float y1 = (yMinAngle / (yMinAngle + yMaxAngle)) * yLength;
	const float yTanMinAngle = tanf(yMinAngle);
	
	const float xDepth = abs((x1 - (focalLength * xTanMinAngle)) / xTanMinAngle);
	const float yDepth = abs((y1 - (focalLength * yTanMinAngle)) / yTanMinAngle);
	
	bool close = abs(xDepth - yDepth) / yDepth < 0.1;
	if(close)
	{
		estimatedDistance = (xDepth + yDepth) / 2.0;
		if (estimatedDistance > maxRange) { return -1; } //Out of range
		return estimatedDistance;
	}
	const bool xBordered = xMax == borderSize.width || xMin == 0;
	const bool yBordered = yMax == borderSize.height || yMin == 0;

	if(xBordered || yBordered)
	{
		if (estimatedDistance > maxRange) { return -1; } //Out of range
		return estimatedDistance;
	}

	//Then one of them probably went wrong, pick the bigger one
	estimatedDistance = max(xDepth, yDepth);
	if (estimatedDistance > maxRange) { return -1; } //Out of range
	return estimatedDistance;
}



bool possibleObject::overlap(const possibleObject& other) const
{
	Point l1 = Point(min(bounds.tl().x, bounds.br().x), max(bounds.tl().y, bounds.br().y));
	Point r1 = Point(max(bounds.tl().x, bounds.br().x), min(bounds.tl().y, bounds.br().y));

	Point l2 = Point(min(other.bounds.tl().x, other.bounds.br().x), max(other.bounds.tl().y, other.bounds.br().y));
	Point r2 = Point(max(other.bounds.tl().x, other.bounds.br().x), min(other.bounds.tl().y, other.bounds.br().y));


	// If one rectangle is on left side of other 
	if (l1.x > r2.x || l2.x > r1.x)
	{
		return false;
	}


	// If one rectangle is above other 
	if (l1.y < r2.y || l2.y < r1.y)
	{
		return false;
	}


	return true;
}

bool possibleObject::operator<(const possibleObject& other) const
{
	if (avgVal == other.avgVal)
	{
		return bounds.area() < other.bounds.area();
	}

	return avgVal < other.avgVal;
}

bool possibleObject::operator==(const possibleObject& other) const
{
	if (bounds != other.bounds)
	{
		return false;
	}

	if (avgVal != other.avgVal)
	{
		return false;
	}

	return true;
}



