#include "Map.h"

#include <opencv2/core.hpp>
#include <Statistics.h>

using namespace cv;

Map::Map(int viewDistance)
{
	int size = viewDistance % 2 == 1 ? viewDistance : viewDistance + 1;
	map = Mat(size, size, CV_8UC1, Scalar(0, 0, 0));
}

void Map::increaseSize(int additional)
{
	copyMakeBorder(map, map, additional, additional, additional, additional, BORDER_CONSTANT, Scalar::all(0));
}

void Map::addMeasurement(Pose currentPose, float depth, Size2f& maxMinAngles)
{
	Point2f origin = getMapCentre();
	origin.x += currentPose.x;
	origin.y += currentPose.y;
	const int distance = cvRound(depth);

	while(origin.x+distance > map.cols || origin.x - distance < 0 || origin.y + distance > map.rows || origin.y - distance < 0)
	{
		increaseSize(distance);
		//Reselect origin of drawing
		origin = getMapCentre();
		origin.x += currentPose.x;
		origin.y += currentPose.y;
	}
	
	const int x0 = origin.x + (depth * cos(currentPose.heading + maxMinAngles.width));
	const int y0 = origin.y + (depth * sin(currentPose.heading + maxMinAngles.width));
	const int x1 = origin.x + (depth * cos(currentPose.heading + maxMinAngles.height));
	const int y1 = origin.y + (depth * sin(currentPose.heading + maxMinAngles.height));

	drawIncrementingLine(x0, y0, x1, y1);
}

cv::Mat Map::getDisplayMap(Pose robotPose, float scale) const
{
	const Size displaySize = Size(map.cols * scale, map.rows * scale);
	Mat displayMap;
	resize(map, displayMap, displaySize);
	cvtColor(displayMap, displayMap, COLOR_GRAY2RGB);
	Point2f startPoint = getMapCentre();
	startPoint.x += robotPose.x;
	startPoint.y += robotPose.y;
	startPoint *= scale;
	Point2f endPoint(startPoint.x + scale * cos(robotPose.heading), startPoint.y + scale * sin(robotPose.heading));
	circle(displayMap, startPoint, scale, Scalar(255, 0, 0));
	line(displayMap, startPoint, endPoint, Scalar(255, 0, 0));
	return displayMap;
}

void Map::incrementPixel(int y, int x) //Flip it because of dimensions
{
	auto pixel = map.at<uchar>(x-1, y-1);
	if(pixel < 255)
	{
		pixel = (pixel + 1);
		map.at<uchar>(x-1, y-1) = pixel;
	}
}

void Map::drawLineLow(int x0, int y0, int x1, int y1)
{
	int dx = x1 - x0;
	int dy = y1 - y0;
	int yi = 1;

	if(dy < 0)
	{
		yi = -1;
		dy = -dy;
	}

	int D = 2 * dy - dx;
	int y = y0;

	for(int x = x0; x <= x1; ++x)
	{
		incrementPixel(x, y);
		if(D > 0)
		{
			y = y + yi;
			D = D - 2 * dx;
		}
		D = D + 2 * dy;
	}
}

void Map::drawLineHigh(int x0, int y0, int x1, int y1)
{
	int dx = x1 - x0;
	int dy = y1 - y0;
	int xi = 1;

	if (dx < 0)
	{
		xi = -1;
		dx = -dx;
	}

	int D = 2 * dx - dy;
	int x = x0;

	for (int y = y0; y <= y1; ++y)
	{
		incrementPixel(x, y);
		if (D > 0)
		{
			x = x + xi;
			D = D - 2 * dy;
		}
		D = D + 2 * dx;
	}
}

void Map::drawIncrementingLine(int x0, int y0, int x1, int y1)
{
	if(abs(y1 - y0) < abs(x1 - x0))
	{
		if(x0 > x1)
		{
			drawLineLow(x1, y1, x0, y0);
		} else
		{
			drawLineLow(x0, y0, x1, y1);
		}
	} else
	{
		if(y0 > y1)
		{
			drawLineHigh(x1, y1, x0, y0);
		}
		else
		{
			drawLineHigh(x0, y0, x1, y1);
		}
	}
}

cv::Point2f Map::getMapCentre() const
{
	return Point2f((map.cols / 2) + 1, (map.rows / 2) + 1);
}

float Pose::rotateDeg(float angle)
{
	return heading = radBound(heading + degToRad(angle));
}

std::string Pose::toString() const
{
	return "(" + to_string(x) + "," + to_string(y) + "," + to_string(heading) + ")";
}
