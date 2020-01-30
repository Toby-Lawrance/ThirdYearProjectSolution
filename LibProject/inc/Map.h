#pragma once

#include <opencv2/opencv.hpp>

class Pose
{
public:
	int x;
	int y;
	float heading;

	Pose() : x(0), y(0), heading(0) {}
	Pose(int _x, int _y, float _heading) : x(_x), y(_y), heading(_heading) {}

	float rotateDeg(float angle);
	std::string toString() const;
};

/*
 * The map is represented by a matrix in greyscale.
 * From the robot pose, we can update our belief in the map such that 0 means a belief of nothing or no information, with steadily increasing values showing greater belief.
 */
class Map
{
public:
	cv::Mat map;

	Map(int viewDistance = 30);

	void increaseSize(int additional);
	
	void addMeasurement(Pose currentPose, float depth, cv::Size2f& maxMinAngles);

	cv::Mat getDisplayMap(Pose robotPose, float scale = 10) const;

private:
	void incrementPixel(int x, int y);
	void drawLineLow(int x0, int y0, int x1, int y1);
	void drawLineHigh(int x0, int y0, int x1, int y1);
	void drawIncrementingLine(int x0, int y0, int x1, int y1);

	cv::Point2f getMapCentre() const;
};