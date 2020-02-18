#pragma once

#include <opencv2/opencv.hpp>

class Point
{
 public:
	int x,y;

	Point(): x(0),y(0) {}
	Point(int _x,int_y): x(_x),y(_y) {}

	inline bool operator ==(const Point& lhs, const Point& rhs)
	{
		return lhs.x == rhs.x && lhs.y == rhs.y;
	}

	bool operator <(const Point& rhs) const
	{
		return this->toString() < rhs.toString();
	}

	Point operator-(const Point& rhs) const
	{
		return Point(x - rhs.x,y - rhs.y);
	}

	Point operator+(const Point& rhs) const
	{
		return Point(x + rhs.x,y + rhs.y);
	}

	Point operator*(const int& rhs) const
	{
		return Point(x * rhs,y * rhs);
	}

	std::string toString() const;
};

class Pose
{
public:
	Point loc;
	float heading;

	Pose() : heading(0) {}
	Pose(int _x, int _y, float _heading) : loc(_x,_y), heading(_heading) {}

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
	cv::Point2f getMapCentre() const;

private:
	void incrementPixel(int x, int y);
	void drawLineLow(int x0, int y0, int x1, int y1);
	void drawLineHigh(int x0, int y0, int x1, int y1);
	void drawIncrementingLine(int x0, int y0, int x1, int y1);


};