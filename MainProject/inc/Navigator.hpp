#pragma once

#include <Map.h>
#include <nav_msgs/msg/odometry.hpp>
#include <random>

class Navigator
{
public:
	Map* navMap;
	Pose* robotPose;
	std::random_device rd;

	 bool lineCheckObstacle(Point2f checkPoint);
	 virtual geometry_msgs::msg::Twist nextMove() = 0;

 private:
	bool checkPixel(int y, int x);
	bool checkLineLow(int x0, int y0, int x1, int y1);
	bool checkLineHigh(int x0, int y0, int x1, int y1);
};

class Explorer : public Navigator
{
public:
	int minX,maxX;
	int minY,maxY;
		
	virtual override geometry_msgs::msg::Twist nextMove();
};

class RandomExplorer : public Explorer
{
public:
	virtual override geometry_msgs::msg::Twist nextMove();
};

class Searcher : public Navigator
{
public:
	virtual override geometry_msgs::msg::Twist nextMove();
};