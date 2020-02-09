#include "Navigator.hpp"


bool Navigator::lineCheckObstacle(Point2f checkPoint)
{
	Point2f start = Point2f(robotPose->x,robotPose->y);
	const int x0 = start.x;
	const int y0 = start.y;
	const int x1 = checkPoint.x;
	const int y1 = checkPoint.y;

	if(abs(y1 - y0) < abs(x1 - x0))
	{
		if(x0 > x1)
		{
			return checkLineLow(x1, y1, x0, y0);
		} else
		{
			return checkLineLow(x0, y0, x1, y1);
		}
	} else
	{
		if(y0 > y1)
		{
			return checkLineHigh(x1, y1, x0, y0);
		}
		else
		{
			return checkLineHigh(x0, y0, x1, y1);
		}
	}
}

bool Navigator::checkLineHigh(int x0, int y0, int x1, int y1)
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
		if(checkPixel(x, y)) {return true;}
		if (D > 0)
		{
			x = x + xi;
			D = D - 2 * dy;
		}
		D = D + 2 * dx;
	}
	return false;
}

bool checkLineLow(int x0, int y0, int x1, int y1)
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
		if(checkPixel(x, y)) {return true;}
		if(D > 0)
		{
			y = y + yi;
			D = D - 2 * dx;
		}
		D = D + 2 * dy;
	}
	return false;
}

bool checkPixel(int y, int x) //Flip it because of dimensions
{
	auto pixel = map.at<uchar>(x-1, y-1);
	return pixel > 1;
}

geometry_msgs::msg::Twist Explorer::nextMove()
{
	geometry_msgs::msg::Twist movement;
	return movement;
}

geometry_msgs::msg::Twist RandomExplorer::nextMove()
{
	const int checkRange = 20;
	Point2f endPoint(robotPose->x + checkRange * cos(robotPose->heading), robotPose->y + checkRange * sin(robotPose->heading));
	bool obstructed = lineCheckObstacle(endPoint);
	geometry_msgs::msg::Twist movement;
	movement.angular.z = 0.0;
	movement.linear.x = 0.0;
	if(obstructed)
	{
		std::mt19937 gen(rd());
		std::uniform_real_distribution<float> dis(0.0, 1.0);
		auto lrChoice = dis(gen);
		if(lrChoice <= 0.5)
		{
			movement.angular.z = -0.25;
		} else
		{
			movement.angular.z = 0.25;
		}
	} else
	{
		movement.linear.x = 0.25;
	}
	return movement;
}

geometry_msgs::msg::Twist Searcher::nextMove()
{
	geometry_msgs::msg::Twist movement;
	return movement;
}