#pragma once

#include <Map.h>
#include <nav_msgs/msg/odometry.hpp>
#include <random>

class Node
{
 public:
	Point loc;
	Node* parent;
	float gCost,hCost,fCost;

	Node(Point s, Node* p, float path) : loc(s),parent(p),gCost(path) {}

	inline bool operator <(const Node& lhs, const Node& rhs)
	{
		return lhs.fCost < rhs.fCost;
	}

	inline bool operator ==(const Node& lhs, const Node& rhs)
	{
		return lhs.loc == rhs.loc;
	}

	static float calculateH(Point loc, Node dest, Map* m)
	{
		const float dist = (sqrt((loc.x - dest.loc.x)*(loc.x - dest.loc.x)
					 + (loc.y - dest.loc.y)*(loc.y - dest.loc.y)));
		const auto mapPoint = convertToMap(m,loc);
		const auto val = map->at<uchar>(mapPoint.y-1, mapPoint.x-1);

		return val > 0 ? dist * val : dist;
	}

	float calculateH(Node dest, Map* m)
	{
		const float dist = (sqrt((loc.x - dest.loc.x)*(loc.x - dest.loc.x)
								 + (loc.y - dest.loc.y)*(loc.y - dest.loc.y)));
		const auto mapPoint = convertToMap(m,loc);
		const auto val = map->at<uchar>(mapPoint.y-1, mapPoint.x-1);

		return hCost = val > 0 ? dist * val : dist;
	}

	float calculateF(Node dest, Map* m)
	{
		return fCost = gCost + this->calculateH(dest,m);
	}

	Point convertToMap(Map* m) const;
	static Point convertToMap(Map* m, Point pathLoc) const;
	static Point convertToPath(Map* m, Point mapLoc) const;
};

class Navigator
{
public:
	Map* navMap;
	Pose* robotPose;
	std::random_device rd;

	 bool lineCheckObstacle(Point2f checkPoint);
	 virtual geometry_msgs::msg::Twist nextMove() = 0;

	 vector<Point> pathTo(Point to);

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