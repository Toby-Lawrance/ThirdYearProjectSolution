#pragma once

#include <Map.h>
#include <nav_msgs/msg/odometry.hpp>
#include <random>
#include <vector>
#include <string>
#include <memory>

class Node
{
 public:
	util::Point loc;
	std::shared_ptr<Node> parent;
	float gCost,hCost,fCost;

	Node(util::Point s, std::shared_ptr<Node> p, float path) : loc(s),parent(std::move(p)),gCost(path) {}

	inline bool operator <(const Node& rhs)
	{
		return fCost < rhs.fCost;
	}

	inline bool operator ==(const Node& rhs)
	{
		return loc == rhs.loc;
	}

	static float calculateH(util::Point loc, Node dest, Map* m)
	{
		const float dist = (sqrt((loc.x - dest.loc.x)*(loc.x - dest.loc.x)
					 + (loc.y - dest.loc.y)*(loc.y - dest.loc.y)));
		const auto mapPoint = convertToMap(m,loc);
		const auto val = m->map.at<uchar>(mapPoint.y-1, mapPoint.x-1);

		return val > 0 ? dist * val : dist;
	}

	float calculateH(Node dest, Map* m)
	{
		const float dist = (sqrt((loc.x - dest.loc.x)*(loc.x - dest.loc.x)
								 + (loc.y - dest.loc.y)*(loc.y - dest.loc.y)));
		const auto mapPoint = convertToMap(m,loc);
		const auto val = m->map.at<uchar>(mapPoint.y-1, mapPoint.x-1);

		return hCost = val > 0 ? dist * val : dist;
	}

	float calculateF(Node dest, Map* m)
	{
		return fCost = gCost + this->calculateH(dest,m);
	}

	std::string toString()
	{
		return loc.toString() + ":" + std::to_string(fCost);
	}

	util::Point convertToMap(Map* m) const;
	static util::Point convertToMap(Map* m, util::Point pathLoc);
	static util::Point convertToPath(Map* m, util::Point mapLoc);
};

class Navigator
{
public:
	Map* navMap = nullptr;
	Pose* robotPose = nullptr;
	std::random_device rd;

	 bool lineCheckObstacle(cv::Point2f checkPoint);
	 virtual geometry_msgs::msg::Twist nextMove() = 0;


 private:
	bool checkPixel(int y, int x);
	bool checkLineLow(int x0, int y0, int x1, int y1);
	bool checkLineHigh(int x0, int y0, int x1, int y1);
};

class Explorer : public Navigator
{
public:
	int minX=0,maxX=100;
	int minY=0,maxY=100;

	std::vector<util::Point> pathTo(util::Point to);
	geometry_msgs::msg::Twist goTo(util::Point loc);
	std::vector<util::Point> getNeighbours(util::Point p);
	virtual geometry_msgs::msg::Twist nextMove();
};

class RandomExplorer : public Explorer
{
	geometry_msgs::msg::Twist lastMove;
public:
	virtual geometry_msgs::msg::Twist nextMove();
};

class Searcher : public Explorer
{
public:
	virtual geometry_msgs::msg::Twist nextMove();
};