#include "Navigator.hpp"

#include <set>
#include <unordered_map>
#include <memory>
#include <algorithm>
#include <math.h>

using namespace std;

vector<Point> getAndReducePath(Node* solvedGoal)
{
	vector<Point> precisePath;
	Node* n = solvedGoal;
	while(n->parent != nullptr)
	{
		precisePath.push_back(n->loc);
		n = n->parent;
	}

	vector<Point> waypointPath;
	waypointPath.push_back(solvedGoal->loc);
	int wayPointRef = 0;
	Point diff;
	int splitCount = 1;
	for(auto it = precisePath.rbegin()+1; it != precisePath.rend(); it++)
	{
		const auto change = waypointPath[waypointRef] - *it;
		if(diff == Point())
		{
			diff == change;
			splitCount++;
		} else 	if(change == diff * splitCount)
		{
			splitCount++;
		} else
		{
			waypointPath.push_back(*it);
			diff = Point();
			wayPointRef++;
			splitCount = 1;
		}
	}
	//Don't need to prepend where we are.
	//waypointPath.push_back(precisePath.back());

	reverse(waypointPath.begin(),waypointPath.end());
	return waypointPath;
}

vector<Point> getNeighbours(Point p)
{
	vector<Point> neighbours;
	for(int x = -1; x <= 1; x++)
	{
		for(int y = -1; y<=1; y++)
		{
			if(x == 0 && y == 0) { continue; }

			neighbours.push_back(Point(x,y));
		}
	}
	return neighbours;
}

vector<Point> Navigator::pathTo(Point to)
{
	auto goal = make_shared<Node>(to, nullptr,0);
	auto frontier = set<shared_ptr<Node>>();
	auto start = make_shared<Node>(robotPose->loc, nullptr,0);
	start->calculateF(*goal,navMap);
	frontier.insert(move(start));

	map<Point,shared_ptr<Node>> KnownNodes;

	while(!fronter.empty())
	{
		auto current = *(frontier.begin());
		KnownNodes[current->loc] = current;

		if(current == goal)
		{return getAndReducePath(current.get());}

		auto neighbours = getNeighbours(current.loc);
		for(auto it = neighbours.begin(); it != neighbours.end(); it++)
		{
			Node newNode(*it,current.get(),current.gCost + 1);
			if(KnownNodes.count(*it) == 0)
			{
				newNode.calculateF(goal,navMap);
				frontier.insert(newNode);
			} else if(newNode.gCost < KnownNodes[*it].gCost)
			{
				KnownNodes[*it] = newNode;
			}

		}
	}
}

//Max Linear = 0.22
//Max Angular = 2.84
geometry_msgs::msg::Twist Navigator::goTo(Point loc)
{
	geometry_msgs::msg::Twist move;
	const float relAngle = atan((float)loc.x/(float)loc.y) - robotPose->heading;
	move.angular.z = relAngle > 0 ? min(2.84, relAngle*1.42) : max(-2.84,relAngle*1.42);
	const float manhattandistance = max(abs(loc.x - robotPose->loc.x), abs(loc.y - robotPose->loc.y));
	move.linear.x = min(0.22,manhattandistance/5.0);
	return move;
}

Point Node::convertToMap(Map* m) const
{
	const auto centre = m->getMapCentre();
	return Point(loc.x + centre.x,loc.y + centre.y);
}


Point Node::convertToMap(Map* m, Point pathLoc) const
{
	const auto centre = m->getMapCentre();
	return Point(pathLoc.x + centre.x,pathLoc.y + centre.y);
}

Point Node::convertToPath(Map* m, Point mapLoc) const
{
	const auto centre = m->getMapCentre();
	return Point(mapLoc.x - centre.x,mapLoc.y - centre.y);
}

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
		movement.linear.x = 0.15;
	}
	return movement;
}

geometry_msgs::msg::Twist Searcher::nextMove()
{
	const Point goal(100,100);
	auto nextWayPoint = pathTo(goal).front();
	geometry_msgs::msg::Twist movement = goTo(nextWayPoint);
	return movement;
}