#include "Navigator.hpp"

#include <iostream>
#include <queue>
#include <set>
#include <unordered_map>
#include <memory>
#include <algorithm>
#include <math.h>

using namespace std;

vector<util::Point> getAndReducePath(shared_ptr<Node> solvedGoal)
{
	cout << "We have a path!" << endl;
	vector<util::Point> precisePath;
	auto n = solvedGoal;
	while(n->parent)
	{
		precisePath.push_back(n->loc);
		n = n->parent;
	}

	vector<util::Point> waypointPath;
	waypointPath.push_back(solvedGoal->loc);
	int wayPointRef = 0;
	util::Point diff;
	int splitCount = 1;
	cout << "Precise path length: " << precisePath.size() << endl;
	for(auto it = precisePath.rbegin()+1; it != precisePath.rend(); it++)
	{
		const auto change = waypointPath[wayPointRef] - *it;
		if(diff == util::Point())
		{
			diff == change;
			splitCount++;
		} else 	if(change == (diff * splitCount))
		{
			splitCount++;
		} else
		{
			waypointPath.push_back(*it);
			diff = util::Point();
			wayPointRef++;
			splitCount = 1;
		}
	}
	//Don't need to prepend where we are.
	//waypointPath.push_back(precisePath.back());

	reverse(waypointPath.begin(),waypointPath.end());
	return waypointPath;
}

vector<util::Point> Explorer::getNeighbours(util::Point p)
{
	vector<util::Point> neighbours;
	for(int x = -1; x <= 1; x++)
	{
		for(int y = -1; y<=1; y++)
		{
			if(x == 0 && y == 0) { continue; }
			int newX = p.x + x,newY = p.y + y;
			if((newX < minX) || (newX > maxX) || (newY < minY) || (newY > maxY))
			{
				continue;
			} //Skip neighbours that would be out of bounds
			neighbours.push_back(util::Point(newX,newY));
		}
	}
	return neighbours;
}

vector<util::Point> Explorer::pathTo(util::Point to)
{
	auto goal = make_shared<Node>(to, nullptr,0);
	auto frontier = priority_queue<shared_ptr<Node>>();
	auto start = make_shared<Node>(robotPose->loc, nullptr,0);
	start->calculateF(*goal,navMap);
	frontier.push(move(start));

	map<util::Point,shared_ptr<Node>> KnownNodes;
	cout << "About to path to: " << to.toString() << endl;
	while(!frontier.empty() && frontier.size() < 50000)
	{
		auto current = (frontier.top());
		frontier.pop();
		KnownNodes[current->loc] = current;

		if(current->loc == goal->loc)
		{return getAndReducePath(current);}

		auto neighbours = getNeighbours(current->loc);
		for(auto it = neighbours.begin(); it != neighbours.end(); it++)
		{
			auto newNode = make_shared<Node>(*it,current,current->gCost + 1);
			if(KnownNodes.count(*it) == 0)
			{
				newNode->calculateF(*goal,navMap);
				frontier.push(newNode);
			} else if(newNode->gCost < KnownNodes[*it]->gCost)
			{
				KnownNodes[*it] = newNode;
			}

		}
		//cout << "Still pathing: " << frontier.size() << " nodes left" << endl;
	}
	cout << "No path found" << endl;
	return vector<util::Point>();
}

//Max Linear = 0.22
//Max Angular = 2.84
geometry_msgs::msg::Twist Explorer::goTo(util::Point loc)
{
	geometry_msgs::msg::Twist move;
	const float relAngle = atan((float)loc.x/(float)loc.y) - robotPose->heading;
	if(abs(relAngle) < 0.35) //About 20 degrees
	{
		move.linear.x = 0.01;
	}
	move.angular.z = relAngle > 0 ? -0.05 : 0.05;
	return move;
}

util::Point Node::convertToMap(Map* m) const
{
	const auto centre = m->getMapCentre();
	return util::Point(loc.x + centre.x,loc.y + centre.y);
}


util::Point Node::convertToMap(Map* m, util::Point pathLoc)
{
	const auto centre = m->getMapCentre();
	return util::Point(pathLoc.x + centre.x,pathLoc.y + centre.y);
}

util::Point Node::convertToPath(Map* m, util::Point mapLoc)
{
	const auto centre = m->getMapCentre();
	return util::Point(mapLoc.x - centre.x,mapLoc.y - centre.y);
}

bool Navigator::lineCheckObstacle(cv::Point2f checkPoint)
{
	cv::Point2f start = cv::Point2f(navMap->getMapCentre().x + robotPose->loc.x,navMap->getMapCentre().y + robotPose->loc.y);
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
		if(checkPixel(x, y)) {
			cout << "obstacle at: " << x << "," << y << endl;
			return true;
		}
		if (D > 0)
		{
			x = x + xi;
			D = D - 2 * dy;
		}
		D = D + 2 * dx;
	}
	return false;
}

bool Navigator::checkLineLow(int x0, int y0, int x1, int y1)
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
		if(checkPixel(x, y)) {
                        cout << "obstacle at: " << x << "," << y << endl;
                        return true;
                }
		if(D > 0)
		{
			y = y + yi;
			D = D - 2 * dx;
		}
		D = D + 2 * dy;
	}
	return false;
}

bool Navigator::checkPixel(int y, int x) //Flip it because of dimensions
{
	auto pixel = navMap->map.at<uchar>(x-1, y-1);
	return pixel > 1;
}

geometry_msgs::msg::Twist Explorer::nextMove()
{
	if(robotPose->loc.x < minX || robotPose->loc.y < minY || robotPose->loc.x > maxX || robotPose->loc.y > maxY)
	{
		util::Point centre((maxX+minX)/2,(maxY+minY)/2);
		cout << "Out of bounds, pathing to: " << centre.toString() << endl;
		auto nextWayPoint = pathTo(centre).front();
		return goTo(nextWayPoint);
	}

	//If we're in bounds, lets path to a random point inside
	std::mt19937 gen(rd());
	std::uniform_int_distribution<> disX(minX, maxX);
	std::uniform_int_distribution<> disY(minY, maxY);

	util::Point newLoc(disX(gen),disY(gen)); //Pick a random point in bounds
	auto nextWayPoint = pathTo(newLoc).front();
	return goTo(nextWayPoint);
}

geometry_msgs::msg::Twist RandomExplorer::nextMove()
{
	const int checkRange = 7;
	cout << "Robot at: (" << (navMap->getMapCentre().x +  robotPose->loc.x) << "," << (navMap->getMapCentre().y +  robotPose->loc.y) << ") on map" << endl;
	cv::Point2f endPoint(navMap->getMapCentre().x + robotPose->loc.x + checkRange * cos(robotPose->heading),navMap->getMapCentre().y + robotPose->loc.y + checkRange * sin(robotPose->heading));
	cv::Point2f endPointH(navMap->getMapCentre().x + robotPose->loc.x + checkRange * cos(robotPose->heading+0.1),navMap->getMapCentre().y + robotPose->loc.y + checkRange * sin(robotPose->heading+0.1));
	cv::Point2f endPointL(navMap->getMapCentre().x + robotPose->loc.x + checkRange * cos(robotPose->heading-0.1),navMap->getMapCentre().y + robotPose->loc.y + checkRange * sin(robotPose->heading-0.1));
	bool obstructed = lineCheckObstacle(endPoint) || lineCheckObstacle(endPointH) || lineCheckObstacle(endPointL);
	geometry_msgs::msg::Twist movement;
	movement.angular.z = 0.0;
	movement.linear.x = 0.0;
	//obstructed = false;

	if(obstructed)
	{
		if(lastMove.angular.z == 0 || lastMove.linear.x > 0)
		{
			cout << "Obstacle, avoiding" << endl;
			std::mt19937 gen(rd());
			std::uniform_real_distribution<float> dis(0.0, 1.0);
			auto lrChoice = dis(gen);
			if(lrChoice <= 0.5)
			{
				movement.angular.z = -0.1;
			} else
			{
				movement.angular.z = 0.1;
			}
			lastMove = movement;
			return movement;
		}
		return lastMove;
	}

	if(robotPose->loc.x < minX || robotPose->loc.y < minY || robotPose->loc.x > maxX || robotPose->loc.y > maxY)
	{
		util::Point centre((maxX+minX)/2,(maxY+minY)/2);
		cout << "Out of bounds, pointing to: " << centre.toString() << endl;
		movement = lastMove = goTo(centre);
		return movement;
	}

	movement.linear.x = 0.01;
	lastMove = movement;
	return movement;
}

geometry_msgs::msg::Twist Searcher::nextMove()
{
	const util::Point goal(100,100);
	auto nextWayPoint = pathTo(goal).front();
	geometry_msgs::msg::Twist movement = goTo(nextWayPoint);
	return movement;
}
