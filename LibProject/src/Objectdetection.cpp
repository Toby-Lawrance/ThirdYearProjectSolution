#include "ObjectDetection.h"
#include "imgProcessing.h"
#include "Statistics.h"
#include "Camera.h"

#include <iostream>

using namespace cv;
using namespace std;

void ObjectDetector::processFrame(cv::Mat camFrame, Pose currentPose, bool drawGraph)
{
	Mat frame;

	copyMakeBorder(camFrame, frame, borderAdditional, borderAdditional, borderAdditional, borderAdditional, BORDER_CONSTANT, Scalar(0, 0, 0));
	Mat lPlane = applyChannelFilter(frame) * contrastChange;
	const Mat distant = increaseDistance(lPlane, distanceThreshold)*1.5;
	lPlane = applyMask(lPlane, distant);
	//Remove noise?
	fastNlMeansDenoising(lPlane, lPlane);

	auto rowCol = calculateRowColumnHistograms(lPlane);

	auto lightnessRow = rowCol.first;
	auto lightnessCol = rowCol.second;

	auto filteredRow = applyExponentialFilter<float>(lightnessRow, graphAlpha);
	auto filteredCol = applyExponentialFilter<float>(lightnessCol, graphAlpha);
	auto rowChange = changeValue<float>(filteredRow);
	auto colChange = changeValue<float>(filteredCol);

	const int maxPosObjs = 500;
	float thresholdMultiplier = lastMultiplier * 0.75;
	vector<possibleObject> possible_objects;
	vector<int> rowTriggers;
	vector<int> colTriggers;
	do
	{
		possible_objects.clear();
		float rowThresh = calcThreshold(filteredRow, HorizontalThreshold * thresholdMultiplier);
		float colThresh = calcThreshold(filteredCol, VerticalThreshold * thresholdMultiplier);

		rowTriggers = changeDetect(rowChange, rowThresh);
		rowTriggers = removeAdjacent<int>(rowTriggers);
		colTriggers = changeDetect(colChange, colThresh);
		colTriggers = removeAdjacent<int>(colTriggers);

		vector<Point> intersects;

		for (auto it = rowTriggers.begin(); it != rowTriggers.end(); ++it)
		{
			for (auto cit = colTriggers.begin(); cit != colTriggers.end(); ++cit)
			{
				Point intersect = Point(*cit, *it);
				intersects.push_back(intersect);
			}
		}
		if (intersects.size() > 1 && intersects.size() < 1000) //Arbritrary cap
		{
			for (int i = 0; i < intersects.size() - 1; ++i)
			{
				Point x = intersects[i];
				for (int j = i + 1; j < intersects.size(); ++j)
				{
					Point y = intersects[j];

					if (x.x == y.x || x.y == y.y) { continue; }
					
					auto r = Rect(x, y);
					const float viewArea = frame.cols * frame.rows;
					if (r.area() >= viewArea * minPctSize)
					{
						possible_objects.push_back(possibleObject(Rect(x, y)));
						if(possible_objects.size() > maxPosObjs)
						{
							break;
						}
					}
				}
				if(possible_objects.size() > maxPosObjs)
				{
					break;
				}
			}
		}
		thresholdMultiplier += 0.1;
		cout << "Possible Objects found at Multiplier:" << std::to_string(thresholdMultiplier) << " :" << possible_objects.size() << endl;
	} while (possible_objects.size() > maxPosObjs); //Attempting to bound the time it takes to run.
	lastMultiplier = thresholdMultiplier;
	std::sort(possible_objects.rbegin(), possible_objects.rend());
	for (auto it = possible_objects.rbegin(); it != possible_objects.rend(); ++it)
	{
		it->computeAvgVal(distant);
		if (it->avgVal <= 125) //Halfway value, meaning half or more of the box isn't on the object
		{
			possible_objects.erase(std::next(it).base()); //We use reverse looping and this weird iterator flick to remove the unimportant objects
			it->avgVal = 0;
		}
		else if (it->avgVal > 230) //90% of the box is over the object
		{
			it->avgVal = 255;
		}
	}
	std::sort(possible_objects.rbegin(), possible_objects.rend());
	cout << "Object thresholding complete on: " << possible_objects.size() << " possible objects" << endl;
	auto detectedObjects = RemoveOverlaps(possible_objects);
	cout << detectedObjects.size() << " detected objects in frame" << endl;
	const float ObjectWidth = 3.8; //cm

	for (auto it = detectedObjects.rbegin(); it != detectedObjects.rend(); ++it)
	{
		const float xyRatio = it->bounds.width / it->bounds.height;
		if(xyRatio < 1) //taller than wide
		{
			const float heightRatio = it->bounds.height / it->bounds.width;
			if(heightRatio > 3) //Strange artifact
			{
				cout << "Artifact?" << endl;
				//continue;
			}
			const float height = heightRatio * ObjectWidth;
			it->estimatedSize = Size2f(ObjectWidth, height);
		} else //wider than tall
		{
			const float widthRatio = xyRatio;
			if (widthRatio > 3) //Strange artifact
			{
				cout << "Artifact?" << endl;
				//continue;
			}
			const float width = widthRatio * ObjectWidth;
			it->estimatedSize = Size2f(width, ObjectWidth);
		}

		float estimateDepth = it->computeDistance(it->estimatedSize, Size2f(frame.cols, frame.rows), Size2f(HorizontalFOV, VerticalFOV), focalLength);
		if(estimateDepth > 0.0)
		{
			cout << "Object at: " << to_string(estimateDepth) << "cm" << endl;
			it->estimatedDistance = estimateDepth;
			detectionMap.addMeasurement(currentPose, estimateDepth, it->maxMinAngles);
		} else
		{
			cout << "Object beyond reasonable estimate" << endl;
		}
	}
	cout << "Depth estimation complete" << endl;

	detectedInFrame = vector<possibleObject>(detectedObjects);
	
	if(drawGraph)
	{
		cout << "Graphing" << endl;
		infoGraph = Graph(camFrame);
		int graph_h = infoGraph.drawing.rows, graph_w = infoGraph.drawing.cols;

		for (auto it = possible_objects.begin(); it != possible_objects.end(); ++it)
		{
			rectangle(infoGraph.drawing, it->bounds, Scalar::all(it->avgVal), FILLED);
		}
		cout << "Rectangles drawn" << endl;

		infoGraph.drawLineGraph<float>(filteredRow, Scalar(255, 0, 0), false);
		infoGraph.drawLineGraph<float>(filteredCol, Scalar(0, 0, 255), true);

		int h_bin_w = cvRound((double)graph_h / (double)lightnessRow.size());
		int v_bin_w = cvRound((double)graph_w / (double)lightnessCol.size());

		for (auto it = rowTriggers.begin(); it != rowTriggers.end(); ++it)
		{
			infoGraph.drawLine(Point(0, h_bin_w * (*it)), Point(graph_w, h_bin_w * (*it)), Scalar(0, 255, 0));
		}

		for (auto it = colTriggers.begin(); it != colTriggers.end(); ++it)
		{
			infoGraph.drawLine(Point(v_bin_w * (*it), 0), Point(v_bin_w * (*it), graph_h), Scalar(0, 255, 0));
		}

		cout << "Triggers drawn" << endl;

		for (auto it = detectedObjects.begin(); it != detectedObjects.end(); ++it)
		{
			int xMax, yMax, xMin, yMin;
			xMax = max(it->bounds.tl().x, it->bounds.br().x);
			yMax = max(it->bounds.tl().y, it->bounds.br().y);
			xMin = min(it->bounds.tl().x, it->bounds.br().x);
			yMin = min(it->bounds.tl().y, it->bounds.br().y);
			infoGraph.drawLine(Point(xMin, yMax), Point(xMax, yMin), Scalar(255, 0, 0), 3);
			infoGraph.drawLine(Point(xMax, yMax), Point(xMin, yMin), Scalar(255, 0, 0), 3);

			string depthMsg = std::to_string(it->estimatedDistance) + "cm";
			int font = FONT_HERSHEY_SIMPLEX;
			Size textSize = getTextSize(depthMsg, font, 1, 2, 0);
			Point textLoc(it->bounds.x, it->bounds.y);
			putText(infoGraph.drawing, depthMsg, textLoc, font, 1, Scalar::all(75), 2);
		}
		cout << "Object distance drawn" << endl;
	}
}

//BOTTLENECK
/* Known assumptions:
 *	possibleObjects are sorted by AvgValue, then by size. AvgVal is more important, where equal, bigger. Reverse order
 */
vector<possibleObject> ObjectDetector::RemoveOverlaps(vector<possibleObject> sortedData)
{
	int mainCount = 0, innerCount = 0;
	if (sortedData.size() < 2) { return sortedData; }
	for (auto it = sortedData.begin(); it < sortedData.end(); it++)
	{
		cout << "Main loop: " << (++mainCount) << endl;
		for (auto cit = it + 1; cit < sortedData.end(); cit++)
		{
			cout << "Inner loop: " << (++innerCount) << endl;
			if (it == cit)
			{
				continue;
			}

			bool overlaps = it->overlap(*cit);

			if (overlaps)
			{
				sortedData.erase(cit);
				cit = it + 1;
			}
		}
	}
	return sortedData;
}
