#include "ObjectDetection.h"
#include "imgProcessing.h"
#include "Statistics.h"
#include "Camera.h"

using namespace cv;

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

	float rowThresh = calcThreshold(filteredRow, HorizontalThreshold);
	float colThresh = calcThreshold(filteredCol, VerticalThreshold);

	auto rowTriggers = changeDetect(rowChange, rowThresh);
	rowTriggers = removeAdjacent<int>(rowTriggers);
	auto colTriggers = changeDetect(colChange, colThresh);
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

	vector<possibleObject> possible_objects;
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
				}

			}
		}
	}

	std::sort(possible_objects.begin(), possible_objects.end());
	
	for (auto it = possible_objects.begin(); it != possible_objects.end(); ++it)
	{
		it->computeAvgVal(distant);
		if (it->avgVal <= 125) //Halfway value, meaning half or more of the box isn't on the object
		{
			it->avgVal = 0;
		}
		else if (it->avgVal > 230) //90% of the box is over the object
		{
			it->avgVal = 255;
		}
	}
	std::sort(possible_objects.begin(), possible_objects.end());
	auto detectedObjects = RemoveOverlaps(possible_objects);

	const float ObjectWidth = 3.8; //cm

	for (auto it = detectedObjects.begin(); it != detectedObjects.end(); ++it)
	{
		const float xyRatio = it->bounds.width / it->bounds.height;
		if(xyRatio < 1) //taller than wide
		{
			const float heightRatio = it->bounds.height / it->bounds.width;
			if(heightRatio > 3) //Strange artifact
			{
				cout << "Object too big?" << heightRatio << endl;
				//it->estimatedSize = Size2f(0, 0);
			}
			const float height = heightRatio * ObjectWidth;
			it->estimatedSize = Size2f(ObjectWidth, height);
		} else //wider than tall
		{
			const float widthRatio = xyRatio;
			if (widthRatio > 3) //Strange artifact
			{
				cout << "Object too big?" << widthRatio << endl;
				it->estimatedSize = Size2f(0, 0);
			}
			const float width = widthRatio * ObjectWidth;
			it->estimatedSize = Size2f(width, ObjectWidth);
		}

		float estimateDepth = it->computeDistance(it->estimatedSize, Size2f(frame.cols, frame.rows), Size2f(HorizontalFOV, VerticalFOV), focalLength);
		if(estimateDepth > 0.0)
		{
			it->estimatedDistance = estimateDepth;
			detectionMap.addMeasurement(currentPose, estimateDepth, it->maxMinAngles);
		}
	}
	
	if(drawGraph)
	{
		infoGraph = Graph(camFrame);
		int graph_h = infoGraph.drawing.rows, graph_w = infoGraph.drawing.cols;

		for (auto it = possible_objects.begin(); it != possible_objects.end(); ++it)
		{
			rectangle(infoGraph.drawing, it->bounds, Scalar::all(it->avgVal), FILLED);
		}

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
	}
}

vector<possibleObject> ObjectDetector::RemoveOverlaps(vector<possibleObject> sortedData)
{
	if (sortedData.size() < 2) { return sortedData; }
	reverse(sortedData.begin(), sortedData.end());
	for (auto it = sortedData.begin(); it != sortedData.end(); ++it)
	{
		if (it->avgVal == 0)
		{
			sortedData.erase(it);
			it = sortedData.begin();
		}

		for (auto cit = it; cit != sortedData.end(); ++cit)
		{
			if (it == cit)
			{
				continue;
			}

			bool overlaps = it->overlap(*cit);

			if (overlaps)
			{
				sortedData.erase(cit);
				cit = it;
			}
		}
	}

	return sortedData;
}
