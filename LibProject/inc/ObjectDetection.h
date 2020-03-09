#pragma once
#include <Map.h>
#include "Graph.h"
#include "possibleObject.h"


class ObjectDetector
{
public:
	Map detectionMap;
	Graph infoGraph;

	const int borderAdditional = 3;
	const int distanceThreshold = 50;
	const float graphAlpha = 0.6;
	const int HorizontalThreshold = 45;
	const int VerticalThreshold = 35;
	const float contrastChange = 1.5;
	const float minPctSize = 0.00;

	float lastMultiplier = 1.0;

	vector<possibleObject> detectedInFrame;

	ObjectDetector(cv::Mat openingFrame) : infoGraph(openingFrame) {}

	void processFrame(cv::Mat frame, Pose currentPose, bool drawGraph = false);

private:
	static vector<possibleObject> RemoveOverlaps(vector<possibleObject> sortedData);
	
};
