#pragma once

#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;

Mat increaseDistance(Mat img, double thresh = 200);

Mat red(Mat bgrImg);
Mat blue(Mat bgrImg);
Mat green(Mat bgrImg);

Mat applyChannelFilter(Mat bgrImg);

Mat calculateLightness(Mat frame);

Mat applyMask(Mat frame, Mat mask);

float calcThreshold(vector<float> data, float factor = 100.0);

pair<vector<float>, vector<float>> calculateRowColumnHistograms(Mat frame);