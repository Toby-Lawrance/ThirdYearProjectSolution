#include "imgProcessing.h"

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>     // Basic OpenCV structures (cv::Mat, Scalar)
#include <opencv2/imgproc.hpp>  // Gaussian Blur
#include <opencv2/highgui.hpp>  // OpenCV window I/O
#include <opencv2/features2d.hpp>

using namespace cv;
using namespace std;

Mat increaseDistance(Mat img, double thresh)
{
	Mat edited;
	threshold(img, edited, thresh, 255, img.type());
	return edited;
}

Mat red(Mat bgrImg)
{
	vector<Mat> bgr_planes;
	split(bgrImg, bgr_planes);
	return max(bgr_planes[2] - 0.5 * bgr_planes[1] - 0.5 * bgr_planes[0], 0);
}

Mat blue(Mat bgrImg)
{
	vector<Mat> bgr_planes;
	split(bgrImg, bgr_planes);
	return max(bgr_planes[1] - 0.5 * bgr_planes[0] - 0.5 * bgr_planes[0], 0);
}

Mat green(Mat bgrImg)
{
	vector<Mat> bgr_planes;
	split(bgrImg, bgr_planes);
	return max(bgr_planes[0] - 0.5 * bgr_planes[1] - 0.5 * bgr_planes[2], 0);
}

Mat applyChannelFilter(Mat bgrImg)
{
	return max(red(bgrImg), max(green(bgrImg), blue(bgrImg)));
}

Mat calculateLightness(Mat frame)
{
	vector<Mat> bgr_planes;
	split(frame, bgr_planes);
	Mat maxRGB = max(bgr_planes[2], max(bgr_planes[1], bgr_planes[0]));
	Mat minRGB = min(bgr_planes[2], max(bgr_planes[1], bgr_planes[0]));
	return (maxRGB - minRGB) / 2.0;
}

Mat applyMask(Mat frame, Mat mask)
{
	Mat masked;
	frame.copyTo(masked, mask);
	return masked;
}

float calcThreshold(vector<float> data, float factor)
{
	float max = 0;
	float min = FP_INFINITE;

	for (auto it = data.begin(); it != data.end(); ++it)
	{
		float val = *it;
		if (val > max)
		{
			max = val;
		}
		if (val < min)
		{
			min = val;
		}
	}

	return (max - min) / factor;
}

pair<vector<float>,vector<float>> calculateRowColumnHistograms(Mat frame)
{
	vector<float> lightnessRow = vector<float>(frame.rows);
	vector<float> lightnessCol = vector<float>(frame.cols);

	for (int i = 0; i < lightnessRow.size(); i++)
	{
		auto row = frame.row(i);
		float total = 0;
		for (int j = 0; j < row.cols; ++j)
		{
			auto pixel = row.at<uchar>(0, j);
			float value = static_cast<float>(pixel);
			total += value;
		}
		lightnessRow[i] = total / row.cols;
	}

	for (int i = 0; i < lightnessCol.size(); i++)
	{
		auto col = frame.col(i);
		float total = 0;
		for (int j = 0; j < col.rows; ++j)
		{
			auto pixel = col.at<uchar>(j, 0);
			float value = static_cast<float>(pixel);
			total += value;
		}
		lightnessCol[i] = total / col.rows;
	}

	return { lightnessRow,lightnessCol };
}

