#pragma once

#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;

class Graph
{
public:
	Mat drawing;

	Graph(Mat base)
	{
		drawing = Mat(base.rows, base.cols, base.type(), Scalar::all(0));
	}

	void drawLine(Point x, Point y, Scalar color, int thickness = 1);
	
	template<typename T>
	void drawLineGraph(vector<typename enable_if<is_arithmetic<T>::value, T>::type> data, Scalar color, bool vertical = true, float scale = 1.0)
	{
		int bin_w = vertical ? cvRound((double)drawing.cols / (double)data.size()) : cvRound((double)drawing.rows / (double)data.size());
		for (int i = 1; i < data.size(); i++)
		{

			Point p1 = vertical ? Point(bin_w * (i - 1), drawing.rows - (data[i - 1])) : Point(cvRound(data[i - 1] * scale), bin_w * (i - 1));
			Point p2 = vertical ? Point(bin_w * (i), drawing.rows - (data[i])) : Point(cvRound(data[i] * scale), bin_w * (i));
			drawLine(p1, p2, color, 2);
		}
	}
};


