#pragma once

#include <algorithm>
#include <vector>
#define _USE_MATH_DEFINES
#include <math.h>

using namespace std;

template<typename T>
vector<typename enable_if<is_arithmetic<T>::value, T>::type> applyExponentialFilter(
	vector<typename enable_if<is_arithmetic<T>::value, T>::type> data, float alpha)
{
	vector<T> filtered = vector<T>(data);
	for (int i = 1; i < data.size(); ++i)
	{
		filtered[i] = alpha * filtered[i - 1] + (1 - alpha) * data[i];
	}
	return filtered;
}

template<typename T>
vector<typename enable_if<is_arithmetic<T>::value, T>::type> changeValue(
	vector<typename enable_if<is_arithmetic<T>::value, T>::type> data)
{
	vector<T> change_values;
	for (auto it = data.begin(); it != (data.end() - 1); ++it)
	{
		change_values.push_back(abs(*(it + 1) - *it));
	}
	return change_values;
}

template<typename T>
vector<typename enable_if<is_arithmetic<T>::value, T>::type> removeAdjacent(
	vector<typename enable_if<is_arithmetic<T>::value, T>::type> data)
{
	if (data.size() == 0) { return data; }
	std::sort(data.begin(), data.end());
	vector<int> cleansed;
	for (int i = 0; i < data.size() - 1; ++i)
	{
		int val = data[i];
		if (data[i + 1] == val + 1)
		{
			int j = i + 1;
			int last = val;
			while (j < data.size() && data[j] == last + 1)
			{
				last = data[j];
				j++;
			}
			int mid = ceil((last + val) / 2.0);
			cleansed.push_back(mid);
			i = j;
		}
		else
		{
			cleansed.push_back(val);
		}
	}
	return cleansed;
}

template<typename T>
vector<T> getInterquartileRange(vector<T> data)
{
	if (data.size() < 3) { return data; }
	int fq = ceil(data.size() / 4.0);
	int tq = 3 * fq;
	fq = fq < 0 ? 0 : fq;
	fq = fq >= data.size() ? data.size() - 1 : fq;
	tq = tq >= data.size() ? data.size() - 1 : tq;

	vector<T> iqr;

	for (int i = fq; i <= tq; ++i)
	{
		iqr.push_back(data[i]);
	}

	return iqr;
}

template <typename T>
pair<int, int> getFQTQ(vector<T> data) //Get First and Third quartile
{
	if (data.size() == 0) { return pair<int, int>(0, 0); }
	if (data.size() == 2) { return pair<int, int>(data[0], data[1]); }
	int fq, tq;
	std::sort(data.begin(), data.end());
	fq = (int)ceil(data.size() / 4.0);
	tq = 3 * fq;

	fq = fq < 0 ? 0 : fq;
	fq = fq >= data.size() ? data.size() - 1 : fq;
	tq = tq >= data.size() ? data.size() - 1 : tq;
	return pair<int, int>(data[fq], data[tq]);
}

vector<int> changeDetect(vector<float> change_values, float threshold);

float degToRad(float ang);

float radBound(float radAng);