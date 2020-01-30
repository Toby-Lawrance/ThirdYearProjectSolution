#include "Statistics.h"

vector<int> changeDetect(vector<float> change_values, float threshold)
{
	vector<int> triggerPoints;
	for (int i = 0; i < change_values.size(); ++i)
	{
		auto val = change_values[i];
		if (val > threshold)
		{
			triggerPoints.push_back(i);
		}
	}
	return triggerPoints;
}

float degToRad(float ang)
{
	const static float degToRadMult = M_PI / 180.0;
	const static float TwoPi = 2.0 * M_PI;

	return fmod(ang * degToRadMult, TwoPi);
}

float radBound(float radAng)
{
	const static float TwoPi = 2.0 * M_PI;
	return fmod(radAng, TwoPi);
}