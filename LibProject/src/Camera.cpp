#include <Camera.h>

#include <iostream>
#include <opencv2/videoio.hpp>

using namespace cv;
using namespace std;

bool Initialise_Camera(VideoCapture& cap, VideoWriter& videoWriter, Mat& baseFrame,int width, int height, int fps,string fileName)
{
	if (!cap.isOpened())
	{
		cout << "Cannot open webcam" << endl;
		cap.open(0);
		if (!cap.isOpened())
		{
			cout << "Really can't open it" << endl;
			return false;
		}
	}

	cap.set(CAP_PROP_FRAME_WIDTH,width);
	cap.set(CAP_PROP_FRAME_HEIGHT, height);
	cap.set(CAP_PROP_FPS, fps);

	Mat frame;
	bool success = cap.read(frame);
	if(!success)
	{
		cout << "Could not read from camera" << endl;
		return false;
	}

	
	int frames_per_second = static_cast<int>(cap.get(CAP_PROP_FPS));//cap.get(CAP_PROP_XI_FRAMERATE);

	cout << "Frame dimensions: " << frame.size() << endl;
	cout << "FPS: " << frames_per_second << endl;

	videoWriter = VideoWriter(fileName, VideoWriter::fourcc('M', 'J', 'P', 'G'),
		frames_per_second, frame.size(), true);

	if (!videoWriter.isOpened())
	{
		cout << "Cannot save video to file" << endl;
		return false;
	}
	return true;
}