#include "markerRecognizer.h"
#include "fstream"
#include "getConfig.h"
#include <sstream>

#define IMAGE_WIDTH 640    //Í¼Ïñ¿í¶È
#define IMAGE_HEIGHT 480   //Í¼Ïñ¸ß¶È
#define BLUE
#define MaxTargetNum 2

bool undistortMode = true;
bool thresholdMode = false;
bool paused = false;

const char *g_szTitle = "Camera";
map<string, string> config;

int main()
{
	VideoCapture video("C:\\Users\\Lee\\Music\\MV\\Ìïð¥Õç - Ð¡ÐÒÔË.mp4");
	//video.set(CAP_PROP_POS_FRAMES, 15500);
	Mat showImg;
	//video.read(showImg);
	//Mat showImg = imread("1.jpg");

	//VideoCapture cap("1.mp4");
	//VideoCapture cap("mul/8_f.mp4");
	VideoCapture cap(0);
	if (!cap.isOpened()|| !video.isOpened())
	{
		cerr << "Can't open the source!Exit!\n";
		return 0;
	}

	namedWindow(g_szTitle);
	ReadConfig("video.cfg", config);
	int t = atoi(config["t"].c_str());
	createTrackbar("t", g_szTitle, &t, 256, 0);

	//VideoWriter writer("recorder.avi", CV_FOURCC('M', 'J', 'P', 'G'), 25, Size(640, 480));

	Mat imgOriginal;
	MakerRecognizer maker(IMAGE_WIDTH, IMAGE_HEIGHT, 1, MaxTargetNum);

	while (true)
	{
		if (!paused)
		{
			if (!cap.read(imgOriginal) || !video.read(showImg))
				break;
			maker.find(imgOriginal, t, blue);
			maker.drawARImg(showImg);
		
			//writer << imgOriginal;
		}

		if (thresholdMode)
			imshow(g_szTitle, maker.imgThresholded);
		else
			imshow(g_szTitle, maker.outputImg);

		char key = (char)waitKey(1);
		if (key == 27)  break;
		switch (key)
		{
		case 'b':
			thresholdMode = !thresholdMode;
			break;
		case 'p':
			paused = !paused;
			break;
		case 'd':
			undistortMode = !undistortMode;
			break;
		default:
			;
		}

	}

	config["t"] = to_string(t);
	WriteConfig("video.cfg", config);
	return 0;
}
