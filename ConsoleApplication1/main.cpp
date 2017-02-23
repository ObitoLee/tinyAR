#include "myHead.hpp"
#include "fstream"
#include "getConfig.h"
#include <sstream>

#define IMAGE_WIDTH 640    //图像宽度
#define IMAGE_HEIGHT 480   //图像高度
#define ZOOM_FACTOR 1     //为加快处理速度，对图像进行等比例缩小，此处为1倍
#define BLUE
#define TARGETNUM 2
#define minContour 5
#define maxContour 6000

bool stopProc = false;//多线程控制位
bool undistortMode = true;
bool thresholdMode = false;
bool paused = false;

const char *g_szTitle = "Camera";
map<string, string> config;

#define a 30
#define b 30

int main()
{
	vector<Point3f> worldTop;
	worldTop.push_back({ -a, b, a });
	worldTop.push_back({ -a, -b, a });
	worldTop.push_back({ a, -b, a });
	worldTop.push_back({ a, b, a });

	ReadConfig("video.cfg", config);
	int t = atoi(config["t"].c_str());
	namedWindow(g_szTitle);
	createTrackbar("t", g_szTitle, &t, 256, 0);
	
	//VideoCapture cap("1.mp4");
	//VideoCapture cap("d/5.mp4");
	//VideoCapture cap("mul/8_f.mp4");
	VideoCapture cap(1);
	VideoWriter writer("recorder.avi", CV_FOURCC('M', 'J', 'P', 'G'), 25, Size(640, 480));
	if (!cap.isOpened())
	{
		cerr << "Can't open the source!Exit!\n";
		return 0;
	}

	Mat imgOriginal, imgThresholded;
	cap >> imgOriginal;
	
	Rect roiImg = Rect(0, 0, imgOriginal.cols * ZOOM_FACTOR, imgOriginal.rows * ZOOM_FACTOR);

	vector<vector<Point> > contours;
	vector<RotatedRect> vEllipse;//符合条件的椭圆
	Armors armors(imgOriginal.cols*ZOOM_FACTOR, imgOriginal.rows*ZOOM_FACTOR);

	int sendFilter = 0;
	bool sendBool = false;
	int frames = 0;
	int	vaildFrames[TARGETNUM] = { 0 };
	
	int64 startTime = getTickCount();

	while (true)
	{
		String noTargetReason;
		int64 t0 = getTickCount();
		if (!paused)
		{
			frames++;
			if (!cap.read(imgOriginal))
				break;

			
			resize(imgOriginal, imgOriginal, Size(imgOriginal.cols * ZOOM_FACTOR, imgOriginal.rows * ZOOM_FACTOR));//等比例缩小
			
#ifdef BLUE
			//Canny(imgOriginal, imgThresholded, t, 3 * t, 3);
			GetDiffImage(imgOriginal, imgThresholded, t, blue, roiImg);//蓝色
#else
			GetDiffImage(imgOriginal, imgThresholded, t, red, roiImg);//红色
#endif
			preProcess(imgThresholded);
			findContours(imgThresholded, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);
			drawContours(imgThresholded, contours, -1, Scalar(255, 255, 255));

			for (auto itContours = contours.begin(); itContours != contours.end(); ++itContours)
			{
				vector<Point> points = *itContours;//将一个轮廓的所有点存入points
				if (itContours->size() > minContour && itContours->size() < maxContour)//筛选轮廓上的点大于100的轮廓
				{
					RotatedRect s = fitEllipse(points);//拟合椭圆
					if (s.size.height * s.size.width > 18000 * ZOOM_FACTOR * ZOOM_FACTOR
						|| (s.size.height < 1.2 * s.size.width) || (s.size.height > s.size.width * 15))
					{
						noTargetReason += "size don't cater to standard.\n";
						cout << s.size.height << "\t" << s.size.width << endl;
						continue;
					}
						
					Point2f vertex[4]; s.points(vertex);
					circle(imgOriginal, (vertex[0] + vertex[3]) / 2, 4, Scalar(22, 33, 244), 2);
					circle(imgOriginal, (vertex[1] + vertex[2]) / 2, 4, Scalar(0, 244, 233), 2);

					vEllipse.push_back(s);
					ellipse(imgOriginal, s, Scalar(255, 0, 166), 2);
				}

				points.swap(vector<Point>());
			}

			armors.inputEllipse(vEllipse);//输入将测到的椭圆
			vector<Point2f> target = armors.getTarget(TARGETNUM);//求目标坐标

			for (int i = 0; i < armors.getTargetNum(); ++i)//检测到目标
			{
				vaildFrames[i]++;
				//roiImg = armors.getROIbox(imgOriginal);
				//armors.drawAllArmors(imgOriginal);
				//circle(imgOriginal, target[i], 5, Scalar(0, 255, 255), 1, 8, 3);
				//putText(imgOriginal, to_string(armors.getScore(i)), target[i], FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255));
				vector<Point2f> imgPointBottom = armors.getVertex(i);
				vector<Point2f> imgPointTop;
				projectPoints(worldTop, armors.getRT(i).rM, armors.getRT(i).t,intrinsic_matrix,distCoeffs,imgPointTop);
				for (int ni = 0; ni < 4; ni++)
				{
					line(imgOriginal, imgPointBottom[ni], imgPointBottom[(ni + 1) % 4], Scalar(0, 255, 255), 2, LINE_AA);
					line(imgOriginal, imgPointTop[ni], imgPointBottom[ni], Scalar(0, 255, 255), 2, LINE_AA);
				}
				for (int ni = 0; ni < 4; ni++)
					line(imgOriginal, imgPointTop[ni], imgPointTop[(ni + 1) % 4], Scalar(0, 0, 255), 2, LINE_AA);

				
				//cout << armors.getRT(i).t.at<double>(2) << endl;
			}
// 			else
// 			{
// 				//cout << "目标丢失~~~" << endl;
// 				putText(imgOriginal, "Looking for the enemy.......", Point(90 * ZOOM_FACTOR, 90 * ZOOM_FACTOR), FONT_HERSHEY_PLAIN, 2 * ZOOM_FACTOR, Scalar(0, 0, 255), 2, 8);
// 				//roiImg = Rect(0, 0, imgOriginal.cols, imgOriginal.rows);
// 			}
			vEllipse.clear();
			contours.swap(vector<vector<Point>>());
			cout << "\n======================\n";
		}

		writer << imgOriginal;

 		if (thresholdMode)
 			imshow(g_szTitle, imgThresholded);
 		else
 			imshow(g_szTitle, imgOriginal);
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
		//int64 t = getTickCount() - t0;
		//cout << 1000 / (1000 * t / getTickFrequency()) << "fps" << "\n";
	}

	int64 spendTime = getTickCount() - startTime;
	cout << frames * 1000 / (1000 * spendTime / getTickFrequency()) << "fps" << "\n";
	cout << "total：\t" << frames << endl;
	for (int i = 0; i < TARGETNUM;++i)
		cout << i+1 << ":\t" << vaildFrames[i]<<endl;
 
	config["t"] = to_string(t);
	WriteConfig("video.cfg", config);
	return 0;
}
