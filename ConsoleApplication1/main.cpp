#include "myHead.hpp"
#include <opencv.hpp>
#include "fstream"
#include "getConfig.h"
#include <sstream>

using namespace cv;
using namespace std;

#define IMAGE_WIDTH 640    //图像宽度
#define IMAGE_HEIGHT 480   //图像高度
#define ZOOM_FACTOR 1      //为加快处理速度，对图像进行等比例缩小，此处为0.5倍
#define DEBUG            //图像显示
#define BLUE
#define CAM
#define TARGETNUM 4
#define minContour 40
#define maxContour 6000

bool stopProc = false;
bool undistortMode = true;
bool imgCatched = false;
bool thresholdMode = false;
bool paused = false;

const char *g_szTitle = "Camera";
map<string, string> config;

int main()
{
	ReadConfig("video.cfg", config);
	int t = atoi(config["t"].c_str());
	namedWindow(g_szTitle);
	createTrackbar("t", g_szTitle, &t, 256, 0);

	VideoCapture cap("3.mp4");

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
			//imgOriginal -= Scalar(B,G,R);
			
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
					//RotatedRect s = minAreaRect((points));//拟合椭圆
					// 					Rect colorRect = (s.boundingRect() - Point(7, 7) + Size(14, 14)) & roiImg;
					// 					int color = colorJudge(imgOriginal(colorRect));
					// 					if (color != blue && color != blue - 1 && color != blue + 1)
					// 						continue;

					if (s.size.height * s.size.width > 18000 * ZOOM_FACTOR*ZOOM_FACTOR
						|| (s.size.height < 4 * s.size.width) || (s.size.height > s.size.width * 11))
					{
						noTargetReason += "size don't cater to standard.\n";
						continue;
					}
					//cout << s.size.height << "\t" << s.size.width << endl;
					Point2f vertex[4]; s.points(vertex);
					circle(imgOriginal, (vertex[0] + vertex[3]) / 2, 4, Scalar(22, 33, 244), 2);
					circle(imgOriginal, (vertex[1] + vertex[2]) / 2, 4, Scalar(0, 244, 233), 2);

					vEllipse.push_back(s);
					ellipse(imgOriginal, s, Scalar(255, 255, 66), 2);
				}

				points.swap(vector<Point>());
			}
			armors.inputEllipse(vEllipse);//输入将测到的椭圆，寻找装甲
			vector<Point2f> target = armors.getTarget(TARGETNUM);//求目标坐标
			//target = armors.track();//追踪
			//cout << "target detected:" << armors.getTargetNum() << endl;
			for (int i = 0; i < armors.getTargetNum(); ++i)//检测到目标
			{
				vaildFrames[i]++;
				//roiImg = armors.getROIbox(imgOriginal);
				armors.drawAllArmors(imgOriginal);

				circle(imgOriginal, target[i] * ZOOM_FACTOR, 5, Scalar(0, 255, 255), 1, 8, 3);
				putText(imgOriginal, to_string(armors.getScore(i)), target[i] * ZOOM_FACTOR, FONT_HERSHEY_PLAIN, 2 * ZOOM_FACTOR, Scalar(0, 0, 255), 2, 8);

				if (sendFilter >= 3)
					sendBool = true;
				else
					sendFilter++;
			}
			if (armors.getTargetNum() == 0)
			{
				noTargetReason += armors.error;
				cout << noTargetReason << endl;
				sendFilter--;
				if (sendFilter < 0)
				{
					sendFilter = 0;
					sendBool = false;
				}
			}
			if (sendBool)
			{
				armors.getRT();
			}
			else
			{
				//cout << "目标丢失~~~" << endl;
				putText(imgOriginal, "Looking for the enemy.......", Point(90 * ZOOM_FACTOR, 90 * ZOOM_FACTOR), FONT_HERSHEY_PLAIN, 2 * ZOOM_FACTOR, Scalar(0, 0, 255), 2, 8);
				//roiImg = Rect(0, 0, imgOriginal.cols, imgOriginal.rows);

			}
			vEllipse.clear();
			contours.swap(vector<vector<Point>>());
			cout << "\n======================\n";
		}

		char key = (char)waitKey(1);
		if (key == 27)  break;  //esc键退出
		if (thresholdMode)
			imshow(g_szTitle, imgThresholded);
		else
			imshow(g_szTitle, imgOriginal);
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
		int64 t = getTickCount() - t0;
		//cout << "主线程：" << 1000 * t / getTickFrequency() << "ms" << "\n";
		
	}

	cout << "total：\t" << frames << endl;
	for (int i = 0; i < TARGETNUM;++i)
	{
		cout << i+1 << ":\t" << vaildFrames[i]<<endl;
	}
 
	config["t"] = to_string(t);
	WriteConfig("video.cfg", config);
	return 0;
}
