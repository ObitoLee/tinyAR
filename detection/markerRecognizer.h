#pragma once

#include "opencv.hpp"
#include <iostream>

#define minContour 20
#define maxContour 6000

#define T_ANGLE_THRE 59.0
#define T_SIZE_THRE 4.0

enum color
{
	red = 0,
	green = 4,
	blue = 6,
	purple = 9
};

using namespace cv;
using namespace std;

struct poseMat
{
	Mat rM;
	Mat t;
};

//@brief：装甲类，继承RotatedRect类，表示单一一个装甲
//@note：相当于定义一个RotatedRect，多了一个属性：likelihood——用于度量是装甲的可能性
class Armor :public RotatedRect
{
public:
	Armor() { initial(); };
	~Armor() {};
	double likelihood;
	inline float area() { return size.area(); }
	inline void calcLikelihood(double _possibility) { likelihood *= _possibility; }
	inline void initial() { likelihood = 1; vertex.resize(4); }
	vector<Point2f>  vertex;
private:
};

class MakerRecognizer
{
public:
	MakerRecognizer(int width = 640, int height = 480, int ZOOM = 1, size_t TARGETNUM = 1);
	~MakerRecognizer();
	void find(Mat &img, int t, int color);		//根据颜色、阈值查找maker
	void drawARImg(Mat imgToDraw);	//在标志增加AR图片，仅支持单目标
	void drawARcube(float a, float b);				//在标志增加AR立方体
	void drawAllMarkers(Scalar _color = Scalar(100, 0, 211));//@brief：在img上绘出颜色为color目标
	Mat outputImg, imgThresholded;
	String error;	//没检测到的原因
private:
	void preProcess(Mat& _img);
	void GetDiffImage(Mat _img, Mat &_dst, int _threshold, int _color, Rect _roi);

	int imgWidth, imgHeight;//缩放后图像的宽高
	int ZOOM_FACTOR;//缩放比例
	size_t MaxTargetNum;

	Rect roiImg;
	Rect boundingBox;//bounding of img

private:
	void findTargetsByEllispses(vector<RotatedRect> _ellipses);//@brief：返回打击目标的像素坐标

	poseMat getRT(int index);//@brief：得到位姿

	//@brief：通过平面上长方形四点估算位姿， 即 P4P
	//@param: _vertex is 图像上的长方形的4个顶点
	//@param: A is 内参矩阵3x3
	//@param: d is 畸变系数5x1
	//@param: width is 长方形宽
	//@param: height is 长方形高
	poseMat poseEstimation(vector<Point2f> _vertex, Mat_<double> A, Mat_<float> d, float width, float height);


	Rect getROIbox(int i);//@brief：返回感兴趣区域Rect
	inline Mat getROI(int i) { return outputImg(getROIbox(i)); }//@brief：返回感兴趣区域Mat
	//inline size_t getTargetNum() { return targrtNum; }
	inline float getScore(int i) { return vRlt[i].likelihood; }
	inline vector<Point2f> getVertex(int i) { return vRlt[i].vertex; }

	size_t findTargrtNum;//找到的目标数量 = min（设定个数，检测到的个数）
	vector<Armor> vRlt;//符合条件的目标
	vector<Rect> targetBox;
	vector<Point2f> targetPos;

private://用于最后的检测到的结果
	int frames = 0;
	vector<int>	vaildFrames;
	int64 startTime;
};