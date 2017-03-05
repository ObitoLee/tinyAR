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

//@brief��װ���࣬�̳�RotatedRect�࣬��ʾ��һһ��װ��
//@note���൱�ڶ���һ��RotatedRect������һ�����ԣ�likelihood�������ڶ�����װ�׵Ŀ�����
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
	void find(Mat &img, int t, int color);		//������ɫ����ֵ����maker
	void drawARImg(Mat imgToDraw);	//�ڱ�־����ARͼƬ����֧�ֵ�Ŀ��
	void drawARcube(float a, float b);				//�ڱ�־����AR������
	void drawAllMarkers(Scalar _color = Scalar(100, 0, 211));//@brief����img�ϻ����ɫΪcolorĿ��
	Mat outputImg, imgThresholded;
	String error;	//û��⵽��ԭ��
private:
	void preProcess(Mat& _img);
	void GetDiffImage(Mat _img, Mat &_dst, int _threshold, int _color, Rect _roi);

	int imgWidth, imgHeight;//���ź�ͼ��Ŀ��
	int ZOOM_FACTOR;//���ű���
	size_t MaxTargetNum;

	Rect roiImg;
	Rect boundingBox;//bounding of img

private:
	void findTargetsByEllispses(vector<RotatedRect> _ellipses);//@brief�����ش��Ŀ�����������

	poseMat getRT(int index);//@brief���õ�λ��

	//@brief��ͨ��ƽ���ϳ������ĵ����λ�ˣ� �� P4P
	//@param: _vertex is ͼ���ϵĳ����ε�4������
	//@param: A is �ڲξ���3x3
	//@param: d is ����ϵ��5x1
	//@param: width is �����ο�
	//@param: height is �����θ�
	poseMat poseEstimation(vector<Point2f> _vertex, Mat_<double> A, Mat_<float> d, float width, float height);


	Rect getROIbox(int i);//@brief�����ظ���Ȥ����Rect
	inline Mat getROI(int i) { return outputImg(getROIbox(i)); }//@brief�����ظ���Ȥ����Mat
	//inline size_t getTargetNum() { return targrtNum; }
	inline float getScore(int i) { return vRlt[i].likelihood; }
	inline vector<Point2f> getVertex(int i) { return vRlt[i].vertex; }

	size_t findTargrtNum;//�ҵ���Ŀ������ = min���趨��������⵽�ĸ�����
	vector<Armor> vRlt;//����������Ŀ��
	vector<Rect> targetBox;
	vector<Point2f> targetPos;

private://�������ļ�⵽�Ľ��
	int frames = 0;
	vector<int>	vaildFrames;
	int64 startTime;
};