#include <iostream>
#include "opencv2/opencv.hpp"
#include <stdio.h>
#include <stdlib.h>
#include "omp.h"
//Serialport need:
#include <string.h>
#include <sys/types.h>
#include <errno.h>
#include <sys/stat.h>
#include <fcntl.h>

#ifndef WIN32
#include <unistd.h>
#include <termios.h>
#endif // LINUX

#define angleThreshold 90.0

#define purple 9
#define blue 6
#define green 4
#define red 0
#define T_ANGLE_THRE 25.0
#define T_SIZE_THRE 4.0

#define DIVID_ROWS 13
#define DIVID_COLS 13//��ͼ�񻮷�ΪDIVID_ROWS�У�DIVID_COLS�У��ж�Ŀ�����ĸ�������

using namespace cv;
using namespace std;

Mat_<float> intrinsic_matrix = (Mat_<float>(3, 3) << 488.31706, 0, 334.75664,
														0, 520.21521, 246.92317,
														0, 0, 1);
Mat_<float> distCoeffs = (Mat_<float>(5, 1) << -0.04312539,-0.005736560, 0, 0, 0);

Mat_<float> cameraPos = (Mat_<float>(3, 1) << 0, 0, 0);

//@brief��ͼƬԤ�������ͣ���ʴ���˲���
//@param��img is 8UC1
void preProcess(Mat& _img)
{
	Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
	Mat element2 = getStructuringElement(MORPH_RECT, Size(5, 5));
	dilate(_img, _img, element, Point(-1, -1), 2);
	erode(_img, _img, element2, Point(-1, -1), 1);
	//blur( _img, _img, Size(3, 3));
	//medianBlur ( img, img, 7); ̫��ʱ
}

/*@brief������ת����ͼ�����굽������꣩
@param�������ά�������
@param��imgXY is �����ά��������
@param��intrinsic_matrix is �ڲξ���**/
Point3f img2camera(Point2f _imgXY, Mat_<float> _intrinsic_matrix)
{
	Mat_<float> target2D = (Mat_<float>(3, 1) << _imgXY.x, _imgXY.y, 1);
	Mat_<float> target3D = _intrinsic_matrix.inv() * target2D;
	Point3f cameraXYZ;
	cameraXYZ.x = target3D(0);
	cameraXYZ.y = target3D(1);
	cameraXYZ.z = target3D(2);
	return cameraXYZ;
}

//@brief��judge the main color of input img by using histogram
//@param��src:����ͼ��8UC3��
//@param��output color is red,green,blue or purple ,see macro definitions
int colorJudge(Mat _src, int _histSize = 10)
{
	Mat hsv, hue, hist;
	cvtColor(_src, hsv, COLOR_BGR2HSV);
	hue.create(hsv.size(), hsv.depth());
	int ch[] = { 0, 0 };
	mixChannels(&hsv, 1, &hue, 1, ch, 1);
	float hranges[] = { 0, 180 };
	const float* phranges = hranges;
	calcHist(&hue, 1, 0, Mat(), hist, 1, &_histSize, &phranges);
	normalize(hist, hist, 0, 255, CV_MINMAX);
	// cout << hist;
	int color = -1;
	for (int i = 0; i < _histSize; ++i)
	{
		float *p = hist.ptr<float>(i);
		if (p[0] > 254)
			color = i;//return i;
	}

	if (color == red || color == purple)
	{
		for (int i = 1; i < _histSize - 1; ++i)
		{
			float *p = hist.ptr<float>(i);
			if (p[0] > 150)
			{
				color = -1;//retu
				break;
			}
		}
	}

	return color;
}

void poseEstimation(vector<Point2f> _vertex, Mat_<double> A, Mat_<float> d, float a, float b)
{
	vector<Point3f> world;
	world.push_back({ -a, b, 0 });
	world.push_back({ -a, -b, 0 });
	world.push_back({ a, -b, 0 });
	world.push_back({ a, b, 0 });

	Mat r, t, rM;
	solvePnP(world, _vertex, A, d, r, t, false);
	Rodrigues(r, rM);
	//cout << "��ת���󣨵�������" << rM << "\nƽ������:" << t << endl << endl;

	solvePnP(world, _vertex, A, d, r, t, false, CV_P3P);
	//Rodrigues(r, rM);
	//cout << "P3P��"<<r << endl << rM << endl << t << endl << endl;

	// 	solvePnP(world, _vertex, A, d, r, t, false, CV_EPNP);
	// 	Rodrigues(r, rM);
	// 	cout <<  "EPNP��"<<r << endl << rM << endl << t << endl << endl;
}

/*@brief����img��roi������Ѱ�����ȴ���threshold�����򣬷��ض�ֵͼ
@param��
@param��**/
void GetBrightImage(Mat _img, Mat &_dst, int _threshold, Rect _roi)
{
	_dst = Mat::zeros(_img.rows, _img.cols, CV_8UC1);
	Mat imgRoi = _img(_roi);
	Mat dstRoi = _dst(_roi);
	inRange(imgRoi, Scalar(_threshold, _threshold, _threshold), Scalar(255, 255, 255), dstRoi);
}

//@brief����ɫɸѡ
//@param��img:����ͼ��8UC3��
//@param��dst:���ͼ��8UC1��
//@param��threshold:��ֵ
//@param��color:��ɫ
void GetDiffImage(Mat _img, Mat &_dst, int _threshold, int _color, Rect _roi)
{
	Mat imgRoi = _img(_roi);
	_dst = Mat::zeros(_img.rows, _img.cols, CV_8UC1);
	Mat dstRoi = _dst(_roi);
	vector<Mat> channels;
	split(imgRoi, channels);
	Mat pBImage = channels[0];
	Mat pGImage = channels[1];
	Mat pRImage = channels[2];
	if (_color == blue)
		dstRoi = pBImage - pRImage;
	else if (_color == red)
		dstRoi = pRImage - pGImage;
	else
		cerr << "��ֻ֧�ֺ�ɫ����ɫ";

	inRange(dstRoi, _threshold, 255, dstRoi);
}

//@brief������������
//
void drawGrid(Mat &_img, int _thickness = 1)
{
	for (int i = 1; i < DIVID_COLS; ++i)
		line(_img, Point2f((float)_img.cols / DIVID_COLS*i, 0), Point2f((float)_img.cols / DIVID_COLS*i, _img.rows), Scalar(255, 255, 255), _thickness, CV_AA);
	for (int i = 1; i < DIVID_ROWS; ++i)
		line(_img, Point2f(0, (float)_img.rows / DIVID_ROWS*i), Point2f(_img.cols, (float)_img.rows / DIVID_ROWS*i), Scalar(255, 255, 255), _thickness, CV_AA);
}

//@brief����άƽ����point1��point2��ŷ�Ͼ���
float lineLength(Point2f _point1, Point2f _point2)
{
	return sqrt((_point1.x - _point2.x) * (_point1.x - _point2.x) + (_point1.y - _point2.y) * (_point1.y - _point2.y));
}

void rectangle(Mat &_src, RotatedRect _rec, Scalar _color = Scalar(255, 255, 255), int _thickness = 2)
{
	Point2f vertex[4];
	_rec.points(vertex);
	for (int ni = 0; ni < 4; ni++)
		line(_src, vertex[ni], vertex[(ni + 1) % 4], _color, _thickness, LINE_AA);
}

//@brief��װ���࣬�̳�RotatedRect�࣬��ʾ��һһ��װ��
//@note���൱�ڶ���һ��RotatedRect������һ�����ԣ�likelihood�������ڶ�����װ�׵Ŀ�����
class Armor :public RotatedRect
{
public:
	Armor();
	~Armor();
	double likelihood;
	float area(){ return size.area(); }
	void calcLikelihood(double _possibility){ likelihood *= _possibility; }
	void initial(){ likelihood = 1; vertex.resize(4); }
	vector<Point2f>  vertex;
private:
};

Armor::Armor()
{
	initial();
}
Armor::~Armor()
{
}

//@brief��������ԲѰ��װ�ף����Ի��ơ��������ꡢ׷��
//@code��Armor a��vector<RotatedRect> ellipses����һ��RotatedRect��ʼ��Armor�࣬���Զ����������װ�״���˽�б���
class Armors
{
public:
	Armors(int, int);
	~Armors();
	int number(){ return vRlt.size(); }//@brief�����ؼ�⵽��װ������
	void inputEllipse(vector<RotatedRect> _ellipses);
	vector<Point2f> getTarget(size_t);
	void drawAllArmors(Mat _img, Scalar _color = Scalar(100, 0, 211));
	Mat getROI(Mat, int);
	Rect getROIbox(Mat, int);
	void getRT();
	int getTargetNum(){ return targrtNum; }
	float getScore(int i){ return vRlt[i].likelihood; }
	String error;//û��⵽װ�׵�ԭ��
private:
	int imgWidth;
	int imgHeight;
	int targrtNum;
	vector<Armor> vRlt;//����������װ��
	vector<Rect> targetBox;
	vector<Point2f> target;
};

Armors::Armors(int _width, int _height)
{
	imgWidth = _width;
	imgHeight = _height;
}


//@brief�������⵽����Բ��Ѱ��װ��
void Armors::inputEllipse(vector<RotatedRect> _ellipse)
{
	error.clear();
	Armor armor;
	vRlt.clear();
	int nL, nW; //װ�׵Ŀ�͸�
	if (_ellipse.size() < 2)//С��2����Բ��˵����װ��
		return;

	//ellipse[i]�Ƕȷ�Χ��0~180����ʱ����ת��y��ĽǶ�
	for (unsigned int i = 0; i < _ellipse.size() - 1; i++)
	{
		for (unsigned int j = i + 1; j < _ellipse.size(); j++)
		{
			armor.initial();

			double diffAngle = abs(_ellipse[i].angle - _ellipse[j].angle);
			bool around180 = false;
			if (diffAngle > 180 - T_ANGLE_THRE)
			{
				diffAngle = 180 - diffAngle;
				around180 = true;
			}

			if (diffAngle < T_ANGLE_THRE)
				// 				&& abs(_ellipse[i].size.height - _ellipse[j].size.height) <
				// 				(_ellipse[i].size.height + _ellipse[j].size.height) / T_SIZE_THRE
				// 				&& abs(_ellipse[i].size.width - _ellipse[j].size.width) <
				// 				(_ellipse[i].size.width + _ellipse[j].size.width) / T_SIZE_THRE)
			{
				armor.center = (_ellipse[i].center + _ellipse[j].center) * 0.5;
				if (around180)
				{
					armor.angle = (180 + _ellipse[i].angle + _ellipse[j].angle) * 0.5;
					if (armor.angle > 180)
						armor.angle -= 180;
				}
				else
					armor.angle = (_ellipse[i].angle + _ellipse[j].angle) * 0.5;

				nL = (_ellipse[i].size.height + _ellipse[j].size.height) * 0.5;
				nW = lineLength(_ellipse[i].center, _ellipse[j].center);
				if (nW < 25)                 //����Բ����̫������ȥ
				{
					error = "two ellipses are too close\n";
					continue;
				}
				armor.size = (nL < nW) ? Size(nW, nL) : Size(nL, nW);

				if (armor.size.width > 1.5 * armor.size.height)//װ��̫ϸ������ȥ || ������б����ȥ
				{
					//cout << armor.size << endl;
					error += "Armor is too thin\n";
					continue;
				}
				armor.calcLikelihood(min(_ellipse[i].size.height, _ellipse[j].size.height)
									/ max(_ellipse[i].size.height, _ellipse[j].size.height));
				cout << armor.likelihood << "\t";
				armor.calcLikelihood(min(_ellipse[i].size.width, _ellipse[j].size.width)
					/ max(_ellipse[i].size.width, _ellipse[j].size.width));
				cout << min(_ellipse[i].size.width, _ellipse[j].size.width)	/ max(_ellipse[i].size.width, _ellipse[j].size.width) << "\t";
				armor.calcLikelihood(1 - pow((diffAngle / T_ANGLE_THRE), 2));
				cout << (1 - pow((diffAngle / T_ANGLE_THRE), 2)) << "\t";

// 				double angleOfEllipsesCenter = 90;
// 				if (_ellipse[i].center.y - _ellipse[j].center.y != 0)
// 					angleOfEllipsesCenter = abs(atan((_ellipse[i].center.x - _ellipse[j].center.x) / (_ellipse[i].center.y - _ellipse[j].center.y)) * 180 / CV_PI);
// 				
				Point2f centerSub = (_ellipse[i].center.x > _ellipse[j].center.x) ? (_ellipse[i].center - _ellipse[j].center) : (_ellipse[j].center - _ellipse[i].center);
				double angleOfEllipsesCenter = acos(-centerSub.y / lineLength(_ellipse[i].center, _ellipse[j].center)) * 180 / CV_PI;
				armor.calcLikelihood(1 - pow(abs(90 - abs(angleOfEllipsesCenter - armor.angle)) / 30, 2));

				cout << 1 - pow(abs(90 - abs(angleOfEllipsesCenter - armor.angle)) / 30, 2) << " = ";

				cout << angleOfEllipsesCenter << "   " << armor.angle << endl;
				//cout << armor.likelihood << "\n";
				if (armor.likelihood > 0.3)
				{
					Point2f vertexi[4], vertexj[4];
					_ellipse[i].points(vertexi);
					_ellipse[j].points(vertexj);
					if (_ellipse[i].center.x <= _ellipse[j].center.x)
					{
						armor.vertex[0] = ((vertexi[1] + vertexi[2]) / 2);
						armor.vertex[1] = ((vertexi[0] + vertexi[3]) / 2);
						armor.vertex[2] = ((vertexj[0] + vertexj[3]) / 2);
						armor.vertex[3] = ((vertexj[1] + vertexj[2]) / 2);
					}
					else
					{
						armor.vertex[0] = ((vertexj[1] + vertexj[2]) / 2);
						armor.vertex[1] = ((vertexj[0] + vertexj[3]) / 2);
						armor.vertex[2] = ((vertexi[0] + vertexi[3]) / 2);
						armor.vertex[3] = ((vertexi[1] + vertexi[2]) / 2);
					}
					if ((armor.vertex[0] - armor.center).cross(armor.vertex[1] - armor.center) > 0)
						swap(armor.vertex[0], armor.vertex[1]);
					if ((armor.vertex[2] - armor.center).cross(armor.vertex[3] - armor.center) > 0)
						swap(armor.vertex[2], armor.vertex[3]);

					vRlt.push_back(armor);
				}
				else
					error += "unlike\n";

				//cout<<armor.size<<endl;// cout<<armor.angle<<endl;
			}
			//else
			//cout << "no angle:" << _ellipse[i].angle << "\t" << _ellipse[j].angle <<"\t"<< _ellipse[i].size << "\t" << _ellipse[j].size << endl;
		}
	}

}

bool sortByLikelihood(Armor &a, Armor &b)
{
	return a.likelihood > b.likelihood;//��������
}

//@brief�����ش��Ŀ�����������
vector<Point2f> Armors::getTarget(size_t _num_Of_Target = 1)
{
	target.clear();
	targrtNum = min(_num_Of_Target, vRlt.size());
	sort(vRlt.begin(), vRlt.end(), sortByLikelihood);
	for (size_t i = 0; i < targrtNum; ++i)
	{
		//cout << vRlt[i].likelihood << endl;
		target.push_back(vRlt[i].center);
		targetBox.push_back(vRlt[i].boundingRect());
	}
	return target;
}

//@brief����img�ϻ����ɫΪcolor������װ��
void Armors::drawAllArmors(Mat img, Scalar color)
{
	for (size_t i = 0; i < targrtNum; i++)
	{
		vector<Point2f> points = vRlt[i].vertex;

		putText(img, "A", points[0], FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255));
		putText(img, "B", points[1], FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255));
		putText(img, "C", points[2], FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255));
		putText(img, "D", points[3], FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255));

		for (int ni = 0; ni < 4; ni++)
			line(img, points[ni], points[(ni + 1) % 4], Scalar::all(0), 2, LINE_AA);
	}
}

//@brief���õ�λ��
void Armors::getRT()
{
	for (size_t i = 0; i < targrtNum; i++)
		poseEstimation(vRlt[i].vertex, intrinsic_matrix, distCoeffs, 30, 30);
}
//@brief�����ظ���Ȥ����Mat
Mat Armors::getROI(Mat _img, int i = 0)
{
	return _img(getROIbox(_img, i));
}
//@brief�����ظ���Ȥ����Rect
Rect Armors::getROIbox(Mat _img, int i = 0)
{
	Rect boundingBox = Rect(Point2f(0, 0), Point2f(_img.cols, _img.rows));//bounding of img
	return Rect(target[i] - Point2f(1.2*targetBox[i].width, 1.2*targetBox[i].height), target[i] + Point2f(1.2*targetBox[i].width, 1.2*targetBox[i].height)) & boundingBox;
}
Armors::~Armors()
{
	vector<Armor>  free;
	vRlt.swap(free);
}

//@brief:linux�µĴ���ͨ���࣬����ͨ�����캯��ֱ�Ӵ�һ�����ڣ�����ʼ����Ĭ��9600�����ʣ�8λ���ݣ�����żУ�飬1λֹͣλ��
//            send( )��Ա��������ֱ�ӷ����ַ�����set_opt( )���Ĳ��������ڻ��������������Զ��ر�
//@example:Serialport exp("/dev/ttyUSB0");
//                  exp.set_opt(115200,8,'N',1);
//                  exp.send("1123dd");
class Serialport
{
public:
	Serialport(char *port);
	Serialport();
	~Serialport();
	int open_port(char *port);
	int set_opt(int nSpeed = 9600, int nBits = 8, char nEvent = 'N', int nStop = 1);
	bool send(char *str);
	bool sendAngle(short angleYaw, short anglePitch);
private:
	int fd;
	char tmpchar[6];
	const char *buffer;
};

#ifndef WIN32
Serialport::Serialport(char *port)
{
	open_port(port);
	set_opt();
}
int Serialport::open_port(char *port)
{
	// char *dev[]={"/dev/ttyS0","/dev/ttyS1","/dev/ttyS2"};
	//long vdisable;
	//  fd = open( "/dev/ttyS0", O_RDWR|O_NOCTTY|O_NDELAY);
	// int fd = open( "/dev/ttyUSB0", O_RDWR|O_NOCTTY|O_NDELAY);
	fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
	if (-1 == fd)
	{
		perror("Can't Open Serial Port");
	}
	else
	{
		fcntl(fd, F_SETFL, 0);
	}
	if (fcntl(fd, F_SETFL, 0)<0)
		printf("fcntl failed!\n");
	else
		printf("fcntl=%d\n", fcntl(fd, F_SETFL, 0));
	if (isatty(STDIN_FILENO) == 0)
		printf("standard input is not a terminal device\n");
	else
		printf("isatty success!\n");
	printf("fd-open=%d\n", fd);
	return fd;
}
/*���ô������ԣ�
fd: �ļ�������
nSpeed: ������
nBits: ����λ
nEvent: ��żУ��
nStop: ֹͣλ*/
int Serialport::set_opt(int nSpeed, int nBits, char nEvent, int nStop)
{
	struct termios newtio, oldtio;
	if (tcgetattr(fd, &oldtio) != 0)
	{
		perror("SetupSerial error");
		return -1;
	}
	bzero(&newtio, sizeof(newtio));
	newtio.c_cflag |= CLOCAL | CREAD;
	newtio.c_cflag &= ~CSIZE;
	switch (nBits)
	{
	case 7:
		newtio.c_cflag |= CS7;
		break;
	case 8:
		newtio.c_cflag |= CS8;
		break;
	}
	switch (nEvent)
	{
	case 'O':
		newtio.c_cflag |= PARENB;
		newtio.c_cflag |= PARODD;
		newtio.c_iflag |= (INPCK | ISTRIP);
		break;
	case 'E':
		newtio.c_iflag |= (INPCK | ISTRIP);
		newtio.c_cflag |= PARENB;
		newtio.c_cflag &= ~PARODD;
		break;
	case 'N':
		newtio.c_cflag &= ~PARENB;
		break;
	}
	switch (nSpeed)
	{
	case 2400:
		cfsetispeed(&newtio, B2400);
		cfsetospeed(&newtio, B2400);
		break;
	case 4800:
		cfsetispeed(&newtio, B4800);
		cfsetospeed(&newtio, B4800);
		break;
	case 9600:
		cfsetispeed(&newtio, B9600);
		cfsetospeed(&newtio, B9600);
		break;
	case 115200:
		cfsetispeed(&newtio, B115200);
		cfsetospeed(&newtio, B115200);
		break;
	default:
		cfsetispeed(&newtio, B9600);
		cfsetospeed(&newtio, B9600);
		break;
	}
	if (nStop == 1)
		newtio.c_cflag &= ~CSTOPB;
	else if (nStop == 2)
		newtio.c_cflag |= CSTOPB;
	newtio.c_cc[VTIME] = 0;
	newtio.c_cc[VMIN] = 0;
	tcflush(fd, TCIFLUSH);
	if ((tcsetattr(fd, TCSANOW, &newtio)) != 0)
	{
		perror("com set error");
		return -1;
	}
	printf("Serial port set done!\n");
	return 0;
}
bool Serialport::send(char *str)
{
	buffer = str;
	if (write(fd, buffer, strlen(str))<0)
	{
		perror("write error");
		return false;
	}
	return true;
}
bool Serialport::sendAngle(short _angle1, short _angle2)
{
	memset(tmpchar, 0x00, sizeof(tmpchar));    //��tempchar����
	tmpchar[0] = 0xAA;                                        //��ʼ��־
	tmpchar[1] = _angle1 & 0x00ff;                      //��һ���Ƕȵĵ�8λ
	tmpchar[2] = _angle1 >> 8;                            //��һ���Ƕȵĸ�8λ
	tmpchar[3] = _angle2 & 0x00ff;
	tmpchar[4] = _angle2 >> 8;
	tmpchar[5] = 0xBB;                                        //������־
	if (send(tmpchar))
		return true;
	else
		return false;
}
Serialport:: ~Serialport()
{
	close(fd);
}
#endif // LINUX
