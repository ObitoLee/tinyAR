#include "markerRecognizer.h"

Mat_<float> intrinsic_matrix = (Mat_<float>(3, 3) << 705.1, 0, 294.79,
	0, 700, 260.98,
	0, 0, 1);
Mat_<float> distCoeffs = (Mat_<float>(5, 1) << -0.04312539, -0.005736560, 0, 0, 0);

//@brief����άƽ����point1��point2��ŷ�Ͼ���
float lineLength(Point2f _point1, Point2f _point2)
{
	return sqrt((_point1.x - _point2.x) * (_point1.x - _point2.x) + (_point1.y - _point2.y) * (_point1.y - _point2.y));
}

//@brief��������ת����
void rectangle(Mat &_src, RotatedRect _rec, Scalar _color = Scalar(255, 255, 255), int _thickness = 2)
{
	Point2f vertex[4];
	_rec.points(vertex);
	for (int ni = 0; ni < 4; ni++)
		line(_src, vertex[ni], vertex[(ni + 1) % 4], _color, _thickness, CV_AA);//LINE_AA
}

//@brief����ʼ����
MakerRecognizer::MakerRecognizer(int width, int height, int zoom, size_t targetNum)
{
	ZOOM_FACTOR = zoom;
	imgWidth = width * zoom;
	imgHeight = height * zoom;
	
	boundingBox = Rect(Point2f(0, 0), Point2f(imgWidth, imgHeight));
	roiImg = boundingBox;
	MaxTargetNum = targetNum;

	vaildFrames.resize(MaxTargetNum);
	int64 startTime = getTickCount();
}

void MakerRecognizer::find(Mat &img, int t, int color)
{
	frames++;
	outputImg = img;
	resize(outputImg, outputImg, Size(imgWidth, imgHeight));//�ȱ�����С
	GetDiffImage(outputImg, imgThresholded, t, color, roiImg);//��ɫ
	preProcess(imgThresholded);

	vector<vector<Point> > contours;
	vector<RotatedRect> ellipses;//������������Բ
	findContours(imgThresholded, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);
	//drawContours(imgThresholded, contours, -1, Scalar(255, 255, 255));

	for (auto iterContour = contours.begin(); iterContour != contours.end(); ++iterContour)
	{
		if (iterContour->size() > minContour && iterContour->size() < maxContour)//ɸѡ�����ϵĵ����100������
		{
			RotatedRect s = fitEllipse(*iterContour);//�����Բ
			if (s.size.height * s.size.width > 18000 * ZOOM_FACTOR * ZOOM_FACTOR
				|| (s.size.height < 1.2 * s.size.width) 
				|| (s.size.height > s.size.width * 15))
			{
				cout << "size don't cater to standard:  " <<s.size.height << "\t" << s.size.width << endl;
				continue;
			}
			ellipses.push_back(s);
			ellipse(outputImg, s, Scalar(255, 0, 166), 2);
		}
	}

	findTargetsByEllispses(ellipses);//���뽫�⵽����Բ

	for (int i = 0; i < findTargrtNum; i++)//��⵽Ŀ��
	{
		vaildFrames[i]++;
		//cout << getRT(i).t.at<double>(2) << endl;
		//putText(imgOriginal, to_string(getScore(i)), target[i], FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255));
		if (MaxTargetNum == 1)
			roiImg = getROIbox(0);
	}

	if (findTargrtNum == 0)
	{
		cout << "Ŀ�궪ʧ~~~" << endl;
		putText(outputImg, "Looking for the enemy.......", Point(90 * ZOOM_FACTOR, 90 * ZOOM_FACTOR), FONT_HERSHEY_PLAIN, 2 * ZOOM_FACTOR, Scalar(0, 0, 255), 2, 8);
		roiImg = boundingBox;
	}
	cout << "\n======================\n";
// ellipses.clear();contours.swap(vector<vector<Point>>());
}

MakerRecognizer::~MakerRecognizer()
{
	vector<Armor>  free;
	vRlt.swap(free);

	int64 spendTime = getTickCount() - startTime;
	cout << frames * 1000 / (1000 * spendTime / getTickFrequency()) << "fps" << "\n";
	cout << "total��\t" << frames << endl;
	for (int i = 0; i < MaxTargetNum; ++i)
		cout << i + 1 << ":\t" << vaildFrames[i] << endl;
}

bool sortByLikelihood(Armor &a, Armor &b)
{
	return a.likelihood > b.likelihood;//��������
}

void MakerRecognizer::findTargetsByEllispses(vector<RotatedRect> _ellipses)
{
	findTargrtNum = 0;
	if (_ellipses.size() < 2)//С��2����Բ��˵����
		return;
	error.clear();
	vRlt.clear();
	int nL, nW; //��͸�
	Armor armor;

	//ellipse[i]�Ƕȷ�Χ��0~180����ʱ����ת��y��ĽǶ�
	for (unsigned int i = 0; i < _ellipses.size() - 1; i++)
	{
		for (unsigned int j = i + 1; j < _ellipses.size(); j++)
		{
			armor.initial();

			double diffAngle = abs(_ellipses[i].angle - _ellipses[j].angle);
			bool around180 = false;
			if (diffAngle > 180 - T_ANGLE_THRE)
			{
				diffAngle = 180 - diffAngle;
				around180 = true;
			}

			if (diffAngle < T_ANGLE_THRE && abs(_ellipses[i].size.height - _ellipses[j].size.height) <	(_ellipses[i].size.height + _ellipses[j].size.height) / T_SIZE_THRE && abs(_ellipses[i].size.width - _ellipses[j].size.width) <	(_ellipses[i].size.width + _ellipses[j].size.width) / T_SIZE_THRE)
			{
				armor.center = (_ellipses[i].center + _ellipses[j].center) * 0.5;
				if (around180)
				{
					armor.angle = (180 + _ellipses[i].angle + _ellipses[j].angle) * 0.5;
					if (armor.angle > 180)
						armor.angle -= 180;
				}
				else
					armor.angle = (_ellipses[i].angle + _ellipses[j].angle) * 0.5;

				nL = (_ellipses[i].size.height + _ellipses[j].size.height) * 0.5;
				nW = lineLength(_ellipses[i].center, _ellipses[j].center);
				if (nW < 15)  //����Բ����̫������ȥ
				{
					error = "two ellipses are too close\n";
					continue;
				}
				armor.size = (nL < nW) ? Size(nW, nL) : Size(nL, nW);

				if (armor.size.width > 4 * armor.size.height)//װ��̫ϸ������ȥ || ������б����ȥ
				{
					cout << armor.size << endl;
					error += "Armor is too thin\n";
					continue;
				}
				armor.calcLikelihood(min(_ellipses[i].size.height, _ellipses[j].size.height)
					/ max(_ellipses[i].size.height, _ellipses[j].size.height));
				cout << armor.likelihood << "\t";
				armor.calcLikelihood(min(_ellipses[i].size.width, _ellipses[j].size.width)
					/ max(_ellipses[i].size.width, _ellipses[j].size.width));
				cout << min(_ellipses[i].size.width, _ellipses[j].size.width) / max(_ellipses[i].size.width, _ellipses[j].size.width) << "\t";
				armor.calcLikelihood(1 - pow((diffAngle / T_ANGLE_THRE), 3));
				cout << (1 - pow((diffAngle / T_ANGLE_THRE), 3)) << "\t";

				Point2f centerSub = (_ellipses[i].center.x > _ellipses[j].center.x) ? (_ellipses[i].center - _ellipses[j].center) : (_ellipses[j].center - _ellipses[i].center);
				double angleOfEllipsesCenter = acos(-centerSub.y / lineLength(_ellipses[i].center, _ellipses[j].center)) * 180 / CV_PI;
				armor.calcLikelihood(1 - pow(abs(90 - abs(angleOfEllipsesCenter - armor.angle)) / 70, 3));

				cout << 1 - pow(abs(90 - abs(angleOfEllipsesCenter - armor.angle)) / 70, 3) << " = ";
				//cout << angleOfEllipsesCenter << "   " << armor.angle << endl;
				cout << armor.likelihood << "\n";
				if (armor.likelihood > 0.3)
				{
					Point2f vertexi[4], vertexj[4];
					_ellipses[i].points(vertexi);
					_ellipses[j].points(vertexj);
					if (_ellipses[i].center.x <= _ellipses[j].center.x)
					{
						armor.vertex[0] = ((vertexi[1] + vertexi[2]) *0.5);
						armor.vertex[1] = ((vertexi[0] + vertexi[3]) *0.5);
						armor.vertex[2] = ((vertexj[0] + vertexj[3]) *0.5);
						armor.vertex[3] = ((vertexj[1] + vertexj[2]) *0.5);
					}
					else
					{
						armor.vertex[0] = ((vertexj[1] + vertexj[2]) *0.5);
						armor.vertex[1] = ((vertexj[0] + vertexj[3]) *0.5);
						armor.vertex[2] = ((vertexi[0] + vertexi[3]) *0.5);
						armor.vertex[3] = ((vertexi[1] + vertexi[2]) *0.5);
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
			// 			else
			// 				cout << "no angle:" << _ellipse[i].angle << "\t" << _ellipse[j].angle <<"\t"<< _ellipse[i].size << "\t" << _ellipse[j].size << endl;
		}
	}

	targetPos.clear();
	findTargrtNum = min(MaxTargetNum, vRlt.size());
	sort(vRlt.begin(), vRlt.end(), sortByLikelihood);
	for (size_t i = 0; i < findTargrtNum; ++i)
	{
		//cout << vRlt[i].likelihood << endl;
		targetPos.push_back(vRlt[i].center);
		targetBox.push_back(vRlt[i].boundingRect());
	}
}


poseMat MakerRecognizer::getRT(int i)
{
	if (i < findTargrtNum)
		return poseEstimation(vRlt[i].vertex, intrinsic_matrix, distCoeffs, 30, 30);
	else
	{
		cerr << "The parameter "<< i << "of 'getRT(int)' is beyond the region!Return pose of the first target\n";
		return poseEstimation(vRlt[0].vertex, intrinsic_matrix, distCoeffs, 30, 30);
	}
}

Rect MakerRecognizer::getROIbox(int i)
{
	return Rect(targetPos[i] - Point2f(1.2*targetBox[i].width, 1.2*targetBox[i].height), targetPos[i] + Point2f(1.2*targetBox[i].width, 1.2*targetBox[i].height)) & boundingBox;
}


poseMat MakerRecognizer::poseEstimation(vector<Point2f> _vertex, Mat_<double> A, Mat_<float> d, float a, float b)
{
	vector<Point3f> world;
	world.push_back({ -a, b, 0 });
	world.push_back({ -a, -b, 0 });
	world.push_back({ a, -b, 0 });
	world.push_back({ a, b, 0 });

	Mat r, t, rM;

	solvePnP(world, _vertex, A, d, r, t, false); //3.0��SOLVEPNP_ITERATIVE
	// 	solvePnP(world, _vertex, A, d, r, t, false, CV_P3P);
	// 	solvePnP(world, _vertex, A, d, r, t, false, CV_EPNP);
	Rodrigues(r, rM);
	poseMat result = { rM, t };
	return result;
}

//@brief��ͼƬԤ�������ͣ���ʴ���˲���
//@param��img is 8UC1
void MakerRecognizer::preProcess(Mat& _img)
{
	Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
	Mat element2 = getStructuringElement(MORPH_RECT, Size(5, 5));
	dilate(_img, _img, element, Point(-1, -1), 2);
	erode(_img, _img, element2, Point(-1, -1), 1);
	//blur( _img, _img, Size(3, 3));
}

//@brief����ɫɸѡ
//@param��img:����ͼ��8UC3��
//@param��dst:���ͼ��8UC1��
//@param��threshold:��ֵ
//@param��color:��ɫ
void MakerRecognizer::GetDiffImage(Mat _img, Mat &_dst, int _threshold, int _color, Rect _roi)
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

void MakerRecognizer::drawARcube(float a, float b)
{
	vector<Point3f> worldTop;
	worldTop.push_back({ a, b , a });
	worldTop.push_back({ a, -b, a });
	worldTop.push_back({ a, -b, a });
	worldTop.push_back({ a, b, a });

	for (int i = 0; i < findTargrtNum; i++)//��⵽Ŀ��
	{
		vector<Point2f> imgPointBottom = getVertex(i);
		vector<Point2f> imgPointTop;
		projectPoints(worldTop, getRT(i).rM, getRT(i).t, intrinsic_matrix, distCoeffs, imgPointTop);

		for (int ni = 0; ni < 4; ni++)
		{
			line(outputImg, imgPointBottom[ni], imgPointBottom[(ni + 1) % 4], Scalar(0, 255, 255), 2, CV_AA);
			line(outputImg, imgPointTop[ni], imgPointBottom[ni], Scalar(0, 255, 255), 2, CV_AA);
		}
		for (int ni = 0; ni < 4; ni++)
			line(outputImg, imgPointTop[ni], imgPointTop[(ni + 1) % 4], Scalar(0, 0, 255), 2, CV_AA);
	}
}

void MakerRecognizer::drawAllMarkers(Scalar _color)
{
	for (size_t i = 0; i < findTargrtNum; i++)
	{
		//vector<Point2f> points = vRlt[i].vertex;
		vector<Point2f> points = getVertex(i);
		putText(outputImg, "A", points[0], FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255));
		putText(outputImg, "B", points[1], FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255));
		putText(outputImg, "C", points[2], FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255));
		putText(outputImg, "D", points[3], FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255));

		for (int ni = 0; ni < 4; ni++)
			line(outputImg, points[ni], points[(ni + 1) % 4], _color, 2, CV_AA);
	}
}

void MakerRecognizer::drawARImg(Mat showImg)
{
	int i = 0;
	if (findTargrtNum != 0)
	{
		int img_width = showImg.cols;
		int img_height = showImg.rows;
		vector<Point2f> corners(4);
		corners[0] = Point2f(0, 0);
		corners[1] = Point2f(0, img_height - 1);
		corners[2] = Point2f(img_width - 1, img_height - 1);
		corners[3] = Point2f(img_width - 1, 0);
		//Mat warpImg = Mat::zeros(showImg.rows, showImg.cols, showImg.type());

		vector<Point3f> worldTop;
		worldTop.push_back({ -45, 0, 90 });
		worldTop.push_back({ -45, -30, 15 });
		worldTop.push_back({ 45, -30, 15 });
		worldTop.push_back({ 45, 0, 90.0 });

		vector<Point2f> imgPointBottom = getVertex(i);
		vector<Point2f> imgPointTop;
		projectPoints(worldTop, getRT(i).rM, getRT(i).t, intrinsic_matrix, distCoeffs, imgPointTop);

		Mat transform = getPerspectiveTransform(corners, imgPointTop);

		Mat out = Mat::zeros(imgWidth, imgHeight, CV_8UC3);
		//cv::Size size(img_width, img_height);
		warpPerspective(showImg, out, transform, outputImg.size(), 1, 0, 0);

		Mat mask;
		cvtColor(out, mask, CV_BGRA2GRAY);
		out.copyTo(outputImg, mask);
	}
}

// /*@brief������ת����ͼ�����굽������꣩
// @param�������ά�������
// @param��imgXY is �����ά��������
// @param��intrinsic_matrix is �ڲξ���**/
// Point3f img2camera(Point2f _imgXY, Mat_<float> _intrinsic_matrix)
// {
// 	Mat_<float> target2D = (Mat_<float>(3, 1) << _imgXY.x, _imgXY.y, 1);
// 	Mat_<float> target3D = _intrinsic_matrix.inv() * target2D;
// 	Point3f cameraXYZ;
// 	cameraXYZ.x = target3D(0);
// 	cameraXYZ.y = target3D(1);
// 	cameraXYZ.z = target3D(2);
// 	return cameraXYZ;
// }