///////////////////////////////////
///�������� vs2015 +opencv 2.13.5//
///////////////////////////////////
////ע�⻬��������ʱֵ��Ҫ̫�󣡣�����������
#include <highgui.h>
#include<opencv2/opencv.hpp>
#include<iostream>
#include<math.h>
#include<algorithm>
using namespace cv;
using namespace std;

Mat src;//ԭʼͼ��
Mat gray;//ԭʼͼ��ת��Ϊ�ĻҶ�ͼ��
void ChgOfIntensity();//����ǵ���Ӧ����
void Threshold();//����������ֵ�Ĺ��������й�����ѡ����ֵ�Ĵ�С
void on_threshold(int,void*);//����������ֵ�����ı�ʱ����Ӧ����
void NMS();//�Ǽ���ֵ����
void circle_corner();//����ǵ�λ��


int main()
{

	//��������ѡ��һ��Ҫ�����ͼƬ
	//src = imread("Miss.bmp");
	src = imread("building.jpg");
	//src = imread("road.jpg");
	//src = imread("test.jpg");
	if (src.empty())
	{
		cout << "picture download fail!!!";
	}
	else
		cout << "picture download successfully!Please wait a few seconds!";
	cvtColor(src, gray, COLOR_BGR2GRAY);
	
	//step1�������ڼ���Ҷ�ֵ�仯��������ǵ���Ӧ����
	ChgOfIntensity();
	cout << "step1 successful!" << endl;

	/*step2������������ͨ�������������ֵ��
	����������ֵ�����ı��ʱ�򣬵���on_threshold()������
	��on_threshold()�����У�����ֵ����������step3�Ǽ���ֵ���ƣ�
	���Ȧ����⵽��step4�ǵ㣬��ʾͼƬ��*/
	Threshold();

	//ϵͳ����
	vector<Point2f> corners;
	Mat img = src.clone();
	goodFeaturesToTrack(gray, corners, 50, 0.05, 5, noArray(), 5, true);
	int i;
	for (i = 0; i < corners.size(); i++)
		circle(img, corners[i], 3, Scalar(0, 0, 255));
	imshow("goodfeaturetotrack", img);
	waitKey(0);
}
int **R;//�ǵ���Ӧ����
void ChgOfIntensity()//����Ҷ�ֵ�仯���ͽǵ���Ӧ����
{
	int i, j, a, b;

	int E[4];
	for (i = 0; i < 4; i++)
		E[i] = 0;

	R = new int*[gray.rows];
	for (i = 0; i < gray.rows; i++)
		R[i] = new int[gray.cols];

	int k = 2;//���ڵİ뾶�����������е�k��Ϊ��ֵ
	for (i = k; i < src.rows - k-1; i++)
	{
		for (j = k+1; j < src.cols - k-1; j++)
		{
			E[3] = E[2] = E[1] = E[0] = 0;
			for (a = -k; a <= k; a++)//ѡȡ2k+1*2k+1��С�Ĵ���
			{
				for (b = -k; b <= k; b++)
				{
					//��1��0������
					E[0] += pow((int)gray.at<uchar>(i + a, j + b) - (int)gray.at<uchar>(i + a, j + b + 1), 2);
					//��1��1������
					E[1] += pow((int)gray.at<uchar>(i + a, j + b) - (int)gray.at<uchar>(i + a + 1, j + b + 1), 2);
					//(0,1)����
					E[2] += pow((int)gray.at<uchar>(i + a, j + b) - (int)gray.at<uchar>(i + a + 1, j + b), 2);
					//(-1,1)����
					E[3] += pow((int)gray.at<uchar>(i + a, j + b) - (int)gray.at<uchar>(i + a + 1, j + b - 1), 2);
				}
			}
			//����ǵ���Ӧ������Ϊ�ĸ������ϻҶ�ֵ�仯����Сֵ,R=min{E}
			int a;
			a = E[0] < E[1] ? E[0]:E[1];
			a = a < E[2] ? a:E[2];
			a = a < E[3] ? a : E[3];
			R[i][j] = a;
		}
	}
}

void Threshold()//����������
{
	int th = 18;//����켣���ĳ�ʼλ��
	int threshold_max = 1000;//����켣�������ֵ
	char TrackBarName[20];//�켣��������
	sprintf_s(TrackBarName, "��ֵ*10^3");
	int r = gray.rows;
	int c = gray.cols;
	namedWindow("Moravec");//�켣��������������
	resizeWindow("Moravec", r, c);
	createTrackbar(TrackBarName, "Moravec", &th, threshold_max, on_threshold);
	on_threshold(th,0);//��������֮�仯�˵���Ӧ����
}

Mat dst;//����Ҫ��ʾ��ͼƬ
int **d;//��Ǹõ��Ƿ�Ϊ�ǵ�
void on_threshold(int th ,void*)//�켣������Ӧ�������ڸú���������ֵ����ͷǼ���ֵ���ƣ���Ȧ���ǵ㣬��ʾͼƬ��
{
	int i, j;
	dst = src.clone();
	d = new int*[gray.rows];
	for (i = 0; i < gray.rows; i++)
		d[i] = new int[gray.cols];

	int k = 2;
	for (i = k; i < gray.rows-k-1; i++)
	{
		for (j = k+1; j < gray.cols - k-1; j++)
		{
			
			if (R[i][j] < (th*1000))//thΪ�켣����ֵ��*1000Ϊ��ֵ
				d[i][j]= 0;//�����ǰ��ĻҶ�ֵ�仯С����ֵ�����ǽǵ�
		    
			else d[i][j] = R[i][j];
		}
	}
	cout << "step2 successful!" << endl;
	//step3�Ǽ���ֵ����
	NMS();
	cout << "step3 successful!" << endl;

	//step4Ȧ�Ͻǵ�
   circle_corner();
	//cout << "step4 successful!" << endl;

	imshow("Moravec", dst);

	for (i = 0; i < gray.rows; i++)
		delete[]d[i];
	delete[]d;
}
void NMS()//�Ǽ���ֵ����
{
	int i, j;
	int a, b;

	int k = 2;
	int m = 6;//�Ǽ���ֵ���ƵĴ��ڵİ뾶���������������е�mҲΪ��ֵ
	for (i = k+m; i < gray.rows-k-m-1; i++)
	{
		for (j = k+m+1; j < gray.cols - k-m-1; j++)
		{
			for (a = -m; a <= m; a++)
			{
				for (b = -m; b <= m; b++)
				{
					double data = d[i + a][j + b];
					if (data > d[i][j])//�����2m+1*2m+1��С�Ĵ����е�ǰ��ĻҶȱ仯�������ģ����ǽǵ�
						d[i][j] = 0;
				}
			}

		}
	}
}

void circle_corner()//Ȧ���ǵ�
{
	int i, j;
	int k = 2, m = 6;
	for (i = k+m; i < gray.rows - k-m-1; i++)
	{
		for (j = k+m+1; j < gray.cols - k-m-1; j++)
		{
			
			if (d[i][j] != 0)
			{
				circle(dst, Point(j, i), 3, Scalar(0, 0, 255));
			}
		}
	}
}