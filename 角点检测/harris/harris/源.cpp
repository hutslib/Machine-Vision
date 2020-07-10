///////////////////////////////////
///�������� vs2015 +opencv 2.13.5//
///////////////////////////////////
#include <highgui.h>
#include<opencv2/opencv.hpp>
#include<iostream>
#include<math.h>
#include<algorithm>
#define pi 3.14159
using namespace cv;
using namespace std;
Mat src;
Mat gray;
void ChgOfIntensity();
void Threshold();
void on_threshold(int, void*);
void NMS();
void circle_corner();
int main()
{
	//��������ѡ��һ��ͼƬ
	//src = imread("Miss.bmp");
	src = imread("building.jpg");
	//src = imread("road.jpg");
	//src = imread("test.jpg");
	if (src.empty())
	{
		cout << "picture download fail!!!" << endl;
	}
	else
		cout << "picture download successfully!" << endl;
	cvtColor(src, gray, COLOR_BGR2GRAY);
	
	//step1����ǵ���Ӧ����
	ChgOfIntensity();
	cout << "step1 successful!" << endl;

	//step2����������ͨ����������ѡ����ֵ
	Threshold();
	cout << "step2 successful!" << endl;
	

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
double **R;
void ChgOfIntensity()
{
	int i, j;
	R = new double*[gray.rows];
	for (i = 0; i < gray.rows; i++)
		R[i] = new double[gray.cols];
	int **Ix, **Iy;
	double **A, **B, **C;
	double **W;
	Ix = new int*[gray.rows];
	for (i = 0; i < gray.rows; i++)
		Ix[i] = new int[gray.cols];
	Iy = new int*[gray.rows];
	for (i = 0; i < gray.rows; i++)
		Iy[i] = new int[gray.cols];
	A = new double*[gray.rows];
	for (i = 0; i < gray.rows; i++)
		A[i] = new double[gray.cols];
	B = new double*[gray.rows];
	for (i = 0; i < gray.rows; i++)
		B[i] = new double[gray.cols];
	C = new double*[gray.rows];
	for (i = 0; i < gray.rows; i++)
		C[i] = new double[gray.cols];
	W = new double*[gray.rows];
	for (i = 0; i < gray.rows; i++)
		W[i] = new double[gray.cols];

	for (i = 1; i < gray.rows - 1; i++)
	{
		for (j = 1; j < gray.cols - 1; j++)
		{
			//����x�����ƫ����
			Ix[i][j] = (-1*(int)gray.at<uchar>(i -1, j-1) + (int)gray.at<uchar>(i -1, j+1)-2* (int)gray.at<uchar>(i , j-1)+2* (int)gray.at<uchar>(i , j+1) - (int)gray.at<uchar>(i+1, j-1)+ (int)gray.at<uchar>(i + 1, j+1))/8;
			//����y�����ƫ����
			Iy[i][j] = (-1 * (int)gray.at<uchar>(i - 1, j - 1) + (int)gray.at<uchar>(i + 1, j - 1) - 2 * (int)gray.at<uchar>(i-1, j ) + 2 * (int)gray.at<uchar>(i+1, j ) - (int)gray.at<uchar>(i -1, j + 1) + (int)gray.at<uchar>(i + 1, j + 1)) / 8;
		}
	}

	//���ɸ�˹����2k+1*2k+1��С
	int k = 5;//��˹���󴰿ڰ뾶�����ຯ���е�k���ֵһ��
	double sigma = 1.5;
	double sum = 0;
	for (i = 0; i <2*k+1; i++)
	{
		for (j = 0; j <2*k+1; j++)
		{
			//�����˹�����ֵ
			W[i][j] = (1. / (2.*pi*sigma*sigma))*exp(-((i - k)*(i - k) + (j - k)*(j - k)) / (2.*sigma*sigma));
			sum += W[i][j];
		}
	}
	//��˹�����һ��
	for (i = 0; i < 2 * k + 1; i++)
	{
		for (j = 0; j < 2 * k + 1; j++)
		{
			W[i][j] /= sum;
		}
	}
	int a, b;
	for (i = k+1; i < gray.rows - k-1; i++)
	{
		for (j = k+1; j < gray.cols - k-1; j++)
		{
			A[i][j] = 0;
			B[i][j] = 0;
			C[i][j] = 0;
			for (a = -k; a <= k; a++)//ѡȡ2k+1*2k+1��С�Ĵ���
			{
				for (b = -k; b <= k; b++)
				{
					A[i][j]+= Ix[i + a][j + b] * Ix[i + a][j + b]* W[k+a][k+b];
					B[i][j] += Iy[i + a][j + b] * Iy[i + a][j + b]*W[k+a][k+b];
					C[i][j] += Ix[i + a][j + b]* Iy[i + a][j + b] * W[k+a][k+b];
				}
			}
			//����ǵ���Ӧ����
			R[i][j] = A[i][j] * B[i][j] - C[i][j] * C[i][j] - 0.05*(A[i][j] + B[i][j])*(A[i][j] + B[i][j]);
		}
	}
}
void Threshold()
{
	int th = 100;//����켣���ĳ�ʼλ��
	int threshold_max = 1000;//����켣�������ֵ
	char TrackBarName[20];//�켣��������
	sprintf_s(TrackBarName, "��ֵ*10^3");
	int r = gray.rows;
	int c = gray.cols;
	namedWindow("Harris");
	resizeWindow("Harris", r, c);
	createTrackbar(TrackBarName, "Harris", &th, threshold_max, on_threshold);
	on_threshold(th, 0);//�켣����ֵ�����仯ʱ����Ӧ����
}
Mat dst;
int **d;
//�켣����ʾ����ֵ�����仯ʱ����Ӧ�������ڸú���������ֵ�����Ǽ���ֵ���ƣ���Ȧ���ǵ�
void on_threshold(int th, void*)
{
	int i, j;
	dst = src.clone();
	int k = 5;
	d = new int*[gray.rows];
	for (i = 0; i < gray.rows; i++)
		d[i] = new int[gray.cols];
	for (i = k+1; i < gray.rows - k-1; i++)
	{
		for (j = k+1; j < gray.cols - k-1; j++)
		{
			
			if (R[i][j] < (th * 1000))//thΪ�켣����ֵ��*1000Ϊ��ֵ
				d[i][j] = 0;//�����ǰ��ĻҶ�ֵ�仯С����ֵ�����ǽǵ�
			
			else d[i][j] = R[i][j];
		}
	}
	cout << "step2 successful!" << endl;
	//step3�Ǽ���ֵ����
	NMS();
	cout << "step3 successful!" << endl;
	//step4Ȧ�Ͻǵ�
	circle_corner();
	cout << "step4 successful!" << endl;
	imshow("Harris", dst);
	for (i = 0; i < gray.rows; i++)
		delete[]d[i];
	delete[]d;
}
void NMS()
{
	int i, j;
	int a, b;
	int k = 5;
	int m = 3;//�Ǽ���ֵ���ƵĴ��ڵİ뾶���������������е�mҲΪ��ֵ
	for (i = k+m+1; i < gray.rows - k-m-1; i++)
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
void circle_corner()
{
	int i, j;
	int k = 5, m = 3;
	for (i = k+m+1; i < gray.rows - k-m-1; i++)
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