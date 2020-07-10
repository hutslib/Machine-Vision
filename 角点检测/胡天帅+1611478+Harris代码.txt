///////////////////////////////////
///开发环境 vs2015 +opencv 2.13.5//
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
	//请在这里选择一张图片
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
	
	//step1计算角点响应函数
	ChgOfIntensity();
	cout << "step1 successful!" << endl;

	//step2创建滚动条通过滚动条来选择阈值
	Threshold();
	cout << "step2 successful!" << endl;
	

	//系统函数
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
			//计算x方向的偏导数
			Ix[i][j] = (-1*(int)gray.at<uchar>(i -1, j-1) + (int)gray.at<uchar>(i -1, j+1)-2* (int)gray.at<uchar>(i , j-1)+2* (int)gray.at<uchar>(i , j+1) - (int)gray.at<uchar>(i+1, j-1)+ (int)gray.at<uchar>(i + 1, j+1))/8;
			//计算y方向的偏导数
			Iy[i][j] = (-1 * (int)gray.at<uchar>(i - 1, j - 1) + (int)gray.at<uchar>(i + 1, j - 1) - 2 * (int)gray.at<uchar>(i-1, j ) + 2 * (int)gray.at<uchar>(i+1, j ) - (int)gray.at<uchar>(i -1, j + 1) + (int)gray.at<uchar>(i + 1, j + 1)) / 8;
		}
	}

	//生成高斯矩阵2k+1*2k+1大小
	int k = 5;//高斯矩阵窗口半径，其余函数中的k与该值一样
	double sigma = 1.5;
	double sum = 0;
	for (i = 0; i <2*k+1; i++)
	{
		for (j = 0; j <2*k+1; j++)
		{
			//计算高斯矩阵的值
			W[i][j] = (1. / (2.*pi*sigma*sigma))*exp(-((i - k)*(i - k) + (j - k)*(j - k)) / (2.*sigma*sigma));
			sum += W[i][j];
		}
	}
	//高斯矩阵归一化
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
			for (a = -k; a <= k; a++)//选取2k+1*2k+1大小的窗口
			{
				for (b = -k; b <= k; b++)
				{
					A[i][j]+= Ix[i + a][j + b] * Ix[i + a][j + b]* W[k+a][k+b];
					B[i][j] += Iy[i + a][j + b] * Iy[i + a][j + b]*W[k+a][k+b];
					C[i][j] += Ix[i + a][j + b]* Iy[i + a][j + b] * W[k+a][k+b];
				}
			}
			//计算角点响应函数
			R[i][j] = A[i][j] * B[i][j] - C[i][j] * C[i][j] - 0.05*(A[i][j] + B[i][j])*(A[i][j] + B[i][j]);
		}
	}
}
void Threshold()
{
	int th = 100;//定义轨迹条的初始位置
	int threshold_max = 1000;//定义轨迹条的最大值
	char TrackBarName[20];//轨迹条的名字
	sprintf_s(TrackBarName, "阈值*10^3");
	int r = gray.rows;
	int c = gray.cols;
	namedWindow("Harris");
	resizeWindow("Harris", r, c);
	createTrackbar(TrackBarName, "Harris", &th, threshold_max, on_threshold);
	on_threshold(th, 0);//轨迹条的值发生变化时的响应函数
}
Mat dst;
int **d;
//轨迹条表示的阈值发生变化时的响应函数，在该函数中做阈值处理，非极大值抑制，并圈出角点
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
			
			if (R[i][j] < (th * 1000))//th为轨迹条的值，*1000为阈值
				d[i][j] = 0;//如果当前点的灰度值变化小于阈值，则不是角点
			
			else d[i][j] = R[i][j];
		}
	}
	cout << "step2 successful!" << endl;
	//step3非极大值抑制
	NMS();
	cout << "step3 successful!" << endl;
	//step4圈上角点
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
	int m = 3;//非极大值抑制的窗口的半径，其他其他函数中的m也为该值
	for (i = k+m+1; i < gray.rows - k-m-1; i++)
	{
		for (j = k+m+1; j < gray.cols - k-m-1; j++)
		{
			for (a = -m; a <= m; a++)
			{
				for (b = -m; b <= m; b++)
				{
					double data = d[i + a][j + b];
					if (data > d[i][j])//如果在2m+1*2m+1大小的窗口中当前点的灰度变化不是最大的，则不是角点
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