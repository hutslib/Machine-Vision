///////////////////////////////////
///开发环境 vs2015 +opencv 2.13.5//
///////////////////////////////////
////注意滑动滚动条时值不要太大！！！！！！！
#include <highgui.h>
#include<opencv2/opencv.hpp>
#include<iostream>
#include<math.h>
#include<algorithm>
using namespace cv;
using namespace std;

Mat src;//原始图像
Mat gray;//原始图像转变为的灰度图像
void ChgOfIntensity();//计算角点响应函数
void Threshold();//创建设置阈值的滚动条，有滚动条选择阈值的大小
void on_threshold(int,void*);//当滚动条的值发生改变时的响应函数
void NMS();//非极大值抑制
void circle_corner();//标出角点位置


int main()
{

	//请在这里选择一张要处理的图片
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
	
	//step1滑动窗口计算灰度值变化，并计算角点响应函数
	ChgOfIntensity();
	cout << "step1 successful!" << endl;

	/*step2创建滚动条，通过滚动条设低阈值，
	当滚动条的值发生改变的时候，调用on_threshold()函数，
	在on_threshold()函数中，做阈值处理，并进行step3非极大值抑制，
	最后圈出检测到的step4角点，显示图片。*/
	Threshold();

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
int **R;//角点响应函数
void ChgOfIntensity()//计算灰度值变化，和角点响应函数
{
	int i, j, a, b;

	int E[4];
	for (i = 0; i < 4; i++)
		E[i] = 0;

	R = new int*[gray.rows];
	for (i = 0; i < gray.rows; i++)
		R[i] = new int[gray.cols];

	int k = 2;//窗口的半径，其他函数中的k均为该值
	for (i = k; i < src.rows - k-1; i++)
	{
		for (j = k+1; j < src.cols - k-1; j++)
		{
			E[3] = E[2] = E[1] = E[0] = 0;
			for (a = -k; a <= k; a++)//选取2k+1*2k+1大小的窗口
			{
				for (b = -k; b <= k; b++)
				{
					//（1，0）方向
					E[0] += pow((int)gray.at<uchar>(i + a, j + b) - (int)gray.at<uchar>(i + a, j + b + 1), 2);
					//（1，1）方向
					E[1] += pow((int)gray.at<uchar>(i + a, j + b) - (int)gray.at<uchar>(i + a + 1, j + b + 1), 2);
					//(0,1)方向
					E[2] += pow((int)gray.at<uchar>(i + a, j + b) - (int)gray.at<uchar>(i + a + 1, j + b), 2);
					//(-1,1)方向
					E[3] += pow((int)gray.at<uchar>(i + a, j + b) - (int)gray.at<uchar>(i + a + 1, j + b - 1), 2);
				}
			}
			//计算角点响应函数，为四个方向上灰度值变化的最小值,R=min{E}
			int a;
			a = E[0] < E[1] ? E[0]:E[1];
			a = a < E[2] ? a:E[2];
			a = a < E[3] ? a : E[3];
			R[i][j] = a;
		}
	}
}

void Threshold()//创建滚动条
{
	int th = 18;//定义轨迹条的初始位置
	int threshold_max = 1000;//定义轨迹条的最大值
	char TrackBarName[20];//轨迹调的名字
	sprintf_s(TrackBarName, "阈值*10^3");
	int r = gray.rows;
	int c = gray.cols;
	namedWindow("Moravec");//轨迹条依附窗口名字
	resizeWindow("Moravec", r, c);
	createTrackbar(TrackBarName, "Moravec", &th, threshold_max, on_threshold);
	on_threshold(th,0);//滚动条的之变化了的响应函数
}

Mat dst;//最重要显示的图片
int **d;//标记该点是否为角点
void on_threshold(int th ,void*)//轨迹条的响应函数，在该函数中做阈值处理和非极大值抑制，并圈出角点，显示图片。
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
			
			if (R[i][j] < (th*1000))//th为轨迹条的值，*1000为阈值
				d[i][j]= 0;//如果当前点的灰度值变化小于阈值，则不是角点
		    
			else d[i][j] = R[i][j];
		}
	}
	cout << "step2 successful!" << endl;
	//step3非极大值抑制
	NMS();
	cout << "step3 successful!" << endl;

	//step4圈上角点
   circle_corner();
	//cout << "step4 successful!" << endl;

	imshow("Moravec", dst);

	for (i = 0; i < gray.rows; i++)
		delete[]d[i];
	delete[]d;
}
void NMS()//非极大值抑制
{
	int i, j;
	int a, b;

	int k = 2;
	int m = 6;//非极大值抑制的窗口的半径，其他其他函数中的m也为该值
	for (i = k+m; i < gray.rows-k-m-1; i++)
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

void circle_corner()//圈出角点
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