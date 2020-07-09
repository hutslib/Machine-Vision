
//环境：
//vs2015 + opencv2.4.13.5

#include <iostream>
#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>  
#include<opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include<math.h>

using namespace std;
using namespace cv;

void gradient();//自己编写的用来计算梯度的大小与方向的函数
void NMS();//自己编写的非极大值抑制函数
void createThresholdTrackbar();//创建选择阈值的滚动条函数
void on_change(int, void*);//当滚动条的值改变时的响应函数，做双阈值处理

Mat imgsrc;//原始图像转化成的灰度图
Mat img_gaussian;//高斯滤波后的图像
Mat grad;//梯度大小
Mat angle;//梯度的方向，0-360°
Mat nms;//非极大值抑制后的图像

int main()
{
	//step1:读入图片

	//请选择一张图片
	// Mat img = imread("Miss.bmp");
	Mat img = imread("building.jpg");
	//Mat img = imread("road.jpg");

	if (!img.data) cout << "picture load fail!" << endl;
	else cout << "step1successful!" << endl;
	//将原始图转化为灰度图
	cvtColor(img, imgsrc, COLOR_BGR2GRAY);

	//step2:对图像进行高斯滤波
	GaussianBlur(imgsrc, img_gaussian, Size(3, 1), 1, 1);
	cout << "step2successful" << endl;
	imshow("高斯", img_gaussian);

	//step3:对滤波后的图像求梯度的大小和方向
	gradient();//调用自己编写的gradient函数，定义在后面
	cout << "step3successful" << endl;

	//step4:非极大值抑制
	NMS();//调用自己编写的NMS函数，定义在后面
	cout << "step4successful" << endl;

	////step5:双阈值检测，通滚动条输入控制高低阈值的大小
	createThresholdTrackbar();//创建选择双阈值的滚动条，滚动条的值改变后响应编写的void on_change(int, void*)函数，进行双阈值处理
	cout << "step5successful" << endl;

	///展示系统自带的canny边缘检测的结果
	Mat canny;
	Canny(imgsrc, canny, 50, 100);
	imshow("系统canny", canny);
	waitKey(0);
}
//求梯度
void gradient()
{
	//对高斯滤波后的图像，利用sobel算子分别求得x,y方向的差分，结果分别保存在sobelx,sobely中
	Mat sobelx;
	Mat sobely;
	Sobel(img_gaussian, sobelx, CV_32F, 1, 0, 3);
	Sobel(img_gaussian, sobely, CV_32F, 0, 1, 3);

	// 求得梯度和方向，梯度的大小最终保存在grand中，方向保存在angle中  
	Mat g;
	cartToPolar(sobelx, sobely, g, angle, 1);//利用笛卡尔坐标转换成极坐标得到梯度的大小和方向
	convertScaleAbs(g, grad);//g的值可能超过255，需要转化成范围在0-255内的8位无符号数 
	imshow("grad", grad);
}
//非极大值抑制
void NMS()
{

	nms = grad;
	int i, j;

	//动态创建标记矩阵
	int **badge = new int *[nms.rows];
	for (i = 0; i < nms.rows; i++)
		badge[i] = new int[nms.cols];
	for (i = 0; i < nms.rows; i++)
	{
		for (j = 0; j < nms.cols; j++)
			badge[i][j] = -2;
	}
	//将360度平均分成8分，求出梯度方向上的前后两个点，比较该点的梯度值是否为此三个点中最大的一个
  /*   2   2
    3         1
  3             1
 4               0
 4               0
  5             7
   5          7
       6   6   */
	for (i = 1; i < nms.rows - 1; i++)
	{
		for (j = 1; j < nms.cols - 1; j++)
		{
			int data = (int)grad.at<uchar>(i, j);
			double ang = (double)angle.at<float>(i, j);
			int pre, next;
			//根据梯度求前后两个点
			if (ang >= 0 && ang < 22.5 || ang >= 337.5 && ang < 360 || ang >= 157.5 && ang < 202.5)
			{
				pre = (int)grad.at<uchar>(i, j - 1);
				next = (int)grad.at<uchar>(i, j + 1);
			}
			if (ang >= 22.5 && ang < 67.5 || ang >= 202.5 && ang < 249.5)
			{
				pre = (int)grad.at<uchar>(i - 1, j + 1);
				next = (int)grad.at<uchar>(i + 1, j - 1);

			}
			if (ang >= 67.5 && ang < 112.5 || ang >= 249.5 && ang < 292.5)
			{

				pre = (int)grad.at<uchar>(i - 1, j);
				next = (int)grad.at<uchar>(i + 1, j);
			}
			if (ang >= 112.5 && ang < 157.5 || ang >= 292.5 && ang < 337.5)
			{

				pre = (int)grad.at<uchar>(i - 1, j - 1);
				next = (int)grad.at<uchar>(i + 1, j + 1);
			}
			//在边缘图像中，对于每个非0幅值的像素，考察由边缘方向指出的两个邻接像素
			if (data == 0)
				nms.at<uchar>(i, j) = 0;
			else
			{//如果两个邻接像素的幅值有一个超过当前考察像素的幅值，则将当前考察像素标记出来
				if (pre > data || next > data)
					badge[i][j] = 0;
			}

		}
	}
	//处理边缘图像上的所有像素，重新扫描图像，将标记像素置为0
	for (i = 0; i < nms.rows; i++)
	{
		for (j = 0; j < nms.cols; j++)
		{
			if (badge[i][j] == 0)
				nms.at<uchar>(i, j) = 0;
		}
	}
	imshow("nms", nms);
	imwrite("nms.jpg", nms);
}
//设置滚动条的初始位置，
int min_threshold = 50;
int max_threshold = 100;
//创建滚动条
void createThresholdTrackbar()
{

	//定义轨迹条最大值参量
	int min_thresholdMaxValue = 255;
	int max_thresholdMaxValue = 255;
	//定义每个轨迹条名字
	char minThresholdTrackBarName[20];
	sprintf_s(minThresholdTrackBarName, "低阈值 %d", min_thresholdMaxValue);
	char maxThresholdTrackBarName[20];
	sprintf_s(maxThresholdTrackBarName, "高阈值 %d", max_thresholdMaxValue);
	//创建轨迹条
	int r = nms.rows;
	int c = nms.cols;
	namedWindow("final");
	resizeWindow("final", r, c);
	createTrackbar(minThresholdTrackBarName, "final", &min_threshold,
		min_thresholdMaxValue, on_change);
	createTrackbar(maxThresholdTrackBarName, "final", &max_threshold,
		max_thresholdMaxValue, on_change);
	on_change(max_threshold, 0);
}
//滚动条值变化的响应函数，做双阈值处理
void on_change(int, void*)
{
	Mat ch = imread("nms.jpg");
	Mat chg;
	cvtColor(ch, chg, COLOR_BGR2GRAY);
	int i, j;
	int h, w;
	h = chg.rows;
	w = chg.cols;

	//将幅值超过max-threshold的所有边缘标注为正确边缘置为255，低于min_threshold的置为0
	for (i = 0; i < h; i++)
	{
		for (j = 0; j < w; j++)
		{
			int data = (int)chg.at<uchar>(i, j);

			if (data <= min_threshold)
				chg.at<uchar>(i, j) = 0;

			if (data >= max_threshold)
				chg.at<uchar>(i, j) = 255;
		}
	}
	//动态创建标记数组
	int **badge = new int *[chg.rows];
	for (i = 0; i < chg.rows; i++)
		badge[i] = new int[chg.cols];
	for (i = 0; i < chg.rows; i++)
	{
		for (j = 0; j < chg.cols; j++)
			badge[i][j] = -2;
	}

	for (i = 1; i < h - 1; i++)
	{
		for (j = 1; j < w - 1; j++)
		{
			int data = (int)chg.at<uchar>(i, j);
			//如果在高低阈值之间
			if (data > min_threshold&&data < max_threshold)
			{
				//考虑8临界区域，该区域中如果有已经标记为255（强边缘）的点则将此位置标记出来，否则此位置不是边缘，置为0
				int a1, a2, a3, a4, a5, a6, a7, a8;
				a1 = (int)chg.at<uchar>(i - 1, j - 1);
				a2 = (int)chg.at<uchar>(i - 1, j);
				a3 = (int)chg.at<uchar>(i - 1, j + 1);
				a4 = (int)chg.at<uchar>(i, j - 1);
				a5 = (int)chg.at<uchar>(i, j + 1);
				a6 = (int)chg.at<uchar>(i + 1, j - 1);
				a7 = (int)chg.at<uchar>(i + 1, j);
				a8 = (int)chg.at<uchar>(i + 1, j + 1);
				if (a1 >= max_threshold || a2 >= max_threshold || a3 >= max_threshold || a4 >= max_threshold || a5 >= max_threshold || a6 >= max_threshold || a7 >= max_threshold || a8 >= max_threshold)
					badge[i][j] = -1;
				else
					chg.at<uchar>(i, j) = 0;
			}
		}
	}
	//扫描图像是否被标记，如果标记了则为边界置为255
	for (i = 1; i < h - 1; i++)
	{
		for (j = 1; j < w - 1; j++)
		{
			if (badge[i][j] == -1)
				chg.at<uchar>(i, j) = 255;
		}
	}
	imshow("final", chg);
}
