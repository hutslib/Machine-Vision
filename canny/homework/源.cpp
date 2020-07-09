
//������
//vs2015 + opencv2.4.13.5

#include <iostream>
#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>  
#include<opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include<math.h>

using namespace std;
using namespace cv;

void gradient();//�Լ���д�����������ݶȵĴ�С�뷽��ĺ���
void NMS();//�Լ���д�ķǼ���ֵ���ƺ���
void createThresholdTrackbar();//����ѡ����ֵ�Ĺ���������
void on_change(int, void*);//����������ֵ�ı�ʱ����Ӧ��������˫��ֵ����

Mat imgsrc;//ԭʼͼ��ת���ɵĻҶ�ͼ
Mat img_gaussian;//��˹�˲����ͼ��
Mat grad;//�ݶȴ�С
Mat angle;//�ݶȵķ���0-360��
Mat nms;//�Ǽ���ֵ���ƺ��ͼ��

int main()
{
	//step1:����ͼƬ

	//��ѡ��һ��ͼƬ
	// Mat img = imread("Miss.bmp");
	Mat img = imread("building.jpg");
	//Mat img = imread("road.jpg");

	if (!img.data) cout << "picture load fail!" << endl;
	else cout << "step1successful!" << endl;
	//��ԭʼͼת��Ϊ�Ҷ�ͼ
	cvtColor(img, imgsrc, COLOR_BGR2GRAY);

	//step2:��ͼ����и�˹�˲�
	GaussianBlur(imgsrc, img_gaussian, Size(3, 1), 1, 1);
	cout << "step2successful" << endl;
	imshow("��˹", img_gaussian);

	//step3:���˲����ͼ�����ݶȵĴ�С�ͷ���
	gradient();//�����Լ���д��gradient�����������ں���
	cout << "step3successful" << endl;

	//step4:�Ǽ���ֵ����
	NMS();//�����Լ���д��NMS�����������ں���
	cout << "step4successful" << endl;

	////step5:˫��ֵ��⣬ͨ������������Ƹߵ���ֵ�Ĵ�С
	createThresholdTrackbar();//����ѡ��˫��ֵ�Ĺ���������������ֵ�ı����Ӧ��д��void on_change(int, void*)����������˫��ֵ����
	cout << "step5successful" << endl;

	///չʾϵͳ�Դ���canny��Ե���Ľ��
	Mat canny;
	Canny(imgsrc, canny, 50, 100);
	imshow("ϵͳcanny", canny);
	waitKey(0);
}
//���ݶ�
void gradient()
{
	//�Ը�˹�˲����ͼ������sobel���ӷֱ����x,y����Ĳ�֣�����ֱ𱣴���sobelx,sobely��
	Mat sobelx;
	Mat sobely;
	Sobel(img_gaussian, sobelx, CV_32F, 1, 0, 3);
	Sobel(img_gaussian, sobely, CV_32F, 0, 1, 3);

	// ����ݶȺͷ����ݶȵĴ�С���ձ�����grand�У����򱣴���angle��  
	Mat g;
	cartToPolar(sobelx, sobely, g, angle, 1);//���õѿ�������ת���ɼ�����õ��ݶȵĴ�С�ͷ���
	convertScaleAbs(g, grad);//g��ֵ���ܳ���255����Ҫת���ɷ�Χ��0-255�ڵ�8λ�޷����� 
	imshow("grad", grad);
}
//�Ǽ���ֵ����
void NMS()
{

	nms = grad;
	int i, j;

	//��̬������Ǿ���
	int **badge = new int *[nms.rows];
	for (i = 0; i < nms.rows; i++)
		badge[i] = new int[nms.cols];
	for (i = 0; i < nms.rows; i++)
	{
		for (j = 0; j < nms.cols; j++)
			badge[i][j] = -2;
	}
	//��360��ƽ���ֳ�8�֣�����ݶȷ����ϵ�ǰ�������㣬�Ƚϸõ���ݶ�ֵ�Ƿ�Ϊ��������������һ��
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
			//�����ݶ���ǰ��������
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
			//�ڱ�Եͼ���У�����ÿ����0��ֵ�����أ������ɱ�Ե����ָ���������ڽ�����
			if (data == 0)
				nms.at<uchar>(i, j) = 0;
			else
			{//��������ڽ����صķ�ֵ��һ��������ǰ�������صķ�ֵ���򽫵�ǰ�������ر�ǳ���
				if (pre > data || next > data)
					badge[i][j] = 0;
			}

		}
	}
	//�����Եͼ���ϵ��������أ�����ɨ��ͼ�񣬽����������Ϊ0
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
//���ù������ĳ�ʼλ�ã�
int min_threshold = 50;
int max_threshold = 100;
//����������
void createThresholdTrackbar()
{

	//����켣�����ֵ����
	int min_thresholdMaxValue = 255;
	int max_thresholdMaxValue = 255;
	//����ÿ���켣������
	char minThresholdTrackBarName[20];
	sprintf_s(minThresholdTrackBarName, "����ֵ %d", min_thresholdMaxValue);
	char maxThresholdTrackBarName[20];
	sprintf_s(maxThresholdTrackBarName, "����ֵ %d", max_thresholdMaxValue);
	//�����켣��
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
//������ֵ�仯����Ӧ��������˫��ֵ����
void on_change(int, void*)
{
	Mat ch = imread("nms.jpg");
	Mat chg;
	cvtColor(ch, chg, COLOR_BGR2GRAY);
	int i, j;
	int h, w;
	h = chg.rows;
	w = chg.cols;

	//����ֵ����max-threshold�����б�Ե��עΪ��ȷ��Ե��Ϊ255������min_threshold����Ϊ0
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
	//��̬�����������
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
			//����ڸߵ���ֵ֮��
			if (data > min_threshold&&data < max_threshold)
			{
				//����8�ٽ����򣬸�������������Ѿ����Ϊ255��ǿ��Ե���ĵ��򽫴�λ�ñ�ǳ����������λ�ò��Ǳ�Ե����Ϊ0
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
	//ɨ��ͼ���Ƿ񱻱�ǣ�����������Ϊ�߽���Ϊ255
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
