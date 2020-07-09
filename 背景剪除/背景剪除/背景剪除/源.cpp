//opencv2.4.13.5+vs2015
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2\opencv.hpp>
#define pi 3.14
using namespace cv;
using namespace std;

struct Guassian//高斯模型
{
	double weight;//权重
	double mu;//均值
	double sigma;//标准差
};
struct pixel
{
	int g_size;//当前像素点使用的高斯模型的数量
	int lt; //匹配上的高斯模型的数量;
	Guassian* p_gmodel;//指向每个像素点建立的高斯模型的指针
};
int main()
{
	VideoCapture capture;
	capture.open("input.avi");//读取视频

	if (!capture.isOpened())
	{
		cout << "Cannot open avi file" << endl;
		return 0;
	}
	else
		cout << "successful！";
	Mat src;//原始视频图像
	Mat gray;//转化的灰度图像
	Mat back, fore;//背景，前景
	cv::BackgroundSubtractorMOG mog(70, 5, 0.7, 0);
	//读取第一帧做初始化
	capture.read(src);
	int width, height;//图像的长、宽
	int cnt = 0; //当前视频帧数
	cnt++;
	cvtColor(src, gray, CV_BGR2GRAY);//将图像转化为灰度图
	width = gray.cols;
	height = gray.rows;
	//各类参数
	int K = 5, sigma_init = 16, T = 70;//高斯模型的最大个数、标准差的初始值 帧数阈值
	double w_init = 0.05, lambda = 2.5, alpha, threold = 0.7;//权重的初始值、置信参数、学习速率、预定阈值（用于选取背景模型）

															 //根据第一帧来初始化高斯模型
	pixel* m_pixel = new pixel[width*height];
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{

			m_pixel[i*width + j].g_size = 1;
			m_pixel[i*width + j].lt = 0;
			m_pixel[i*width + j].p_gmodel = new Guassian[K];
			//初始化每个像素点的第一个高斯模型
			m_pixel[i*width + j].p_gmodel[0].mu = gray.at<uchar>(i, j);
			m_pixel[i*width + j].p_gmodel[0].sigma = sigma_init;
			m_pixel[i*width + j].p_gmodel[0].weight = w_init;
			//初始化每个像素点的其余高斯模型
			for (int k = 1; k < K; k++)
			{
				m_pixel[i*width + j].p_gmodel[k].mu = 0;
				m_pixel[i*width + j].p_gmodel[k].sigma = sigma_init;
				m_pixel[i*width + j].p_gmodel[k].weight = 0;
			}
		}
	}
	while (1)
	{


		if (!capture.read(src))
			break;
		cvtColor(src, gray, CV_BGR2GRAY);

		//计算学习速率
		if (cnt < T) alpha = (double)1 / (2 * cnt);
		else alpha = (double)1 / (2 * T);


		double max_sigma = 0;
		double min_weight = 1;

		for (int i = 0; i < height; i++)
		{
			for (int j = 0; j < width; j++)
			{
				max_sigma = 0;
				min_weight = 1;
				int match = 0;//判断是否匹配上

							  //排序
				for (int k = 0; k < m_pixel[i*width + j].g_size; k++)
				{
					for (int m = k + 1; m < m_pixel[i*width + j].g_size; m++)
					{
						//所有K个高斯函数按照表达式𝜔𝑘𝑡/𝜎𝑘𝑡排序，比值越大，该高斯函数代表背景的可能性越大
						if ((m_pixel[i*width + j].p_gmodel[m].weight / m_pixel[i*width + j].p_gmodel[m].mu) >(m_pixel[i*width + j].p_gmodel[k].weight / m_pixel[i*width + j].p_gmodel[k].mu))
						{
							double temp;
							temp = m_pixel[i*width + j].p_gmodel[m].mu;
							m_pixel[i*width + j].p_gmodel[m].mu = m_pixel[i*width + j].p_gmodel[k].mu;
							m_pixel[i*width + j].p_gmodel[k].mu = temp;
							temp = m_pixel[i*width + j].p_gmodel[m].sigma;
							m_pixel[i*width + j].p_gmodel[m].sigma = m_pixel[i*width + j].p_gmodel[k].sigma;
							m_pixel[i*width + j].p_gmodel[k].sigma = temp;
							temp = m_pixel[i*width + j].p_gmodel[m].weight;
							m_pixel[i*width + j].p_gmodel[m].weight = m_pixel[i*width + j].p_gmodel[k].weight;
							m_pixel[i*width + j].p_gmodel[k].weight = temp;
						}
					}

					//找到最大的sigma和最小的weight
					if (m_pixel[i*width + j].p_gmodel[k].sigma > max_sigma) max_sigma = m_pixel[i*width + j].p_gmodel[k].sigma;
					if (m_pixel[i*width + j].p_gmodel[k].weight < min_weight)min_weight = m_pixel[i*width + j].p_gmodel[k].weight;

					//判断一个像素是否与高斯模型匹配，计算该像素满足高斯模型的最小编号
					int data = gray.at<uchar>(i, j);
					if (fabs(data - m_pixel[i*width + j].p_gmodel[k].mu) < lambda*m_pixel[i*width + j].p_gmodel[k].sigma&&match == 0)
					{
						//如果匹配上
						match = 1;
						m_pixel[i*width + j].lt = k;//记录当前匹配上的高斯模型的编号
													//对匹配上的高斯模型更新权重、均值、标准差
						m_pixel[i*width].p_gmodel[k].weight = (1 - alpha)*m_pixel[i*width + j].p_gmodel[k].weight;
						double N = 1 / (sqrt(2 * pi)*m_pixel[i*width + j].p_gmodel[k].sigma)*exp(-(data - m_pixel[i*width + j].p_gmodel[k].mu)*(data - m_pixel[i*width + j].p_gmodel[k].mu) / (2 * m_pixel[i*width + j].p_gmodel[k].sigma*m_pixel[i*width + j].p_gmodel[k].sigma));
						double p = alpha*N;
						m_pixel[i*width + j].p_gmodel[k].mu = (1 - p)*m_pixel[i*width + j].p_gmodel[k].mu + p*data;
						m_pixel[i*width + j].p_gmodel[k].sigma = sqrt((1 - p)*m_pixel[i*width + j].p_gmodel[k].sigma*m_pixel[i*width + j].p_gmodel[k].sigma + p*(data - m_pixel[i*width + j].p_gmodel[k].mu)*(data - m_pixel[i*width + j].p_gmodel[k].mu));

					}
					else
					{ //对没有匹配到的高斯模型减小权重
						m_pixel[i*width + j].p_gmodel[k].weight = (1 - alpha)*m_pixel[i*width + j].p_gmodel[k].weight;
					}
				}
				//如果所有的高斯模型都没有匹配上
				if (match == 0)
				{
					//增加新的背景模型
					if (m_pixel[i*width + j].g_size < K)
					{
						int n = m_pixel[i*width + j].g_size;
						m_pixel[i*width + j].g_size++;
						m_pixel[i*width + j].p_gmodel[n].mu = gray.at<uchar>(i, j);
						m_pixel[i*width + j].p_gmodel[n].weight = w_init;
						m_pixel[i*width + j].p_gmodel[n].sigma = sigma_init;
					}
					//更改最后一个背景模型
					else
					{
						m_pixel[i*width + j].p_gmodel[K - 1].mu = gray.at<uchar>(i, j);
						m_pixel[i*width + j].p_gmodel[K - 1].weight = 0.5*min_weight;
						m_pixel[i*width + j].p_gmodel[K - 1].sigma = sqrt(2 * max_sigma*max_sigma);
					}
				}

				//权重归一化
				double sum = 0;
				for (int p = 0; p < K; p++)
				{
					sum += m_pixel[i*width + j].p_gmodel[p].weight;
				}
				for (int p = 0; p < K; p++)
				{
					m_pixel[i*width + j].p_gmodel[p].weight /= sum;
				}

				//根据高斯函数的权重，确定描述背景模型的高斯函数
				sum = 0;
				int B = 0;
				for (int p = 0; p < K; p++)
				{

					sum += m_pixel[i*width + j].p_gmodel[p].weight;
					if (sum > threold)
					{
						B = p;
						break;
					}
				}
				//判断当前像素是背景还是前景，如果其匹配上的高斯模型编号在背景模型内为背景，反之位前景
				fore.create(height, width, CV_8UC1);
				if (m_pixel[i*width + j].lt > B || match == 0)
					fore.at<uchar>(i, j) = 255;//前景
				else
					fore.at<uchar>(i, j) = 0;//背景

			}
		}
		//做形态学变换，开运算与闭运算
		Mat open, close;
		Mat se1 = Mat::ones(2, 2, CV_8U);
		Mat se2 = Mat::ones(10, 10, CV_8U);
		morphologyEx(fore, close, MORPH_CLOSE, se2);
		morphologyEx(close, open, MORPH_OPEN, se1);
		Point c1, c2, c3, c4;
		int  minx = height, miny = width, maxx = 0, maxy = 0;

		//找到前景的四角，画出方框
		for (int i = 0; i < height; i++)
		{
			for (int j = 0; j < width; j++)
			{
				if (open.at<uchar>(i, j) == 255)
				{
					if (minx > j)
						minx = j;
					if (miny > i)
						miny = i;
					if (maxx < j)
						maxx = j;
					if (maxy < i)
						maxy = i;
				}
			}
		}

		c1.x = minx - 5;
		c1.y = miny - 5;
		c2.x = minx - 5;
		c2.y = maxy + 5;
		c3.x = maxx + 5;
		c3.y = miny - 5;
		c4.x = maxx + 5;
		c4.y = maxy + 5;

		if (cnt > 1)
		{
			rectangle(src, c1, c4, Scalar(0, 0, 255), 1, 1, 0);
		}
		Mat fg;

		mog(gray, fg, 0.005);

		//显示处理结果
		imshow("原视频", src);
		imshow("front未做过形态学变化的原始前景", fore);
		imshow("open形态学变换后的前景", open);
		imshow("系统自带检测", fg);
		char s = cvWaitKey(80);
		if (s == 27) break;
		cnt++;


	}
	return 0;
}