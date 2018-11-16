#include "wxzSVM.h"
using namespace std;
using namespace cv;
using namespace cv::ml;

Mat svm(Mat &image, int type)
{
	switch (type)
	{
	case 1:
		Ptr<SVM> svm = SVM::load("svmLightYellow.xml");
		break;
	case 2:
		Ptr<SVM> svm = SVM::load("svmRed.xml");
		break;
	case 3:
		Ptr<SVM> svm = SVM::load("svmBlue.xml");
		break;
	}

	// Show the decision regions given by the SVM
	Mat newImge(320, 240, CV_8UC1);
	int step = 3;

	for (int i = 0; i < image.rows; i += step)
	{
		Vec3b *data = image.ptr<Vec3b>(i);
		Vec3b *newData = newImge.ptr<Vec3b>(i);
		for (int j = 0; j < image.cols; j += step)
		{
			//取出该坐标处的像素值

			Mat sampleMat = (Mat_<float>(1, 3) << data[j][0], data[j][1], data[j][2]);
			float response = svm->predict(sampleMat);
			//进行预测，返回1或-1,返回类型为float
			if ((int)response != 1)
			{
				newData[j] = 0;
			}
			else
			{
				newData[j] = 255;
			}
		}
	}
	Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
	morphologyEx(newImge, newImge, MORPH_OPEN, element);
	imshow("SVM New Simple Example", newImge);
	return newImge;
}
Mat SVMFind(int type, Mat image)
{
	Mat bimg;
	bimg = svm(image, type);
	waitKey(50);

	vector<std::vector<cv::Point>> contours;
	findContours(opened, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE); //只检测外轮廓。忽略轮廓内部的洞。
	Mat contoursimg(opened.size(), CV_8U, Scalar(255));						//白图
	Mat conimg = contoursimg.clone();

	drawContours(conimg, contours, -1, Scalar(0), 2); //-1：如果是负数，则绘制所有的轮廓 用黑色绘制图像
	int cmax;
	int s[1000], i = 1, position = 160;
	//找最长轮廓 用最小的矩形将它包围起来
	if (contours.size() > 0)
	{
		vector<std::vector<cv::Point>>::iterator itc = contours.begin();
		while (itc != contours.end())
		{
			s[i] = itc->size();
			//cout << s[i] << endl;
			++i;
			++itc;
		}
		for (int l = 2; l <= i - 1; l++)
			if (s[l - 1] > s[l])
				s[l] = s[l - 1];
		cmax = s[i - 1];
		itc = contours.begin();
		while (itc != contours.end())
		{
			if (itc->size() < cmax)
			{
				itc = contours.erase(itc);
			}
			else
				++itc;
		}
		drawContours(contoursimg, contours, -1, Scalar(0), 2);
		Mat result = contoursimg.clone();
		Rect r = boundingRect(Mat(contours[0])); //矩形将轮廓包围
		rectangle(result, r, Scalar(0), 2);
		imshow("result", result);

		center_y = r.y + r.height / 2; //x、y、width、height，分别为左上角点的坐标和矩形的宽和高
		center_x = r.x + r.width / 2;
		down_y = r.y + r.height;
		up_y = r.y;
		area = r.height * r.width;

		return result;
	}
	else
		cout << "haven't deal it sucessfully" << endl;
}
int getInfo()
{
	cout << "center_y: " << center_y << "||"
		 << "center_x: " << center_x << "||"
		 << "up_y: " << up_y << '||'
		 << "down_y: " << down_y << "||"
		 << "area :" << area << endl;
}
int center_y()
{
	return center_y;
}
int center_x()
{
	return center_x;
}
int down_y()
{
	return down_y;
}
int up_y()
{
	return up_y;
}
int area()
{
	return area;
}