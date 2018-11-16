// svm.cpp : 定义控制台应用程序的入口点。

#include "opencv2/opencv.hpp"
using namespace cv;
using namespace cv::ml;
using namespace std;

class wxzSVM
{
private:
	int center_y;
	int center_x;
	int down_y;
	int up_y;
	int area;

public:
	Mat Capt(int type);
	Mat svm(Mat &, int type); //svm分类
	Mat SVMFind(int type, Mat image);
	int getInfo();
	int center_y();
	int center_x();
	int down_y();
	int up_y();
	int area();
}
