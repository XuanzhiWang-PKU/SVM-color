#include "opencv2/opencv.hpp"
#include<iostream>
using namespace std;
using namespace cv;

void main()
{
	Mat img = imread("robot.jpg");
	//imshow("src", img);
	Mat result = img.clone();
	Mat gray;
	cvtColor(img, gray, CV_BGR2GRAY);

	//Shi-Tomasi角点检测
	vector<Point2f> corners;
	goodFeaturesToTrack(gray, corners, 100, 0.01, 10, Mat(), 3, false, 0.04);
	cout << "角点数量" << corners.size() << endl;

	for (int i = 0; i<corners.size(); i++)
	{
		cout << "像素坐标:(" << corners[i].x << ", " << corners[i].y << ")" << endl;
		circle(result, corners[i], 5, Scalar(0, 255, 0), 2, 8);
	}
	imshow("result", result);

	Size winSize = Size(5, 5);
	Size zeroZone = Size(-1, -1);
	//精度或最大迭代数目，其中任意一个达到  迭代次数40，精度0.001
	TermCriteria criteria = TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 40, 0.001);
	cornerSubPix(gray, corners, winSize, zeroZone, criteria);

	for (int j = 0; j<corners.size(); j++)
	{
		cout << "亚像素坐标:(" << corners[j].x << ", " << corners[j].y << ")" << endl;
		circle(img, corners[j], 5, Scalar(0, 255, 0), -1, 8);
	}
	imshow("subPix", img);

	waitKey(0);
}

//#include "opencv2/opencv.hpp"
//using namespace cv;
//using namespace std;
//
//int main() {
//	string path[8];
//	Mat img[8];
//	for (int i = 0; i < 8; i++) {
//		int aa = i;
//		stringstream ss;
//		ss << aa;
//		string s1 = ss.str();
//		//cout << s1 << endl; // 30
//
//		string s2;
//		ss >> s2;
//		//cout << s2 << endl; // 30
//
//		path[i] = "G:/robot/img" + s2 + ".jpg";
//		cout << path[i] << endl;
//
//		img[i] = imread(path[i]);
//		resize(img[i], img[i], Size(240, 320));
//
//	}
//
//	Mat combine1, combine2, combine3, combine4, combine5, combine6, combine;
//	hconcat(img[0], img[1], combine1);
//	hconcat(img[2], img[3], combine2);
//	hconcat(img[4], img[5], combine3);
//	hconcat(img[6], img[7], combine4);
//
//	hconcat(combine1, combine2, combine5);
//	hconcat(combine3, combine4, combine6);
//
//	vconcat(combine6, combine5, combine);
//	//namedWindow("Combine", CV_WINDOW_AUTOSIZE);
//	imshow("Combine", combine);
//	imwrite("robot.jpg", combine);
//	waitKey(0);
//
//	system("pause");
//
//}