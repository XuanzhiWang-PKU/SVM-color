//#include "opencv2/opencv.hpp"
//using namespace cv;
//using namespace cv::ml;
//using namespace std;
//
//Mat img, image, orig;
//Mat targetData, backData;
//bool flag = true;
//string wdname = "image";
//
//void on_mouse(int event, int x, int y, int flags, void* ustc);
//void getTrainData(Mat &train_data, Mat &train_label);
//Mat svm();
//void svmFind(Mat bimage);
//
//
//int main(int argc, char** argv)
//{
//	string path = "C:/Users/yinquan/Documents/Visual Studio 2015/Projects/SVM2.0/SVM2.0/eight.jpg";
//	orig = imread(path);
//	cvtColor(orig, img, COLOR_BGR2Lab);//
//	//imshow("Lab", img);
//	resize(img, img, Size(800, 400));
//	resize(orig, orig, Size(800, 400));
//	img.copyTo(image);
//	if (img.empty())
//	{
//		cout << "Image load error";
//		return 0;
//	}
//	namedWindow(wdname);
//	setMouseCallback(wdname, on_mouse, 0);
//
//	for (;;)
//	{
//		imshow("image", orig);
//
//		int c = waitKey(0);
//		if ((c & 255) == 27)
//		{
//			cout << "Exiting ...\n";
//			break;
//		}
//		if ((char)c == 'c')
//		{
//			flag = false;
//		}
//		if ((char)c == 'q')
//		{
//			destroyAllWindows();
//			break;
//		}
//	}
//	Mat result1 = svm();
//	svmFind(result1);
//	return 0;
//}
//
//
//void on_mouse(int event, int x, int y, int flags, void* ustc)
//{
//	if (event == CV_EVENT_LBUTTONDOWN)
//	{
//		Point pt = Point(x, y);
//		Vec3b point = img.at<Vec3b>(y, x);
//		Mat tmp = (Mat_<float>(1, 3) << point[0], point[1], point[2]);
//		if (flag)
//		{
//			targetData.push_back(tmp);
//			circle(orig, pt, 2, Scalar(0, 255, 255), -1, 8);//
//
//		}
//
//		else
//		{
//			backData.push_back(tmp);
//			circle(orig, pt, 2, Scalar(255, 0, 0), -1, 8);//
//
//		}
//		imshow(wdname, orig);//
//	}
//}
//
//void getTrainData(Mat &train_data, Mat &train_label)
//{
//	int m = targetData.rows;
//	int n = backData.rows;
//	cout << "target:" << m << endl;
//	cout << "back:" << n << endl;
//	vconcat(targetData, backData, train_data);
//	train_label = Mat(m + n, 1, CV_32S, Scalar::all(1));
//	for (int i = m; i < m + n; i++)
//		train_label.at<int>(i, 0) = -1;
//}
//
//Mat svm()
//{
//	Mat train_data, train_label;
//	getTrainData(train_data, train_label);
//
//	Ptr<SVM> svm = SVM::create();
//	svm->setType(SVM::C_SVC);
//	svm->setKernel(SVM::RBF);
//	svm->setGamma(0.01);
//	svm->setC(10.0);
//	svm->setTermCriteria(TermCriteria(CV_TERMCRIT_EPS, 1000, FLT_EPSILON));
//
//	Ptr<TrainData> tData = TrainData::create(train_data, ROW_SAMPLE, train_label);
//	//svm->train(tData);
//	svm->trainAuto(tData, 10,
//		SVM::getDefaultGrid(SVM::C),
//		SVM::getDefaultGrid(SVM::GAMMA),
//		SVM::getDefaultGrid(SVM::P),
//		SVM::getDefaultGrid(SVM::NU),
//		SVM::getDefaultGrid(SVM::COEF),
//		SVM::getDefaultGrid(SVM::DEGREE),
//		true);
//	svm->save("svmYellow-lab.xml");
//
//
//	Mat newImge(Size(800, 400), CV_8UC1, Scalar(0));
//	int step = 3;
//
//	for (int i = 0; i < image.rows - step; i += step) {
//		Vec3b *data = image.ptr<Vec3b>(i);
//		uchar *newData = newImge.ptr<uchar>(i);
//		for (int j = 0; j < image.cols - step; j += step)
//		{
//			Mat sampleMat = (Mat_<float>(1, 3) << data[j][0], data[j][1], data[j][2]);
//			float response = svm->predict(sampleMat);
//
//			if ((int)response == -1) {
//				newData[j] = 0;
//			}
//			else {
//				newData[j] = 255;
//			}
//		}
//	}
//	imshow("SVM New Simple Example", newImge);
//	Mat element = getStructuringElement(MORPH_RECT, Size(7, 7));
//	morphologyEx(newImge, newImge, MORPH_CLOSE, element);
//	//imshow("SVM Simple Example", image);
//	return newImge;
//}
//
//void svmFind(Mat bimage) {
//	Mat bimg;
//	//imshow("bimg", bimage);
//	bimg = bimage;
//	vector<std::vector<cv::Point> > contours;
//	findContours(bimg, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE); //只检测外轮廓。忽略轮廓内部的洞。
//	Mat contoursimg(bimg.size(), CV_8U, Scalar(0));					  //黑图
//	Mat conimg = contoursimg.clone();
//	//drawContours(conimg, contours, -1, Scalar(0), 2); //-1：如果是负数，则绘制所有的轮廓 用黑色绘制图像
//	int cmax;
//	int s[1000], i = 1, position = 160;
//	//找最长轮廓 用最小的矩形将它包围起来
//	if (contours.size() > 0)
//	{
//		vector<std::vector<cv::Point> >::iterator itc = contours.begin();
//		while (itc != contours.end())
//		{
//			s[i] = itc->size();
//			//cout << s[i] << endl;
//			++i;
//			++itc;
//		}
//		for (int l = 2; l <= i - 1; l++)
//			if (s[l - 1] > s[l])
//				s[l] = s[l - 1];
//		cmax = s[i - 1];
//		itc = contours.begin();
//		while (itc != contours.end())
//		{
//			if (itc->size() < cmax)
//			{
//				itc = contours.erase(itc);
//			}
//			else
//				++itc;
//		}
//		drawContours(contoursimg, contours, -1, Scalar(255), 2);
//		Mat result = contoursimg.clone();
//		Rect r = boundingRect(Mat(contours[0])); //矩形将轮廓包围
//		rectangle(result, r, Scalar(255), 2);
//		imshow("result", result);
//		waitKey(0);
//
//	}
//	else
//		cout << "haven't deal it sucessfully" << endl;
//		waitKey(0);
//}

 #include "opencv2/opencv.hpp"
 #include <ctime>
 using namespace cv;
 using namespace cv::ml;
 using namespace std;

 Mat img, image;
 Mat targetData, backData;
 bool flag = true;
 string wdname = "image";

 void svmFind(Mat bimage);
 Mat svm(Mat &);

 int main(int argc, char **argv)
 {

 	VideoCapture cap(1);
 	while ((waitKey(10) & 255) != 27)
 	{
 		cap >> img;
 		resize(img, img, Size(320, 240));
 		imshow("Camera", img);
 		double start = clock();
 		cvtColor(img, img, COLOR_BGR2Lab);//
 		Mat result1 = svm(img);
 		svmFind(result1);
 		double end = clock();
 		cout << (end - start) / CLOCKS_PER_SEC << endl;
 	}
 	return 0;
 }

 Mat svm(Mat &image)
 {

 	Ptr<SVM> svm = SVM::load("svmYellow-lab.xml");

 	Vec3b color(0, 0, 0);
 	// Show the decision regions given by the SVM

 	Mat newImge(image.size(), CV_8UC1, Scalar(0));
 	int step = 3;

 	for (int i = 0; i < image.rows - step; i += step)
 	{
 		Vec3b *data = image.ptr<Vec3b>(i);
 		uchar *newData = newImge.ptr<uchar>(i);
 		for (int j = 0; j < image.cols - step; j += step)
 		{
 			Mat sampleMat = (Mat_<float>(1, 3) << data[j][0], data[j][1], data[j][2]);
 			float response = svm->predict(sampleMat);

 			if ((int)response == -1)
 			{
 				newData[j] = 0;
 			}
 			else
 			{
 				newData[j] = 255;
 			}
 		}
 	}
 	imshow("SVM New Simple Example", newImge);
 	Mat element = getStructuringElement(MORPH_RECT, Size(7, 7));
 	morphologyEx(newImge, newImge, MORPH_CLOSE, element);
 	//imshow("SVM Simple Example", image);
 	return newImge;
 }

 void svmFind(Mat bimage)
 {
 	Mat bimg;
 	//imshow("bimg", bimage);
 	bimg = bimage;
 	vector<std::vector<cv::Point>> contours;
 	findContours(bimg, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE); //只检测外轮廓。忽略轮廓内部的洞。
 	Mat contoursimg(bimg.size(), CV_8U, Scalar(0));						  //黑图
 	Mat conimg = contoursimg.clone();
 	//drawContours(conimg, contours, -1, Scalar(0), 2); //-1：如果是负数，则绘制所有的轮廓 用黑色绘制图像
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
 		drawContours(contoursimg, contours, -1, Scalar(255), 2);
 		Mat result = contoursimg.clone();
 		Rect r = boundingRect(Mat(contours[0])); //矩形将轮廓包围
 		rectangle(result, r, Scalar(255), 2);
 		imshow("result", result);
 		waitKey(1);
 	}
 	else
 		cout << "haven't deal it sucessfully" << endl;
 	waitKey(1);
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
//		path[i] = "G:/img" + s2 + ".jpg";
//		//cout << path[i] << endl;
//
//		img[i] = imread(path[i]);
//
//	}
//
//	Mat combine1, combine2, combine3, combine4, combine5, combine6, combine;
//	hconcat(img[0], img[1], combine1);
//	hconcat(img[2], img[3], combine2);
//	hconcat(img[4], img[5], combine3);
//	hconcat(img[6], img[6], combine4);
//
//	hconcat(combine1, combine2, combine5);
//	hconcat(combine3, combine4, combine6);
//
//	vconcat(combine6, combine5, combine);
//	//namedWindow("Combine", CV_WINDOW_AUTOSIZE);
//	imshow("Combine", combine);
//	imwrite("red.jpg", combine);
//	waitKey(0);
//
//	system("pause");
//
//}
