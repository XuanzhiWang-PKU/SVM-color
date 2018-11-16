#include <boost/shared_ptr.hpp>
#include <alproxies/alvideodeviceproxy.h>
#include <alvision/alimage.h>
#include <alvision/alvisiondefinitions.h>
#include <alerror/alerror.h>
#include <alproxies/almotionproxy.h>
#include <alproxies/alrobotpostureproxy.h>
#include <alproxies/altexttospeechproxy.h>
#include <qi/os.hpp>
#include <qi/log.hpp>
#include <alcommon/alproxy.h>
#include <alcommon/albroker.h>

// Opencv includes.
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "pickup.h"
#include "wxzSVM.h"
#include <iostream>
#include <string>

using namespace cv;
using namespace std;
using namespace AL;
#define PI 3.14159

pickup::pickup(boost::shared_ptr<ALBroker> pBroker,
			   const std::string &pName) : ALModule(pBroker, pName), camProxy(getParentBroker()), motionProxy(getParentBroker()), postureProxy(getParentBroker()), clientName(" ")
{
	setModuleDescription(
		"");

	functionName("start", getName(), "");
	BIND_METHOD(pickup::start);
}

void pickup::exit()
{
	AL::ALModule::exit();
}

void pickup::init()
{
	start();
}

pickup::~pickup()
{
}

void pickup::start()
{
	motionProxy.wakeUp();
	motionProxy.setMoveArmsEnabled(true, true); //第二次运行时把胳膊松开
	postureProxy.goToPosture("StandInit", 0.5); //0.5代表速度
	AL::ALValue config = AL::ALValue::array(AL::ALValue::array("LeftStepHeight", 0.025), AL::ALValue::array("RightStepHeight", 0.03), AL::ALValue::array("LeftMaxStepX", 0.05), AL::ALValue::array("RightMaxStepX", 0.065));
	AL::ALValue configMoveto = AL::ALValue::array(AL::ALValue::array("StepHeight", 0.02), AL::ALValue::array("MaxStepX", 0.04));
	camProxy.setActiveCamera(1); //打开下摄像头//////////
	clientName = camProxy.subscribe("test", kQVGA, kBGRColorSpace, 5);
	imgHeader = cv::Mat(cv::Size(320, 240), CV_8UC3); //创建一张图片
	/***********************************************************************************/
	wxzSVM imgSVM;
	int pickBall = 10;		  //球在中间的阈值
	int closetoballmid = 100; //可以捡球
	int closetoballbot = 220; //原来210

	int throwBall = 15;		 //垃圾箱在中间的阈值 20
	int closeToTarget = 190; //可以扔球

	int upCloseToTarget = 155;   //判断上边缘扔垃圾
	int downCloseToTarget = 170; //判断下边缘 结束盲走

	int pause = 1;
	/***********************************************************************************/
	AL::ALValue names1 = "HeadPitch";		 //names1关节名字
	AL::ALValue angleLists1 = 20 * PI / 180; //角度
	AL::ALValue timeLists = 1.0f;			 //时间
	bool isAbsolute = true;					 //绝对角度
	motionProxy.angleInterpolation(names1, angleLists1, timeLists, isAbsolute);
	/*************看到垃圾就停**********************************************************************/
	while (true)
	{
		ALValue img = camProxy.getImageRemote(clientName); //从摄像头得到一幅图片
		imgHeader.data = (uchar *)img[6].GetBinary();	  //把图片的data保留在imgheader中，img作废
		camProxy.releaseImage(clientName);
		Mat image = imgHeader.clone(); //复制一个imgheader给image
		imshow("image", image);		   //显示图片
		waitKey(50);				   //等待5秒

		imgSVM.SVMFind(1, image);						  //得到中心坐标和左下角y坐标
		if (imgSVM.down_y() > 0 && (imgSVM.area() > 800)) //判断是否看到垃圾
		{
			cout << "看到垃圾了" << endl;
			break;
		}
		motionProxy.moveToward(0.6, 0, 0, config); //直走
	}

	while (true)
	{
		ALValue img = camProxy.getImageRemote(clientName); //从摄像头得到一幅图片
		imgHeader.data = (uchar *)img[6].GetBinary();	  //把图片的data保留在imgheader中，img作废
		camProxy.releaseImage(clientName);
		Mat image = imgHeader.clone();													  //复制一个imgheader给image
		imshow("image", image);															  //显示图片
		waitKey(50);																	  //等待5秒
		imgSVM.SVMFind(1, image);														  //得到中心坐标和左下角y坐标
		if (imgSVM.center_x() < (160 + pickBall) && imgSVM.center_x() > (160 - pickBall)) //如果在中间离得近
			motionProxy.moveToward(0.6, 0, 0);											  //0.6为速度
		if (imgSVM.center_x() < (160 - pickBall))										  //如果在左边离得近
			motionProxy.moveToward(0.3, 0.15, 0);
		if (imgSVM.center_x() > (160 + pickBall)) //如果在右边离得近
			motionProxy.moveToward(0.3, -0.15, 0);
		if (imgSVM.down_y() > closetoballbot && imgSVM.center_x() > (160 - pickBall * 2) && imgSVM.center_x() < (160 + pickBall * 2)) //如果可以捡到
		{
			motionProxy.moveToward(0, 0, 0);
			break;
		}
		if (imgSVM.down_y() > closetoballbot && imgSVM.center_x() < (160 - pickBall))
		{
			motionProxy.moveToward(0, 0.25, 0); //0.15
			cout << "我在向左走" << endl;
		}
		if (imgSVM.down_y() > closetoballbot && imgSVM.center_x() > (160 + pickBall))
		{
			motionProxy.moveToward(0, -0.25, 0);
			cout << "我在向右走" << endl;
		}
	}

	/*****************************************************************************************/
	bool pEnable = false;
	motionProxy.setCollisionProtectionEnabled("Arms", pEnable);
	if (motionProxy.getCollisionProtectionEnabled("Arms") == false)
	{
		cout << "保护机制已关闭" << endl;
	}
	/*****************************************************************************************/
	pickUpBalls(); //捡球
	cout << "我捡球****************************************************" << endl;
	camProxy.setActiveCamera(0);				  //开上摄像头
	motionProxy.setMoveArmsEnabled(false, false); //关节锁死
	motionProxy.moveToward(0, 0, 0);

	motionProxy.moveTo(0, 0, 15 * PI / 180); //向左转15度

	// while (true)
	// {

	// 	ALValue img = camProxy.getImageRemote(clientName);//从摄像头得到一幅图片
	// 	imgHeader.data = (uchar*) img[6].GetBinary();     //把图片的data保留在imgheader中，img作废
	// 	camProxy.releaseImage(clientName);
	// 	Mat image = imgHeader.clone();                    //复制一个imgheader给image
	// 	imshow("image", image);//显示图片
	// 	waitKey(50);//等待5秒
	// 	find(2, image);//得到中心坐标和左下角y坐标

	// 	if (center_x > 140)
	// 	{
	// 		motionProxy.moveToward(0, 0, 0);
	// 		break;
	// 	}

	// 	motionProxy.moveToward(0, 0, 0.3);
	// }

	while (true)
	{

		ALValue img = camProxy.getImageRemote(clientName); //从摄像头得到一幅图片
		imgHeader.data = (uchar *)img[6].GetBinary();	  //把图片的data保留在imgheader中，img作废
		camProxy.releaseImage(clientName);
		Mat image = imgHeader.clone(); //复制一个imgheader给image
		imshow("image", image);		   //显示图片
		waitKey(50);				   //等待5秒
		imgSVM.find(2, image);		   //得到中心坐标和左下角y坐标

		if (imgSVM.down_y() > downCloseToTarget)
		{
			motionProxy.moveToward(0, 0, 0);
			break;
		}
		//motionProxy.moveToward(0.6, 0, 0);//感觉还行
		motionProxy.moveToward(0.6, 0, 0, config); //0.03的还行 走得大一点 要抖 0.04要摔
	}

	motionProxy.angleInterpolation(names1, angleLists1, timeLists, isAbsolute); //

	while (true)
	{
		ALValue img = camProxy.getImageRemote(clientName); //从摄像头得到一幅图片
		imgHeader.data = (uchar *)img[6].GetBinary();	  //把图片的data保留在imgheader中，img作废
		camProxy.releaseImage(clientName);
		Mat image = imgHeader.clone(); //复制一个imgheader给image
		imshow("image", image);
		//waitKey(50);
		imgSVM.find(2, image);																//得到中心坐标和左下角y坐标
		if (imgSVM.center_x() < (160 + throwBall) && imgSVM.center_x() > (160 - throwBall)) //如果在中间离得远
			motionProxy.moveToward(0.4, 0, 0, config);
		if (imgSVM.center_x() < (160 - throwBall)) //如果在左边离得远
		{
			motionProxy.moveToward(0.2, 0.5, 0, config);
			cout << "我在向左走" << endl;
		}
		if (imgSVM.center_x() > (160 + throwBall)) //如果在右边离得远
		{
			motionProxy.moveToward(0.2, -0.5, 0, config);
			cout << "我在向右走" << endl;
		}

		if (imgSVM.center_y() > closeToTarget && std::fabs(imgSVM.center_x() - 160.f) <= throwBall) //如果可以放下
		{
			motionProxy.moveToward(0, 0, 0); //停住
			break;
		}
		if (imgSVM.center_y() > closeToTarget && std::fabs(imgSVM.center_x() - 160.f) > throwBall)
		{
			if (imgSVM.center_y() > 160 + throwBall)
				motionProxy.moveToward(0, -0.5, 0);
			else if (imgSVM.center_y() < 160 - throwBall)
				motionProxy.moveToward(0, 0.5, 0);
		}
	}

	while (true)
	{

		ALValue img = camProxy.getImageRemote(clientName); //从摄像头得到一幅图片
		imgHeader.data = (uchar *)img[6].GetBinary();	  //把图片的data保留在imgheader中，img作废
		camProxy.releaseImage(clientName);
		Mat image = imgHeader.clone(); //复制一个imgheader给image
		imshow("image", image);		   //显示图片
		waitKey(50);				   //等待5秒
		imgSVM.find(3, image);		   //得到中心坐标和左下角y坐标

		if (up_y > upCloseToTarget)
		{
			motionProxy.moveToward(0, 0, 0);
			break;
		}
		//motionProxy.moveToward(0.6, 0, 0);//感觉还行
		motionProxy.moveToward(0.6, 0, 0, config); //0.03的还行 走得大一点 要抖 0.04要摔
	}

	throwOutBalls();
	cout << "hahaha" << endl;
	postureProxy.goToPosture("StandInit", 0.5); //0.5表示速度
	camProxy.unsubscribe(clientName);
}

// void pickup::find(int num, Mat image)
// {
// 	/***********************************************************************************/
// 	Mat sample1, sample2, sample3, bimg;
// 	sample1 = imread("/home/yinquan/pictures/yellow.jpg");
// 	sample2 = imread("/home/yinquan/pictures/red.jpg");
// 	sample3 = imread("/home/yinquan/pictures/red2.jpg");
// 	/***********************************************************************************/
// 	if (num == 1)
// 	{
// 		/////hsv
// 		Mat result1_hsv = imgProcess(image, sample1, 0.014f, 1); //threshold 0.05~0.09
// 		///rgb
// 		ColorHistogram h;
// 		sample1 = h.colorReduce(sample1, 75); //reduce 100~200
// 		image = h.colorReduce(image, 75);
// 		Mat result1_rgb = imgProcess(image, sample1, 0.017f, 0); //RGB
// 		Mat result1_j = result1_hsv & result1_rgb;
// 		Mat result1_b = result1_hsv | result1_rgb; // "&"  "|"
// 		bimg = result1_b;						   //
// 		//bimg=result1_rgb;
// 		imshow("result_hsv", result1_hsv);
// 		imshow("result_rgb", result1_rgb);
// 		imshow("result_&", result1_j);
// 		imshow("result_|", result1_b);
// 		waitKey(50);
// 		moveWindow("result_hsv", 100, 100);
// 		moveWindow("result_rgb", 1000, 100);
// 		moveWindow("result_&", 100, 1000);
// 		moveWindow("result_|", 1000, 1000);
// 	}
// 	if (num == 2)
// 	{
// 		///hsv
// 		Mat result2_hsv = imgProcess(image, sample2, 0.004f, 1);
// 		///rgb
// 		ColorHistogram h;
// 		sample2 = h.colorReduce(sample2, 62);
// 		image = h.colorReduce(image, 62);
// 		Mat result2_rgb = imgProcess(image, sample2, 0.001f, 0);
// 		Mat result2_j = result2_hsv & result2_rgb;
// 		Mat result2_b = result2_hsv | result2_rgb; // "&"  "|"
// 		bimg = result2_b;

// 		imshow("result_hsv", result2_hsv);
// 		imshow("result_rgb", result2_rgb);
// 		imshow("result_&", result2_j);
// 		imshow("result_|", result2_b);

// 		moveWindow("result_hsv", 100, 100);
// 		moveWindow("result_rgb", 1000, 100);
// 		moveWindow("result_&", 100, 1000);
// 		moveWindow("result_|", 1000, 1000);
// 		waitKey(50);
// 	}
// 	if (num == 3)
// 	{
// 		///hsv
// 		Mat result2_hsv = imgProcess(image, sample3, 0.073f, 1);
// 		///rgb
// 		ColorHistogram h;
// 		sample3 = h.colorReduce(sample3, 50);
// 		image = h.colorReduce(image, 50);
// 		Mat result2_rgb = imgProcess(image, sample3, 0.057f, 0);
// 		Mat result2_j = result2_hsv & result2_rgb;
// 		Mat result2_b = result2_hsv | result2_rgb; // "&"  "|"
// 		bimg = result2_j;

// 		imshow("result_hsv", result2_hsv);
// 		imshow("result_rgb", result2_rgb);
// 		imshow("result_&", result2_j);
// 		imshow("result_|", result2_b);

// 		moveWindow("result_hsv", 100, 100);
// 		moveWindow("result_rgb", 1000, 100);
// 		moveWindow("result_&", 100, 1000);
// 		moveWindow("result_|", 1000, 1000);
// 		waitKey(50);
// 	}

// 	/*ColorHistogram hc;
// 		image = hc.colorReduce(image, 240);                //减色
// 		sample = hc.colorReduce(sample, 240);

// 		MatND hist = hc.getHistogram(sample);             //把减色的样图的直方图给hist
// 		ObjectFinder finder;
// 		finder.setHistogram(hist);
// 		finder.setThreshold(0.05f);
// 		Mat bimg = finder.find(image);                    //用finder来找image
// 		//imshow("bimg",bimg);                              //bimg是找到的图像
// 		//waitKey(500);*/
// 	Mat element5(5, 5, CV_8U, Scalar(1)); //内核5*5
// 	Mat closed;
// 	morphologyEx(bimg, closed, MORPH_CLOSE, element5); //闭运算 前两个参数为输入和输出图像，第三个为操作类型，第四个参数为内核。

// 	Mat opened;
// 	morphologyEx(closed, opened, MORPH_OPEN, element5); //开运算

// 	vector<std::vector<cv::Point>> contours;
// 	findContours(opened, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE); //只检测外轮廓。忽略轮廓内部的洞。
// 	Mat contoursimg(opened.size(), CV_8U, Scalar(255));						//白图
// 	Mat conimg = contoursimg.clone();

// 	drawContours(conimg, contours, -1, Scalar(0), 2); //-1：如果是负数，则绘制所有的轮廓 用黑色绘制图像
// 	int cmax;
// 	int s[1000], i = 1, position = 160;
// 	//找最长轮廓 用最小的矩形将它包围起来
// 	if (contours.size() > 0)
// 	{
// 		vector<std::vector<cv::Point>>::iterator itc = contours.begin();
// 		while (itc != contours.end())
// 		{
// 			s[i] = itc->size();
// 			//cout << s[i] << endl;
// 			++i;
// 			++itc;
// 		}
// 		for (int l = 2; l <= i - 1; l++)
// 			if (s[l - 1] > s[l])
// 				s[l] = s[l - 1];
// 		cmax = s[i - 1];
// 		itc = contours.begin();
// 		while (itc != contours.end())
// 		{
// 			if (itc->size() < cmax)
// 			{
// 				itc = contours.erase(itc);
// 			}
// 			else
// 				++itc;
// 		}
// 		drawContours(contoursimg, contours, -1, Scalar(0), 2);
// 		Mat result = contoursimg.clone();
// 		Rect r = boundingRect(Mat(contours[0])); //矩形将轮廓包围
// 		rectangle(result, r, Scalar(0), 2);
// 		imshow("result", result);
// 		moveWindow("result", 1500, 100);
// 		// waitKey(50);

// 		//坐标从左上角开始 向右为x正方向 向下为y正方向
// 		center_y = r.y + r.height / 2; //x、y、width、height，分别为左上角点的坐标和矩形的宽和高
// 		center_x = r.x + r.width / 2;
// 		down_y = r.y + r.height;
// 		up_y = r.y;
// 		area = r.height * r.width;
// 		cout << "上边缘y： " << r.y << endl;
// 		cout << "(" << center_x << "," << center_y << ")" << endl;
// 		//cout << "(" << down_y << ")" << endl;
// 		//cout << "面积是:" << area << endl;
// 	}
// 	else
// 		cout << "haven't deal it sucessfully" << endl;
// }

void pickup::stayPosture()
{
	std::vector<std::string> names;
	AL::ALValue times, keys;
	names.reserve(12);
	times.arraySetSize(12);
	keys.arraySetSize(12);

	names.push_back("LElbowRoll");
	times[0].arraySetSize(1);
	keys[0].arraySetSize(1);

	times[0][0] = 16.92;
	keys[0][0] = -1.48947;

	names.push_back("LElbowYaw");
	times[1].arraySetSize(1);
	keys[1].arraySetSize(1);

	times[1][0] = 16.92;
	keys[1][0] = -0.581429;

	names.push_back("LHand");
	times[2].arraySetSize(1);
	keys[2].arraySetSize(1);

	times[2][0] = 16.92;
	keys[2][0] = 0.968;

	names.push_back("LShoulderPitch");
	times[3].arraySetSize(1);
	keys[3].arraySetSize(1);

	times[3][0] = 16.92;
	keys[3][0] = 0.921892;

	names.push_back("LShoulderRoll");
	times[4].arraySetSize(1);
	keys[4].arraySetSize(1);

	times[4][0] = 16.92;
	keys[4][0] = 0.107338;

	names.push_back("LWristYaw");
	times[5].arraySetSize(1);
	keys[5].arraySetSize(1);

	times[5][0] = 16.92;
	keys[5][0] = -0.019984;

	names.push_back("RElbowRoll");
	times[6].arraySetSize(1);
	keys[6].arraySetSize(1);

	times[6][0] = 16.92;
	keys[6][0] = 1.17355;

	names.push_back("RElbowYaw");
	times[7].arraySetSize(1);
	keys[7].arraySetSize(1);

	times[7][0] = 16.92;
	keys[7][0] = 0.141086;

	names.push_back("RHand");
	times[8].arraySetSize(1);
	keys[8].arraySetSize(1);

	times[8][0] = 16.92;
	keys[8][0] = 0.8476;

	names.push_back("RShoulderPitch");
	times[9].arraySetSize(1);
	keys[9].arraySetSize(1);

	times[9][0] = 16.92;
	keys[9][0] = 0.65506;

	names.push_back("RShoulderRoll");
	times[10].arraySetSize(1);
	keys[10].arraySetSize(1);

	times[10][0] = 16.92;
	keys[10][0] = -0.075208;

	names.push_back("RWristYaw");
	times[11].arraySetSize(1);
	keys[11].arraySetSize(1);

	times[11][0] = 16.92;
	keys[11][0] = 1.61373;

	motionProxy.angleInterpolation(names, keys, times, true);
}

void pickup::pickUpBalls()
{
	//王炫之 改进版
	std::vector<std::string> names;
	AL::ALValue times, keys;
	names.reserve(26);
	times.arraySetSize(26);
	keys.arraySetSize(26);

	names.push_back("HeadPitch");
	times[0].arraySetSize(7);
	keys[0].arraySetSize(7);

	times[0][0] = 3.16;
	keys[0][0] = AL::ALValue::array(0.0183661, AL::ALValue::array(3, -1.05333, 0), AL::ALValue::array(3, 0.8, 0));
	times[0][1] = 5.56;
	keys[0][1] = AL::ALValue::array(-0.656595, AL::ALValue::array(3, -0.8, 0), AL::ALValue::array(3, 0.613333, 0));
	times[0][2] = 7.4;
	keys[0][2] = AL::ALValue::array(-0.656595, AL::ALValue::array(3, -0.613333, 0), AL::ALValue::array(3, 0.706667, 0));
	times[0][3] = 9.52;
	keys[0][3] = AL::ALValue::array(-0.628982, AL::ALValue::array(3, -0.706667, 0), AL::ALValue::array(3, 0.08, 0));
	times[0][4] = 9.76;
	keys[0][4] = AL::ALValue::array(-0.628982, AL::ALValue::array(3, -0.08, 0), AL::ALValue::array(3, 1.18667, 0));
	times[0][5] = 13.32;
	keys[0][5] = AL::ALValue::array(-0.635118, AL::ALValue::array(3, -1.18667, 0), AL::ALValue::array(3, 0.986667, 0));
	times[0][6] = 16.28;
	keys[0][6] = AL::ALValue::array(-0.010472, AL::ALValue::array(3, -0.986667, 0), AL::ALValue::array(3, 0, 0));

	names.push_back("HeadYaw");
	times[1].arraySetSize(7);
	keys[1].arraySetSize(7);

	times[1][0] = 3.16;
	keys[1][0] = AL::ALValue::array(0.00916195, AL::ALValue::array(3, -1.05333, 0), AL::ALValue::array(3, 0.8, 0));
	times[1][1] = 5.56;
	keys[1][1] = AL::ALValue::array(0.024502, AL::ALValue::array(3, -0.8, 0), AL::ALValue::array(3, 0.613333, 0));
	times[1][2] = 7.4;
	keys[1][2] = AL::ALValue::array(0.024502, AL::ALValue::array(3, -0.613333, 0), AL::ALValue::array(3, 0.706667, 0));
	times[1][3] = 9.52;
	keys[1][3] = AL::ALValue::array(0.0168321, AL::ALValue::array(3, -0.706667, 0), AL::ALValue::array(3, 0.08, 0));
	times[1][4] = 9.76;
	keys[1][4] = AL::ALValue::array(0.0168321, AL::ALValue::array(3, -0.08, 0), AL::ALValue::array(3, 1.18667, 0));
	times[1][5] = 13.32;
	keys[1][5] = AL::ALValue::array(0.0152981, AL::ALValue::array(3, -1.18667, 0.000558384), AL::ALValue::array(3, 0.986667, -0.000464274));
	times[1][6] = 16.28;
	keys[1][6] = AL::ALValue::array(0.0137641, AL::ALValue::array(3, -0.986667, 0), AL::ALValue::array(3, 0, 0));

	names.push_back("LAnklePitch");
	times[2].arraySetSize(8);
	keys[2].arraySetSize(8);

	times[2][0] = 3.16;
	keys[2][0] = AL::ALValue::array(-1.18944, AL::ALValue::array(3, -1.05333, 0), AL::ALValue::array(3, 0.8, 0));
	times[2][1] = 5.56;
	keys[2][1] = AL::ALValue::array(-0.740964, AL::ALValue::array(3, -0.8, -0.138743), AL::ALValue::array(3, 0.613333, 0.10637));
	times[2][2] = 7.4;
	keys[2][2] = AL::ALValue::array(-0.454105, AL::ALValue::array(3, -0.613333, 0), AL::ALValue::array(3, 0.706667, 0));
	times[2][3] = 9.52;
	keys[2][3] = AL::ALValue::array(-0.455641, AL::ALValue::array(3, -0.706667, 0), AL::ALValue::array(3, 0.08, 0));
	times[2][4] = 9.76;
	keys[2][4] = AL::ALValue::array(-0.455641, AL::ALValue::array(3, -0.08, 0), AL::ALValue::array(3, 1.18667, 0));
	times[2][5] = 13.32;
	keys[2][5] = AL::ALValue::array(-0.780848, AL::ALValue::array(3, -1.18667, 0), AL::ALValue::array(3, 0.986667, 0));
	times[2][6] = 16.28;
	keys[2][6] = AL::ALValue::array(-0.342125, AL::ALValue::array(3, -0.986667, 0), AL::ALValue::array(3, 0.52, 0));
	times[2][7] = 17.84;
	keys[2][7] = AL::ALValue::array(-0.352862, AL::ALValue::array(3, -0.52, 0), AL::ALValue::array(3, 0, 0));

	names.push_back("LAnkleRoll");
	times[3].arraySetSize(8);
	keys[3].arraySetSize(8);

	times[3][0] = 3.16;
	keys[3][0] = AL::ALValue::array(0.023052, AL::ALValue::array(3, -1.05333, 0), AL::ALValue::array(3, 0.8, 0));
	times[3][1] = 5.56;
	keys[3][1] = AL::ALValue::array(-0.022968, AL::ALValue::array(3, -0.8, 0), AL::ALValue::array(3, 0.613333, 0));
	times[3][2] = 7.4;
	keys[3][2] = AL::ALValue::array(0.147306, AL::ALValue::array(3, -0.613333, -0.00532547), AL::ALValue::array(3, 0.706667, 0.00613587));
	times[3][3] = 9.52;
	keys[3][3] = AL::ALValue::array(0.153442, AL::ALValue::array(3, -0.706667, 0), AL::ALValue::array(3, 0.08, 0));
	times[3][4] = 9.76;
	keys[3][4] = AL::ALValue::array(0.153442, AL::ALValue::array(3, -0.08, 0), AL::ALValue::array(3, 1.18667, 0));
	times[3][5] = 13.32;
	keys[3][5] = AL::ALValue::array(0.154976, AL::ALValue::array(3, -1.18667, 0), AL::ALValue::array(3, 0.986667, 0));
	times[3][6] = 16.28;
	keys[3][6] = AL::ALValue::array(-0.00149202, AL::ALValue::array(3, -0.986667, 0.00582126), AL::ALValue::array(3, 0.52, -0.00306796));
	times[3][7] = 17.84;
	keys[3][7] = AL::ALValue::array(-0.00455999, AL::ALValue::array(3, -0.52, 0), AL::ALValue::array(3, 0, 0));

	names.push_back("LElbowRoll");
	times[4].arraySetSize(9);
	keys[4].arraySetSize(9);

	times[4][0] = 3.16;
	keys[4][0] = AL::ALValue::array(-0.987855, AL::ALValue::array(3, -1.05333, 0), AL::ALValue::array(3, 0.8, 0));
	times[4][1] = 5.56;
	keys[4][1] = AL::ALValue::array(-0.450955, AL::ALValue::array(3, -0.8, 0), AL::ALValue::array(3, 0.613333, 0));
	times[4][2] = 7.4;
	keys[4][2] = AL::ALValue::array(-0.450955, AL::ALValue::array(3, -0.613333, 0), AL::ALValue::array(3, 0.706667, 0));
	times[4][3] = 9.52;
	keys[4][3] = AL::ALValue::array(-0.375789, AL::ALValue::array(3, -0.706667, -0.0293975), AL::ALValue::array(3, 0.08, 0.00332802));
	times[4][4] = 9.76;
	keys[4][4] = AL::ALValue::array(-0.352778, AL::ALValue::array(3, -0.08, 0), AL::ALValue::array(3, 0.4, 0));
	times[4][5] = 10.96;
	keys[4][5] = AL::ALValue::array(-0.352778, AL::ALValue::array(3, -0.4, 0), AL::ALValue::array(3, 0.786667, 0));
	times[4][6] = 13.32;
	keys[4][6] = AL::ALValue::array(-0.361981, AL::ALValue::array(3, -0.786667, 0), AL::ALValue::array(3, 0.986667, 0));
	times[4][7] = 16.28;
	keys[4][7] = AL::ALValue::array(-0.34971, AL::ALValue::array(3, -0.986667, 0), AL::ALValue::array(3, 0.52, 0));
	times[4][8] = 17.84;
	keys[4][8] = AL::ALValue::array(-0.990921, AL::ALValue::array(3, -0.52, 0), AL::ALValue::array(3, 0, 0));

	names.push_back("LElbowYaw");
	times[5].arraySetSize(9);
	keys[5].arraySetSize(9);

	times[5][0] = 3.16;
	keys[5][0] = AL::ALValue::array(-1.37144, AL::ALValue::array(3, -1.05333, 0), AL::ALValue::array(3, 0.8, 0));
	times[5][1] = 5.56;
	keys[5][1] = AL::ALValue::array(0.170232, AL::ALValue::array(3, -0.8, -0.0340139), AL::ALValue::array(3, 0.613333, 0.0260773));
	times[5][2] = 7.4;
	keys[5][2] = AL::ALValue::array(0.196309, AL::ALValue::array(3, -0.613333, 0), AL::ALValue::array(3, 0.706667, 0));
	times[5][3] = 9.52;
	keys[5][3] = AL::ALValue::array(0.170232, AL::ALValue::array(3, -0.706667, 0), AL::ALValue::array(3, 0.08, 0));
	times[5][4] = 9.76;
	keys[5][4] = AL::ALValue::array(0.190175, AL::ALValue::array(3, -0.08, 0), AL::ALValue::array(3, 0.4, 0));
	times[5][5] = 10.96;
	keys[5][5] = AL::ALValue::array(0.190175, AL::ALValue::array(3, -0.4, 0), AL::ALValue::array(3, 0.786667, 0));
	times[5][6] = 13.32;
	keys[5][6] = AL::ALValue::array(0.182504, AL::ALValue::array(3, -0.786667, 0.00272206), AL::ALValue::array(3, 0.986667, -0.00341411));
	times[5][7] = 16.28;
	keys[5][7] = AL::ALValue::array(0.171766, AL::ALValue::array(3, -0.986667, 0.0107378), AL::ALValue::array(3, 0.52, -0.00565911));
	times[5][8] = 17.84;
	keys[5][8] = AL::ALValue::array(0.052114, AL::ALValue::array(3, -0.52, 0), AL::ALValue::array(3, 0, 0));

	names.push_back("LHand");
	times[6].arraySetSize(9);
	keys[6].arraySetSize(9);

	times[6][0] = 3.16;
	keys[6][0] = AL::ALValue::array(0.246, AL::ALValue::array(3, -1.05333, 0), AL::ALValue::array(3, 0.8, 0));
	times[6][1] = 5.56;
	keys[6][1] = AL::ALValue::array(0.9692, AL::ALValue::array(3, -0.8, -0.0172174), AL::ALValue::array(3, 0.613333, 0.0132));
	times[6][2] = 7.4;
	keys[6][2] = AL::ALValue::array(0.9824, AL::ALValue::array(3, -0.613333, 0), AL::ALValue::array(3, 0.706667, 0));
	times[6][3] = 9.52;
	keys[6][3] = AL::ALValue::array(0.88, AL::ALValue::array(3, -0.706667, 0.1024), AL::ALValue::array(3, 0.08, -0.0115925));
	times[6][4] = 9.76;
	keys[6][4] = AL::ALValue::array(0.5828, AL::ALValue::array(3, -0.08, 0), AL::ALValue::array(3, 0.4, 0));
	times[6][5] = 10.96;
	keys[6][5] = AL::ALValue::array(0.5828, AL::ALValue::array(3, -0.4, 0), AL::ALValue::array(3, 0.786667, 0));
	times[6][6] = 13.32;
	keys[6][6] = AL::ALValue::array(0.5916, AL::ALValue::array(3, -0.786667, -0.00189274), AL::ALValue::array(3, 0.986667, 0.00237394));
	times[6][7] = 16.28;
	keys[6][7] = AL::ALValue::array(0.5956, AL::ALValue::array(3, -0.986667, -0.00400001), AL::ALValue::array(3, 0.52, 0.00210811));
	times[6][8] = 17.84;
	keys[6][8] = AL::ALValue::array(0.6476, AL::ALValue::array(3, -0.52, 0), AL::ALValue::array(3, 0, 0));

	names.push_back("LHipPitch");
	times[7].arraySetSize(8);
	keys[7].arraySetSize(8);

	times[7][0] = 3.16;
	keys[7][0] = AL::ALValue::array(-0.569072, AL::ALValue::array(3, -1.05333, 0), AL::ALValue::array(3, 0.8, 0));
	times[7][1] = 5.56;
	keys[7][1] = AL::ALValue::array(-0.929562, AL::ALValue::array(3, -0.8, 0.169608), AL::ALValue::array(3, 0.613333, -0.130033));
	times[7][2] = 7.4;
	keys[7][2] = AL::ALValue::array(-1.468, AL::ALValue::array(3, -0.613333, 0.0013315), AL::ALValue::array(3, 0.706667, -0.00153411));
	times[7][3] = 9.52;
	keys[7][3] = AL::ALValue::array(-1.46953, AL::ALValue::array(3, -0.706667, 0), AL::ALValue::array(3, 0.08, 0));
	times[7][4] = 9.76;
	keys[7][4] = AL::ALValue::array(-1.46953, AL::ALValue::array(3, -0.08, 0), AL::ALValue::array(3, 1.18667, 0));
	times[7][5] = 13.32;
	keys[7][5] = AL::ALValue::array(-0.538392, AL::ALValue::array(3, -1.18667, -0.114387), AL::ALValue::array(3, 0.986667, 0.0951082));
	times[7][6] = 16.28;
	keys[7][6] = AL::ALValue::array(-0.443284, AL::ALValue::array(3, -0.986667, 0), AL::ALValue::array(3, 0.52, 0));
	times[7][7] = 17.84;
	keys[7][7] = AL::ALValue::array(-0.452487, AL::ALValue::array(3, -0.52, 0), AL::ALValue::array(3, 0, 0));

	names.push_back("LHipRoll");
	times[8].arraySetSize(8);
	keys[8].arraySetSize(8);

	times[8][0] = 3.16;
	keys[8][0] = AL::ALValue::array(0.039926, AL::ALValue::array(3, -1.05333, 0), AL::ALValue::array(3, 0.8, 0));
	times[8][1] = 5.56;
	keys[8][1] = AL::ALValue::array(-0.174835, AL::ALValue::array(3, -0.8, 0.0590445), AL::ALValue::array(3, 0.613333, -0.0452674));
	times[8][2] = 7.4;
	keys[8][2] = AL::ALValue::array(-0.27301, AL::ALValue::array(3, -0.613333, 0.00133152), AL::ALValue::array(3, 0.706667, -0.00153415));
	times[8][3] = 9.52;
	keys[8][3] = AL::ALValue::array(-0.274544, AL::ALValue::array(3, -0.706667, 0), AL::ALValue::array(3, 0.08, 0));
	times[8][4] = 9.76;
	keys[8][4] = AL::ALValue::array(-0.274544, AL::ALValue::array(3, -0.08, 0), AL::ALValue::array(3, 1.18667, 0));
	times[8][5] = 13.32;
	keys[8][5] = AL::ALValue::array(-0.049046, AL::ALValue::array(3, -1.18667, -0.0569556), AL::ALValue::array(3, 0.986667, 0.0473563));
	times[8][6] = 16.28;
	keys[8][6] = AL::ALValue::array(0.038392, AL::ALValue::array(3, -0.986667, 0), AL::ALValue::array(3, 0.52, 0));
	times[8][7] = 17.84;
	keys[8][7] = AL::ALValue::array(4.19617e-05, AL::ALValue::array(3, -0.52, 0), AL::ALValue::array(3, 0, 0));

	names.push_back("LHipYawPitch");
	times[9].arraySetSize(8);
	keys[9].arraySetSize(8);

	times[9][0] = 3.16;
	keys[9][0] = AL::ALValue::array(-0.612024, AL::ALValue::array(3, -1.05333, 0), AL::ALValue::array(3, 0.8, 0));
	times[9][1] = 5.56;
	keys[9][1] = AL::ALValue::array(-0.984786, AL::ALValue::array(3, -0.8, 0.0975392), AL::ALValue::array(3, 0.613333, -0.0747801));
	times[9][2] = 7.4;
	keys[9][2] = AL::ALValue::array(-1.12898, AL::ALValue::array(3, -0.613333, 0.00665759), AL::ALValue::array(3, 0.706667, -0.0076707));
	times[9][3] = 9.52;
	keys[9][3] = AL::ALValue::array(-1.13665, AL::ALValue::array(3, -0.706667, 0), AL::ALValue::array(3, 0.08, 0));
	times[9][4] = 9.76;
	keys[9][4] = AL::ALValue::array(-1.13665, AL::ALValue::array(3, -0.08, 0), AL::ALValue::array(3, 1.18667, 0));
	times[9][5] = 13.32;
	keys[9][5] = AL::ALValue::array(-1.12131, AL::ALValue::array(3, -1.18667, -0.0153397), AL::ALValue::array(3, 0.986667, 0.0127543));
	times[9][6] = 16.28;
	keys[9][6] = AL::ALValue::array(0.00157595, AL::ALValue::array(3, -0.986667, 0), AL::ALValue::array(3, 0.52, 0));
	times[9][7] = 17.84;
	keys[9][7] = AL::ALValue::array(-0.00455999, AL::ALValue::array(3, -0.52, 0), AL::ALValue::array(3, 0, 0));

	names.push_back("LKneePitch");
	times[10].arraySetSize(8);
	keys[10].arraySetSize(8);

	times[10][0] = 3.16;
	keys[10][0] = AL::ALValue::array(2.10461, AL::ALValue::array(3, -1.05333, 0), AL::ALValue::array(3, 0.8, 0));
	times[10][1] = 5.56;
	keys[10][1] = AL::ALValue::array(2.04019, AL::ALValue::array(3, -0.8, 0), AL::ALValue::array(3, 0.613333, 0));
	times[10][2] = 7.4;
	keys[10][2] = AL::ALValue::array(2.10921, AL::ALValue::array(3, -0.613333, -0.001333), AL::ALValue::array(3, 0.706667, 0.00153584));
	times[10][3] = 9.52;
	keys[10][3] = AL::ALValue::array(2.11075, AL::ALValue::array(3, -0.706667, 0), AL::ALValue::array(3, 0.08, 0));
	times[10][4] = 9.76;
	keys[10][4] = AL::ALValue::array(2.11075, AL::ALValue::array(3, -0.08, 0), AL::ALValue::array(3, 1.18667, 0));
	times[10][5] = 13.32;
	keys[10][5] = AL::ALValue::array(1.98955, AL::ALValue::array(3, -1.18667, 0.121196), AL::ALValue::array(3, 0.986667, -0.100769));
	times[10][6] = 16.28;
	keys[10][6] = AL::ALValue::array(0.696393, AL::ALValue::array(3, -0.986667, 0), AL::ALValue::array(3, 0.52, 0));
	times[10][7] = 17.84;
	keys[10][7] = AL::ALValue::array(0.708667, AL::ALValue::array(3, -0.52, 0), AL::ALValue::array(3, 0, 0));

	names.push_back("LShoulderPitch");
	times[11].arraySetSize(9);
	keys[11].arraySetSize(9);

	times[11][0] = 3.16;
	keys[11][0] = AL::ALValue::array(1.42504, AL::ALValue::array(3, -1.05333, 0), AL::ALValue::array(3, 0.8, 0));
	times[11][1] = 5.56;
	keys[11][1] = AL::ALValue::array(0.406468, AL::ALValue::array(3, -0.8, 0.122053), AL::ALValue::array(3, 0.613333, -0.0935741));
	times[11][2] = 7.4;
	keys[11][2] = AL::ALValue::array(0.312894, AL::ALValue::array(3, -0.613333, 0), AL::ALValue::array(3, 0.706667, 0));
	times[11][3] = 9.52;
	keys[11][3] = AL::ALValue::array(0.377323, AL::ALValue::array(3, -0.706667, -0.0229667), AL::ALValue::array(3, 0.08, 0.00260001));
	times[11][4] = 9.76;
	keys[11][4] = AL::ALValue::array(0.389594, AL::ALValue::array(3, -0.08, 0), AL::ALValue::array(3, 0.4, 0));
	times[11][5] = 10.96;
	keys[11][5] = AL::ALValue::array(0.389594, AL::ALValue::array(3, -0.4, 0), AL::ALValue::array(3, 0.786667, 0));
	times[11][6] = 13.32;
	keys[11][6] = AL::ALValue::array(0.44175, AL::ALValue::array(3, -0.786667, -0.0174662), AL::ALValue::array(3, 0.986667, 0.0219067));
	times[11][7] = 16.28;
	keys[11][7] = AL::ALValue::array(0.507713, AL::ALValue::array(3, -0.986667, -0.065963), AL::ALValue::array(3, 0.52, 0.0347643));
	times[11][8] = 17.84;
	keys[11][8] = AL::ALValue::array(1.02007, AL::ALValue::array(3, -0.52, 0), AL::ALValue::array(3, 0, 0));

	names.push_back("LShoulderRoll");
	times[12].arraySetSize(9);
	keys[12].arraySetSize(9);

	times[12][0] = 3.16;
	keys[12][0] = AL::ALValue::array(0.288349, AL::ALValue::array(3, -1.05333, 0), AL::ALValue::array(3, 0.8, 0));
	times[12][1] = 5.56;
	keys[12][1] = AL::ALValue::array(0.133416, AL::ALValue::array(3, -0.8, 0), AL::ALValue::array(3, 0.613333, 0));
	times[12][2] = 7.4;
	keys[12][2] = AL::ALValue::array(0.136484, AL::ALValue::array(3, -0.613333, 0), AL::ALValue::array(3, 0.706667, 0));
	times[12][3] = 9.52;
	keys[12][3] = AL::ALValue::array(-0.093616, AL::ALValue::array(3, -0.706667, 0), AL::ALValue::array(3, 0.08, 0));
	times[12][4] = 9.76;
	keys[12][4] = AL::ALValue::array(-0.0567998, AL::ALValue::array(3, -0.08, 0), AL::ALValue::array(3, 0.4, 0));
	times[12][5] = 10.96;
	keys[12][5] = AL::ALValue::array(-0.0567998, AL::ALValue::array(3, -0.4, 0), AL::ALValue::array(3, 0.786667, 0));
	times[12][6] = 13.32;
	keys[12][6] = AL::ALValue::array(-0.046062, AL::ALValue::array(3, -0.786667, -0.00362929), AL::ALValue::array(3, 0.986667, 0.004552));
	times[12][7] = 16.28;
	keys[12][7] = AL::ALValue::array(-0.032256, AL::ALValue::array(3, -0.986667, -0.0138061), AL::ALValue::array(3, 0.52, 0.00727617));
	times[12][8] = 17.84;
	keys[12][8] = AL::ALValue::array(0.095066, AL::ALValue::array(3, -0.52, 0), AL::ALValue::array(3, 0, 0));

	names.push_back("LWristYaw");
	times[13].arraySetSize(9);
	keys[13].arraySetSize(9);

	times[13][0] = 3.16;
	keys[13][0] = AL::ALValue::array(-0.00771189, AL::ALValue::array(3, -1.05333, 0), AL::ALValue::array(3, 0.8, 0));
	times[13][1] = 5.56;
	keys[13][1] = AL::ALValue::array(-1.62148, AL::ALValue::array(3, -0.8, 0.0460198), AL::ALValue::array(3, 0.613333, -0.0352818));
	times[13][2] = 7.4;
	keys[13][2] = AL::ALValue::array(-1.65676, AL::ALValue::array(3, -0.613333, 0.030411), AL::ALValue::array(3, 0.706667, -0.0350388));
	times[13][3] = 9.52;
	keys[13][3] = AL::ALValue::array(-1.81783, AL::ALValue::array(3, -0.706667, 0), AL::ALValue::array(3, 0.08, 0));
	times[13][4] = 9.76;
	keys[13][4] = AL::ALValue::array(-1.78715, AL::ALValue::array(3, -0.08, 0), AL::ALValue::array(3, 0.4, 0));
	times[13][5] = 10.96;
	keys[13][5] = AL::ALValue::array(-1.78715, AL::ALValue::array(3, -0.4, 0), AL::ALValue::array(3, 0.786667, 0));
	times[13][6] = 13.32;
	keys[13][6] = AL::ALValue::array(-1.78715, AL::ALValue::array(3, -0.786667, 0), AL::ALValue::array(3, 0.986667, 0));
	times[13][7] = 16.28;
	keys[13][7] = AL::ALValue::array(-1.7979, AL::ALValue::array(3, -0.986667, 0.00703298), AL::ALValue::array(3, 0.52, -0.00370657));
	times[13][8] = 17.84;
	keys[13][8] = AL::ALValue::array(-1.81937, AL::ALValue::array(3, -0.52, 0), AL::ALValue::array(3, 0, 0));

	names.push_back("RAnklePitch");
	times[14].arraySetSize(8);
	keys[14].arraySetSize(8);

	times[14][0] = 3.16;
	keys[14][0] = AL::ALValue::array(-1.18421, AL::ALValue::array(3, -1.05333, 0), AL::ALValue::array(3, 0.8, 0));
	times[14][1] = 5.56;
	keys[14][1] = AL::ALValue::array(-0.716335, AL::ALValue::array(3, -0.8, -0.12793), AL::ALValue::array(3, 0.613333, 0.0980794));
	times[14][2] = 7.4;
	keys[14][2] = AL::ALValue::array(-0.506179, AL::ALValue::array(3, -0.613333, 0), AL::ALValue::array(3, 0.706667, 0));
	times[14][3] = 9.52;
	keys[14][3] = AL::ALValue::array(-0.51845, AL::ALValue::array(3, -0.706667, 0), AL::ALValue::array(3, 0.08, 0));
	times[14][4] = 9.76;
	keys[14][4] = AL::ALValue::array(-0.51845, AL::ALValue::array(3, -0.08, 0), AL::ALValue::array(3, 1.18667, 0));
	times[14][5] = 13.32;
	keys[14][5] = AL::ALValue::array(-0.828318, AL::ALValue::array(3, -1.18667, 0), AL::ALValue::array(3, 0.986667, 0));
	times[14][6] = 16.28;
	keys[14][6] = AL::ALValue::array(-0.345107, AL::ALValue::array(3, -0.986667, 0), AL::ALValue::array(3, 0.52, 0));
	times[14][7] = 17.84;
	keys[14][7] = AL::ALValue::array(-0.358915, AL::ALValue::array(3, -0.52, 0), AL::ALValue::array(3, 0, 0));

	names.push_back("RAnkleRoll");
	times[15].arraySetSize(8);
	keys[15].arraySetSize(8);

	times[15][0] = 3.16;
	keys[15][0] = AL::ALValue::array(0.046062, AL::ALValue::array(3, -1.05333, 0), AL::ALValue::array(3, 0.8, 0));
	times[15][1] = 5.56;
	keys[15][1] = AL::ALValue::array(0.047596, AL::ALValue::array(3, -0.8, 0), AL::ALValue::array(3, 0.613333, 0));
	times[15][2] = 7.4;
	keys[15][2] = AL::ALValue::array(-0.139552, AL::ALValue::array(3, -0.613333, 0), AL::ALValue::array(3, 0.706667, 0));
	times[15][3] = 9.52;
	keys[15][3] = AL::ALValue::array(-0.131882, AL::ALValue::array(3, -0.706667, 0), AL::ALValue::array(3, 0.08, 0));
	times[15][4] = 9.76;
	keys[15][4] = AL::ALValue::array(-0.131882, AL::ALValue::array(3, -0.08, 0), AL::ALValue::array(3, 1.18667, 0));
	times[15][5] = 13.32;
	keys[15][5] = AL::ALValue::array(-0.178554, AL::ALValue::array(3, -1.18667, 0), AL::ALValue::array(3, 0.986667, 0));
	times[15][6] = 16.28;
	keys[15][6] = AL::ALValue::array(0.00464392, AL::ALValue::array(3, -0.986667, 0), AL::ALValue::array(3, 0.52, 0));
	times[15][7] = 17.84;
	keys[15][7] = AL::ALValue::array(0.00464392, AL::ALValue::array(3, -0.52, 0), AL::ALValue::array(3, 0, 0));

	names.push_back("RElbowRoll");
	times[16].arraySetSize(9);
	keys[16].arraySetSize(9);

	times[16][0] = 3.16;
	keys[16][0] = AL::ALValue::array(1.00328, AL::ALValue::array(3, -1.05333, 0), AL::ALValue::array(3, 0.8, 0));
	times[16][1] = 5.56;
	keys[16][1] = AL::ALValue::array(0.308375, AL::ALValue::array(3, -0.8, 0), AL::ALValue::array(3, 0.613333, 0));
	times[16][2] = 7.4;
	keys[16][2] = AL::ALValue::array(0.34826, AL::ALValue::array(3, -0.613333, 0), AL::ALValue::array(3, 0.706667, 0));
	times[16][3] = 9.52;
	keys[16][3] = AL::ALValue::array(0.257754, AL::ALValue::array(3, -0.706667, 0.0335314), AL::ALValue::array(3, 0.08, -0.003796));
	times[16][4] = 9.76;
	keys[16][4] = AL::ALValue::array(0.236277, AL::ALValue::array(3, -0.08, 0), AL::ALValue::array(3, 0.4, 0));
	times[16][5] = 10.96;
	keys[16][5] = AL::ALValue::array(0.236277, AL::ALValue::array(3, -0.4, 0), AL::ALValue::array(3, 0.786667, 0));
	times[16][6] = 13.32;
	keys[16][6] = AL::ALValue::array(0.243948, AL::ALValue::array(3, -0.786667, 0), AL::ALValue::array(3, 0.986667, 0));
	times[16][7] = 16.28;
	keys[16][7] = AL::ALValue::array(0.236277, AL::ALValue::array(3, -0.986667, 0), AL::ALValue::array(3, 0.52, 0));
	times[16][8] = 17.84;
	keys[16][8] = AL::ALValue::array(0.957259, AL::ALValue::array(3, -0.52, 0), AL::ALValue::array(3, 0, 0));

	names.push_back("RElbowYaw");
	times[17].arraySetSize(9);
	keys[17].arraySetSize(9);

	times[17][0] = 3.16;
	keys[17][0] = AL::ALValue::array(1.36522, AL::ALValue::array(3, -1.05333, 0), AL::ALValue::array(3, 0.8, 0));
	times[17][1] = 5.56;
	keys[17][1] = AL::ALValue::array(0.452487, AL::ALValue::array(3, -0.8, 0.0440187), AL::ALValue::array(3, 0.613333, -0.0337477));
	times[17][2] = 7.4;
	keys[17][2] = AL::ALValue::array(0.418739, AL::ALValue::array(3, -0.613333, 0.0337477), AL::ALValue::array(3, 0.706667, -0.0388832));
	times[17][3] = 9.52;
	keys[17][3] = AL::ALValue::array(-0.11049, AL::ALValue::array(3, -0.706667, 0), AL::ALValue::array(3, 0.08, 0));
	times[17][4] = 9.76;
	keys[17][4] = AL::ALValue::array(-0.093616, AL::ALValue::array(3, -0.08, 0), AL::ALValue::array(3, 0.4, 0));
	times[17][5] = 10.96;
	keys[17][5] = AL::ALValue::array(-0.093616, AL::ALValue::array(3, -0.4, 0), AL::ALValue::array(3, 0.786667, 0));
	times[17][6] = 13.32;
	keys[17][6] = AL::ALValue::array(-0.113558, AL::ALValue::array(3, -0.786667, 0.00499031), AL::ALValue::array(3, 0.986667, -0.00625903));
	times[17][7] = 16.28;
	keys[17][7] = AL::ALValue::array(-0.127364, AL::ALValue::array(3, -0.986667, 0), AL::ALValue::array(3, 0.52, 0));
	times[17][8] = 17.84;
	keys[17][8] = AL::ALValue::array(-0.0414601, AL::ALValue::array(3, -0.52, 0), AL::ALValue::array(3, 0, 0));

	names.push_back("RHand");
	times[18].arraySetSize(9);
	keys[18].arraySetSize(9);

	times[18][0] = 3.16;
	keys[18][0] = AL::ALValue::array(0.2464, AL::ALValue::array(3, -1.05333, 0), AL::ALValue::array(3, 0.8, 0));
	times[18][1] = 5.56;
	keys[18][1] = AL::ALValue::array(0.9772, AL::ALValue::array(3, -0.8, -0.00886964), AL::ALValue::array(3, 0.613333, 0.00680006));
	times[18][2] = 7.4;
	keys[18][2] = AL::ALValue::array(0.984, AL::ALValue::array(3, -0.613333, 0), AL::ALValue::array(3, 0.706667, 0));
	times[18][3] = 9.52;
	keys[18][3] = AL::ALValue::array(0.96, AL::ALValue::array(3, -0.706667, 0.024), AL::ALValue::array(3, 0.08, -0.00271699));
	times[18][4] = 9.76;
	keys[18][4] = AL::ALValue::array(0.6684, AL::ALValue::array(3, -0.08, 0), AL::ALValue::array(3, 0.4, 0));
	times[18][5] = 10.96;
	keys[18][5] = AL::ALValue::array(0.6684, AL::ALValue::array(3, -0.4, 0), AL::ALValue::array(3, 0.786667, 0));
	times[18][6] = 13.32;
	keys[18][6] = AL::ALValue::array(0.684, AL::ALValue::array(3, -0.786667, -0.00390376), AL::ALValue::array(3, 0.986667, 0.00489625));
	times[18][7] = 16.28;
	keys[18][7] = AL::ALValue::array(0.6948, AL::ALValue::array(3, -0.986667, -0.00314336), AL::ALValue::array(3, 0.52, 0.00165664));
	times[18][8] = 17.84;
	keys[18][8] = AL::ALValue::array(0.6984, AL::ALValue::array(3, -0.52, 0), AL::ALValue::array(3, 0, 0));

	names.push_back("RHipPitch");
	times[19].arraySetSize(8);
	keys[19].arraySetSize(8);

	times[19][0] = 3.16;
	keys[19][0] = AL::ALValue::array(-0.589097, AL::ALValue::array(3, -1.05333, 0), AL::ALValue::array(3, 0.8, 0));
	times[19][1] = 5.56;
	keys[19][1] = AL::ALValue::array(-1.23338, AL::ALValue::array(3, -0.8, 0.0800333), AL::ALValue::array(3, 0.613333, -0.0613588));
	times[19][2] = 7.4;
	keys[19][2] = AL::ALValue::array(-1.29474, AL::ALValue::array(3, -0.613333, 0), AL::ALValue::array(3, 0.706667, 0));
	times[19][3] = 9.52;
	keys[19][3] = AL::ALValue::array(-1.29474, AL::ALValue::array(3, -0.706667, 0), AL::ALValue::array(3, 0.08, 0));
	times[19][4] = 9.76;
	keys[19][4] = AL::ALValue::array(-1.29474, AL::ALValue::array(3, -0.08, 0), AL::ALValue::array(3, 1.18667, 0));
	times[19][5] = 13.32;
	keys[19][5] = AL::ALValue::array(-0.713353, AL::ALValue::array(3, -1.18667, -0.154953), AL::ALValue::array(3, 0.986667, 0.128837));
	times[19][6] = 16.28;
	keys[19][6] = AL::ALValue::array(-0.443368, AL::ALValue::array(3, -0.986667, 0), AL::ALValue::array(3, 0.52, 0));
	times[19][7] = 17.84;
	keys[19][7] = AL::ALValue::array(-0.451038, AL::ALValue::array(3, -0.52, 0), AL::ALValue::array(3, 0, 0));

	names.push_back("RHipRoll");
	times[20].arraySetSize(8);
	keys[20].arraySetSize(8);

	times[20][0] = 3.16;
	keys[20][0] = AL::ALValue::array(-0.240796, AL::ALValue::array(3, -1.05333, 0), AL::ALValue::array(3, 0.8, 0));
	times[20][1] = 5.56;
	keys[20][1] = AL::ALValue::array(-0.0367741, AL::ALValue::array(3, -0.8, -0.0908822), AL::ALValue::array(3, 0.613333, 0.0696764));
	times[20][2] = 7.4;
	keys[20][2] = AL::ALValue::array(0.24088, AL::ALValue::array(3, -0.613333, 0), AL::ALValue::array(3, 0.706667, 0));
	times[20][3] = 9.52;
	keys[20][3] = AL::ALValue::array(0.239346, AL::ALValue::array(3, -0.706667, 0), AL::ALValue::array(3, 0.08, 0));
	times[20][4] = 9.76;
	keys[20][4] = AL::ALValue::array(0.239346, AL::ALValue::array(3, -0.08, 0), AL::ALValue::array(3, 1.18667, 0));
	times[20][5] = 13.32;
	keys[20][5] = AL::ALValue::array(-0.033706, AL::ALValue::array(3, -1.18667, 0), AL::ALValue::array(3, 0.986667, 0));
	times[20][6] = 16.28;
	keys[20][6] = AL::ALValue::array(-0.0106959, AL::ALValue::array(3, -0.986667, -0.00636225), AL::ALValue::array(3, 0.52, 0.00335308));
	times[20][7] = 17.84;
	keys[20][7] = AL::ALValue::array(-0.00455999, AL::ALValue::array(3, -0.52, 0), AL::ALValue::array(3, 0, 0));

	names.push_back("RHipYawPitch");
	times[21].arraySetSize(8);
	keys[21].arraySetSize(8);

	times[21][0] = 3.16;
	keys[21][0] = AL::ALValue::array(-0.612024, AL::ALValue::array(3, -1.05333, 0), AL::ALValue::array(3, 0.8, 0));
	times[21][1] = 5.56;
	keys[21][1] = AL::ALValue::array(-0.984786, AL::ALValue::array(3, -0.8, 0.0975392), AL::ALValue::array(3, 0.613333, -0.0747801));
	times[21][2] = 7.4;
	keys[21][2] = AL::ALValue::array(-1.12898, AL::ALValue::array(3, -0.613333, 0.00665759), AL::ALValue::array(3, 0.706667, -0.0076707));
	times[21][3] = 9.52;
	keys[21][3] = AL::ALValue::array(-1.13665, AL::ALValue::array(3, -0.706667, 0), AL::ALValue::array(3, 0.08, 0));
	times[21][4] = 9.76;
	keys[21][4] = AL::ALValue::array(-1.13665, AL::ALValue::array(3, -0.08, 0), AL::ALValue::array(3, 1.18667, 0));
	times[21][5] = 13.32;
	keys[21][5] = AL::ALValue::array(-1.12131, AL::ALValue::array(3, -1.18667, -0.0153397), AL::ALValue::array(3, 0.986667, 0.0127543));
	times[21][6] = 16.28;
	keys[21][6] = AL::ALValue::array(0.00157595, AL::ALValue::array(3, -0.986667, 0), AL::ALValue::array(3, 0.52, 0));
	times[21][7] = 17.84;
	keys[21][7] = AL::ALValue::array(-0.00455999, AL::ALValue::array(3, -0.52, 0), AL::ALValue::array(3, 0, 0));

	names.push_back("RKneePitch");
	times[22].arraySetSize(8);
	keys[22].arraySetSize(8);

	times[22][0] = 3.16;
	keys[22][0] = AL::ALValue::array(2.11255, AL::ALValue::array(3, -1.05333, 0), AL::ALValue::array(3, 0.8, 0));
	times[22][1] = 5.56;
	keys[22][1] = AL::ALValue::array(2.10316, AL::ALValue::array(3, -0.8, 0), AL::ALValue::array(3, 0.613333, 0));
	times[22][2] = 7.4;
	keys[22][2] = AL::ALValue::array(2.10469, AL::ALValue::array(3, -0.613333, -0.00145433), AL::ALValue::array(3, 0.706667, 0.00167564));
	times[22][3] = 9.52;
	keys[22][3] = AL::ALValue::array(2.11255, AL::ALValue::array(3, -0.706667, 0), AL::ALValue::array(3, 0.08, 0));
	times[22][4] = 9.76;
	keys[22][4] = AL::ALValue::array(2.11255, AL::ALValue::array(3, -0.08, 0), AL::ALValue::array(3, 1.18667, 0));
	times[22][5] = 13.32;
	keys[22][5] = AL::ALValue::array(2.09549, AL::ALValue::array(3, -1.18667, 0.0170518), AL::ALValue::array(3, 0.986667, -0.0141779));
	times[22][6] = 16.28;
	keys[22][6] = AL::ALValue::array(0.698011, AL::ALValue::array(3, -0.986667, 0), AL::ALValue::array(3, 0.52, 0));
	times[22][7] = 17.84;
	keys[22][7] = AL::ALValue::array(0.699545, AL::ALValue::array(3, -0.52, 0), AL::ALValue::array(3, 0, 0));

	names.push_back("RShoulderPitch");
	times[23].arraySetSize(9);
	keys[23].arraySetSize(9);

	times[23][0] = 3.16;
	keys[23][0] = AL::ALValue::array(1.42206, AL::ALValue::array(3, -1.05333, 0), AL::ALValue::array(3, 0.8, 0));
	times[23][1] = 5.56;
	keys[23][1] = AL::ALValue::array(0.668866, AL::ALValue::array(3, -0.8, 0.114049), AL::ALValue::array(3, 0.613333, -0.0874375));
	times[23][2] = 7.4;
	keys[23][2] = AL::ALValue::array(0.581429, AL::ALValue::array(3, -0.613333, 0.0375391), AL::ALValue::array(3, 0.706667, -0.0432516));
	times[23][3] = 9.52;
	keys[23][3] = AL::ALValue::array(0.426494, AL::ALValue::array(3, -0.706667, 0), AL::ALValue::array(3, 0.08, 0));
	times[23][4] = 9.76;
	keys[23][4] = AL::ALValue::array(0.469446, AL::ALValue::array(3, -0.08, 0), AL::ALValue::array(3, 0.4, 0));
	times[23][5] = 10.96;
	keys[23][5] = AL::ALValue::array(0.469446, AL::ALValue::array(3, -0.4, 0), AL::ALValue::array(3, 0.786667, 0));
	times[23][6] = 13.32;
	keys[23][6] = AL::ALValue::array(0.526205, AL::ALValue::array(3, -0.786667, -0.0172391), AL::ALValue::array(3, 0.986667, 0.0216219));
	times[23][7] = 16.28;
	keys[23][7] = AL::ALValue::array(0.586029, AL::ALValue::array(3, -0.986667, -0.0598246), AL::ALValue::array(3, 0.52, 0.0315292));
	times[23][8] = 17.84;
	keys[23][8] = AL::ALValue::array(1.00941, AL::ALValue::array(3, -0.52, 0), AL::ALValue::array(3, 0, 0));

	names.push_back("RShoulderRoll");
	times[24].arraySetSize(9);
	keys[24].arraySetSize(9);

	times[24][0] = 3.16;
	keys[24][0] = AL::ALValue::array(-0.276162, AL::ALValue::array(3, -1.05333, 0), AL::ALValue::array(3, 0.8, 0));
	times[24][1] = 5.56;
	keys[24][1] = AL::ALValue::array(-0.131966, AL::ALValue::array(3, -0.8, -0.0555713), AL::ALValue::array(3, 0.613333, 0.0426047));
	times[24][2] = 7.4;
	keys[24][2] = AL::ALValue::array(0.0183661, AL::ALValue::array(3, -0.613333, -0.064862), AL::ALValue::array(3, 0.706667, 0.0747323));
	times[24][3] = 9.52;
	keys[24][3] = AL::ALValue::array(0.286817, AL::ALValue::array(3, -0.706667, 0), AL::ALValue::array(3, 0.08, 0));
	times[24][4] = 9.76;
	keys[24][4] = AL::ALValue::array(0.23466, AL::ALValue::array(3, -0.08, 0), AL::ALValue::array(3, 0.4, 0));
	times[24][5] = 10.96;
	keys[24][5] = AL::ALValue::array(0.23466, AL::ALValue::array(3, -0.4, 0), AL::ALValue::array(3, 0.786667, 0));
	times[24][6] = 13.32;
	keys[24][6] = AL::ALValue::array(0.210117, AL::ALValue::array(3, -0.786667, 0), AL::ALValue::array(3, 0.986667, 0));
	times[24][7] = 16.28;
	keys[24][7] = AL::ALValue::array(0.217786, AL::ALValue::array(3, -0.986667, 0), AL::ALValue::array(3, 0.52, 0));
	times[24][8] = 17.84;
	keys[24][8] = AL::ALValue::array(-0.130432, AL::ALValue::array(3, -0.52, 0), AL::ALValue::array(3, 0, 0));

	names.push_back("RWristYaw");
	times[25].arraySetSize(9);
	keys[25].arraySetSize(9);

	times[25][0] = 3.16;
	keys[25][0] = AL::ALValue::array(-0.0138481, AL::ALValue::array(3, -1.05333, 0), AL::ALValue::array(3, 0.8, 0));
	times[25][1] = 5.56;
	keys[25][1] = AL::ALValue::array(1.35601, AL::ALValue::array(3, -0.8, -0.0240103), AL::ALValue::array(3, 0.613333, 0.0184079));
	times[25][2] = 7.4;
	keys[25][2] = AL::ALValue::array(1.37442, AL::ALValue::array(3, -0.613333, -0.0184079), AL::ALValue::array(3, 0.706667, 0.0212091));
	times[25][3] = 9.52;
	keys[25][3] = AL::ALValue::array(1.78093, AL::ALValue::array(3, -0.706667, 0), AL::ALValue::array(3, 0.08, 0));
	times[25][4] = 9.76;
	keys[25][4] = AL::ALValue::array(1.75332, AL::ALValue::array(3, -0.08, 0), AL::ALValue::array(3, 0.4, 0));
	times[25][5] = 10.96;
	keys[25][5] = AL::ALValue::array(1.75332, AL::ALValue::array(3, -0.4, 0), AL::ALValue::array(3, 0.786667, 0));
	times[25][6] = 13.32;
	keys[25][6] = AL::ALValue::array(1.76406, AL::ALValue::array(3, -0.786667, 0), AL::ALValue::array(3, 0.986667, 0));
	times[25][7] = 16.28;
	keys[25][7] = AL::ALValue::array(1.76406, AL::ALValue::array(3, -0.986667, 0), AL::ALValue::array(3, 0.52, 0));
	times[25][8] = 17.84;
	keys[25][8] = AL::ALValue::array(1.75639, AL::ALValue::array(3, -0.52, 0), AL::ALValue::array(3, 0, 0));

	try
	{
		getParentBroker()->getMotionProxy()->angleInterpolationBezier(names, times, keys);
	}
	catch (const std::exception &)
	{
	}
}

void pickup::throwOutBalls()
{
	std::vector<std::string> names;
	AL::ALValue times, keys;
	names.reserve(26);
	times.arraySetSize(26);
	keys.arraySetSize(26);

	names.push_back("HeadPitch");
	times[0].arraySetSize(4);
	keys[0].arraySetSize(4);

	times[0][0] = 1.64;
	keys[0][0] = AL::ALValue::array(0.377323, AL::ALValue::array(3, -0.546667, 0), AL::ALValue::array(3, 0.546667, 0));
	times[0][1] = 3.28;
	keys[0][1] = AL::ALValue::array(-0.289967, AL::ALValue::array(3, -0.546667, 0), AL::ALValue::array(3, 0.226667, 0));
	times[0][2] = 3.96;
	keys[0][2] = AL::ALValue::array(-0.289967, AL::ALValue::array(3, -0.226667, 0), AL::ALValue::array(3, 0.866667, 0));
	times[0][3] = 6.56;
	keys[0][3] = AL::ALValue::array(0.0168321, AL::ALValue::array(3, -0.866667, 0), AL::ALValue::array(3, 0, 0));

	names.push_back("HeadYaw");
	times[1].arraySetSize(4);
	keys[1].arraySetSize(4);

	times[1][0] = 1.64;
	keys[1][0] = AL::ALValue::array(-0.00771189, AL::ALValue::array(3, -0.546667, 0), AL::ALValue::array(3, 0.546667, 0));
	times[1][1] = 3.28;
	keys[1][1] = AL::ALValue::array(-0.00771189, AL::ALValue::array(3, -0.546667, 0), AL::ALValue::array(3, 0.226667, 0));
	times[1][2] = 3.96;
	keys[1][2] = AL::ALValue::array(-0.00771189, AL::ALValue::array(3, -0.226667, 0), AL::ALValue::array(3, 0.866667, 0));
	times[1][3] = 6.56;
	keys[1][3] = AL::ALValue::array(-0.00771189, AL::ALValue::array(3, -0.866667, 0), AL::ALValue::array(3, 0, 0));

	names.push_back("LAnklePitch");
	times[2].arraySetSize(4);
	keys[2].arraySetSize(4);

	times[2][0] = 1.64;
	keys[2][0] = AL::ALValue::array(-0.352862, AL::ALValue::array(3, -0.546667, 0), AL::ALValue::array(3, 0.546667, 0));
	times[2][1] = 3.28;
	keys[2][1] = AL::ALValue::array(-0.282298, AL::ALValue::array(3, -0.546667, 0), AL::ALValue::array(3, 0.226667, 0));
	times[2][2] = 3.96;
	keys[2][2] = AL::ALValue::array(-0.282298, AL::ALValue::array(3, -0.226667, 0), AL::ALValue::array(3, 0.866667, 0));
	times[2][3] = 6.56;
	keys[2][3] = AL::ALValue::array(-0.349794, AL::ALValue::array(3, -0.866667, 0), AL::ALValue::array(3, 0, 0));

	names.push_back("LAnkleRoll");
	times[3].arraySetSize(4);
	keys[3].arraySetSize(4);

	times[3][0] = 1.64;
	keys[3][0] = AL::ALValue::array(-0.00455999, AL::ALValue::array(3, -0.546667, 0), AL::ALValue::array(3, 0.546667, 0));
	times[3][1] = 3.28;
	keys[3][1] = AL::ALValue::array(0.0353239, AL::ALValue::array(3, -0.546667, 0), AL::ALValue::array(3, 0.226667, 0));
	times[3][2] = 3.96;
	keys[3][2] = AL::ALValue::array(0.0353239, AL::ALValue::array(3, -0.226667, 0), AL::ALValue::array(3, 0.866667, 0));
	times[3][3] = 6.56;
	keys[3][3] = AL::ALValue::array(4.19617e-05, AL::ALValue::array(3, -0.866667, 0), AL::ALValue::array(3, 0, 0));

	names.push_back("LElbowRoll");
	times[4].arraySetSize(4);
	keys[4].arraySetSize(4);

	times[4][0] = 1.64;
	keys[4][0] = AL::ALValue::array(-0.990921, AL::ALValue::array(3, -0.546667, 0), AL::ALValue::array(3, 0.546667, 0));
	times[4][1] = 3.28;
	keys[4][1] = AL::ALValue::array(-0.921892, AL::ALValue::array(3, -0.546667, -0.0212898), AL::ALValue::array(3, 0.226667, 0.00882747));
	times[4][2] = 3.96;
	keys[4][2] = AL::ALValue::array(-0.900569, AL::ALValue::array(3, -0.226667, 0), AL::ALValue::array(3, 0.866667, 0));
	times[4][3] = 6.56;
	keys[4][3] = AL::ALValue::array(-0.952573, AL::ALValue::array(3, -0.866667, 0), AL::ALValue::array(3, 0, 0));

	names.push_back("LElbowYaw");
	times[5].arraySetSize(4);
	keys[5].arraySetSize(4);

	times[5][0] = 1.64;
	keys[5][0] = AL::ALValue::array(0.052114, AL::ALValue::array(3, -0.546667, 0), AL::ALValue::array(3, 0.546667, 0));
	times[5][1] = 3.28;
	keys[5][1] = AL::ALValue::array(-0.30224, AL::ALValue::array(3, -0.546667, 0), AL::ALValue::array(3, 0.226667, 0));
	times[5][2] = 3.96;
	keys[5][2] = AL::ALValue::array(-0.292901, AL::ALValue::array(3, -0.226667, 0), AL::ALValue::array(3, 0.866667, 0));
	times[5][3] = 6.56;
	keys[5][3] = AL::ALValue::array(-1.34076, AL::ALValue::array(3, -0.866667, 0), AL::ALValue::array(3, 0, 0));

	names.push_back("LHand");
	times[6].arraySetSize(4);
	keys[6].arraySetSize(4);

	times[6][0] = 1.64;
	keys[6][0] = AL::ALValue::array(0.6476, AL::ALValue::array(3, -0.546667, 0), AL::ALValue::array(3, 0.546667, 0));
	times[6][1] = 3.28;
	keys[6][1] = AL::ALValue::array(0.6456, AL::ALValue::array(3, -0.546667, 0), AL::ALValue::array(3, 0.226667, 0));
	times[6][2] = 3.96;
	keys[6][2] = AL::ALValue::array(1, AL::ALValue::array(3, -0.226667, 0), AL::ALValue::array(3, 0.866667, 0));
	times[6][3] = 6.56;
	keys[6][3] = AL::ALValue::array(0.9988, AL::ALValue::array(3, -0.866667, 0), AL::ALValue::array(3, 0, 0));

	names.push_back("LHipPitch");
	times[7].arraySetSize(4);
	keys[7].arraySetSize(4);

	times[7][0] = 1.64;
	keys[7][0] = AL::ALValue::array(-0.452487, AL::ALValue::array(3, -0.546667, 0), AL::ALValue::array(3, 0.546667, 0));
	times[7][1] = 3.28;
	keys[7][1] = AL::ALValue::array(-0.450955, AL::ALValue::array(3, -0.546667, 0), AL::ALValue::array(3, 0.226667, 0));
	times[7][2] = 3.96;
	keys[7][2] = AL::ALValue::array(-0.450955, AL::ALValue::array(3, -0.226667, 0), AL::ALValue::array(3, 0.866667, 0));
	times[7][3] = 6.56;
	keys[7][3] = AL::ALValue::array(-0.460158, AL::ALValue::array(3, -0.866667, 0), AL::ALValue::array(3, 0, 0));

	names.push_back("LHipRoll");
	times[8].arraySetSize(4);
	keys[8].arraySetSize(4);

	times[8][0] = 1.64;
	keys[8][0] = AL::ALValue::array(4.19617e-05, AL::ALValue::array(3, -0.546667, 0), AL::ALValue::array(3, 0.546667, 0));
	times[8][1] = 3.28;
	keys[8][1] = AL::ALValue::array(0.00771189, AL::ALValue::array(3, -0.546667, 0), AL::ALValue::array(3, 0.226667, 0));
	times[8][2] = 3.96;
	keys[8][2] = AL::ALValue::array(0.00771189, AL::ALValue::array(3, -0.226667, 0), AL::ALValue::array(3, 0.866667, 0));
	times[8][3] = 6.56;
	keys[8][3] = AL::ALValue::array(0.00157595, AL::ALValue::array(3, -0.866667, 0), AL::ALValue::array(3, 0, 0));

	names.push_back("LHipYawPitch");
	times[9].arraySetSize(4);
	keys[9].arraySetSize(4);

	times[9][0] = 1.64;
	keys[9][0] = AL::ALValue::array(-0.00455999, AL::ALValue::array(3, -0.546667, 0), AL::ALValue::array(3, 0.546667, 0));
	times[9][1] = 3.28;
	keys[9][1] = AL::ALValue::array(0.00617791, AL::ALValue::array(3, -0.546667, 0), AL::ALValue::array(3, 0.226667, 0));
	times[9][2] = 3.96;
	keys[9][2] = AL::ALValue::array(0.00617791, AL::ALValue::array(3, -0.226667, 0), AL::ALValue::array(3, 0.866667, 0));
	times[9][3] = 6.56;
	keys[9][3] = AL::ALValue::array(0.00617791, AL::ALValue::array(3, -0.866667, 0), AL::ALValue::array(3, 0, 0));

	names.push_back("LKneePitch");
	times[10].arraySetSize(4);
	keys[10].arraySetSize(4);

	times[10][0] = 1.64;
	keys[10][0] = AL::ALValue::array(0.708667, AL::ALValue::array(3, -0.546667, 0), AL::ALValue::array(3, 0.546667, 0));
	times[10][1] = 3.28;
	keys[10][1] = AL::ALValue::array(0.759288, AL::ALValue::array(3, -0.546667, 0), AL::ALValue::array(3, 0.226667, 0));
	times[10][2] = 3.96;
	keys[10][2] = AL::ALValue::array(0.759288, AL::ALValue::array(3, -0.226667, 0), AL::ALValue::array(3, 0.866667, 0));
	times[10][3] = 6.56;
	keys[10][3] = AL::ALValue::array(0.694859, AL::ALValue::array(3, -0.866667, 0), AL::ALValue::array(3, 0, 0));

	names.push_back("LShoulderPitch");
	times[11].arraySetSize(4);
	keys[11].arraySetSize(4);

	times[11][0] = 1.64;
	keys[11][0] = AL::ALValue::array(1.02007, AL::ALValue::array(3, -0.546667, 0), AL::ALValue::array(3, 0.546667, 0));
	times[11][1] = 3.28;
	keys[11][1] = AL::ALValue::array(0.391128, AL::ALValue::array(3, -0.546667, 0), AL::ALValue::array(3, 0.226667, 0));
	times[11][2] = 3.96;
	keys[11][2] = AL::ALValue::array(0.418177, AL::ALValue::array(3, -0.226667, -0.0270492), AL::ALValue::array(3, 0.866667, 0.103423));
	times[11][3] = 6.56;
	keys[11][3] = AL::ALValue::array(1.39283, AL::ALValue::array(3, -0.866667, 0), AL::ALValue::array(3, 0, 0));

	names.push_back("LShoulderRoll");
	times[12].arraySetSize(4);
	keys[12].arraySetSize(4);

	times[12][0] = 1.64;
	keys[12][0] = AL::ALValue::array(0.095066, AL::ALValue::array(3, -0.546667, 0), AL::ALValue::array(3, 0.546667, 0));
	times[12][1] = 3.28;
	keys[12][1] = AL::ALValue::array(0.116542, AL::ALValue::array(3, -0.546667, -0.0214761), AL::ALValue::array(3, 0.226667, 0.00890473));
	times[12][2] = 3.96;
	keys[12][2] = AL::ALValue::array(0.356366, AL::ALValue::array(3, -0.226667, 0), AL::ALValue::array(3, 0.866667, 0));
	times[12][3] = 6.56;
	keys[12][3] = AL::ALValue::array(0.24233, AL::ALValue::array(3, -0.866667, 0), AL::ALValue::array(3, 0, 0));

	names.push_back("LWristYaw");
	times[13].arraySetSize(4);
	keys[13].arraySetSize(4);

	times[13][0] = 1.64;
	keys[13][0] = AL::ALValue::array(-1.81937, AL::ALValue::array(3, -0.546667, 0), AL::ALValue::array(3, 0.546667, 0));
	times[13][1] = 3.28;
	keys[13][1] = AL::ALValue::array(-1.25639, AL::ALValue::array(3, -0.546667, 0), AL::ALValue::array(3, 0.226667, 0));
	times[13][2] = 3.96;
	keys[13][2] = AL::ALValue::array(-1.26606, AL::ALValue::array(3, -0.226667, 0), AL::ALValue::array(3, 0.866667, 0));
	times[13][3] = 6.56;
	keys[13][3] = AL::ALValue::array(-0.023052, AL::ALValue::array(3, -0.866667, 0), AL::ALValue::array(3, 0, 0));

	names.push_back("RAnklePitch");
	times[14].arraySetSize(4);
	keys[14].arraySetSize(4);

	times[14][0] = 1.64;
	keys[14][0] = AL::ALValue::array(-0.358915, AL::ALValue::array(3, -0.546667, 0), AL::ALValue::array(3, 0.546667, 0));
	times[14][1] = 3.28;
	keys[14][1] = AL::ALValue::array(-0.274544, AL::ALValue::array(3, -0.546667, 0), AL::ALValue::array(3, 0.226667, 0));
	times[14][2] = 3.96;
	keys[14][2] = AL::ALValue::array(-0.274544, AL::ALValue::array(3, -0.226667, 0), AL::ALValue::array(3, 0.866667, 0));
	times[14][3] = 6.56;
	keys[14][3] = AL::ALValue::array(-0.360449, AL::ALValue::array(3, -0.866667, 0), AL::ALValue::array(3, 0, 0));

	names.push_back("RAnkleRoll");
	times[15].arraySetSize(4);
	keys[15].arraySetSize(4);

	times[15][0] = 1.64;
	keys[15][0] = AL::ALValue::array(0.00464392, AL::ALValue::array(3, -0.546667, 0), AL::ALValue::array(3, 0.546667, 0));
	times[15][1] = 3.28;
	keys[15][1] = AL::ALValue::array(4.19617e-05, AL::ALValue::array(3, -0.546667, 0), AL::ALValue::array(3, 0.226667, 0));
	times[15][2] = 3.96;
	keys[15][2] = AL::ALValue::array(4.19617e-05, AL::ALValue::array(3, -0.226667, 0), AL::ALValue::array(3, 0.866667, 0));
	times[15][3] = 6.56;
	keys[15][3] = AL::ALValue::array(0.00157595, AL::ALValue::array(3, -0.866667, 0), AL::ALValue::array(3, 0, 0));

	names.push_back("RElbowRoll");
	times[16].arraySetSize(4);
	keys[16].arraySetSize(4);

	times[16][0] = 1.64;
	keys[16][0] = AL::ALValue::array(0.957259, AL::ALValue::array(3, -0.546667, 0), AL::ALValue::array(3, 0.546667, 0));
	times[16][1] = 3.28;
	keys[16][1] = AL::ALValue::array(0.900499, AL::ALValue::array(3, -0.546667, 0.0202109), AL::ALValue::array(3, 0.226667, -0.00838015));
	times[16][2] = 3.96;
	keys[16][2] = AL::ALValue::array(0.871486, AL::ALValue::array(3, -0.226667, 0), AL::ALValue::array(3, 0.866667, 0));
	times[16][3] = 6.56;
	keys[16][3] = AL::ALValue::array(0.951122, AL::ALValue::array(3, -0.866667, 0), AL::ALValue::array(3, 0, 0));

	names.push_back("RElbowYaw");
	times[17].arraySetSize(4);
	keys[17].arraySetSize(4);

	times[17][0] = 1.64;
	keys[17][0] = AL::ALValue::array(-0.0414601, AL::ALValue::array(3, -0.546667, 0), AL::ALValue::array(3, 0.546667, 0));
	times[17][1] = 3.28;
	keys[17][1] = AL::ALValue::array(0.429478, AL::ALValue::array(3, -0.546667, 0), AL::ALValue::array(3, 0.226667, 0));
	times[17][2] = 3.96;
	keys[17][2] = AL::ALValue::array(0.422778, AL::ALValue::array(3, -0.226667, 0), AL::ALValue::array(3, 0.866667, 0));
	times[17][3] = 6.56;
	keys[17][3] = AL::ALValue::array(1.33607, AL::ALValue::array(3, -0.866667, 0), AL::ALValue::array(3, 0, 0));

	names.push_back("RHand");
	times[18].arraySetSize(4);
	keys[18].arraySetSize(4);

	times[18][0] = 1.64;
	keys[18][0] = AL::ALValue::array(0.6984, AL::ALValue::array(3, -0.546667, 0), AL::ALValue::array(3, 0.546667, 0));
	times[18][1] = 3.28;
	keys[18][1] = AL::ALValue::array(0.696, AL::ALValue::array(3, -0.546667, 0), AL::ALValue::array(3, 0.226667, 0));
	times[18][2] = 3.96;
	keys[18][2] = AL::ALValue::array(0.9996, AL::ALValue::array(3, -0.226667, -0.000104617), AL::ALValue::array(3, 0.866667, 0.000400007));
	times[18][3] = 6.56;
	keys[18][3] = AL::ALValue::array(1, AL::ALValue::array(3, -0.866667, 0), AL::ALValue::array(3, 0, 0));

	names.push_back("RHipPitch");
	times[19].arraySetSize(4);
	keys[19].arraySetSize(4);

	times[19][0] = 1.64;
	keys[19][0] = AL::ALValue::array(-0.451038, AL::ALValue::array(3, -0.546667, 0), AL::ALValue::array(3, 0.546667, 0));
	times[19][1] = 3.28;
	keys[19][1] = AL::ALValue::array(-0.395814, AL::ALValue::array(3, -0.546667, 0), AL::ALValue::array(3, 0.226667, 0));
	times[19][2] = 3.96;
	keys[19][2] = AL::ALValue::array(-0.395814, AL::ALValue::array(3, -0.226667, 0), AL::ALValue::array(3, 0.866667, 0));
	times[19][3] = 6.56;
	keys[19][3] = AL::ALValue::array(-0.451038, AL::ALValue::array(3, -0.866667, 0), AL::ALValue::array(3, 0, 0));

	names.push_back("RHipRoll");
	times[20].arraySetSize(4);
	keys[20].arraySetSize(4);

	times[20][0] = 1.64;
	keys[20][0] = AL::ALValue::array(-0.00455999, AL::ALValue::array(3, -0.546667, 0), AL::ALValue::array(3, 0.546667, 0));
	times[20][1] = 3.28;
	keys[20][1] = AL::ALValue::array(-0.0106959, AL::ALValue::array(3, -0.546667, 0), AL::ALValue::array(3, 0.226667, 0));
	times[20][2] = 3.96;
	keys[20][2] = AL::ALValue::array(-0.0106959, AL::ALValue::array(3, -0.226667, 0), AL::ALValue::array(3, 0.866667, 0));
	times[20][3] = 6.56;
	keys[20][3] = AL::ALValue::array(4.19617e-05, AL::ALValue::array(3, -0.866667, 0), AL::ALValue::array(3, 0, 0));

	names.push_back("RHipYawPitch");
	times[21].arraySetSize(4);
	keys[21].arraySetSize(4);

	times[21][0] = 1.64;
	keys[21][0] = AL::ALValue::array(-0.00455999, AL::ALValue::array(3, -0.546667, 0), AL::ALValue::array(3, 0.546667, 0));
	times[21][1] = 3.28;
	keys[21][1] = AL::ALValue::array(0.00617791, AL::ALValue::array(3, -0.546667, 0), AL::ALValue::array(3, 0.226667, 0));
	times[21][2] = 3.96;
	keys[21][2] = AL::ALValue::array(0.00617791, AL::ALValue::array(3, -0.226667, 0), AL::ALValue::array(3, 0.866667, 0));
	times[21][3] = 6.56;
	keys[21][3] = AL::ALValue::array(0.00617791, AL::ALValue::array(3, -0.866667, 0), AL::ALValue::array(3, 0, 0));

	names.push_back("RKneePitch");
	times[22].arraySetSize(4);
	keys[22].arraySetSize(4);

	times[22][0] = 1.64;
	keys[22][0] = AL::ALValue::array(0.699545, AL::ALValue::array(3, -0.546667, 0), AL::ALValue::array(3, 0.546667, 0));
	times[22][1] = 3.28;
	keys[22][1] = AL::ALValue::array(0.691876, AL::ALValue::array(3, -0.546667, 0), AL::ALValue::array(3, 0.226667, 0));
	times[22][2] = 3.96;
	keys[22][2] = AL::ALValue::array(0.691876, AL::ALValue::array(3, -0.226667, 0), AL::ALValue::array(3, 0.866667, 0));
	times[22][3] = 6.56;
	keys[22][3] = AL::ALValue::array(0.690342, AL::ALValue::array(3, -0.866667, 0), AL::ALValue::array(3, 0, 0));

	names.push_back("RShoulderPitch");
	times[23].arraySetSize(4);
	keys[23].arraySetSize(4);

	times[23][0] = 1.64;
	keys[23][0] = AL::ALValue::array(1.00941, AL::ALValue::array(3, -0.546667, 0), AL::ALValue::array(3, 0.546667, 0));
	times[23][1] = 3.28;
	keys[23][1] = AL::ALValue::array(0.461776, AL::ALValue::array(3, -0.546667, 0), AL::ALValue::array(3, 0.226667, 0));
	times[23][2] = 3.96;
	keys[23][2] = AL::ALValue::array(0.51064, AL::ALValue::array(3, -0.226667, -0.0488641), AL::ALValue::array(3, 0.866667, 0.186834));
	times[23][3] = 6.56;
	keys[23][3] = AL::ALValue::array(1.39598, AL::ALValue::array(3, -0.866667, 0), AL::ALValue::array(3, 0, 0));

	names.push_back("RShoulderRoll");
	times[24].arraySetSize(4);
	keys[24].arraySetSize(4);

	times[24][0] = 1.64;
	keys[24][0] = AL::ALValue::array(-0.130432, AL::ALValue::array(3, -0.546667, 0), AL::ALValue::array(3, 0.546667, 0));
	times[24][1] = 3.28;
	keys[24][1] = AL::ALValue::array(-0.04913, AL::ALValue::array(3, -0.546667, 0), AL::ALValue::array(3, 0.226667, 0));
	times[24][2] = 3.96;
	keys[24][2] = AL::ALValue::array(-0.35487, AL::ALValue::array(3, -0.226667, 0), AL::ALValue::array(3, 0.866667, 0));
	times[24][3] = 6.56;
	keys[24][3] = AL::ALValue::array(-0.243948, AL::ALValue::array(3, -0.866667, 0), AL::ALValue::array(3, 0, 0));

	names.push_back("RWristYaw");
	times[25].arraySetSize(4);
	keys[25].arraySetSize(4);

	times[25][0] = 1.64;
	keys[25][0] = AL::ALValue::array(1.75639, AL::ALValue::array(3, -0.546667, 0), AL::ALValue::array(3, 0.546667, 0));
	times[25][1] = 3.28;
	keys[25][1] = AL::ALValue::array(1.20415, AL::ALValue::array(3, -0.546667, 0.00229555), AL::ALValue::array(3, 0.226667, -0.000951814));
	times[25][2] = 3.96;
	keys[25][2] = AL::ALValue::array(1.2032, AL::ALValue::array(3, -0.226667, 0.000951814), AL::ALValue::array(3, 0.866667, -0.00363929));
	times[25][3] = 6.56;
	keys[25][3] = AL::ALValue::array(0.06592, AL::ALValue::array(3, -0.866667, 0), AL::ALValue::array(3, 0, 0));

	try
	{
		getParentBroker()->getMotionProxy()->angleInterpolationBezier(names, times, keys);
	}
	catch (const std::exception &)
	{
	}
}

/*void pickup::pickUpBalls()
{
	//王炫之版本
	std::vector<std::string> names;
	AL::ALValue times, keys;
	names.reserve(26);
	times.arraySetSize(26);
	keys.arraySetSize(26);

	names.push_back("HeadPitch");
	times[0].arraySetSize(7);
	keys[0].arraySetSize(7);

	times[0][0] = 3.16;
	keys[0][0] = AL::ALValue::array(0.0183661, AL::ALValue::array(3, -1.05333, 0), AL::ALValue::array(3, 0.8, 0));
	times[0][1] = 5.56;
	keys[0][1] = AL::ALValue::array(-0.656595, AL::ALValue::array(3, -0.8, 0), AL::ALValue::array(3, 0.6, 0));
	times[0][2] = 7.36;
	keys[0][2] = AL::ALValue::array(-0.656595, AL::ALValue::array(3, -0.6, 0), AL::ALValue::array(3, 0.733333, 0));
	times[0][3] = 9.56;
	keys[0][3] = AL::ALValue::array(-0.628982, AL::ALValue::array(3, -0.733333, 0), AL::ALValue::array(3, 0.08, 0));
	times[0][4] = 9.8;
	keys[0][4] = AL::ALValue::array(-0.628982, AL::ALValue::array(3, -0.08, 0), AL::ALValue::array(3, 1.18667, 0));
	times[0][5] = 13.36;
	keys[0][5] = AL::ALValue::array(-0.635118, AL::ALValue::array(3, -1.18667, 0), AL::ALValue::array(3, 1, 0));
	times[0][6] = 16.36;
	keys[0][6] = AL::ALValue::array(-0.010472, AL::ALValue::array(3, -1, 0), AL::ALValue::array(3, 0, 0));

	names.push_back("HeadYaw");
	times[1].arraySetSize(7);
	keys[1].arraySetSize(7);

	times[1][0] = 3.16;
	keys[1][0] = AL::ALValue::array(0.00916195, AL::ALValue::array(3, -1.05333, 0), AL::ALValue::array(3, 0.8, 0));
	times[1][1] = 5.56;
	keys[1][1] = AL::ALValue::array(0.024502, AL::ALValue::array(3, -0.8, 0), AL::ALValue::array(3, 0.6, 0));
	times[1][2] = 7.36;
	keys[1][2] = AL::ALValue::array(0.024502, AL::ALValue::array(3, -0.6, 0), AL::ALValue::array(3, 0.733333, 0));
	times[1][3] = 9.56;
	keys[1][3] = AL::ALValue::array(0.0168321, AL::ALValue::array(3, -0.733333, 0), AL::ALValue::array(3, 0.08, 0));
	times[1][4] = 9.8;
	keys[1][4] = AL::ALValue::array(0.0168321, AL::ALValue::array(3, -0.08, 0), AL::ALValue::array(3, 1.18667, 0));
	times[1][5] = 13.36;
	keys[1][5] = AL::ALValue::array(0.0152981, AL::ALValue::array(3, -1.18667, 0.000554979), AL::ALValue::array(3, 1, -0.000467679));
	times[1][6] = 16.36;
	keys[1][6] = AL::ALValue::array(0.0137641, AL::ALValue::array(3, -1, 0), AL::ALValue::array(3, 0, 0));

	names.push_back("LAnklePitch");
	times[2].arraySetSize(8);
	keys[2].arraySetSize(8);

	times[2][0] = 3.16;
	keys[2][0] = AL::ALValue::array(-1.18944, AL::ALValue::array(3, -1.05333, 0), AL::ALValue::array(3, 0.8, 0));
	times[2][1] = 5.56;
	keys[2][1] = AL::ALValue::array(-0.740964, AL::ALValue::array(3, -0.8, -0.140064), AL::ALValue::array(3, 0.6, 0.105048));
	times[2][2] = 7.36;
	keys[2][2] = AL::ALValue::array(-0.454105, AL::ALValue::array(3, -0.6, 0), AL::ALValue::array(3, 0.733333, 0));
	times[2][3] = 9.56;
	keys[2][3] = AL::ALValue::array(-0.455641, AL::ALValue::array(3, -0.733333, 0), AL::ALValue::array(3, 0.08, 0));
	times[2][4] = 9.8;
	keys[2][4] = AL::ALValue::array(-0.455641, AL::ALValue::array(3, -0.08, 0), AL::ALValue::array(3, 1.18667, 0));
	times[2][5] = 13.36;
	keys[2][5] = AL::ALValue::array(-0.780848, AL::ALValue::array(3, -1.18667, 0), AL::ALValue::array(3, 1, 0));
	times[2][6] = 16.36;
	keys[2][6] = AL::ALValue::array(-0.342125, AL::ALValue::array(3, -1, 0), AL::ALValue::array(3, 0.56, 0));
	times[2][7] = 18.04;
	keys[2][7] = AL::ALValue::array(-0.347174, AL::ALValue::array(3, -0.56, 0), AL::ALValue::array(3, 0, 0));

	names.push_back("LAnkleRoll");
	times[3].arraySetSize(8);
	keys[3].arraySetSize(8);

	times[3][0] = 3.16;
	keys[3][0] = AL::ALValue::array(0.023052, AL::ALValue::array(3, -1.05333, 0), AL::ALValue::array(3, 0.8, 0));
	times[3][1] = 5.56;
	keys[3][1] = AL::ALValue::array(-0.022968, AL::ALValue::array(3, -0.8, 0), AL::ALValue::array(3, 0.6, 0));
	times[3][2] = 7.36;
	keys[3][2] = AL::ALValue::array(0.147306, AL::ALValue::array(3, -0.6, -0.00502026), AL::ALValue::array(3, 0.733333, 0.00613587));
	times[3][3] = 9.56;
	keys[3][3] = AL::ALValue::array(0.153442, AL::ALValue::array(3, -0.733333, 0), AL::ALValue::array(3, 0.08, 0));
	times[3][4] = 9.8;
	keys[3][4] = AL::ALValue::array(0.153442, AL::ALValue::array(3, -0.08, 0), AL::ALValue::array(3, 1.18667, 0));
	times[3][5] = 13.36;
	keys[3][5] = AL::ALValue::array(0.154976, AL::ALValue::array(3, -1.18667, 0), AL::ALValue::array(3, 1, 0));
	times[3][6] = 16.36;
	keys[3][6] = AL::ALValue::array(-0.00149202, AL::ALValue::array(3, -1, 0), AL::ALValue::array(3, 0.56, 0));
	times[3][7] = 18.04;
	keys[3][7] = AL::ALValue::array(0.000308941, AL::ALValue::array(3, -0.56, 0), AL::ALValue::array(3, 0, 0));

	names.push_back("LElbowRoll");
	times[4].arraySetSize(9);
	keys[4].arraySetSize(9);

	times[4][0] = 3.16;
	keys[4][0] = AL::ALValue::array(-0.987855, AL::ALValue::array(3, -1.05333, 0), AL::ALValue::array(3, 0.8, 0));
	times[4][1] = 5.56;
	keys[4][1] = AL::ALValue::array(-0.450955, AL::ALValue::array(3, -0.8, 0), AL::ALValue::array(3, 0.6, 0));
	times[4][2] = 7.36;
	keys[4][2] = AL::ALValue::array(-0.450955, AL::ALValue::array(3, -0.6, 0), AL::ALValue::array(3, 0.733333, 0));
	times[4][3] = 9.56;
	keys[4][3] = AL::ALValue::array(-0.582879, AL::ALValue::array(3, -0.733333, 0.0502532), AL::ALValue::array(3, 0.08, -0.00548217));
	times[4][4] = 9.8;
	keys[4][4] = AL::ALValue::array(-0.618161, AL::ALValue::array(3, -0.08, 0), AL::ALValue::array(3, 0.4, 0));
	times[4][5] = 11;
	keys[4][5] = AL::ALValue::array(-0.53379, AL::ALValue::array(3, -0.4, -0.00857993), AL::ALValue::array(3, 0.786667, 0.0168739));
	times[4][6] = 13.36;
	keys[4][6] = AL::ALValue::array(-0.516916, AL::ALValue::array(3, -0.786667, 0), AL::ALValue::array(3, 1, 0));
	times[4][7] = 16.36;
	keys[4][7] = AL::ALValue::array(-0.516916, AL::ALValue::array(3, -1, 0), AL::ALValue::array(3, 0.56, 0));
	times[4][8] = 18.04;
	keys[4][8] = AL::ALValue::array(-0.507712, AL::ALValue::array(3, -0.56, 0), AL::ALValue::array(3, 0, 0));

	names.push_back("LElbowYaw");
	times[5].arraySetSize(9);
	keys[5].arraySetSize(9);

	times[5][0] = 3.16;
	keys[5][0] = AL::ALValue::array(-1.37144, AL::ALValue::array(3, -1.05333, 0), AL::ALValue::array(3, 0.8, 0));
	times[5][1] = 5.56;
	keys[5][1] = AL::ALValue::array(0.170232, AL::ALValue::array(3, -0.8, -0.0347697), AL::ALValue::array(3, 0.6, 0.0260773));
	times[5][2] = 7.36;
	keys[5][2] = AL::ALValue::array(0.196309, AL::ALValue::array(3, -0.6, 0), AL::ALValue::array(3, 0.733333, 0));
	times[5][3] = 9.56;
	keys[5][3] = AL::ALValue::array(0.167164, AL::ALValue::array(3, -0.733333, 0.0291453), AL::ALValue::array(3, 0.08, -0.00317948));
	times[5][4] = 9.8;
	keys[5][4] = AL::ALValue::array(0.095066, AL::ALValue::array(3, -0.08, 0.00122717), AL::ALValue::array(3, 0.4, -0.00613587));
	times[5][5] = 11;
	keys[5][5] = AL::ALValue::array(0.0889301, AL::ALValue::array(3, -0.4, 0), AL::ALValue::array(3, 0.786667, 0));
	times[5][6] = 13.36;
	keys[5][6] = AL::ALValue::array(0.0966001, AL::ALValue::array(3, -0.786667, 0), AL::ALValue::array(3, 1, 0));
	times[5][7] = 16.36;
	keys[5][7] = AL::ALValue::array(0.0966001, AL::ALValue::array(3, -1, 0), AL::ALValue::array(3, 0.56, 0));
	times[5][8] = 18.04;
	keys[5][8] = AL::ALValue::array(0.110406, AL::ALValue::array(3, -0.56, 0), AL::ALValue::array(3, 0, 0));

	names.push_back("LHand");
	times[6].arraySetSize(9);
	keys[6].arraySetSize(9);

	times[6][0] = 3.16;
	keys[6][0] = AL::ALValue::array(0.246, AL::ALValue::array(3, -1.05333, 0), AL::ALValue::array(3, 0.8, 0));
	times[6][1] = 5.56;
	keys[6][1] = AL::ALValue::array(0.9692, AL::ALValue::array(3, -0.8, -0.0176), AL::ALValue::array(3, 0.6, 0.0132));
	times[6][2] = 7.36;
	keys[6][2] = AL::ALValue::array(0.9824, AL::ALValue::array(3, -0.6, 0), AL::ALValue::array(3, 0.733333, 0));
	times[6][3] = 9.56;
	keys[6][3] = AL::ALValue::array(0.9716, AL::ALValue::array(3, -0.733333, 0.0108), AL::ALValue::array(3, 0.08, -0.00117818));
	times[6][4] = 9.8;
	keys[6][4] = AL::ALValue::array(0.6676, AL::ALValue::array(3, -0.08, 0.00575999), AL::ALValue::array(3, 0.4, -0.0288));
	times[6][5] = 11;
	keys[6][5] = AL::ALValue::array(0.6388, AL::ALValue::array(3, -0.4, 0.00440449), AL::ALValue::array(3, 0.786667, -0.00866216));
	times[6][6] = 13.36;
	keys[6][6] = AL::ALValue::array(0.6284, AL::ALValue::array(3, -0.786667, 0), AL::ALValue::array(3, 1, 0));
	times[6][7] = 16.36;
	keys[6][7] = AL::ALValue::array(0.6284, AL::ALValue::array(3, -1, 0), AL::ALValue::array(3, 0.56, 0));
	times[6][8] = 18.04;
	keys[6][8] = AL::ALValue::array(0.6384, AL::ALValue::array(3, -0.56, 0), AL::ALValue::array(3, 0, 0));

	names.push_back("LHipPitch");
	times[7].arraySetSize(8);
	keys[7].arraySetSize(8);

	times[7][0] = 3.16;
	keys[7][0] = AL::ALValue::array(-0.569072, AL::ALValue::array(3, -1.05333, 0), AL::ALValue::array(3, 0.8, 0));
	times[7][1] = 5.56;
	keys[7][1] = AL::ALValue::array(-0.929562, AL::ALValue::array(3, -0.8, 0.171224), AL::ALValue::array(3, 0.6, -0.128418));
	times[7][2] = 7.36;
	keys[7][2] = AL::ALValue::array(-1.468, AL::ALValue::array(3, -0.6, 0.00125518), AL::ALValue::array(3, 0.733333, -0.00153411));
	times[7][3] = 9.56;
	keys[7][3] = AL::ALValue::array(-1.46953, AL::ALValue::array(3, -0.733333, 0), AL::ALValue::array(3, 0.08, 0));
	times[7][4] = 9.8;
	keys[7][4] = AL::ALValue::array(-1.46953, AL::ALValue::array(3, -0.08, 0), AL::ALValue::array(3, 1.18667, 0));
	times[7][5] = 13.36;
	keys[7][5] = AL::ALValue::array(-0.538392, AL::ALValue::array(3, -1.18667, -0.112862), AL::ALValue::array(3, 1, 0.0951082));
	times[7][6] = 16.36;
	keys[7][6] = AL::ALValue::array(-0.443284, AL::ALValue::array(3, -1, 0), AL::ALValue::array(3, 0.56, 0));
	times[7][7] = 18.04;
	keys[7][7] = AL::ALValue::array(-0.444827, AL::ALValue::array(3, -0.56, 0), AL::ALValue::array(3, 0, 0));

	names.push_back("LHipRoll");
	times[8].arraySetSize(8);
	keys[8].arraySetSize(8);

	times[8][0] = 3.16;
	keys[8][0] = AL::ALValue::array(0.039926, AL::ALValue::array(3, -1.05333, 0), AL::ALValue::array(3, 0.8, 0));
	times[8][1] = 5.56;
	keys[8][1] = AL::ALValue::array(-0.174835, AL::ALValue::array(3, -0.8, 0.0596068), AL::ALValue::array(3, 0.6, -0.0447051));
	times[8][2] = 7.36;
	keys[8][2] = AL::ALValue::array(-0.27301, AL::ALValue::array(3, -0.6, 0.00125521), AL::ALValue::array(3, 0.733333, -0.00153415));
	times[8][3] = 9.56;
	keys[8][3] = AL::ALValue::array(-0.274544, AL::ALValue::array(3, -0.733333, 0), AL::ALValue::array(3, 0.08, 0));
	times[8][4] = 9.8;
	keys[8][4] = AL::ALValue::array(-0.274544, AL::ALValue::array(3, -0.08, 0), AL::ALValue::array(3, 1.18667, 0));
	times[8][5] = 13.36;
	keys[8][5] = AL::ALValue::array(-0.049046, AL::ALValue::array(3, -1.18667, -0.0566083), AL::ALValue::array(3, 1, 0.0477036));
	times[8][6] = 16.36;
	keys[8][6] = AL::ALValue::array(0.038392, AL::ALValue::array(3, -1, 0), AL::ALValue::array(3, 0.56, 0));
	times[8][7] = 18.04;
	keys[8][7] = AL::ALValue::array(0.0352925, AL::ALValue::array(3, -0.56, 0), AL::ALValue::array(3, 0, 0));

	names.push_back("LHipYawPitch");
	times[9].arraySetSize(8);
	keys[9].arraySetSize(8);

	times[9][0] = 3.16;
	keys[9][0] = AL::ALValue::array(-0.612024, AL::ALValue::array(3, -1.05333, 0), AL::ALValue::array(3, 0.8, 0));
	times[9][1] = 5.56;
	keys[9][1] = AL::ALValue::array(-0.984786, AL::ALValue::array(3, -0.8, 0.0984681), AL::ALValue::array(3, 0.6, -0.0738511));
	times[9][2] = 7.36;
	keys[9][2] = AL::ALValue::array(-1.12898, AL::ALValue::array(3, -0.6, 0.00627603), AL::ALValue::array(3, 0.733333, -0.0076707));
	times[9][3] = 9.56;
	keys[9][3] = AL::ALValue::array(-1.13665, AL::ALValue::array(3, -0.733333, 0), AL::ALValue::array(3, 0.08, 0));
	times[9][4] = 9.8;
	keys[9][4] = AL::ALValue::array(-1.13665, AL::ALValue::array(3, -0.08, 0), AL::ALValue::array(3, 1.18667, 0));
	times[9][5] = 13.36;
	keys[9][5] = AL::ALValue::array(-1.12131, AL::ALValue::array(3, -1.18667, -0.0153397), AL::ALValue::array(3, 1, 0.0129267));
	times[9][6] = 16.36;
	keys[9][6] = AL::ALValue::array(0.00157595, AL::ALValue::array(3, -1, 0), AL::ALValue::array(3, 0.56, 0));
	times[9][7] = 18.04;
	keys[9][7] = AL::ALValue::array(-0.00467966, AL::ALValue::array(3, -0.56, 0), AL::ALValue::array(3, 0, 0));

	names.push_back("LKneePitch");
	times[10].arraySetSize(8);
	keys[10].arraySetSize(8);

	times[10][0] = 3.16;
	keys[10][0] = AL::ALValue::array(2.10461, AL::ALValue::array(3, -1.05333, 0), AL::ALValue::array(3, 0.8, 0));
	times[10][1] = 5.56;
	keys[10][1] = AL::ALValue::array(2.04019, AL::ALValue::array(3, -0.8, 0), AL::ALValue::array(3, 0.6, 0));
	times[10][2] = 7.36;
	keys[10][2] = AL::ALValue::array(2.10921, AL::ALValue::array(3, -0.6, -0.0012566), AL::ALValue::array(3, 0.733333, 0.00153584));
	times[10][3] = 9.56;
	keys[10][3] = AL::ALValue::array(2.11075, AL::ALValue::array(3, -0.733333, 0), AL::ALValue::array(3, 0.08, 0));
	times[10][4] = 9.8;
	keys[10][4] = AL::ALValue::array(2.11075, AL::ALValue::array(3, -0.08, 0), AL::ALValue::array(3, 1.18667, 0));
	times[10][5] = 13.36;
	keys[10][5] = AL::ALValue::array(1.98955, AL::ALValue::array(3, -1.18667, 0.121196), AL::ALValue::array(3, 1, -0.102131));
	times[10][6] = 16.36;
	keys[10][6] = AL::ALValue::array(0.696393, AL::ALValue::array(3, -1, 0), AL::ALValue::array(3, 0.56, 0));
	times[10][7] = 18.04;
	keys[10][7] = AL::ALValue::array(0.702828, AL::ALValue::array(3, -0.56, 0), AL::ALValue::array(3, 0, 0));

	names.push_back("LShoulderPitch");
	times[11].arraySetSize(9);
	keys[11].arraySetSize(9);

	times[11][0] = 3.16;
	keys[11][0] = AL::ALValue::array(1.42504, AL::ALValue::array(3, -1.05333, 0), AL::ALValue::array(3, 0.8, 0));
	times[11][1] = 5.56;
	keys[11][1] = AL::ALValue::array(0.406468, AL::ALValue::array(3, -0.8, 0.124765), AL::ALValue::array(3, 0.6, -0.0935741));
	times[11][2] = 7.36;
	keys[11][2] = AL::ALValue::array(0.312894, AL::ALValue::array(3, -0.6, 0), AL::ALValue::array(3, 0.733333, 0));
	times[11][3] = 9.56;
	keys[11][3] = AL::ALValue::array(0.455555, AL::ALValue::array(3, -0.733333, -0.0470261), AL::ALValue::array(3, 0.08, 0.00513012));
	times[11][4] = 9.8;
	keys[11][4] = AL::ALValue::array(0.469363, AL::ALValue::array(3, -0.08, -0.003068), AL::ALValue::array(3, 0.4, 0.01534));
	times[11][5] = 11;
	keys[11][5] = AL::ALValue::array(0.510779, AL::ALValue::array(3, -0.4, -0.00775613), AL::ALValue::array(3, 0.786667, 0.0152537));
	times[11][6] = 13.36;
	keys[11][6] = AL::ALValue::array(0.538392, AL::ALValue::array(3, -0.786667, 0), AL::ALValue::array(3, 1, 0));
	times[11][7] = 16.36;
	keys[11][7] = AL::ALValue::array(0.538392, AL::ALValue::array(3, -1, 0), AL::ALValue::array(3, 0.56, 0));
	times[11][8] = 18.04;
	keys[11][8] = AL::ALValue::array(0.547596, AL::ALValue::array(3, -0.56, 0), AL::ALValue::array(3, 0, 0));

	names.push_back("LShoulderRoll");
	times[12].arraySetSize(9);
	keys[12].arraySetSize(9);

	times[12][0] = 3.16;
	keys[12][0] = AL::ALValue::array(0.288349, AL::ALValue::array(3, -1.05333, 0), AL::ALValue::array(3, 0.8, 0));
	times[12][1] = 5.56;
	keys[12][1] = AL::ALValue::array(0.133416, AL::ALValue::array(3, -0.8, 0), AL::ALValue::array(3, 0.6, 0));
	times[12][2] = 7.36;
	keys[12][2] = AL::ALValue::array(0.136484, AL::ALValue::array(3, -0.6, 0), AL::ALValue::array(3, 0.733333, 0));
	times[12][3] = 9.56;
	keys[12][3] = AL::ALValue::array(-0.00771189, AL::ALValue::array(3, -0.733333, 0), AL::ALValue::array(3, 0.08, 0));
	times[12][4] = 9.8;
	keys[12][4] = AL::ALValue::array(0.0152981, AL::ALValue::array(3, -0.08, 0), AL::ALValue::array(3, 0.4, 0));
	times[12][5] = 11;
	keys[12][5] = AL::ALValue::array(-0.073674, AL::ALValue::array(3, -0.4, 0.00467991), AL::ALValue::array(3, 0.786667, -0.00920382));
	times[12][6] = 13.36;
	keys[12][6] = AL::ALValue::array(-0.0828778, AL::ALValue::array(3, -0.786667, 0), AL::ALValue::array(3, 1, 0));
	times[12][7] = 16.36;
	keys[12][7] = AL::ALValue::array(-0.0828778, AL::ALValue::array(3, -1, 0), AL::ALValue::array(3, 0.56, 0));
	times[12][8] = 18.04;
	keys[12][8] = AL::ALValue::array(-0.092082, AL::ALValue::array(3, -0.56, 0), AL::ALValue::array(3, 0, 0));

	names.push_back("LWristYaw");
	times[13].arraySetSize(9);
	keys[13].arraySetSize(9);

	times[13][0] = 3.16;
	keys[13][0] = AL::ALValue::array(-0.00771189, AL::ALValue::array(3, -1.05333, 0), AL::ALValue::array(3, 0.8, 0));
	times[13][1] = 5.56;
	keys[13][1] = AL::ALValue::array(-1.62148, AL::ALValue::array(3, -0.8, 0.0470424), AL::ALValue::array(3, 0.6, -0.0352818));
	times[13][2] = 7.36;
	keys[13][2] = AL::ALValue::array(-1.65676, AL::ALValue::array(3, -0.6, 0.0246196), AL::ALValue::array(3, 0.733333, -0.0300906));
	times[13][3] = 9.56;
	keys[13][3] = AL::ALValue::array(-1.78561, AL::ALValue::array(3, -0.733333, 0.0502233), AL::ALValue::array(3, 0.08, -0.0054789));
	times[13][4] = 9.8;
	keys[13][4] = AL::ALValue::array(-1.82387, AL::ALValue::array(3, -0.08, 0), AL::ALValue::array(3, 0.4, 0));
	times[13][5] = 11;
	keys[13][5] = AL::ALValue::array(-1.82387, AL::ALValue::array(3, -0.4, 0), AL::ALValue::array(3, 0.786667, 0));
	times[13][6] = 13.36;
	keys[13][6] = AL::ALValue::array(-1.79942, AL::ALValue::array(3, -0.786667, 0), AL::ALValue::array(3, 1, 0));
	times[13][7] = 16.36;
	keys[13][7] = AL::ALValue::array(-1.79942, AL::ALValue::array(3, -1, 0), AL::ALValue::array(3, 0.56, 0));
	times[13][8] = 18.04;
	keys[13][8] = AL::ALValue::array(-1.80403, AL::ALValue::array(3, -0.56, 0), AL::ALValue::array(3, 0, 0));

	names.push_back("RAnklePitch");
	times[14].arraySetSize(8);
	keys[14].arraySetSize(8);

	times[14][0] = 3.16;
	keys[14][0] = AL::ALValue::array(-1.18421, AL::ALValue::array(3, -1.05333, 0), AL::ALValue::array(3, 0.8, 0));
	times[14][1] = 5.56;
	keys[14][1] = AL::ALValue::array(-0.716335, AL::ALValue::array(3, -0.8, -0.129148), AL::ALValue::array(3, 0.6, 0.096861));
	times[14][2] = 7.36;
	keys[14][2] = AL::ALValue::array(-0.506179, AL::ALValue::array(3, -0.6, 0), AL::ALValue::array(3, 0.733333, 0));
	times[14][3] = 9.56;
	keys[14][3] = AL::ALValue::array(-0.51845, AL::ALValue::array(3, -0.733333, 0), AL::ALValue::array(3, 0.08, 0));
	times[14][4] = 9.8;
	keys[14][4] = AL::ALValue::array(-0.51845, AL::ALValue::array(3, -0.08, 0), AL::ALValue::array(3, 1.18667, 0));
	times[14][5] = 13.36;
	keys[14][5] = AL::ALValue::array(-0.828318, AL::ALValue::array(3, -1.18667, 0), AL::ALValue::array(3, 1, 0));
	times[14][6] = 16.36;
	keys[14][6] = AL::ALValue::array(-0.345107, AL::ALValue::array(3, -1, 0), AL::ALValue::array(3, 0.56, 0));
	times[14][7] = 18.04;
	keys[14][7] = AL::ALValue::array(-0.354382, AL::ALValue::array(3, -0.56, 0), AL::ALValue::array(3, 0, 0));

	names.push_back("RAnkleRoll");
	times[15].arraySetSize(8);
	keys[15].arraySetSize(8);

	times[15][0] = 3.16;
	keys[15][0] = AL::ALValue::array(0.046062, AL::ALValue::array(3, -1.05333, 0), AL::ALValue::array(3, 0.8, 0));
	times[15][1] = 5.56;
	keys[15][1] = AL::ALValue::array(0.047596, AL::ALValue::array(3, -0.8, 0), AL::ALValue::array(3, 0.6, 0));
	times[15][2] = 7.36;
	keys[15][2] = AL::ALValue::array(-0.139552, AL::ALValue::array(3, -0.6, 0), AL::ALValue::array(3, 0.733333, 0));
	times[15][3] = 9.56;
	keys[15][3] = AL::ALValue::array(-0.131882, AL::ALValue::array(3, -0.733333, 0), AL::ALValue::array(3, 0.08, 0));
	times[15][4] = 9.8;
	keys[15][4] = AL::ALValue::array(-0.131882, AL::ALValue::array(3, -0.08, 0), AL::ALValue::array(3, 1.18667, 0));
	times[15][5] = 13.36;
	keys[15][5] = AL::ALValue::array(-0.178554, AL::ALValue::array(3, -1.18667, 0), AL::ALValue::array(3, 1, 0));
	times[15][6] = 16.36;
	keys[15][6] = AL::ALValue::array(0.00464392, AL::ALValue::array(3, -1, 0), AL::ALValue::array(3, 0.56, 0));
	times[15][7] = 18.04;
	keys[15][7] = AL::ALValue::array(0.00112737, AL::ALValue::array(3, -0.56, 0), AL::ALValue::array(3, 0, 0));

	names.push_back("RElbowRoll");
	times[16].arraySetSize(9);
	keys[16].arraySetSize(9);

	times[16][0] = 3.16;
	keys[16][0] = AL::ALValue::array(1.00328, AL::ALValue::array(3, -1.05333, 0), AL::ALValue::array(3, 0.8, 0));
	times[16][1] = 5.56;
	keys[16][1] = AL::ALValue::array(0.308375, AL::ALValue::array(3, -0.8, 0), AL::ALValue::array(3, 0.6, 0));
	times[16][2] = 7.36;
	keys[16][2] = AL::ALValue::array(0.34826, AL::ALValue::array(3, -0.6, -0.0142662), AL::ALValue::array(3, 0.733333, 0.0174365));
	times[16][3] = 9.56;
	keys[16][3] = AL::ALValue::array(0.403483, AL::ALValue::array(3, -0.733333, -0.0193639), AL::ALValue::array(3, 0.08, 0.00211242));
	times[16][4] = 9.8;
	keys[16][4] = AL::ALValue::array(0.412688, AL::ALValue::array(3, -0.08, -0.00315323), AL::ALValue::array(3, 0.4, 0.0157661));
	times[16][5] = 11;
	keys[16][5] = AL::ALValue::array(0.460242, AL::ALValue::array(3, -0.4, 0), AL::ALValue::array(3, 0.786667, 0));
	times[16][6] = 13.36;
	keys[16][6] = AL::ALValue::array(0.420357, AL::ALValue::array(3, -0.786667, 0), AL::ALValue::array(3, 1, 0));
	times[16][7] = 16.36;
	keys[16][7] = AL::ALValue::array(0.420357, AL::ALValue::array(3, -1, 0), AL::ALValue::array(3, 0.56, 0));
	times[16][8] = 18.04;
	keys[16][8] = AL::ALValue::array(0.40195, AL::ALValue::array(3, -0.56, 0), AL::ALValue::array(3, 0, 0));

	names.push_back("RElbowYaw");
	times[17].arraySetSize(9);
	keys[17].arraySetSize(9);

	times[17][0] = 3.16;
	keys[17][0] = AL::ALValue::array(1.36522, AL::ALValue::array(3, -1.05333, 0), AL::ALValue::array(3, 0.8, 0));
	times[17][1] = 5.56;
	keys[17][1] = AL::ALValue::array(0.452487, AL::ALValue::array(3, -0.8, 0.0449969), AL::ALValue::array(3, 0.6, -0.0337477));
	times[17][2] = 7.36;
	keys[17][2] = AL::ALValue::array(0.418739, AL::ALValue::array(3, -0.6, 0.0337477), AL::ALValue::array(3, 0.733333, -0.0412472));
	times[17][3] = 9.56;
	keys[17][3] = AL::ALValue::array(-0.150374, AL::ALValue::array(3, -0.733333, 0), AL::ALValue::array(3, 0.08, 0));
	times[17][4] = 9.8;
	keys[17][4] = AL::ALValue::array(-0.147306, AL::ALValue::array(3, -0.08, 0), AL::ALValue::array(3, 0.4, 0));
	times[17][5] = 11;
	keys[17][5] = AL::ALValue::array(-0.147306, AL::ALValue::array(3, -0.4, 0), AL::ALValue::array(3, 0.786667, 0));
	times[17][6] = 13.36;
	keys[17][6] = AL::ALValue::array(-0.154976, AL::ALValue::array(3, -0.786667, 0), AL::ALValue::array(3, 1, 0));
	times[17][7] = 16.36;
	keys[17][7] = AL::ALValue::array(-0.154976, AL::ALValue::array(3, -1, 0), AL::ALValue::array(3, 0.56, 0));
	times[17][8] = 18.04;
	keys[17][8] = AL::ALValue::array(-0.159578, AL::ALValue::array(3, -0.56, 0), AL::ALValue::array(3, 0, 0));

	names.push_back("RHand");
	times[18].arraySetSize(9);
	keys[18].arraySetSize(9);

	times[18][0] = 3.16;
	keys[18][0] = AL::ALValue::array(0.2464, AL::ALValue::array(3, -1.05333, 0), AL::ALValue::array(3, 0.8, 0));
	times[18][1] = 5.56;
	keys[18][1] = AL::ALValue::array(0.9772, AL::ALValue::array(3, -0.8, -0.00906674), AL::ALValue::array(3, 0.6, 0.00680006));
	times[18][2] = 7.36;
	keys[18][2] = AL::ALValue::array(0.984, AL::ALValue::array(3, -0.6, -0.00126), AL::ALValue::array(3, 0.733333, 0.00154));
	times[18][3] = 9.56;
	keys[18][3] = AL::ALValue::array(0.9856, AL::ALValue::array(3, -0.733333, 0), AL::ALValue::array(3, 0.08, 0));
	times[18][4] = 9.8;
	keys[18][4] = AL::ALValue::array(0.7148, AL::ALValue::array(3, -0.08, 0.000160003), AL::ALValue::array(3, 0.4, -0.000800014));
	times[18][5] = 11;
	keys[18][5] = AL::ALValue::array(0.714, AL::ALValue::array(3, -0.4, 0.000314605), AL::ALValue::array(3, 0.786667, -0.000618724));
	times[18][6] = 13.36;
	keys[18][6] = AL::ALValue::array(0.712, AL::ALValue::array(3, -0.786667, 0), AL::ALValue::array(3, 1, 0));
	times[18][7] = 16.36;
	keys[18][7] = AL::ALValue::array(0.712, AL::ALValue::array(3, -1, 0), AL::ALValue::array(3, 0.56, 0));
	times[18][8] = 18.04;
	keys[18][8] = AL::ALValue::array(0.7172, AL::ALValue::array(3, -0.56, 0), AL::ALValue::array(3, 0, 0));

	names.push_back("RHipPitch");
	times[19].arraySetSize(8);
	keys[19].arraySetSize(8);

	times[19][0] = 3.16;
	keys[19][0] = AL::ALValue::array(-0.589097, AL::ALValue::array(3, -1.05333, 0), AL::ALValue::array(3, 0.8, 0));
	times[19][1] = 5.56;
	keys[19][1] = AL::ALValue::array(-1.23338, AL::ALValue::array(3, -0.8, 0.0818118), AL::ALValue::array(3, 0.6, -0.0613588));
	times[19][2] = 7.36;
	keys[19][2] = AL::ALValue::array(-1.29474, AL::ALValue::array(3, -0.6, 0), AL::ALValue::array(3, 0.733333, 0));
	times[19][3] = 9.56;
	keys[19][3] = AL::ALValue::array(-1.29474, AL::ALValue::array(3, -0.733333, 0), AL::ALValue::array(3, 0.08, 0));
	times[19][4] = 9.8;
	keys[19][4] = AL::ALValue::array(-1.29474, AL::ALValue::array(3, -0.08, 0), AL::ALValue::array(3, 1.18667, 0));
	times[19][5] = 13.36;
	keys[19][5] = AL::ALValue::array(-0.713353, AL::ALValue::array(3, -1.18667, -0.154008), AL::ALValue::array(3, 1, 0.129782));
	times[19][6] = 16.36;
	keys[19][6] = AL::ALValue::array(-0.443368, AL::ALValue::array(3, -1, 0), AL::ALValue::array(3, 0.56, 0));
	times[19][7] = 18.04;
	keys[19][7] = AL::ALValue::array(-0.449222, AL::ALValue::array(3, -0.56, 0), AL::ALValue::array(3, 0, 0));

	names.push_back("RHipRoll");
	times[20].arraySetSize(8);
	keys[20].arraySetSize(8);

	times[20][0] = 3.16;
	keys[20][0] = AL::ALValue::array(-0.240796, AL::ALValue::array(3, -1.05333, 0), AL::ALValue::array(3, 0.8, 0));
	times[20][1] = 5.56;
	keys[20][1] = AL::ALValue::array(-0.0367741, AL::ALValue::array(3, -0.8, -0.0917478), AL::ALValue::array(3, 0.6, 0.0688109));
	times[20][2] = 7.36;
	keys[20][2] = AL::ALValue::array(0.24088, AL::ALValue::array(3, -0.6, 0), AL::ALValue::array(3, 0.733333, 0));
	times[20][3] = 9.56;
	keys[20][3] = AL::ALValue::array(0.239346, AL::ALValue::array(3, -0.733333, 0), AL::ALValue::array(3, 0.08, 0));
	times[20][4] = 9.8;
	keys[20][4] = AL::ALValue::array(0.239346, AL::ALValue::array(3, -0.08, 0), AL::ALValue::array(3, 1.18667, 0));
	times[20][5] = 13.36;
	keys[20][5] = AL::ALValue::array(-0.033706, AL::ALValue::array(3, -1.18667, 0), AL::ALValue::array(3, 1, 0));
	times[20][6] = 16.36;
	keys[20][6] = AL::ALValue::array(-0.0106959, AL::ALValue::array(3, -1, -0.00703284), AL::ALValue::array(3, 0.56, 0.00393839));
	times[20][7] = 18.04;
	keys[20][7] = AL::ALValue::array(-0.00079229, AL::ALValue::array(3, -0.56, 0), AL::ALValue::array(3, 0, 0));

	names.push_back("RHipYawPitch");
	times[21].arraySetSize(8);
	keys[21].arraySetSize(8);

	times[21][0] = 3.16;
	keys[21][0] = AL::ALValue::array(-0.612024, AL::ALValue::array(3, -1.05333, 0), AL::ALValue::array(3, 0.8, 0));
	times[21][1] = 5.56;
	keys[21][1] = AL::ALValue::array(-0.984786, AL::ALValue::array(3, -0.8, 0.0984681), AL::ALValue::array(3, 0.6, -0.0738511));
	times[21][2] = 7.36;
	keys[21][2] = AL::ALValue::array(-1.12898, AL::ALValue::array(3, -0.6, 0.00627603), AL::ALValue::array(3, 0.733333, -0.0076707));
	times[21][3] = 9.56;
	keys[21][3] = AL::ALValue::array(-1.13665, AL::ALValue::array(3, -0.733333, 0), AL::ALValue::array(3, 0.08, 0));
	times[21][4] = 9.8;
	keys[21][4] = AL::ALValue::array(-1.13665, AL::ALValue::array(3, -0.08, 0), AL::ALValue::array(3, 1.18667, 0));
	times[21][5] = 13.36;
	keys[21][5] = AL::ALValue::array(-1.12131, AL::ALValue::array(3, -1.18667, -0.0153397), AL::ALValue::array(3, 1, 0.0129267));
	times[21][6] = 16.36;
	keys[21][6] = AL::ALValue::array(0.00157595, AL::ALValue::array(3, -1, 0), AL::ALValue::array(3, 0.56, 0));
	times[21][7] = 18.04;
	keys[21][7] = AL::ALValue::array(-0.00467966, AL::ALValue::array(3, -0.56, 0), AL::ALValue::array(3, 0, 0));

	names.push_back("RKneePitch");
	times[22].arraySetSize(8);
	keys[22].arraySetSize(8);

	times[22][0] = 3.16;
	keys[22][0] = AL::ALValue::array(2.11255, AL::ALValue::array(3, -1.05333, 0), AL::ALValue::array(3, 0.8, 0));
	times[22][1] = 5.56;
	keys[22][1] = AL::ALValue::array(2.10316, AL::ALValue::array(3, -0.8, 0), AL::ALValue::array(3, 0.6, 0));
	times[22][2] = 7.36;
	keys[22][2] = AL::ALValue::array(2.10469, AL::ALValue::array(3, -0.6, -0.00140849), AL::ALValue::array(3, 0.733333, 0.00172148));
	times[22][3] = 9.56;
	keys[22][3] = AL::ALValue::array(2.11255, AL::ALValue::array(3, -0.733333, 0), AL::ALValue::array(3, 0.08, 0));
	times[22][4] = 9.8;
	keys[22][4] = AL::ALValue::array(2.11255, AL::ALValue::array(3, -0.08, 0), AL::ALValue::array(3, 1.18667, 0));
	times[22][5] = 13.36;
	keys[22][5] = AL::ALValue::array(2.09549, AL::ALValue::array(3, -1.18667, 0.0170518), AL::ALValue::array(3, 1, -0.0143695));
	times[22][6] = 16.36;
	keys[22][6] = AL::ALValue::array(0.698011, AL::ALValue::array(3, -1, 0), AL::ALValue::array(3, 0.56, 0));
	times[22][7] = 18.04;
	keys[22][7] = AL::ALValue::array(0.700641, AL::ALValue::array(3, -0.56, 0), AL::ALValue::array(3, 0, 0));

	names.push_back("RShoulderPitch");
	times[23].arraySetSize(9);
	keys[23].arraySetSize(9);

	times[23][0] = 3.16;
	keys[23][0] = AL::ALValue::array(1.42206, AL::ALValue::array(3, -1.05333, 0), AL::ALValue::array(3, 0.8, 0));
	times[23][1] = 5.56;
	keys[23][1] = AL::ALValue::array(0.668866, AL::ALValue::array(3, -0.8, 0.116583), AL::ALValue::array(3, 0.6, -0.0874375));
	times[23][2] = 7.36;
	keys[23][2] = AL::ALValue::array(0.581429, AL::ALValue::array(3, -0.6, 0.0283023), AL::ALValue::array(3, 0.733333, -0.0345918));
	times[23][3] = 9.56;
	keys[23][3] = AL::ALValue::array(0.480184, AL::ALValue::array(3, -0.733333, 0), AL::ALValue::array(3, 0.08, 0));
	times[23][4] = 9.8;
	keys[23][4] = AL::ALValue::array(0.512397, AL::ALValue::array(3, -0.08, -0.0021478), AL::ALValue::array(3, 0.4, 0.010739));
	times[23][5] = 11;
	keys[23][5] = AL::ALValue::array(0.523136, AL::ALValue::array(3, -0.4, -0.00448138), AL::ALValue::array(3, 0.786667, 0.00881338));
	times[23][6] = 13.36;
	keys[23][6] = AL::ALValue::array(0.552281, AL::ALValue::array(3, -0.786667, 0), AL::ALValue::array(3, 1, 0));
	times[23][7] = 16.36;
	keys[23][7] = AL::ALValue::array(0.552281, AL::ALValue::array(3, -1, 0), AL::ALValue::array(3, 0.56, 0));
	times[23][8] = 18.04;
	keys[23][8] = AL::ALValue::array(0.576826, AL::ALValue::array(3, -0.56, 0), AL::ALValue::array(3, 0, 0));

	names.push_back("RShoulderRoll");
	times[24].arraySetSize(10);
	keys[24].arraySetSize(10);

	times[24][0] = 1;
	keys[24][0] = AL::ALValue::array(0.165806, AL::ALValue::array(3, -0.333333, 0), AL::ALValue::array(3, 0.72, 0));
	times[24][1] = 3.16;
	keys[24][1] = AL::ALValue::array(-0.276162, AL::ALValue::array(3, -0.72, 0), AL::ALValue::array(3, 0.8, 0));
	times[24][2] = 5.56;
	keys[24][2] = AL::ALValue::array(-0.131966, AL::ALValue::array(3, -0.8, -0.0561005), AL::ALValue::array(3, 0.6, 0.0420754));
	times[24][3] = 7.36;
	keys[24][3] = AL::ALValue::array(0.0183661, AL::ALValue::array(3, -0.6, -0.0450996), AL::ALValue::array(3, 0.733333, 0.0551217));
	times[24][4] = 9.56;
	keys[24][4] = AL::ALValue::array(0.168698, AL::ALValue::array(3, -0.733333, -0.0567078), AL::ALValue::array(3, 0.08, 0.0061863));
	times[24][5] = 9.8;
	keys[24][5] = AL::ALValue::array(0.207048, AL::ALValue::array(3, -0.08, 0), AL::ALValue::array(3, 0.4, 0));
	times[24][6] = 11;
	keys[24][6] = AL::ALValue::array(0.145688, AL::ALValue::array(3, -0.4, 0.00858001), AL::ALValue::array(3, 0.786667, -0.016874));
	times[24][7] = 13.36;
	keys[24][7] = AL::ALValue::array(0.128814, AL::ALValue::array(3, -0.786667, 0), AL::ALValue::array(3, 1, 0));
	times[24][8] = 16.36;
	keys[24][8] = AL::ALValue::array(0.128814, AL::ALValue::array(3, -1, 0), AL::ALValue::array(3, 0.56, 0));
	times[24][9] = 18.04;
	keys[24][9] = AL::ALValue::array(0.187106, AL::ALValue::array(3, -0.56, 0), AL::ALValue::array(3, 0, 0));

	names.push_back("RWristYaw");
	times[25].arraySetSize(9);
	keys[25].arraySetSize(9);

	times[25][0] = 3.16;
	keys[25][0] = AL::ALValue::array(-0.0138481, AL::ALValue::array(3, -1.05333, 0), AL::ALValue::array(3, 0.8, 0));
	times[25][1] = 5.56;
	keys[25][1] = AL::ALValue::array(1.35601, AL::ALValue::array(3, -0.8, -0.0245439), AL::ALValue::array(3, 0.6, 0.0184079));
	times[25][2] = 7.36;
	keys[25][2] = AL::ALValue::array(1.37442, AL::ALValue::array(3, -0.6, -0.0184079), AL::ALValue::array(3, 0.733333, 0.0224985));
	times[25][3] = 9.56;
	keys[25][3] = AL::ALValue::array(1.81774, AL::ALValue::array(3, -0.733333, 0), AL::ALValue::array(3, 0.08, 0));
	times[25][4] = 9.8;
	keys[25][4] = AL::ALValue::array(1.81008, AL::ALValue::array(3, -0.08, 0), AL::ALValue::array(3, 0.4, 0));
	times[25][5] = 11;
	keys[25][5] = AL::ALValue::array(1.81008, AL::ALValue::array(3, -0.4, 0), AL::ALValue::array(3, 0.786667, 0));
	times[25][6] = 13.36;
	keys[25][6] = AL::ALValue::array(1.80855, AL::ALValue::array(3, -0.786667, 0), AL::ALValue::array(3, 1, 0));
	times[25][7] = 16.36;
	keys[25][7] = AL::ALValue::array(1.80855, AL::ALValue::array(3, -1, 0), AL::ALValue::array(3, 0.56, 0));
	times[25][8] = 18.04;
	keys[25][8] = AL::ALValue::array(1.80548, AL::ALValue::array(3, -0.56, 0), AL::ALValue::array(3, 0, 0));

	try
	{
		getParentBroker()->getMotionProxy()->angleInterpolationBezier(names, times, keys);
	}
	catch (const std::exception&)
	{

	}
}*/

/*void pickup::pickUpBalls()
{
	//学姐未改版本
	std::vector<std::string> names;
	AL::ALValue times, keys;
	names.reserve(26);
	times.arraySetSize(26);
	keys.arraySetSize(26);

	names.push_back("HeadPitch");
	times[0].arraySetSize(8);
	keys[0].arraySetSize(8);

	times[0][0] = 4.16;
	keys[0][0] = -0.671952;
	times[0][1] = 6.4;
	keys[0][1] = -0.671952;
	times[0][2] = 9.16;
	keys[0][2] = -0.671952;
	times[0][3] = 11.44;
	keys[0][3] = -0.671952;
	times[0][4] = 13.68;
	keys[0][4] = -0.671952;
	times[0][5] = 16.88;
	keys[0][5] = -0.671952;
	times[0][6] = 20.68;
	keys[0][6] = -0.671952;
	times[0][7] = 21.16;
	keys[0][7] = 0.00609397;

	names.push_back("HeadYaw");
	times[1].arraySetSize(8);
	keys[1].arraySetSize(8);

	times[1][0] = 4.16;
	keys[1][0] = 0.00762796;
	times[1][1] = 6.4;
	keys[1][1] = 0.00762796;
	times[1][2] = 9.16;
	keys[1][2] = 0.00762796;
	times[1][3] = 11.44;
	keys[1][3] = 0.00762796;
	times[1][4] = 13.68;
	keys[1][4] = 0.00762796;
	times[1][5] = 16.88;
	keys[1][5] = 0.00762796;
	times[1][6] = 20.68;
	keys[1][6] = 0.00762796;
	times[1][7] = 21.16;
	keys[1][7] = 0.00762796;

	names.push_back("LAnklePitch");
	times[2].arraySetSize(8);
	keys[2].arraySetSize(8);

	times[2][0] = 4.16;
	keys[2][0] = -0.659661;
	times[2][1] = 6.4;
	keys[2][1] = -0.659661;
	times[2][2] = 9.16;
	keys[2][2] = -0.659661;
	times[2][3] = 11.44;
	keys[2][3] = -0.656595;
	times[2][4] = 13.68;
	keys[2][4] = -0.437231;
	times[2][5] = 16.88;
	keys[2][5] = -0.737896;
	times[2][6] = 20.68;
	keys[2][6] = -0.352862;
	times[2][7] = 21.16;
	keys[2][7] = -0.35593;

	names.push_back("LAnkleRoll");
	times[3].arraySetSize(8);
	keys[3].arraySetSize(8);

	times[3][0] = 4.16;
	keys[3][0] = -0.164096;
	times[3][1] = 6.4;
	keys[3][1] = -0.164096;
	times[3][2] = 9.16;
	keys[3][2] = -0.16563;
	times[3][3] = 11.44;
	keys[3][3] = -0.15796;
	times[3][4] = 13.68;
	keys[3][4] = -0.148756;
	times[3][5] = 16.88;
	keys[3][5] = -0.0628521;
	times[3][6] = 20.68;
	keys[3][6] = 0.00157595;
	times[3][7] = 21.16;
	keys[3][7] = 0.00157595;

	names.push_back("LElbowRoll");
	times[4].arraySetSize(8);
	keys[4].arraySetSize(8);

	times[4][0] = 4.16;
	keys[4][0] = -1.34374;
	times[4][1] = 6.4;
	keys[4][1] = -0.713267;
	times[4][2] = 9.16;
	keys[4][2] = -0.578276;
	times[4][3] = 11.44;
	keys[4][3] = -1.48947;
	times[4][4] = 13.68;
	keys[4][4] = -1.48947;
	times[4][5] = 16.88;
	keys[4][5] = -1.48947;
	times[4][6] = 20.68;
	keys[4][6] = -1.46033;
	times[4][7] = 21.16;
	keys[4][7] = -1.46033;

	names.push_back("LElbowYaw");
	times[5].arraySetSize(8);
	keys[5].arraySetSize(8);

	times[5][0] = 4.16;
	keys[5][0] = -0.664264;
	times[5][1] = 6.4;
	keys[5][1] = -0.550747;
	times[5][2] = 9.16;
	keys[5][2] = -0.584497;
	times[5][3] = 11.44;
	keys[5][3] = -0.581429;
	times[5][4] = 13.68;
	keys[5][4] = -0.581429;
	times[5][5] = 16.88;
	keys[5][5] = -0.581429;
	times[5][6] = 20.68;
	keys[5][6] = -0.633584;
	times[5][7] = 21.16;
	keys[5][7] = -0.633584;

	names.push_back("LHand");
	times[6].arraySetSize(8);
	keys[6].arraySetSize(8);

	times[6][0] = 4.16;
	keys[6][0] = 0.7892;
	times[6][1] = 6.4;
	keys[6][1] = 0.7876;
	times[6][2] = 9.16;
	keys[6][2] = 0.968;
	times[6][3] = 11.44;
	keys[6][3] = 0.968;
	times[6][4] = 13.68;
	keys[6][4] = 0.968;
	times[6][5] = 16.88;
	keys[6][5] = 0.968;
	times[6][6] = 20.68;
	keys[6][6] = 0.962;
	times[6][7] = 21.16;
	keys[6][7] = 0.962;

	names.push_back("LHipPitch");
	times[7].arraySetSize(8);
	keys[7].arraySetSize(8);

	times[7][0] = 4.16;
	keys[7][0] = -1.52475;
	times[7][1] = 6.4;
	keys[7][1] = -1.52475;
	times[7][2] = 9.16;
	keys[7][2] = -1.52322;
	times[7][3] = 11.44;
	keys[7][3] = -1.53089;
	times[7][4] = 13.68;
	keys[7][4] = -1.37289;
	times[7][5] = 16.88;
	keys[7][5] = -1.53089;
	times[7][6] = 20.68;
	keys[7][6] = -0.455555;
	times[7][7] = 21.16;
	keys[7][7] = -0.450955;

	names.push_back("LHipRoll");
	times[8].arraySetSize(8);
	keys[8].arraySetSize(8);

	times[8][0] = 4.16;
	keys[8][0] = -0.107338;
	times[8][1] = 6.4;
	keys[8][1] = -0.10427;
	times[8][2] = 9.16;
	keys[8][2] = -0.0996681;
	times[8][3] = 11.44;
	keys[8][3] = -0.102736;
	times[8][4] = 13.68;
	keys[8][4] = 0.299172;
	times[8][5] = 16.88;
	keys[8][5] = 0.0337899;
	times[8][6] = 20.68;
	keys[8][6] = 0.00464392;
	times[8][7] = 21.16;
	keys[8][7] = 0.00464392;

	names.push_back("LHipYawPitch");
	times[9].arraySetSize(8);
	keys[9].arraySetSize(8);

	times[9][0] = 4.16;
	keys[9][0] = -0.843657;
	times[9][1] = 6.4;
	keys[9][1] = -0.843657;
	times[9][2] = 9.16;
	keys[9][2] = -0.83292;
	times[9][3] = 11.44;
	keys[9][3] = -0.835988;
	times[9][4] = 13.68;
	keys[9][4] = -0.711735;
	times[9][5] = 16.88;
	keys[9][5] = -0.13495;
	times[9][6] = 20.68;
	keys[9][6] = -0.00762796;
	times[9][7] = 21.16;
	keys[9][7] = 0.00310993;

	names.push_back("LKneePitch");
	times[10].arraySetSize(8);
	keys[10].arraySetSize(8);

	times[10][0] = 4.16;
	keys[10][0] = 2.10307;
	times[10][1] = 6.4;
	keys[10][1] = 2.10307;
	times[10][2] = 9.16;
	keys[10][2] = 2.10307;
	times[10][3] = 11.44;
	keys[10][3] = 2.09541;
	times[10][4] = 13.68;
	keys[10][4] = 2.10614;
	times[10][5] = 16.88;
	keys[10][5] = 2.11228;
	times[10][6] = 20.68;
	keys[10][6] = 0.699462;
	times[10][7] = 21.16;
	keys[10][7] = 0.699462;

	names.push_back("LShoulderPitch");
	times[11].arraySetSize(8);
	keys[11].arraySetSize(8);

	times[11][0] = 4.16;
	keys[11][0] = 0.754686;
	times[11][1] = 6.4;
	keys[11][1] = 0.736278;
	times[11][2] = 9.16;
	keys[11][2] = 0.681054;
	times[11][3] = 11.44;
	keys[11][3] = 0.90962;
	times[11][4] = 13.68;
	keys[11][4] = 0.921892;
	times[11][5] = 16.88;
	keys[11][5] = 0.921892;
	times[11][6] = 20.68;
	keys[11][6] = 0.934165;
	times[11][7] = 21.16;
	keys[11][7] = 0.934165;

	names.push_back("LShoulderRoll");
	times[12].arraySetSize(8);
	keys[12].arraySetSize(8);

	times[12][0] = 4.16;
	keys[12][0] = 0.57214;
	times[12][1] = 6.4;
	keys[12][1] = 0.05825;
	times[12][2] = 9.16;
	keys[12][2] = -0.222472;
	times[12][3] = 11.44;
	keys[12][3] = 0.0858622;
	times[12][4] = 13.68;
	keys[12][4] = 0.107338;
	times[12][5] = 16.88;
	keys[12][5] = 0.107338;
	times[12][6] = 20.68;
	keys[12][6] = 0.177901;
	times[12][7] = 21.16;
	keys[12][7] = 0.177901;

	names.push_back("LWristYaw");
	times[13].arraySetSize(8);
	keys[13].arraySetSize(8);

	times[13][0] = 4.16;
	keys[13][0] = -0.926578;
	times[13][1] = 6.4;
	keys[13][1] = -0.891296;
	times[13][2] = 9.16;
	keys[13][2] = -0.308375;
	times[13][3] = 11.44;
	keys[13][3] = -0.019984;
	times[13][4] = 13.68;
	keys[13][4] = -0.019984;
	times[13][5] = 16.88;
	keys[13][5] = -0.019984;
	times[13][6] = 20.68;
	keys[13][6] = -0.00924586;
	times[13][7] = 21.16;
	keys[13][7] = -0.00924586;

	names.push_back("RAnklePitch");
	times[14].arraySetSize(8);
	keys[14].arraySetSize(8);

	times[14][0] = 4.16;
	keys[14][0] = -0.700996;
	times[14][1] = 6.4;
	keys[14][1] = -0.700996;
	times[14][2] = 9.16;
	keys[14][2] = -0.699462;
	times[14][3] = 11.44;
	keys[14][3] = -0.699462;
	times[14][4] = 13.68;
	keys[14][4] = -0.71787;
	times[14][5] = 16.88;
	keys[14][5] = -0.791502;
	times[14][6] = 20.68;
	keys[14][6] = -0.352778;
	times[14][7] = 21.16;
	keys[14][7] = -0.354312;

	names.push_back("RAnkleRoll");
	times[15].arraySetSize(8);
	keys[15].arraySetSize(8);

	times[15][0] = 4.16;
	keys[15][0] = 0.233211;
	times[15][1] = 6.4;
	keys[15][1] = 0.233211;
	times[15][2] = 9.16;
	keys[15][2] = 0.230143;
	times[15][3] = 11.44;
	keys[15][3] = 0.230143;
	times[15][4] = 13.68;
	keys[15][4] = 0.165714;
	times[15][5] = 16.88;
	keys[15][5] = 0.073674;
	times[15][6] = 20.68;
	keys[15][6] = -0.00149202;
	times[15][7] = 21.16;
	keys[15][7] = -0.00149202;

	names.push_back("RElbowRoll");
	times[16].arraySetSize(8);
	keys[16].arraySetSize(8);

	times[16][0] = 4.16;
	keys[16][0] = 1.17202;
	times[16][1] = 6.4;
	keys[16][1] = 0.527739;
	times[16][2] = 9.16;
	keys[16][2] = 0.454105;
	times[16][3] = 11.44;
	keys[16][3] = 1.17355;
	times[16][4] = 13.68;
	keys[16][4] = 1.17355;
	times[16][5] = 16.88;
	keys[16][5] = 1.17355;
	times[16][6] = 20.68;
	keys[16][6] = 1.10299;
	times[16][7] = 21.16;
	keys[16][7] = 1.10299;

	names.push_back("RElbowYaw");
	times[17].arraySetSize(8);
	keys[17].arraySetSize(8);

	times[17][0] = 4.16;
	keys[17][0] = 0.477032;
	times[17][1] = 6.4;
	keys[17][1] = 0.314428;
	times[17][2] = 9.16;
	keys[17][2] = 0.322099;
	times[17][3] = 11.44;
	keys[17][3] = 0.141086;
	times[17][4] = 13.68;
	keys[17][4] = 0.141086;
	times[17][5] = 16.88;
	keys[17][5] = 0.141086;
	times[17][6] = 20.68;
	keys[17][6] = 0.153358;
	times[17][7] = 21.16;
	keys[17][7] = 0.153358;

	names.push_back("RHand");
	times[18].arraySetSize(8);
	keys[18].arraySetSize(8);

	times[18][0] = 4.16;
	keys[18][0] = 0.7608;
	times[18][1] = 6.4;
	keys[18][1] = 0.7564;
	times[18][2] = 9.16;
	keys[18][2] = 0.8476;
	times[18][3] = 11.44;
	keys[18][3] = 0.8476;
	times[18][4] = 13.68;
	keys[18][4] = 0.8476;
	times[18][5] = 16.88;
	keys[18][5] = 0.8476;
	times[18][6] = 20.68;
	keys[18][6] = 0.8408;
	times[18][7] = 21.16;
	keys[18][7] = 0.8408;

	names.push_back("RHipPitch");
	times[19].arraySetSize(8);
	keys[19].arraySetSize(8);

	times[19][0] = 4.16;
	keys[19][0] = -1.53251;
	times[19][1] = 6.4;
	keys[19][1] = -1.53251;
	times[19][2] = 9.16;
	keys[19][2] = -1.52637;
	times[19][3] = 11.44;
	keys[19][3] = -1.53251;
	times[19][4] = 13.68;
	keys[19][4] = -0.975665;
	times[19][5] = 16.88;
	keys[19][5] = -1.53097;
	times[19][6] = 20.68;
	keys[19][6] = -0.458707;
	times[19][7] = 21.16;
	keys[19][7] = -0.446436;

	names.push_back("RHipRoll");
	times[20].arraySetSize(8);
	keys[20].arraySetSize(8);

	times[20][0] = 4.16;
	keys[20][0] = 0.0153821;
	times[20][1] = 6.4;
	keys[20][1] = 0.016916;
	times[20][2] = 9.16;
	keys[20][2] = 4.19617e-05;
	times[20][3] = 11.44;
	keys[20][3] = 0.00157595;
	times[20][4] = 13.68;
	keys[20][4] = -0.11194;
	times[20][5] = 16.88;
	keys[20][5] = -0.061318;
	times[20][6] = 20.68;
	keys[20][6] = -0.00149202;
	times[20][7] = 21.16;
	keys[20][7] = -0.00149202;

	names.push_back("RHipYawPitch");
	times[21].arraySetSize(8);
	keys[21].arraySetSize(8);

	times[21][0] = 4.16;
	keys[21][0] = -0.843657;
	times[21][1] = 6.4;
	keys[21][1] = -0.843657;
	times[21][2] = 9.16;
	keys[21][2] = -0.83292;
	times[21][3] = 11.44;
	keys[21][3] = -0.835988;
	times[21][4] = 13.68;
	keys[21][4] = -0.711735;
	times[21][5] = 16.88;
	keys[21][5] = -0.13495;
	times[21][6] = 20.68;
	keys[21][6] = -0.00762796;
	times[21][7] = 21.16;
	keys[21][7] = 0.00310993;

	names.push_back("RKneePitch");
	times[22].arraySetSize(8);
	keys[22].arraySetSize(8);

	times[22][0] = 4.16;
	keys[22][0] = 2.11255;
	times[22][1] = 6.4;
	keys[22][1] = 2.11255;
	times[22][2] = 9.16;
	keys[22][2] = 2.10469;
	times[22][3] = 11.44;
	keys[22][3] = 2.11082;
	times[22][4] = 13.68;
	keys[22][4] = 2.11082;
	times[22][5] = 16.88;
	keys[22][5] = 2.11082;
	times[22][6] = 20.68;
	keys[22][6] = 0.702614;
	times[22][7] = 21.16;
	keys[22][7] = 0.702614;

	names.push_back("RShoulderPitch");
	times[23].arraySetSize(8);
	keys[23].arraySetSize(8);

	times[23][0] = 4.16;
	keys[23][0] = 0.579894;
	times[23][1] = 6.4;
	keys[23][1] = 0.667332;
	times[23][2] = 9.16;
	keys[23][2] = 0.671934;
	times[23][3] = 11.44;
	keys[23][3] = 0.644321;
	times[23][4] = 13.68;
	keys[23][4] = 0.65506;
	times[23][5] = 16.88;
	keys[23][5] = 0.65506;
	times[23][6] = 20.68;
	keys[23][6] = 0.721022;
	times[23][7] = 21.16;
	keys[23][7] = 0.721022;

	names.push_back("RShoulderRoll");
	times[24].arraySetSize(8);
	keys[24].arraySetSize(8);

	times[24][0] = 4.16;
	keys[24][0] = -0.480184;
	times[24][1] = 6.4;
	keys[24][1] = 0.00609397;
	times[24][2] = 9.16;
	keys[24][2] = 0.18097;
	times[24][3] = 11.44;
	keys[24][3] = -0.075208;
	times[24][4] = 13.68;
	keys[24][4] = -0.075208;
	times[24][5] = 16.88;
	keys[24][5] = -0.075208;
	times[24][6] = 20.68;
	keys[24][6] = -0.147306;
	times[24][7] = 21.16;
	keys[24][7] = -0.147306;

	names.push_back("RWristYaw");
	times[25].arraySetSize(8);
	keys[25].arraySetSize(8);

	times[25][0] = 4.16;
	keys[25][0] = 1.59532;
	times[25][1] = 6.4;
	keys[25][1] = 1.58458;
	times[25][2] = 9.16;
	keys[25][2] = 1.73645;
	times[25][3] = 11.44;
	keys[25][3] = 1.61373;
	times[25][4] = 13.68;
	keys[25][4] = 1.61373;
	times[25][5] = 16.88;
	keys[25][5] = 1.61373;
	times[25][6] = 20.68;
	keys[25][6] = 1.59378;
	times[25][7] = 21.16;
	keys[25][7] = 1.59378;

	motionProxy.angleInterpolation(names, keys, times, true);
}*/
