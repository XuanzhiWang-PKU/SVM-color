#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;
using namespace cv::ml;

int main()
{
    string imagePath = "/home/xyz/red/";
    char imageName[50];
    char binImagename[50];

    Mat train_data, train_lable;

    for(int imgIdx = 1; imgIdx < 2; imgIdx++){
        sprintf(imageName, (imagePath + "sample%d.jpg").c_str(), imgIdx);
        sprintf(binImagename, (imagePath + "binarySample%d.jpg").c_str(), imgIdx);
        Mat orig = imread(imageName);
        Mat growed = imread(binImagename);

        cvtColor(orig,orig,CV_BGR2Lab);

        //waitKey(1000);

        for(int i = 0; i < orig.rows; i++){
            for(int j = 0; j < orig.cols; j++){
                Vec3b point = orig.at<Vec3b>(i, j);
                Mat tmp = (Mat_<float>(1, 2) << point[1], point[2]);
                train_data.push_back(tmp);
                int lablePt = -1;
                if(growed.at<uchar>(i,j) > 100){
                    lablePt = 1;
                }
                Mat tmpLable = (Mat_<int>(1, 1) << lablePt);
                train_lable.push_back(tmpLable);
            }
        }
    }

    cout << train_data.rows << " " << train_data.cols << endl;
    cout << train_lable.rows << " " << train_lable.cols << endl;

    Ptr<SVM> svm = SVM::create();
    svm->setType(SVM::C_SVC);
    svm->setC(0.1);
    svm->setTermCriteria(TermCriteria(TermCriteria::MAX_ITER, (int)1e4, 1e-5));
    svm->setKernel(SVM::LINEAR);
    Ptr<TrainData> tData = TrainData::create(train_data, ROW_SAMPLE, train_lable);
    svm->train(tData);
    svm->save("/home/xyz/red.xml");

    return 0;
}

