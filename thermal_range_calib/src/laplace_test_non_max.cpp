#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>

using namespace cv;
using namespace std;

// 滑动条名
const string ksize_trackbarname = "ksize";
const string scale_trackbarname = "scale";
const string delta_trackbarname = "delta";
// 窗口名
const string winname = "Laplacian_non_max Demo - Simple Edge Detector";
// 最大值
const int maxNum = 4;

int ksize_value, scale_value, delta_value;

Mat image, src, src_gray;
int ddepth = CV_16S;

// Detect edge by canny, and filter by edge length
cv::Mat NonMaxSupp(cv::Mat sFiltered)
{

    cv::Mat nonMaxSupped = cv::Mat(sFiltered.rows - 2, sFiltered.cols - 2, CV_8UC1);

    for (int i = 1; i < (sFiltered.rows - 1); i++)
    {
        for (int j = 1; j < (sFiltered.cols - 1); j++)
        {

            nonMaxSupped.at<uchar>(i - 1, j - 1) = sFiltered.at<uchar>(i, j);

            // Horizontal Edge
                if ((sFiltered.at<uchar>(i, j) < sFiltered.at<uchar>(i, j + 1)) || 
                    (sFiltered.at<uchar>(i, j) < sFiltered.at<uchar>(i, j - 1)) ||
                    (sFiltered.at<uchar>(i, j) < sFiltered.at<uchar>(i, j + 2)) ||
                    (sFiltered.at<uchar>(i, j) < sFiltered.at<uchar>(i, j - 2))
                ){
                    nonMaxSupped.at<uchar>(i - 1, j - 1) = 0;
                }


            // // Vertical Edge
            //     if ((sFiltered.at<uchar>(i, j) < sFiltered.at<uchar>(i + 1, j)) ||
            //         (sFiltered.at<uchar>(i, j) < sFiltered.at<uchar>(i - 1, j)) ||
            //         (sFiltered.at<uchar>(i, j) < sFiltered.at<uchar>(i + 2, j)) ||
            //         (sFiltered.at<uchar>(i, j) < sFiltered.at<uchar>(i - 2, j))
            //         ){
            //             nonMaxSupped.at<uchar>(i - 1, j - 1) = 0;
            //         }

            //-45 Degree Edge
            //     if ((sFiltered.at<uchar>(i, j) < sFiltered.at<uchar>(i - 1, j + 1)) || (sFiltered.at<uchar>(i, j) < sFiltered.at<uchar>(i + 1, j - 1)))
            //         nonMaxSupped.at<uchar>(i - 1, j - 1) = 0;

            // // 45 Degree Edge
            //     if ((sFiltered.at<uchar>(i, j) < sFiltered.at<uchar>(i + 1, j + 1)) || (sFiltered.at<uchar>(i, j) < sFiltered.at<uchar>(i - 1, j - 1)))
            //         nonMaxSupped.at<uchar>(i - 1, j - 1) = 0;
        }
    }
    return nonMaxSupped;
}

void onLaplacian(int, void *) {
    int ksize = 1 + 2 * (ksize_value % 5); // ksize取值为 1/3/5/7/9
    double scale = 1 + scale_value;        // scale取值为 1/2/3/4/5
    double delta = 10 * delta_value;       // delta取值为 0/10/20/30/40

    Mat grad, abs_grad, non_max_supped;
    Laplacian(src_gray, grad, ddepth, ksize, scale, delta, BORDER_DEFAULT);

    // converting back to CV_8U
    convertScaleAbs(grad, abs_grad);

    non_max_supped = NonMaxSupp(abs_grad);

    imshow(winname, non_max_supped);
}

int main(int argc, char **argv) {
    // cv::CommandLineParser parser(argc, argv,
    //                              "{@input   |../lena.jpg|input image}"
    //                              "{help    h|false|show help message}");
    // cout << "The sample uses Laplacian OpenCV functions for edge detection\n\n";
    // parser.printMessage();

    String imageName = "/home/allen/calib_ws/data_demo/0thermal.png";
    // As usual we load our source image (src)
    image = imread(imageName, IMREAD_COLOR); // Load an image
    // Check if image is loaded fine
    if (image.empty()) {
        printf("Error opening image: %s\n", imageName.c_str());
        return 1;
    }

    // Remove noise by blurring with a Gaussian filter ( kernel size = 3 )
    GaussianBlur(image, src, Size(3, 3), 0, 0, BORDER_DEFAULT);
    // Convert the image to grayscale
    cvtColor(src, src_gray, COLOR_BGR2GRAY);

    namedWindow(winname);
    createTrackbar(ksize_trackbarname, winname, &ksize_value, maxNum, onLaplacian, NULL);
    createTrackbar(scale_trackbarname, winname, &scale_value, maxNum, onLaplacian, NULL);
    createTrackbar(delta_trackbarname, winname, &delta_value, maxNum, onLaplacian, NULL);

    onLaplacian(0, NULL);
    waitKey(0);

    return 0;
}