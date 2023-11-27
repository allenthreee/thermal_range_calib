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
const string winname = "Laplacian Demo - Simple Edge Detector";
// 最大值
const int maxNum = 4;

int ksize_value, scale_value, delta_value;

Mat image, src, src_gray;
int ddepth = CV_16S;
RNG rng(12345);


void onLaplacian(int, void *) {
    int ksize = 1 + 2 * (ksize_value % 5); // ksize取值为 1/3/5/7/9
    double scale = 1 + scale_value;        // scale取值为 1/2/3/4/5
    double delta = 10 * delta_value;       // delta取值为 0/10/20/30/40

    Mat grad, abs_grad, non_max_supped;
    Laplacian(src_gray, grad, ddepth, ksize, scale, delta, BORDER_DEFAULT);

    // converting back to CV_8U
    convertScaleAbs(grad, abs_grad);

    // non_max_supped = NonMaxSupp(abs_grad);

    imshow(winname, abs_grad);
}

void onLaplacian_counter(int, void *) {
    int ksize = 1 + 2 * (ksize_value % 5); // ksize取值为 1/3/5/7/9
    double scale = 1 + scale_value;        // scale取值为 1/2/3/4/5
    double delta = 10 * delta_value;       // delta取值为 0/10/20/30/40

    Mat grad, abs_grad, non_max_supped;
    Laplacian(src_gray, grad, ddepth, ksize, scale, delta, BORDER_DEFAULT);

    // converting back to CV_8U
    convertScaleAbs(grad, abs_grad);
    imshow( "source_window", abs_grad );


    // non_max_supped = NonMaxSupp(abs_grad);

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    // findContours(abs_grad, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE );
    findContours(abs_grad, contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE );
    Mat drawing = Mat::zeros( abs_grad.size(), CV_8UC3 );
    for( size_t i = 0; i< contours.size(); i++ )
    {
        Scalar color = Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
        drawContours( drawing, contours, (int)i, color, 2, LINE_8, hierarchy, 0 );
    }
    imshow( "Contours_laplace", drawing );
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

    // createTrackbar(ksize_trackbarname, winname, &ksize_value, maxNum, onLaplacian, NULL);
    // createTrackbar(scale_trackbarname, winname, &scale_value, maxNum, onLaplacian, NULL);
    // createTrackbar(delta_trackbarname, winname, &delta_value, maxNum, onLaplacian, NULL);
    // onLaplacian(0, NULL);

    const char* source_window = "Source";
    imshow( source_window, src );
    createTrackbar(ksize_trackbarname, source_window, &ksize_value, maxNum, onLaplacian_counter, NULL);
    createTrackbar(scale_trackbarname, source_window, &scale_value, maxNum, onLaplacian_counter, NULL);
    createTrackbar(delta_trackbarname, source_window, &delta_value, maxNum, onLaplacian_counter, NULL);
    onLaplacian_counter(0, NULL);

    waitKey(0);

    return 0;
}