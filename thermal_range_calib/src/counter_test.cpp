
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include "include/thermal_canny.hpp"
using namespace cv;
using namespace std;
Mat src_gray;
int thresh = 100;
int kernel_size = 0;
// int odd_kernel_size = kernel_size*2 + 1;
RNG rng(12345);

void canny_thresh_callback(int, void* );
void laplace_thresh_callback(int, void* );
// data_demo/ywy_collect_thermal_images/1edited.jpeg
int main( int argc, char** argv )
{
    Mat src = imread( samples::findFile("/home/allen/ooad_calib_ws/data_demo/ywy_collect_thermal_images/1edited.jpeg"));
    cvtColor( src, src_gray, COLOR_BGR2GRAY );
    blur( src_gray, src_gray, Size(3,3) );
    const char* source_window = "Source";
    namedWindow( source_window );
    imshow( source_window, src );

    const int max_thresh = 255;
    createTrackbar( "Canny thresh:", source_window, &thresh, max_thresh, canny_thresh_callback );
    canny_thresh_callback( 0, 0 );

    waitKey();
    return 0;
}
void canny_thresh_callback(int, void* )
{
    Mat canny_output;
    ThermalCanny my_thermal_canny;
    cv::Mat sFiltered = my_thermal_canny.Sobel(src_gray);
    cv::Mat ssFiltered = my_thermal_canny.Sobel(sFiltered);
    cv::Mat non = my_thermal_canny.NonMaxSupp(sFiltered);
    cv::Mat ssnon = my_thermal_canny.NonMaxSupp(ssFiltered);

    Canny( src_gray, canny_output, thresh, thresh*2 );
    imshow( "Canny_output", canny_output );

    vector<vector<Point> > raw_contours;
    vector<Vec4i> raw_hierarchy;
    findContours( src_gray, raw_contours, raw_hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE );
    Mat raw_drawing = Mat::zeros( src_gray.size(), CV_8UC3 );
    for( size_t i = 0; i< raw_contours.size(); i++ )
    {
        Scalar color = Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
        drawContours( raw_drawing, raw_contours, (int)i, color, 2, LINE_8, raw_hierarchy, 0 );
    }
    imshow( "Contours_raw", raw_drawing );

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours( canny_output, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE );
    Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
    for( size_t i = 0; i< contours.size(); i++ )
    {
        Scalar color = Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
        drawContours( drawing, contours, (int)i, color, 2, LINE_8, hierarchy, 0 );
    }
    imshow( "Contours_canny", drawing );

    vector<vector<Point> > thermal_sobel_contours;
    vector<Vec4i> hierarchy_thermal_sobel;
    cv::Mat theraml_sobel_threshold = my_thermal_canny.Thresholding(non, thresh);
    cv::imshow("1_sobel_canny", theraml_sobel_threshold);
    // findContours( theraml_sobel_threshold, thermal_sobel_contours, 
    //               hierarchy_thermal_sobel, RETR_TREE, CHAIN_APPROX_SIMPLE );
    findContours( theraml_sobel_threshold, thermal_sobel_contours, 
                  hierarchy_thermal_sobel, RETR_LIST, CHAIN_APPROX_SIMPLE );
    Mat drawing_thermal_sobel = Mat::zeros( canny_output.size(), CV_8UC3 );
    for( size_t i = 0; i< thermal_sobel_contours.size(); i++ )
    {
        Scalar color = Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
        drawContours( drawing_thermal_sobel, thermal_sobel_contours, (int)i, color, 1, LINE_8, hierarchy_thermal_sobel, 0 );
    }
    imshow( "Contours_thermal_sobel", drawing_thermal_sobel );

    vector<vector<Point> > two_sobel_counter;
    vector<Vec4i> two_sobel_hierarchy;
    cv::Mat two_sobel_edge = my_thermal_canny.Thresholding(ssnon, thresh);
    cv::imshow("2_sobel_canny", two_sobel_edge);
    findContours( two_sobel_edge, two_sobel_counter, two_sobel_hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE );
    Mat drawing_thermal_sobel_sobel = Mat::zeros( canny_output.size(), CV_8UC3 );
    for( size_t i = 0; i< two_sobel_counter.size(); i++ )
    {
        if(two_sobel_counter[i].size()<15){
            continue;
        }
        Scalar color = Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
        drawContours( drawing_thermal_sobel_sobel, two_sobel_counter, (int)i, color, 2, LINE_8, two_sobel_hierarchy, 0 );
    }
    imshow( "Contours_thermal_sobel_sobel", drawing_thermal_sobel_sobel );
}
