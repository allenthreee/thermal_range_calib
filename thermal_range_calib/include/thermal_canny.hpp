#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace std;
// using namespace cv;

class ThermalCanny
{
    public:
        ThermalCanny();
        ThermalCanny(const std::string &image_name);
        cv::Mat GaussianBlur(cv::Mat);
        cv::Mat Sobel(cv::Mat ); 
        cv::Mat NonMaxSupp(cv::Mat );
        cv::Mat Thresholding(cv::Mat, int threshold);

    private:
        cv::Mat sourceImage;
        cv::Mat angles;
     

};

ThermalCanny::ThermalCanny(){

}

ThermalCanny::ThermalCanny(const std::string &image_name)
{

    sourceImage = cv::imread(image_name, 0);
    int threshold_val = 20;

    cv::Mat grayscaled = sourceImage;
    // cv::Mat gFiltered = GaussianBlur(grayscaled);
    cv::Mat gFiltered = grayscaled;
    cv::Mat sFiltered = Sobel(gFiltered);
    cv::Mat non = NonMaxSupp(sFiltered);
    cv::Mat threshold = Thresholding(non, threshold_val);

    cv::Mat ssFiltered = Sobel(sFiltered);
    cv::Mat ssnon = NonMaxSupp(ssFiltered);
    cv::Mat ssthreshold = Thresholding(ssnon, threshold_val);

    cv::namedWindow("GrayScaled");
    cv::namedWindow("Gaussian Blur");
    cv::namedWindow("Sobel Filtered");
    cv::namedWindow("Edge Thining");
    cv::namedWindow("Threshold sourceImage");

    cv::imshow("GrayScaled", grayscaled);
    // cv::imshow("Gaussian Blur", GaussianBlur(grayscaled));
    cv::imshow("Sobel Filtered", Sobel(gFiltered));
    cv::imshow("Edge Thining", NonMaxSupp(sFiltered));
    cv::imshow("Threshold sourceImage", Thresholding(non, threshold_val));

    cv::namedWindow("SobelSobel Filtered");
    cv::namedWindow("ssEdge Thining");
    cv::namedWindow("ssThreshold sourceImage");
    cv::imshow("SobelSobel Filtered", Sobel(sFiltered));
    cv::imshow("ssEdge Thining", ssnon);
    cv::imshow("ssThreshold sourceImage", ssthreshold);


    cv::waitKey(0);
}

cv::Mat ThermalCanny::GaussianBlur(cv::Mat grayscaled)
{
    std::cout << "GaussianBlur called" << std::endl;
    cv::Mat Gaussiankernel = cv::Mat(cv::Size(3, 3), CV_8UC1);
    double sigma = 3;
    int size = 3;
    double sum_g = 0;

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            Gaussiankernel.at<uchar>(i, j) = (1 / (M_PI)*pow(sigma, 2)) - ((pow(i, 2) + pow(j, 2))) / (2 * pow(sigma, 2));
            sum_g += Gaussiankernel.at<uchar>(i, j);
        }
    }

    cv::Mat blurImage = cv::Mat((sourceImage.rows - size + 1), (sourceImage.cols - size + 1), sourceImage.type());

    int dx = size / 2;
    int dy = size / 2;
    int sumIndex;

    for (int i = 0; i < sourceImage.rows; i++)
    {
        for (int j = 0; j < sourceImage.cols; j++)
        {
            sumIndex = 0;
            for (int k = 0; k < size; k++)
            {
                for (int l = 0; l < size; l++)
                {
                    int x = j - dx + l;
                    int y = i - dy + k;

                    if (x >= 0 && x < blurImage.cols && y >= 0 && y < blurImage.rows)
                    {
                        sumIndex += (Gaussiankernel.at<uchar>(k, l) * sourceImage.at<uchar>(y, x)) / sum_g;
                    }
                }
            }
            blurImage.at<uchar>(i, j) = cv::saturate_cast<uchar>(sumIndex);
        }
    }
    return blurImage;
}

cv::Mat ThermalCanny::Sobel(cv::Mat gFiltered)
{
    int size = 1;

    double xFilter[3][3] = {
        -1.0, 0, 1.0,
        -2.0, 0, 2.0,
        -1.0, 0, 1.0,
    };

    double yFilter[3][3] = {
        -1.0, -2.0, -1.0,
        0, 0, 0,
        1.0, 2.0, 1.0,
    };

    cv::Mat filteredImg = cv::Mat(gFiltered.rows, gFiltered.cols, CV_8UC1);
    angles = cv::Mat(gFiltered.rows, gFiltered.cols, CV_32FC1);

    for (int i = size; i < gFiltered.rows; i++)
    {
        for (int j = size; j < gFiltered.cols; j++)
        {
            double sumx = 0;
            double sumy = 0;

            for (int x = 0; x < 3; x++)
                for (int y = 0; y < 3; y++)
                {
                    sumx += xFilter[x][y] * gFiltered.at<uchar>(i + x, j + y);
                    sumy += yFilter[x][y] * gFiltered.at<uchar>(i + x, j + y);
                }

            double sumxsq = sumx * sumx;
            double sumysq = sumy * sumy;

            double sq2 = sqrt(sumxsq + sumysq);

            if (sq2 > 255)
            {
                sq2 = 255;
            }

            filteredImg.at<uchar>(i, j) = sq2;

            angles.at<float>(i, j) = atan(sumy / sumx);
        }
    }


    return filteredImg;
}

cv::Mat ThermalCanny::NonMaxSupp(cv::Mat sFiltered)
{

    cv::Mat nonMaxSupped = cv::Mat(sFiltered.rows - 2, sFiltered.cols - 2, CV_8UC1);

    for (int i = 1; i < (sFiltered.rows - 1); i++)
    {
        for (int j = 1; j < (sFiltered.cols - 1); j++)
        {
            float Tangent = angles.at<float>(i, j);

            nonMaxSupped.at<uchar>(i - 1, j - 1) = sFiltered.at<uchar>(i, j);

            // Horizontal Edge
            if (((-22.5 < Tangent) && (Tangent <= 22.5)) || ((157.5 < Tangent) && (Tangent <= -157.5)))
            {
                if ((sFiltered.at<uchar>(i, j) < sFiltered.at<uchar>(i, j + 1)) || (sFiltered.at<uchar>(i, j) < sFiltered.at<uchar>(i, j - 1)))
                    nonMaxSupped.at<uchar>(i - 1, j - 1) = 0;
            }

            // Vertical Edge
            if (((-112.5 < Tangent) && (Tangent <= -67.5)) || ((67.5 < Tangent) && (Tangent <= 112.5)))
            {
                if ((sFiltered.at<uchar>(i, j) < sFiltered.at<uchar>(i + 1, j)) || (sFiltered.at<uchar>(i, j) < sFiltered.at<uchar>(i - 1, j)))
                    nonMaxSupped.at<uchar>(i - 1, j - 1) = 0;
            }

            //-45 Degree Edge
            if (((-67.5 < Tangent) && (Tangent <= -22.5)) || ((112.5 < Tangent) && (Tangent <= 157.5)))
            {
                if ((sFiltered.at<uchar>(i, j) < sFiltered.at<uchar>(i - 1, j + 1)) || (sFiltered.at<uchar>(i, j) < sFiltered.at<uchar>(i + 1, j - 1)))
                    nonMaxSupped.at<uchar>(i - 1, j - 1) = 0;
            }

            // 45 Degree Edge
            if (((-157.5 < Tangent) && (Tangent <= -112.5)) || ((22.5 < Tangent) && (Tangent <= 67.5)))
            {
                if ((sFiltered.at<uchar>(i, j) < sFiltered.at<uchar>(i + 1, j + 1)) || (sFiltered.at<uchar>(i, j) < sFiltered.at<uchar>(i - 1, j - 1)))
                    nonMaxSupped.at<uchar>(i - 1, j - 1) = 0;
            }
        }
    }
    return nonMaxSupped;
}

cv::Mat ThermalCanny::Thresholding(cv::Mat non, int threshold)
{
    cv::Mat thres = non;

    // int threshold = 18;
    for (int i = 0; i < non.rows; i++)
    {
        for (int j = 0; j < non.cols; j++)
        {
            // if( i<2 || i>non.rows-2 || j<2 || j<non.cols-2){
            //     thres.at<uchar>(i,j) = 0;
            //     // cout << i << ", " << j << "set as 0 in if" << endl; 
            // }
            if (thres.at<uchar>(i, j) > threshold)
            {
                thres.at<uchar>(i, j) = 255;
            }

            else
            {
                thres.at<uchar>(i, j) = 0;
                // cout << i << ", " << j << "set as 0 in else" << endl; 
            }
        }
    }
    imwrite("single threshold.png", thres);

    return thres;
}