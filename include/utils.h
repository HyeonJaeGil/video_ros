#include <opencv2/opencv.hpp>
#include <boost_array.hpp>

<typename T, int N>
cv::Mat arrayToMat (boost::array<T, N> input)
{
    float[] result; 
    for (const auto& it : input)
    {
        result << it;
    }
}