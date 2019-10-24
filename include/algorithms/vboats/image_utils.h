#ifndef VBOATS_IMAGE_UTILS_H_
#define VBOATS_IMAGE_UTILS_H_

#include <string>
#include <vector>
#include <cassert>

#include <opencv2/opencv.hpp>

#include "base/matplotlibcpp.h"

using namespace std;
namespace plt = matplotlibcpp;

template<typename... Args>
std::string format(const char* format, Args... args ){
     int length = std::snprintf(nullptr, 0, format, args...);
     assert(length >= 0);

     char* buf = new char[length + 1];
     std::snprintf(buf, length + 1, format, args...);

     std::string str(buf);
     delete[] buf;
     return str;
}

int strip_image(const cv::Mat& input, vector<cv::Mat>* strips, int nstrips = 5, bool cut_horizontally = true, bool visualize=false, bool verbose=false);
int merge_strips(const vector<cv::Mat>& strips, cv::Mat* merged, bool merge_horizontally = true, bool visualize=false, bool verbose=false);

void spread_image(const cv::Mat& input, cv::Mat* output, const cv::Size& target_size, double* fx, double* fy, bool verbose = false);
void spread_image(const cv::Mat& input, cv::Mat* output, double fx, double fy, cv::Size* new_size, bool verbose = false);

void unspread_image(const cv::Mat& input, cv::Mat* output, const cv::Size& target_size, double* fx, double* fy, bool verbose = false);
void unspread_image(const cv::Mat& input, cv::Mat* output, double fx, double fy, cv::Size* new_size, bool verbose = false);

std::string cvtype2str(int type);
std::string cvStrSize(const char* name, const cv::Mat& mat);

void filter_disparity_vmap(const cv::Mat& input, cv::Mat* output, bool verbose = false);


/** TODO */
// def histogram_sliding_filter(hist, window_size=16, flag_plot=False):


/** TODO put these in a seperate place for realsense specific utils */

// cv::Mat frame_to_mat(const rs2::frame& f){
//     using namespace cv;
//     using namespace rs2;
//
//     rs2::video_frame vf = f.as<rs2::video_frame>();
//     const int w = vf.get_width();
//     const int h = vf.get_height();
//
// 	if (f.get_profile().format() == RS2_FORMAT_BGR8){
// 		return cv::Mat(Size(w, h), CV_8UC3, (void*)f.get_data(), cv::Mat::AUTO_STEP);
// 	} else if (f.get_profile().format() == RS2_FORMAT_RGB8){
// 		cv::Mat r = cv::Mat(Size(w, h), CV_8UC3, (void*)f.get_data(), cv::Mat::AUTO_STEP);
// 		cv::cvtColor(r, r, COLOR_RGB2BGR);
// 		return r;
// 	} else if (f.get_profile().format() == RS2_FORMAT_Z16){
// 		return cv::Mat(Size(w, h), CV_16UC1, (void*)f.get_data(), cv::Mat::AUTO_STEP);
// 	} else if (f.get_profile().format() == RS2_FORMAT_Y8){
// 		return cv::Mat(Size(w, h), CV_8UC1, (void*)f.get_data(), cv::Mat::AUTO_STEP);
// 	} else if (f.get_profile().format() == RS2_FORMAT_DISPARITY32){
// 		return cv::Mat(Size(w, h), CV_32FC1, (void*)f.get_data(), cv::Mat::AUTO_STEP);
// 	}
//
//     throw std::runtime_error("Frame format is not supported yet!");
// }
//
// // Converts depth frame to a matrix of doubles with distances in meters
// cv::Mat depth_frame_to_meters(const rs2::pipeline& pipe, const rs2::depth_frame& f){
//     cv::Mat dm = frame_to_mat(f);
//     dm.convertTo(dm, CV_64F);
//     double depth_scale = pipe.get_active_profile().get_device().first<rs2::depth_sensor>().get_depth_scale();
//     dm = dm * depth_scale;
//     return dm;
// }

// ===================================================================================
/**
static void get_field_of_view(const rs2::stream_profile& stream)
    {
        // A sensor's stream (rs2::stream_profile) is in general a stream of data with no specific type.
        // For video streams (streams of images), the sensor that produces the data has a lens and thus has properties such
        //  as a focal point, distortion, and principal point.
        // To get these intrinsics parameters, we need to take a stream and first check if it is a video stream
        if (auto video_stream = stream.as<rs2::video_stream_profile>())
        {
            try
            {
                //If the stream is indeed a video stream, we can now simply call get_intrinsics()
                rs2_intrinsics intrinsics = video_stream.get_intrinsics();

                auto principal_point = std::make_pair(intrinsics.ppx, intrinsics.ppy);
                auto focal_length = std::make_pair(intrinsics.fx, intrinsics.fy);
                rs2_distortion model = intrinsics.model;

                std::cout << "Principal Point         : " << principal_point.first << ", " << principal_point.second << std::endl;
                std::cout << "Focal Length            : " << focal_length.first << ", " << focal_length.second << std::endl;
                std::cout << "Distortion Model        : " << model << std::endl;
                std::cout << "Distortion Coefficients : [" << intrinsics.coeffs[0] << "," << intrinsics.coeffs[1] << "," <<
                    intrinsics.coeffs[2] << "," << intrinsics.coeffs[3] << "," << intrinsics.coeffs[4] << "]" << std::endl;
            }
            catch (const std::exception& e)
            {
                std::cerr << "Failed to get intrinsics for the given stream. " << e.what() << std::endl;
            }
        }
        else if (auto motion_stream = stream.as<rs2::motion_stream_profile>())
        {
            try
            {
                //If the stream is indeed a motion stream, we can now simply call get_motion_intrinsics()
                rs2_motion_device_intrinsic intrinsics = motion_stream.get_motion_intrinsics();

                std::cout << " Scale X      cross axis      cross axis  Bias X \n";
                std::cout << " cross axis    Scale Y        cross axis  Bias Y  \n";
                std::cout << " cross axis    cross axis     Scale Z     Bias Z  \n";
                for (int i = 0; i < 3; i++)
                {
                    for (int j = 0; j < 4; j++)
                    {
                        std::cout << intrinsics.data[i][j] << "    ";
                    }
                    std::cout << "\n";
                }

                std::cout << "Variance of noise for X, Y, Z axis \n";
                for (int i = 0; i < 3; i++)
                    std::cout << intrinsics.noise_variances[i] << " ";
                std::cout << "\n";

                std::cout << "Variance of bias for X, Y, Z axis \n";
                for (int i = 0; i < 3; i++)
                    std::cout << intrinsics.bias_variances[i] << " ";
                std::cout << "\n";
            }
            catch (const std::exception& e)
            {
                std::cerr << "Failed to get intrinsics for the given stream. " << e.what() << std::endl;
            }
        }
        else
        {
            std::cerr << "Given stream profile has no intrinsics data" << std::endl;
        }
    }
*/

// ===================================================================================
/**
void BaseRealSenseNode::publishStaticTransforms()
{
    rs2::stream_profile base_profile = getAProfile(_base_stream);

    // Publish static transforms
    if (_publish_tf)
    {
        for (std::pair<stream_index_pair, bool> ienable : _enable)
        {
            if (ienable.second)
            {
                calcAndPublishStaticTransform(ienable.first, base_profile);
            }
        }
        // Static transform for non-positive values
        if (_tf_publish_rate > 0)
            _tf_t = std::shared_ptr<std::thread>(new std::thread(boost::bind(&BaseRealSenseNode::publishDynamicTransforms, this)));
        else
            _static_tf_broadcaster.sendTransform(_static_tf_msgs);
    }

    // Publish Extrinsics Topics:
    if (_enable[DEPTH] &&
        _enable[FISHEYE])
    {
        static const char* frame_id = "depth_to_fisheye_extrinsics";
        const auto& ex = base_profile.get_extrinsics_to(getAProfile(FISHEYE));

        _depth_to_other_extrinsics[FISHEYE] = ex;
        _depth_to_other_extrinsics_publishers[FISHEYE].publish(rsExtrinsicsToMsg(ex, frame_id));
    }

    if (_enable[DEPTH] &&
        _enable[COLOR])
    {
        static const char* frame_id = "depth_to_color_extrinsics";
        const auto& ex = base_profile.get_extrinsics_to(getAProfile(COLOR));
        _depth_to_other_extrinsics[COLOR] = ex;
        _depth_to_other_extrinsics_publishers[COLOR].publish(rsExtrinsicsToMsg(ex, frame_id));
    }

    if (_enable[DEPTH] &&
        _enable[INFRA1])
    {
        static const char* frame_id = "depth_to_infra1_extrinsics";
        const auto& ex = base_profile.get_extrinsics_to(getAProfile(INFRA1));
        _depth_to_other_extrinsics[INFRA1] = ex;
        _depth_to_other_extrinsics_publishers[INFRA1].publish(rsExtrinsicsToMsg(ex, frame_id));
    }

    if (_enable[DEPTH] &&
        _enable[INFRA2])
    {
        static const char* frame_id = "depth_to_infra2_extrinsics";
        const auto& ex = base_profile.get_extrinsics_to(getAProfile(INFRA2));
        _depth_to_other_extrinsics[INFRA2] = ex;
        _depth_to_other_extrinsics_publishers[INFRA2].publish(rsExtrinsicsToMsg(ex, frame_id));
    }

}
*/

// ===================================================================================
/**
static void change_sensor_option(const rs2::sensor& sensor, rs2_option option_type)
    {
        // Sensors usually have several options to control their properties
        //  such as Exposure, Brightness etc.

        // To control an option, use the following api:

        // First, verify that the sensor actually supports this option
        if (!sensor.supports(option_type))
        {
            std::cerr << "This option is not supported by this sensor" << std::endl;
            return;
        }

        // Each option provides its rs2::option_range to provide information on how it can be changed
        // To get the supported range of an option we do the following:

        std::cout << "Supported range for option " << option_type << ":" << std::endl;

        rs2::option_range range = sensor.get_option_range(option_type);
        float default_value = range.def;
        float maximum_supported_value = range.max;
        float minimum_supported_value = range.min;
        float difference_to_next_value = range.step;
        std::cout << "  Min Value     : " << minimum_supported_value << std::endl;
        std::cout << "  Max Value     : " << maximum_supported_value << std::endl;
        std::cout << "  Default Value : " << default_value << std::endl;
        std::cout << "  Step          : " << difference_to_next_value << std::endl;

        bool change_option = false;
        change_option = prompt_yes_no("Change option's value?");

        if (change_option)
        {
            std::cout << "Enter the new value for this option: ";
            float requested_value;
            std::cin >> requested_value;
            std::cout << std::endl;

            // To set an option to a different value, we can call set_option with a new value
            try
            {
                sensor.set_option(option_type, requested_value);
            }
            catch (const rs2::error& e)
            {
                // Some options can only be set while the camera is streaming,
                // and generally the hardware might fail so it is good practice to catch exceptions from set_option
                std::cerr << "Failed to set option " << option_type << ". (" << e.what() << ")" << std::endl;
            }
        }
    }
*/




#endif // VBOATS_IMAGE_UTILS_H_
