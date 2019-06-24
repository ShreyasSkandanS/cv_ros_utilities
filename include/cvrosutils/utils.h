/*
	OpenCV and ROS Image Processing and Computer Vision Tools
	       Copyright (C) 2019 Shreyas S. Shivakumar

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <https://www.gnu.org/licenses/>.
*/

#ifndef UTILS_BASE_H_
#define UTILS_BASE_H_

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <dirent.h>
#include <sys/time.h>
#include <fstream>

namespace cvrosutils {

    std::vector<cv::Point2f> convertDoublePointVecToFloat(std::vector<cv::Point2d> &cin);

    std::vector<cv::Point2d> convertFloatPointVecToDouble(std::vector<cv::Point2f> &cin);

    bool doesFileExist(std::string fname);

    void getDirectoryContents(std::string directory_in, std::vector<std::string> &files);

    cv::Mat cvMatx33dToMat(cv::Matx33d &mat_in);

    cv::Matx33d cvMatToMatx33d(cv::Mat &mat_in);

    cv::Vec<double, 5> distCoeffMatToVectorPlumbBob(cv::Mat &mat);

    cv::Mat claheHistEq(cv::Mat &in, int cliplimit);

    cv::Mat convertDepthImageTo16bit(cv::Mat &depth_in, int scale_factor, int max_depth);

    void checkConvertBGRToGray(cv::Mat &mat_in);

    void checkConvertRGBToGray(cv::Mat &mat_in);

    void saveImageWithTag(cv::Mat mat_in, std::string pdir, std::string prefix, int idx);
	
    cv::Mat rectifyPlumbBobImage(cv::Mat mat_in, cv::Mat K, cv::Mat K_new, 
				cv::Vec<double, 5> dist_coeffs);

    cv::Mat rectifyFisheyeImage(cv::Mat mat_in, cv::Mat K, cv::Mat K_new, 
				cv::Vec4d dist_coeffs);

}

#endif
//UTILS_BASE_H_
