/*  
      OpenCV and ROS Image Processing and Computer Vision Utilities
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

#include "cvrosutils/utils.h"

namespace cvrosutils {

std::vector<cv::Point2d> convertFloatPointVecToDouble(std::vector<cv::Point2f> &cin)
{
    std::vector<cv::Point2d> cout;
    for (int i = 0; i < (int)cin.size(); i++) {
        cv::Point2f pt_2f = cin[i];
        cout.push_back((cv::Point2d)pt_2f);	
    }
    return cout;
}

std::vector<cv::Point2f> convertDoublePointVecToFloat(std::vector<cv::Point2d> &cin)
{
    std::vector<cv::Point2f> cout;
    for (int i = 0; i < (int)cin.size(); i++) {
        cv::Point2d pt_2d = cin[i];
        cout.push_back((cv::Point2f)pt_2d); 
    }
    return cout;
}

cv::Vec<double, 5> distCoeffMatToVectorPlumbBob(cv::Mat &mat_in)
{
    cv::Vec<double, 5> D_plumb_bob;
    D_plumb_bob = cv::Vec<double, 5>(mat_in.at<double>(0,0),
				     mat_in.at<double>(1,0),
				     mat_in.at<double>(3,0),
				     mat_in.at<double>(4,0),
				     mat_in.at<double>(2,0));
   return D_plumb_bob;
}

void checkConvertBGRToGray(cv::Mat &mat_in)
{
    if (mat_in.channels() > 1) {
        cv::cvtColor(mat_in, mat_in, CV_BGR2GRAY);
    }
}

void checkConvertRGBToGray(cv::Mat &mat_in)
{
    if (mat_in.channels() > 1) {
        cv::cvtColor(mat_in, mat_in, CV_RGB2GRAY);
    }
}

cv::Mat claheHistEq(cv::Mat &mat_in, int cliplimit)
{
    cv::Mat mat_out;
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
    clahe->setClipLimit(cliplimit);
    clahe->apply(mat_in, mat_out);
    return mat_out;
}

cv::Mat cvMatx33dToMat(cv::Matx33d &mat_in)
{
    return cv::Mat(mat_in);
}

cv::Matx33d cvMatToMatx33d(cv::Mat &mat_in)
{
    double* data_mat = reinterpret_cast<double*>(mat_in.data);
    return cv::Matx33d(data_mat);
}

bool doesFileExist(std::string fname)
{
    std::ifstream file_calib(fname.c_str());
    return file_calib.good();
}

void getDirectoryContents(std::string directory_in, std::vector<std::string> &files)
{
    DIR *dir;
    dirent *pdir;
    dir = opendir(directory_in.c_str());
    while ((pdir = readdir(dir))) {
      files.push_back(pdir->d_name);
    }
    std::sort(files.begin(), files.end());
}

cv::Mat convertDepthImageTo16bit(cv::Mat &depth_in, int scale_factor, int max_depth)
{
    cv::Mat depth_out = cv::Mat::zeros(depth_in.size(), CV_16UC1);
    cv::patchNaNs(depth_in, 0);
    if (max_depth > 0) {
        depth_in.setTo(max_depth, depth_in > max_depth);
    }
    if (scale_factor > 0) {
        depth_in = depth_in * scale_factor;
    }
    depth_in.convertTo(depth_out, CV_16UC1);
    return depth_out;
}

void saveImageWithTag(cv::Mat mat_in, std::string pdir, std::string prefix, int idx)
{
    std::vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(0);
    std::string f_idx = std::string(10 - std::to_string(idx).length(), '0') 
			+ std::to_string(idx);
    std::string fname = prefix + f_idx + ".png";
    std::string fpath = pdir + fname;
    std::cout << fpath << std::endl;
    cv::imwrite(fpath.c_str(), mat_in, compression_params); 
}

cv::Mat rectifyPlumbBobImage(cv::Mat mat_in, cv::Mat K, cv::Mat K_new, 
			     cv::Vec<double, 5> dist_coeffs)
{
    cv::Mat undist_out;
    cv::undistort(mat_in, undist_out, K, dist_coeffs, K_new);
    return undist_out;
}

cv::Mat rectifyFisheyeImage(cv::Mat mat_in, cv::Mat K, cv::Mat K_new, 
			    cv::Vec4d dist_coeffs)
{
    cv::Mat undist_out;
    cv::fisheye::undistortImage(mat_in, undist_out, K, dist_coeffs, K_new);
    return undist_out;
}

} // namespace cvrosutils

