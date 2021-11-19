/**
* This file is part of ORB-LINE-SLAM
*
* Copyright (C) 2020-2021 John Alamanos, National Technical University of Athens.
* Copyright (C) 2016-2018, Ruben Gomez-Ojeda, University of Malaga.
* Copyright (C) 2016-2018, David Zuñiga-Noël, University of Malaga.         
* Copyright (C) 2016-2018, MAPIR group, University of Malaga.    
*
* ORB-LINE-SLAM is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-LINE-SLAM is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-LINE-SLAM.
* If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef LINEEXTRACTOR_H
#define LINEEXTRACTOR_H

#include <vector>
#include <list>
#include <opencv/cv.h>

#include <line_descriptor_custom.hpp>
#include <line_descriptor/descriptor_custom.hpp>

using namespace cv;
using namespace line_descriptor;

namespace ORB_SLAM3
{

struct sort_lines_by_response
{
    inline bool operator()(const KeyLine& a, const KeyLine& b){
        return ( a.response > b.response );
    }
};

struct sort_lines_by_length
{
    inline bool operator()(const KeyLine& a, const KeyLine& b){
        return ( a.lineLength > b.lineLength );
    }
};

class Lineextractor
{
public:
    Lineextractor(int _lsd_nfeatures, int _lsd_refine, float _lsd_scale, int _nlevels, float _scale, int _extractor);

    ~Lineextractor(){}

    void operator()( const cv::Mat& image, const cv::Mat& mask,
      std::vector<cv::line_descriptor::KeyLine>& keylines,
      cv::Mat& descriptors_line); 

    // Images on the pyramid
    std::vector<cv::Mat> mvImagePyramid_l;
    std::vector<float> mvScaleFactor_l;
    std::vector<float> mvInvScaleFactor_l;
    std::vector<float> mvLevelSigma2_l;
    std::vector<float> mvInvLevelSigma2_l;
    int nlevels_l;

protected:
    // filtering after extraction
    int    lsd_nfeatures;
    double min_line_length;

    // lines detection
    int    lsd_refine;
    float  lsd_scale;
    double lsd_sigma_scale;
    double lsd_quant;
    double lsd_ang_th;
    double lsd_log_eps;
    double lsd_density_th;
    int    lsd_n_bins;
    int    nlevels;
    float  scale;
    int    extractor;
};

} //namespace ORB_SLAM

#endif

