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

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

#include "LineExtractor.h"

using namespace cv;
using namespace line_descriptor;
using namespace std;

namespace ORB_SLAM3
{

Lineextractor::Lineextractor(int _lsd_nfeatures, int _lsd_refine, float _lsd_scale, int _nlevels, float _scale, int _extractor)
    :lsd_nfeatures(_lsd_nfeatures), lsd_refine(_lsd_refine), lsd_scale(_lsd_scale), nlevels(_nlevels), scale(_scale), extractor(_extractor)
{

}

void Lineextractor::operator()( const cv::Mat& img, const cv::Mat& mask, 
            std::vector<cv::line_descriptor::KeyLine>& keylines, cv::Mat& descriptors_line)
{
    // Line Length Threshold
    min_line_length = 0.025;
    if(extractor==0) // LSD Extractor
    {
        // Detect line features
        Ptr<BinaryDescriptor>   lbd = BinaryDescriptor::createBinaryDescriptor();
        Ptr<line_descriptor::LSDDetectorC> lsd = line_descriptor::LSDDetectorC::createLSDDetectorC();
        keylines.clear();
        // lsd parameters
        lsd_sigma_scale = 0.6;
        lsd_quant = 2.0;
        lsd_ang_th = 22.5;
        lsd_log_eps = 1.0;
        lsd_density_th = 0.6;
        lsd_n_bins = 1024;
        line_descriptor::LSDDetectorC::LSDOptions opts;
        opts.refine       = lsd_refine;
        opts.scale        = lsd_scale;
        opts.sigma_scale  = lsd_sigma_scale;
        opts.quant        = lsd_quant;
        opts.ang_th       = lsd_ang_th;
        opts.log_eps      = lsd_log_eps;
        opts.density_th   = lsd_density_th;
        opts.n_bins       = lsd_n_bins;
        opts.min_length   = min_line_length*(std::min(img.cols,img.rows)); 
        lsd->detect( img, keylines, scale, nlevels, opts);
        // filter keyline
        if( int(keylines.size())>lsd_nfeatures && lsd_nfeatures!=0  )
        {
            // sort keylines by their response or by their length
            sort( keylines.begin(), keylines.end(), sort_lines_by_response() );
            //sort( keylines.begin(), keylines.end(), sort_lines_by_length() );
            keylines.resize(lsd_nfeatures);
            // reassign index
            for( int i = 0; i < lsd_nfeatures; i++  )
                    keylines[i].class_id = i;
        }

        nlevels_l=nlevels;
        mvLevelSigma2_l.resize(nlevels);
        mvLevelSigma2_l[0]=1.0f;
        mvInvLevelSigma2_l.resize(nlevels);
        for (int i = 0; i < nlevels; i++) 
        {
            mvImagePyramid_l.push_back(lsd->gaussianPyrs[i]);
            mvScaleFactor_l.push_back(lsd->mvScaleFactor[i]);
            mvInvScaleFactor_l.push_back(lsd->mvInvScaleFactor[i]);
            if (i>0)  
                mvLevelSigma2_l[i]=mvScaleFactor_l[i]*mvScaleFactor_l[i];
            mvInvLevelSigma2_l[i]=1.0f/mvLevelSigma2_l[i];
        } 

        lbd->compute( img, keylines, descriptors_line);
    }
    else if(extractor==1) // ED Extractor
    {
        // Detect line features
        Ptr<BinaryDescriptor>   lbd = BinaryDescriptor::createBinaryDescriptor();
        Ptr<line_descriptor::LSDDetectorC> lsd = line_descriptor::LSDDetectorC::createLSDDetectorC();
        keylines.clear();
        double min_length = min_line_length*(std::min(img.cols,img.rows)); 
        lsd->detect_ED( img, keylines, scale, nlevels, min_length);
        // filter keyline
        if( int(keylines.size())>lsd_nfeatures && lsd_nfeatures!=0  )
        {
            // sort keylines by their response or by their length
            sort( keylines.begin(), keylines.end(), sort_lines_by_response() );
            //sort( keylines.begin(), keylines.end(), sort_lines_by_length() );
            keylines.resize(lsd_nfeatures);
            // reassign index
            for( int i = 0; i < lsd_nfeatures; i++  )
                    keylines[i].class_id = i;
        }

        nlevels_l=nlevels;
        mvLevelSigma2_l.resize(nlevels);
        mvLevelSigma2_l[0]=1.0f;
        mvInvLevelSigma2_l.resize(nlevels);
        for (int i = 0; i < nlevels; i++) 
        {
            mvImagePyramid_l.push_back(lsd->gaussianPyrs[i]);
            mvScaleFactor_l.push_back(lsd->mvScaleFactor[i]);
            mvInvScaleFactor_l.push_back(lsd->mvInvScaleFactor[i]);
            if (i>0)  
                mvLevelSigma2_l[i]=mvScaleFactor_l[i]*mvScaleFactor_l[i];
            mvInvLevelSigma2_l[i]=1.0f/mvLevelSigma2_l[i];
        } 

        lbd->compute( img, keylines, descriptors_line);        
    }
}

} //namespace ORB_SLAM
