/**
* This file is part of ORB-LINE-SLAM
*
* Copyright (C) 2020-2021 John Alamanos, National Technical University of Athens.
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
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

#include "Frame.h"

#include "G2oTypes.h"
#include "MapPoint.h"
#include "MapLine.h"
#include "KeyFrame.h"
#include "ORBextractor.h"
#include "Converter.h"
#include "ORBmatcher.h"
#include "GeometricCamera.h"
#include "LineMatcher.h"

#include <thread>
#include <include/CameraModels/Pinhole.h>
#include <include/CameraModels/KannalaBrandt8.h>

namespace ORB_SLAM3
{

long unsigned int Frame::nNextId=0;
bool Frame::mbInitialComputations=true;
float Frame::cx, Frame::cy, Frame::fx, Frame::fy, Frame::invfx, Frame::invfy;
float Frame::mnMinX, Frame::mnMinY, Frame::mnMaxX, Frame::mnMaxY;
float Frame::mfGridElementWidthInv, Frame::mfGridElementHeightInv;

//For stereo fisheye matching
cv::BFMatcher Frame::BFmatcher = cv::BFMatcher(cv::NORM_HAMMING);

Frame::Frame(): mpcpi(NULL), mpImuPreintegrated(NULL), mpPrevFrame(NULL), mpImuPreintegratedFrame(NULL), mpReferenceKF(static_cast<KeyFrame*>(NULL)), mbImuPreintegrated(false)
{
}


//Copy Constructor
Frame::Frame(const Frame &frame)
    :mpcpi(frame.mpcpi),mpORBvocabulary(frame.mpORBvocabulary), mpLinevocabulary(frame.mpLinevocabulary), mpORBextractorLeft(frame.mpORBextractorLeft), mpORBextractorRight(frame.mpORBextractorRight),
     mpLineextractorLeft(frame.mpLineextractorLeft), mpLineextractorRight(frame.mpLineextractorRight),
     mTimeStamp(frame.mTimeStamp), mK(frame.mK.clone()), mDistCoef(frame.mDistCoef.clone()),
     mbf(frame.mbf), mb(frame.mb), mThDepth(frame.mThDepth), N(frame.N),  N_l(frame.N_l), mvKeys(frame.mvKeys), mvKeys_Line(frame.mvKeys_Line),
     mvKeysRight(frame.mvKeysRight), mvKeysRight_Line(frame.mvKeysRight_Line), mvKeysUn(frame.mvKeysUn), mvKeysUn_Line(frame.mvKeysUn_Line), mvuRight(frame.mvuRight),
     mvDepth(frame.mvDepth), mvDepth_l(frame.mvDepth_l), mvDisparity_l(frame.mvDisparity_l), mvle_l(frame.mvle_l), mBowVec(frame.mBowVec), mFeatVec(frame.mFeatVec),
     mDescriptors(frame.mDescriptors.clone()), mDescriptors_Line(frame.mDescriptors_Line.clone()), mDescriptorsRight(frame.mDescriptorsRight.clone()), mDescriptorsRight_Line(frame.mDescriptorsRight_Line.clone()),
     mvpMapPoints(frame.mvpMapPoints), mvpMapLines(frame.mvpMapLines), mvbOutlier(frame.mvbOutlier), mvbOutlier_Line(frame.mvbOutlier_Line), mImuCalib(frame.mImuCalib), mnCloseMPs(frame.mnCloseMPs), mnCloseMLs(frame.mnCloseMLs), 
     mpImuPreintegrated(frame.mpImuPreintegrated), mpImuPreintegratedFrame(frame.mpImuPreintegratedFrame), mImuBias(frame.mImuBias),
     mnId(frame.mnId), mpReferenceKF(frame.mpReferenceKF), mnScaleLevels(frame.mnScaleLevels), mnScaleLevels_l(frame.mnScaleLevels_l),
     mfScaleFactor(frame.mfScaleFactor), mfLogScaleFactor(frame.mfLogScaleFactor),
     mvScaleFactors(frame.mvScaleFactors), mvInvScaleFactors(frame.mvInvScaleFactors), mvScaleFactors_l(frame.mvScaleFactors_l), mvInvScaleFactors_l(frame.mvInvScaleFactors_l), mNameFile(frame.mNameFile), mnDataset(frame.mnDataset),
     mvLevelSigma2(frame.mvLevelSigma2), mvInvLevelSigma2(frame.mvInvLevelSigma2), mvLevelSigma2_l(frame.mvLevelSigma2_l), mvInvLevelSigma2_l(frame.mvInvLevelSigma2_l), mpPrevFrame(frame.mpPrevFrame), mpLastKeyFrame(frame.mpLastKeyFrame), mbImuPreintegrated(frame.mbImuPreintegrated), mpMutexImu(frame.mpMutexImu),
     mpCamera(frame.mpCamera), mpCamera2(frame.mpCamera2), Nleft(frame.Nleft), Nright(frame.Nright),
     monoLeft(frame.monoLeft), monoRight(frame.monoRight), mvLeftToRightMatch(frame.mvLeftToRightMatch),
     mvRightToLeftMatch(frame.mvRightToLeftMatch), mvStereo3Dpoints(frame.mvStereo3Dpoints),
     mTlr(frame.mTlr.clone()), mRlr(frame.mRlr.clone()), mtlr(frame.mtlr.clone()), mTrl(frame.mTrl.clone()), mTimeStereoMatch(frame.mTimeStereoMatch), mTimeStereoMatch_Lines(frame.mTimeStereoMatch_Lines), 
     n_inliers(frame.n_inliers), n_inliers_pt(frame.n_inliers_pt), n_inliers_ls(frame.n_inliers_ls), 
     mTimeORB_Ext(frame.mTimeORB_Ext), mTimeLines_Ext(frame.mTimeLines_Ext), inv_width(frame.inv_width), inv_height(frame.inv_width)
{
    for(int i=0;i<FRAME_GRID_COLS;i++)
        for(int j=0; j<FRAME_GRID_ROWS; j++){
            mGrid[i][j]=frame.mGrid[i][j];
            if(frame.Nleft > 0){
                mGridRight[i][j] = frame.mGridRight[i][j];
            }
        }

    if(!frame.mTcw.empty())
    {
        SetPose(frame.mTcw);
        mTcw_prev = frame.mTcw_prev.clone();
    }

    if(!frame.mVw.empty())
        mVw = frame.mVw.clone();

    mmProjectPoints = frame.mmProjectPoints;
    mmMatchedInImage = frame.mmMatchedInImage;
}

Frame::Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor* extractorLeft, ORBextractor* extractorRight, ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth, GeometricCamera* pCamera, Frame* pPrevF, const IMU::Calib &ImuCalib)
    :mpcpi(NULL), mpORBvocabulary(voc),mpORBextractorLeft(extractorLeft),mpORBextractorRight(extractorRight), mTimeStamp(timeStamp), mK(K.clone()), mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth),
     mImuCalib(ImuCalib), mpImuPreintegrated(NULL), mpPrevFrame(pPrevF),mpImuPreintegratedFrame(NULL), mpReferenceKF(static_cast<KeyFrame*>(NULL)), mbImuPreintegrated(false),
     mpCamera(pCamera) ,mpCamera2(nullptr), mTimeStereoMatch(0), mTimeORB_Ext(0)
{
    // Frame ID
    mnId=nNextId++;

    // Scale Level Info
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // ORB extraction
#ifdef SAVE_TIMES
    std::chrono::steady_clock::time_point time_StartExtORB = std::chrono::steady_clock::now();
#endif
    thread threadLeft(&Frame::ExtractORB,this,0,imLeft,0,0);
    thread threadRight(&Frame::ExtractORB,this,1,imRight,0,0);
    threadLeft.join();
    threadRight.join();
#ifdef SAVE_TIMES
    std::chrono::steady_clock::time_point time_EndExtORB = std::chrono::steady_clock::now();

    mTimeORB_Ext = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndExtORB - time_StartExtORB).count();
#endif


    N = mvKeys.size();
    if(mvKeys.empty())
        return;

    UndistortKeyPoints();

#ifdef SAVE_TIMES
    std::chrono::steady_clock::time_point time_StartStereoMatches = std::chrono::steady_clock::now();
#endif
    ComputeStereoMatches();
#ifdef SAVE_TIMES
    std::chrono::steady_clock::time_point time_EndStereoMatches = std::chrono::steady_clock::now();
    
    mTimeStereoMatch = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndStereoMatches - time_StartStereoMatches).count();
#endif


    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));
    mvbOutlier = vector<bool>(N,false);
    mmProjectPoints.clear();// = map<long unsigned int, cv::Point2f>(N, static_cast<cv::Point2f>(NULL));
    mmMatchedInImage.clear();


    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        ComputeImageBounds(imLeft);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/(mnMaxY-mnMinY);



        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    mb = mbf/fx;

    if(pPrevF)
    {
        if(!pPrevF->mVw.empty())
            mVw = pPrevF->mVw.clone();
    }
    else
    {
        mVw = cv::Mat::zeros(3,1,CV_32F);
    }

    AssignFeaturesToGrid();

    mpMutexImu = new std::mutex();

    //Set no stereo fisheye information
    Nleft = -1;
    Nright = -1;
    mvLeftToRightMatch = vector<int>(0);
    mvRightToLeftMatch = vector<int>(0);
    mTlr = cv::Mat(3,4,CV_32F);
    mTrl = cv::Mat(3,4,CV_32F);
    mvStereo3Dpoints = vector<cv::Mat>(0);
    monoLeft = -1;
    monoRight = -1;
}

// Constructor for stereo cameras with lines
Frame::Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor* extractorLeft, ORBextractor* extractorRight, Lineextractor* LineextractorLeft, Lineextractor* LineextractorRight, ORBVocabulary* voc, LineVocabulary* voc_l, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth, GeometricCamera* pCamera, Frame* pPrevF, const IMU::Calib &ImuCalib)
    :mpcpi(NULL), mpORBvocabulary(voc), mpLinevocabulary(voc_l), mpORBextractorLeft(extractorLeft),mpORBextractorRight(extractorRight),
     mpLineextractorLeft(LineextractorLeft),mpLineextractorRight(LineextractorRight),
     mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth),
     mImuCalib(ImuCalib), mpImuPreintegrated(NULL), mpPrevFrame(pPrevF), mpImuPreintegratedFrame(NULL), mpReferenceKF(static_cast<KeyFrame*>(NULL)), mbImuPreintegrated(false),
     mpCamera(pCamera) ,mpCamera2(nullptr), mTimeStereoMatch(0), mTimeStereoMatch_Lines(0), mTimeORB_Ext(0), mTimeLines_Ext(0)
{

    inv_width  = FRAME_GRID_COLS / static_cast<double>(imLeft.cols);
    inv_height = FRAME_GRID_ROWS / static_cast<double>(imRight.rows);

    // Frame ID
    mnId=nNextId++;

    // Scale Level Info
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // ORB extraction
#ifdef SAVE_TIMES
    std::chrono::steady_clock::time_point time_StartExtORB = std::chrono::steady_clock::now();
#endif
    thread threadLeft(&Frame::ExtractORB,this,0,imLeft,0,0);
    thread threadRight(&Frame::ExtractORB,this,1,imRight,0,0);
    threadLeft.join();
    threadRight.join();
#ifdef SAVE_TIMES
    std::chrono::steady_clock::time_point time_EndExtORB = std::chrono::steady_clock::now();

    mTimeORB_Ext = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndExtORB - time_StartExtORB).count();
#endif


    N = mvKeys.size();
    if(mvKeys.empty())
        return;
    UndistortKeyPoints();


    // Line Extraction
#ifdef SAVE_TIMES
    std::chrono::steady_clock::time_point time_StartExtLines = std::chrono::steady_clock::now();
#endif
    thread threadLeft_Line(&Frame::ExtractLine,this,0,imLeft);
    thread threadRight_Line(&Frame::ExtractLine,this,1,imRight);
    threadLeft_Line.join();
    threadRight_Line.join();
#ifdef SAVE_TIMES
    std::chrono::steady_clock::time_point time_EndExtLines = std::chrono::steady_clock::now();

    mTimeLines_Ext = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndExtLines - time_StartExtLines).count();
    mTimeTotal_Ext = mTimeORB_Ext + mTimeLines_Ext;
#endif

    mnScaleLevels_l = mpLineextractorLeft->nlevels_l;
    mvScaleFactors_l = mpLineextractorLeft->mvScaleFactor_l;
    mvInvScaleFactors_l = mpLineextractorLeft->mvInvScaleFactor_l;
    mvLevelSigma2_l =  mpLineextractorLeft->mvLevelSigma2_l;
    mvInvLevelSigma2_l = mpLineextractorLeft->mvInvLevelSigma2_l;

#ifdef SAVE_TIMES
    std::chrono::steady_clock::time_point time_StartStereoMatches = std::chrono::steady_clock::now();
#endif
    ComputeStereoMatches();
#ifdef SAVE_TIMES
    std::chrono::steady_clock::time_point time_EndStereoMatches = std::chrono::steady_clock::now();

    mTimeStereoMatch = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndStereoMatches - time_StartStereoMatches).count();
#endif

    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));
    mvbOutlier = vector<bool>(N,false);
    mmProjectPoints.clear();// = map<long unsigned int, cv::Point2f>(N, static_cast<cv::Point2f>(NULL));
    mmMatchedInImage.clear();  

    N_l = mvKeys_Line.size();

    UndistortKeyLines();
#ifdef SAVE_TIMES
    std::chrono::steady_clock::time_point time_StartStereoMatches_Lines = std::chrono::steady_clock::now();
#endif
    ComputeStereoMatches_Lines();
#ifdef SAVE_TIMES
    std::chrono::steady_clock::time_point time_EndStereoMatches_Lines = std::chrono::steady_clock::now();

    mTimeStereoMatch_Lines = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndStereoMatches_Lines - time_StartStereoMatches_Lines).count();
    mTimeStereoMatchTotal = mTimeStereoMatch + mTimeStereoMatch_Lines;
#endif
    
 
    mvpMapLines = vector<MapLine*>(N_l,static_cast<MapLine*>(NULL));
    mvbOutlier_Line = vector<bool>(N_l,false);

    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        ComputeImageBounds(imLeft);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/(mnMaxY-mnMinY);



        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    mb = mbf/fx;

    if(pPrevF)
    {
        if(!pPrevF->mVw.empty())
            mVw = pPrevF->mVw.clone();
    }
    else
    {
        mVw = cv::Mat::zeros(3,1,CV_32F);
    }

    AssignFeaturesToGrid();

    //Fill in grid & directions -> Used for Line Matching By Projection
    /*list<pair<int, int>> line_coords;
    grid_Line = GridStructure(FRAME_GRID_ROWS, FRAME_GRID_COLS);
    for (unsigned int idx = 0; idx < mvKeysUn_Line.size(); ++idx) 
    {
        const KeyLine &kl = mvKeysUn_Line[idx];

        getLineCoords(kl.startPointX * inv_width, kl.startPointY * inv_height, kl.endPointX * inv_width, kl.endPointY * inv_height, line_coords);
        for (const std::pair<int, int> &p : line_coords)
            grid_Line.at(p.first, p.second).push_back(idx);
    } */ 

    mpMutexImu = new std::mutex();

    //Set no stereo fisheye information
    Nleft = -1;
    Nright = -1;
    mvLeftToRightMatch = vector<int>(0);
    mvRightToLeftMatch = vector<int>(0);
    mTlr = cv::Mat(3,4,CV_32F);
    mTrl = cv::Mat(3,4,CV_32F);
    mvStereo3Dpoints = vector<cv::Mat>(0);
    monoLeft = -1;
    monoRight = -1;
}

Frame::Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth, GeometricCamera* pCamera,Frame* pPrevF, const IMU::Calib &ImuCalib)
    :mpcpi(NULL),mpORBvocabulary(voc),mpORBextractorLeft(extractor),mpORBextractorRight(static_cast<ORBextractor*>(NULL)),
     mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth),
     mImuCalib(ImuCalib), mpImuPreintegrated(NULL), mpPrevFrame(pPrevF), mpImuPreintegratedFrame(NULL), mpReferenceKF(static_cast<KeyFrame*>(NULL)), mbImuPreintegrated(false),
     mpCamera(pCamera),mpCamera2(nullptr), mTimeStereoMatch(0), mTimeORB_Ext(0)
{
    // Frame ID
    mnId=nNextId++;

    // Scale Level Info
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // ORB extraction
#ifdef SAVE_TIMES
    std::chrono::steady_clock::time_point time_StartExtORB = std::chrono::steady_clock::now();
#endif
    ExtractORB(0,imGray,0,0);
#ifdef SAVE_TIMES
    std::chrono::steady_clock::time_point time_EndExtORB = std::chrono::steady_clock::now();

    mTimeORB_Ext = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndExtORB - time_StartExtORB).count();
#endif


    N = mvKeys.size();

    if(mvKeys.empty())
        return;

    UndistortKeyPoints();

    ComputeStereoFromRGBD(imDepth);

    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));

    mmProjectPoints.clear();// = map<long unsigned int, cv::Point2f>(N, static_cast<cv::Point2f>(NULL));
    mmMatchedInImage.clear();

    mvbOutlier = vector<bool>(N,false);

    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        ComputeImageBounds(imGray);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    mb = mbf/fx;

    mpMutexImu = new std::mutex();

    //Set no stereo fisheye information
    Nleft = -1;
    Nright = -1;
    mvLeftToRightMatch = vector<int>(0);
    mvRightToLeftMatch = vector<int>(0);
    mTlr = cv::Mat(3,4,CV_32F);
    mTrl = cv::Mat(3,4,CV_32F);
    mvStereo3Dpoints = vector<cv::Mat>(0);
    monoLeft = -1;
    monoRight = -1;

    AssignFeaturesToGrid();
}


Frame::Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, GeometricCamera* pCamera, cv::Mat &distCoef, const float &bf, const float &thDepth, Frame* pPrevF, const IMU::Calib &ImuCalib)
    :mpcpi(NULL),mpORBvocabulary(voc),mpORBextractorLeft(extractor),mpORBextractorRight(static_cast<ORBextractor*>(NULL)),
     mTimeStamp(timeStamp), mK(static_cast<Pinhole*>(pCamera)->toK()), mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth),
     mImuCalib(ImuCalib), mpImuPreintegrated(NULL),mpPrevFrame(pPrevF),mpImuPreintegratedFrame(NULL), mpReferenceKF(static_cast<KeyFrame*>(NULL)), mbImuPreintegrated(false), mpCamera(pCamera),
     mpCamera2(nullptr), mTimeStereoMatch(0), mTimeORB_Ext(0)
{
    // Frame ID
    mnId=nNextId++;

    // Scale Level Info
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // ORB extraction
#ifdef SAVE_TIMES
    std::chrono::steady_clock::time_point time_StartExtORB = std::chrono::steady_clock::now();
#endif
    ExtractORB(0,imGray,0,1000);
#ifdef SAVE_TIMES
    std::chrono::steady_clock::time_point time_EndExtORB = std::chrono::steady_clock::now();

    mTimeORB_Ext = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndExtORB - time_StartExtORB).count();
#endif


    N = mvKeys.size();
    if(mvKeys.empty())
        return;

    UndistortKeyPoints();

    // Set no stereo information
    mvuRight = vector<float>(N,-1);
    mvDepth = vector<float>(N,-1);
    mnCloseMPs = 0;

    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));

    mmProjectPoints.clear();// = map<long unsigned int, cv::Point2f>(N, static_cast<cv::Point2f>(NULL));
    mmMatchedInImage.clear();

    mvbOutlier = vector<bool>(N,false);

    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        ComputeImageBounds(imGray);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

        fx = static_cast<Pinhole*>(mpCamera)->toK().at<float>(0,0);
        fy = static_cast<Pinhole*>(mpCamera)->toK().at<float>(1,1);
        cx = static_cast<Pinhole*>(mpCamera)->toK().at<float>(0,2);
        cy = static_cast<Pinhole*>(mpCamera)->toK().at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }


    mb = mbf/fx;

    //Set no stereo fisheye information
    Nleft = -1;
    Nright = -1;
    mvLeftToRightMatch = vector<int>(0);
    mvRightToLeftMatch = vector<int>(0);
    mTlr = cv::Mat(3,4,CV_32F);
    mTrl = cv::Mat(3,4,CV_32F);
    mvStereo3Dpoints = vector<cv::Mat>(0);
    monoLeft = -1;
    monoRight = -1;

    AssignFeaturesToGrid();

    // mVw = cv::Mat::zeros(3,1,CV_32F);
    if(pPrevF)
    {
        if(!pPrevF->mVw.empty())
            mVw = pPrevF->mVw.clone();
    }
    else
    {
        mVw = cv::Mat::zeros(3,1,CV_32F);
    }

    mpMutexImu = new std::mutex();
}


void Frame::AssignFeaturesToGrid()
{
    // Fill matrix with points
    const int nCells = FRAME_GRID_COLS*FRAME_GRID_ROWS;

    int nReserve = 0.5f*N/(nCells);

    for(unsigned int i=0; i<FRAME_GRID_COLS;i++)
        for (unsigned int j=0; j<FRAME_GRID_ROWS;j++){
            mGrid[i][j].reserve(nReserve);
            if(Nleft != -1){
                mGridRight[i][j].reserve(nReserve);
            }
        }



    for(int i=0;i<N;i++)
    {
        const cv::KeyPoint &kp = (Nleft == -1) ? mvKeysUn[i]
                                                 : (i < Nleft) ? mvKeys[i]
                                                                 : mvKeysRight[i - Nleft];

        int nGridPosX, nGridPosY;
        if(PosInGrid(kp,nGridPosX,nGridPosY)){
            if(Nleft == -1 || i < Nleft)
                mGrid[nGridPosX][nGridPosY].push_back(i);
            else
                mGridRight[nGridPosX][nGridPosY].push_back(i - Nleft);
        }
    }
}

void Frame::ExtractORB(int flag, const cv::Mat &im, const int x0, const int x1)
{
    vector<int> vLapping = {x0,x1};
    if(flag==0)
        monoLeft = (*mpORBextractorLeft)(im,cv::Mat(),mvKeys,mDescriptors,vLapping);
    else
        monoRight = (*mpORBextractorRight)(im,cv::Mat(),mvKeysRight,mDescriptorsRight,vLapping);
}

void Frame::ExtractLine(int flag, const cv::Mat &im)
{
    if(flag==0)
        (*mpLineextractorLeft)(im,cv::Mat(),mvKeys_Line,mDescriptors_Line);
    else
        (*mpLineextractorRight)(im,cv::Mat(),mvKeysRight_Line,mDescriptorsRight_Line);
}

void Frame::SetPose(cv::Mat Tcw)
{
    mTcw = Tcw.clone();
    UpdatePoseMatrices();
}

void Frame::GetPose(cv::Mat &Tcw)
{
    Tcw = mTcw.clone();
}

void Frame::SetNewBias(const IMU::Bias &b)
{
    mImuBias = b;
    if(mpImuPreintegrated)
        mpImuPreintegrated->SetNewBias(b);
}

void Frame::SetVelocity(const cv::Mat &Vwb)
{
    mVw = Vwb.clone();
}

void Frame::SetImuPoseVelocity(const cv::Mat &Rwb, const cv::Mat &twb, const cv::Mat &Vwb)
{
    mVw = Vwb.clone();
    cv::Mat Rbw = Rwb.t();
    cv::Mat tbw = -Rbw*twb;
    cv::Mat Tbw = cv::Mat::eye(4,4,CV_32F);
    Rbw.copyTo(Tbw.rowRange(0,3).colRange(0,3));
    tbw.copyTo(Tbw.rowRange(0,3).col(3));
    mTcw = mImuCalib.Tcb*Tbw;
    UpdatePoseMatrices();
}



void Frame::UpdatePoseMatrices()
{
    mRcw = mTcw.rowRange(0,3).colRange(0,3);
    mRwc = mRcw.t();
    mtcw = mTcw.rowRange(0,3).col(3);
    mOw = -mRcw.t()*mtcw;
}

cv::Mat Frame::GetImuPosition()
{
    return mRwc*mImuCalib.Tcb.rowRange(0,3).col(3)+mOw;
}

cv::Mat Frame::GetImuRotation()
{
    return mRwc*mImuCalib.Tcb.rowRange(0,3).colRange(0,3);
}

cv::Mat Frame::GetImuPose()
{
    cv::Mat Twb = cv::Mat::eye(4,4,CV_32F);
    Twb.rowRange(0,3).colRange(0,3) = mRwc*mImuCalib.Tcb.rowRange(0,3).colRange(0,3);
    Twb.rowRange(0,3).col(3) = mRwc*mImuCalib.Tcb.rowRange(0,3).col(3)+mOw;
    return Twb.clone();
}


bool Frame::isInFrustum(MapPoint *pMP, float viewingCosLimit)
{
    if(Nleft == -1){
        // cout << "\na";
        pMP->mbTrackInView = false;
        pMP->mTrackProjX = -1;
        pMP->mTrackProjY = -1;

        // 3D in absolute coordinates
        cv::Mat P = pMP->GetWorldPos();

        // cout << "b";

        // 3D in camera coordinates
        const cv::Mat Pc = mRcw*P+mtcw;
        const float Pc_dist = cv::norm(Pc);

        // Check positive depth
        const float &PcZ = Pc.at<float>(2);
        const float invz = 1.0f/PcZ;
        if(PcZ<0.0f)
            return false;

        const cv::Point2f uv = mpCamera->project(Pc);

        // cout << "c";

        if(uv.x<mnMinX || uv.x>mnMaxX)
            return false;
        if(uv.y<mnMinY || uv.y>mnMaxY)
            return false;

        // cout << "d";
        pMP->mTrackProjX = uv.x;
        pMP->mTrackProjY = uv.y;

        // Check distance is in the scale invariance region of the MapPoint
        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();
        const cv::Mat PO = P-mOw;
        const float dist = cv::norm(PO);

        if(dist<minDistance || dist>maxDistance)
            return false;

        // cout << "e";

        // Check viewing angle
        cv::Mat Pn = pMP->GetNormal();

        // cout << "f";

        const float viewCos = PO.dot(Pn)/dist;

        if(viewCos<viewingCosLimit)
            return false;

        // Predict scale in the image
        const int nPredictedLevel = pMP->PredictScale(dist,this);

        // cout << "g";

        // Data used by the tracking
        pMP->mbTrackInView = true;
        pMP->mTrackProjX = uv.x;
        pMP->mTrackProjXR = uv.x - mbf*invz;

        pMP->mTrackDepth = Pc_dist;
        // cout << "h";

        pMP->mTrackProjY = uv.y;
        pMP->mnTrackScaleLevel= nPredictedLevel;
        pMP->mTrackViewCos = viewCos;

        // cout << "i";

        return true;
    }
    else{
        pMP->mbTrackInView = false;
        pMP->mbTrackInViewR = false;
        pMP -> mnTrackScaleLevel = -1;
        pMP -> mnTrackScaleLevelR = -1;

        pMP->mbTrackInView = isInFrustumChecks(pMP,viewingCosLimit);
        pMP->mbTrackInViewR = isInFrustumChecks(pMP,viewingCosLimit,true);

        return pMP->mbTrackInView || pMP->mbTrackInViewR;
    }
}

bool Frame::isInFrustum_l(MapLine *pML, float viewingCosLimit)
{
    pML->mbTrackInView = false;

    // 3D in absolute coordinates
    Vector6d sep = pML->GetWorldPos();
    Vector3d sp_eigen = sep.head(3);
    Vector3d ep_eigen = sep.tail(3);
    cv::Mat sp = Converter::toCvMat(sp_eigen);
    cv::Mat ep = Converter::toCvMat(ep_eigen);
    {
        // 3D in camera coordinates
        const cv::Mat Pc = mRcw*sp+mtcw;
        const float &PcX = Pc.at<float>(0);
        const float &PcY= Pc.at<float>(1);
        const float &PcZ = Pc.at<float>(2);

        // Check positive depth
        if(PcZ<0.0f)
            return false;

        // Project in image and check it is not outside
        const float invz = 1.0f/PcZ;
        const float u=fx*PcX*invz+cx;
        const float v=fy*PcY*invz+cy;

        if(u<mnMinX || u>mnMaxX)
            return false;
        if(v<mnMinY || v>mnMaxY)
            return false;

        pML->mTrackProjsX = u;
        pML->mTrackProjsY = v;
    }
    {
        // 3D in camera coordinates
        const cv::Mat Pc = mRcw*ep+mtcw;
        const float &PcX = Pc.at<float>(0);
        const float &PcY= Pc.at<float>(1);
        const float &PcZ = Pc.at<float>(2);

        // Check positive depth
        if(PcZ<0.0f)
            return false;

        // Project in image and check it is not outside
        const float invz = 1.0f/PcZ;
        const float u=fx*PcX*invz+cx;
        const float v=fy*PcY*invz+cy;

        if(u<mnMinX || u>mnMaxX)
            return false;
        if(v<mnMinY || v>mnMaxY)
            return false;

        pML->mTrackProjeX = u;
        pML->mTrackProjeY = v;
    }

    Vector3d MidPoint = (sp_eigen+ep_eigen)/2;
    cv::Mat P = Converter::toCvMat(MidPoint);

    // Check distance is in the scale invariance region of the MapPoint
    const float maxDistance = pML->GetMaxDistanceInvariance();
    const float minDistance = pML->GetMinDistanceInvariance();
    const cv::Mat PO = P-mOw;
    const float dist = cv::norm(PO);

    if(dist<minDistance || dist>maxDistance)
        return false;

    // Check viewing angle
    cv::Mat Pn = pML->GetNormal();

    const float viewCos = PO.dot(Pn)/dist;

    if(viewCos<viewingCosLimit)
        return false;

    // Data used by the tracking
    pML->mbTrackInView = true;
    pML->mnTrackangle = atan2(pML->mTrackProjeY - pML->mTrackProjsY, pML->mTrackProjeX - pML->mTrackProjsX);

    return true;
}

bool Frame::ProjectPointDistort(MapPoint* pMP, cv::Point2f &kp, float &u, float &v)
{

    // 3D in absolute coordinates
    cv::Mat P = pMP->GetWorldPos();

    // 3D in camera coordinates
    const cv::Mat Pc = mRcw*P+mtcw;
    const float &PcX = Pc.at<float>(0);
    const float &PcY= Pc.at<float>(1);
    const float &PcZ = Pc.at<float>(2);

    // Check positive depth
    if(PcZ<0.0f)
    {
        cout << "Negative depth: " << PcZ << endl;
        return false;
    }

    // Project in image and check it is not outside
    const float invz = 1.0f/PcZ;
    u=fx*PcX*invz+cx;
    v=fy*PcY*invz+cy;

    // cout << "c";

    if(u<mnMinX || u>mnMaxX)
        return false;
    if(v<mnMinY || v>mnMaxY)
        return false;

    float u_distort, v_distort;

    float x = (u - cx) * invfx;
    float y = (v - cy) * invfy;
    float r2 = x * x + y * y;
    float k1 = mDistCoef.at<float>(0);
    float k2 = mDistCoef.at<float>(1);
    float p1 = mDistCoef.at<float>(2);
    float p2 = mDistCoef.at<float>(3);
    float k3 = 0;
    if(mDistCoef.total() == 5)
    {
        k3 = mDistCoef.at<float>(4);
    }

    // Radial distorsion
    float x_distort = x * (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);
    float y_distort = y * (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);

    // Tangential distorsion
    x_distort = x_distort + (2 * p1 * x * y + p2 * (r2 + 2 * x * x));
    y_distort = y_distort + (p1 * (r2 + 2 * y * y) + 2 * p2 * x * y);

    u_distort = x_distort * fx + cx;
    v_distort = y_distort * fy + cy;


    u = u_distort;
    v = v_distort;

    kp = cv::Point2f(u, v);

    return true;
}

cv::Mat Frame::inRefCoordinates(cv::Mat pCw)
{
    return mRcw*pCw+mtcw;
}

vector<size_t> Frame::GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel, const int maxLevel, const bool bRight) const
{
    vector<size_t> vIndices;
    vIndices.reserve(N);

    float factorX = r;
    float factorY = r;

    /*cout << "fX " << factorX << endl;
    cout << "fY " << factorY << endl;*/

    const int nMinCellX = max(0,(int)floor((x-mnMinX-factorX)*mfGridElementWidthInv));
    if(nMinCellX>=FRAME_GRID_COLS)
    {
        return vIndices;
    }

    const int nMaxCellX = min((int)FRAME_GRID_COLS-1,(int)ceil((x-mnMinX+factorX)*mfGridElementWidthInv));
    if(nMaxCellX<0)
    {
        return vIndices;
    }

    const int nMinCellY = max(0,(int)floor((y-mnMinY-factorY)*mfGridElementHeightInv));
    if(nMinCellY>=FRAME_GRID_ROWS)
    {
        return vIndices;
    }

    const int nMaxCellY = min((int)FRAME_GRID_ROWS-1,(int)ceil((y-mnMinY+factorY)*mfGridElementHeightInv));
    if(nMaxCellY<0)
    {
        return vIndices;
    }

    const bool bCheckLevels = (minLevel>0) || (maxLevel>=0);

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            const vector<size_t> vCell = (!bRight) ? mGrid[ix][iy] : mGridRight[ix][iy];
            if(vCell.empty())
                continue;

            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = (Nleft == -1) ? mvKeysUn[vCell[j]]
                                                         : (!bRight) ? mvKeys[vCell[j]]
                                                                     : mvKeysRight[vCell[j]];
                if(bCheckLevels)
                {
                    if(kpUn.octave<minLevel)
                        continue;
                    if(maxLevel>=0)
                        if(kpUn.octave>maxLevel)
                            continue;
                }

                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;

                if(fabs(distx)<factorX && fabs(disty)<factorY)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}

bool Frame::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY)
{
    posX = round((kp.pt.x-mnMinX)*mfGridElementWidthInv);
    posY = round((kp.pt.y-mnMinY)*mfGridElementHeightInv);

    //Keypoint's coordinates are undistorted, which could cause to go out of the image
    if(posX<0 || posX>=FRAME_GRID_COLS || posY<0 || posY>=FRAME_GRID_ROWS)
        return false;

    return true;
}


void Frame::ComputeBoW()
{
    if(mBowVec.empty())
    {
        vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
        mpORBvocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);
    }
}

void Frame::UndistortKeyPoints()
{
    if(mDistCoef.at<float>(0)==0.0)
    {
        mvKeysUn=mvKeys;
        return;
    }

    // Fill matrix with points
    cv::Mat mat(N,2,CV_32F);

    for(int i=0; i<N; i++)
    {
        mat.at<float>(i,0)=mvKeys[i].pt.x;
        mat.at<float>(i,1)=mvKeys[i].pt.y;
    }

    // Undistort points
    mat=mat.reshape(2);
    cv::undistortPoints(mat,mat, static_cast<Pinhole*>(mpCamera)->toK(),mDistCoef,cv::Mat(),mK);
    mat=mat.reshape(1);


    // Fill undistorted keypoint vector
    mvKeysUn.resize(N);
    for(int i=0; i<N; i++)
    {
        cv::KeyPoint kp = mvKeys[i];
        kp.pt.x=mat.at<float>(i,0);
        kp.pt.y=mat.at<float>(i,1);
        mvKeysUn[i]=kp;
    }

}

void Frame::UndistortKeyLines()
{
    if(mDistCoef.at<float>(0)==0.0)
    {
        mvKeysUn_Line = mvKeys_Line;
        return;
    }

    N_l = mvKeys_Line.size();   // update N_l

    // Fill matrix with points
    cv::Mat mat_s(N_l,2,CV_32F);
    cv::Mat mat_e(N_l,2,CV_32F);
    for(int i=0; i<N_l; i++)
    {
        mat_s.at<float>(i,0)=mvKeys_Line[i].startPointX;
        mat_s.at<float>(i,1)=mvKeys_Line[i].startPointY;
        mat_e.at<float>(i,0)=mvKeys_Line[i].endPointX;
        mat_e.at<float>(i,1)=mvKeys_Line[i].endPointY;
    }

    // Undistort points
    mat_s=mat_s.reshape(2);
    mat_e=mat_e.reshape(2);
    cv::undistortPoints(mat_s,mat_s,mK,mDistCoef,cv::Mat(),mK);
    cv::undistortPoints(mat_e,mat_e,mK,mDistCoef,cv::Mat(),mK);
    mat_s=mat_s.reshape(1);
    mat_e=mat_e.reshape(1);

    // Fill undistorted keypoint vector
    mvKeysUn_Line.resize(N_l);
    for(int i=0; i<N_l; i++)
    { 
        mvKeysUn_Line[i].startPointX=mat_s.at<float>(i,0);
        mvKeysUn_Line[i].startPointY=mat_s.at<float>(i,1);
        mvKeysUn_Line[i].endPointX=mat_e.at<float>(i,0);
        mvKeysUn_Line[i].endPointY=mat_e.at<float>(i,1);
    }  
}

void Frame::ComputeImageBounds(const cv::Mat &imLeft)
{
    if(mDistCoef.at<float>(0)!=0.0)
    {
        cv::Mat mat(4,2,CV_32F);
        mat.at<float>(0,0)=0.0; mat.at<float>(0,1)=0.0;
        mat.at<float>(1,0)=imLeft.cols; mat.at<float>(1,1)=0.0;
        mat.at<float>(2,0)=0.0; mat.at<float>(2,1)=imLeft.rows;
        mat.at<float>(3,0)=imLeft.cols; mat.at<float>(3,1)=imLeft.rows;

        mat=mat.reshape(2);
        cv::undistortPoints(mat,mat,static_cast<Pinhole*>(mpCamera)->toK(),mDistCoef,cv::Mat(),mK);
        mat=mat.reshape(1);

        // Undistort corners
        mnMinX = min(mat.at<float>(0,0),mat.at<float>(2,0));
        mnMaxX = max(mat.at<float>(1,0),mat.at<float>(3,0));
        mnMinY = min(mat.at<float>(0,1),mat.at<float>(1,1));
        mnMaxY = max(mat.at<float>(2,1),mat.at<float>(3,1));
    }
    else
    {
        mnMinX = 0.0f;
        mnMaxX = imLeft.cols;
        mnMinY = 0.0f;
        mnMaxY = imLeft.rows;
    }
}

void Frame::ComputeStereoMatches()
{
    mvuRight = vector<float>(N,-1.0f);
    mvDepth = vector<float>(N,-1.0f);

    const int thOrbDist = (ORBmatcher::TH_HIGH+ORBmatcher::TH_LOW)/2;

    const int nRows = mpORBextractorLeft->mvImagePyramid[0].rows;

    //Assign keypoints to row table
    vector<vector<size_t> > vRowIndices(nRows,vector<size_t>());

    for(int i=0; i<nRows; i++)
        vRowIndices[i].reserve(200);

    const int Nr = mvKeysRight.size();

    for(int iR=0; iR<Nr; iR++)
    {
        const cv::KeyPoint &kp = mvKeysRight[iR];
        const float &kpY = kp.pt.y;
        const float r = 2.0f*mvScaleFactors[mvKeysRight[iR].octave];
        const int maxr = ceil(kpY+r);
        const int minr = floor(kpY-r);

        for(int yi=minr;yi<=maxr;yi++)
            vRowIndices[yi].push_back(iR);
    }

    // Set limits for search
    const float minZ = mb;
    const float minD = 0;
    const float maxD = mbf/minZ;

    // For each left keypoint search a match in the right image
    vector<pair<int, int> > vDistIdx;
    vDistIdx.reserve(N);

    for(int iL=0; iL<N; iL++)
    {
        const cv::KeyPoint &kpL = mvKeys[iL];
        const int &levelL = kpL.octave;
        const float &vL = kpL.pt.y;
        const float &uL = kpL.pt.x;

        const vector<size_t> &vCandidates = vRowIndices[vL];

        if(vCandidates.empty())
            continue;

        const float minU = uL-maxD;
        const float maxU = uL-minD;

        if(maxU<0)
            continue;

        int bestDist = ORBmatcher::TH_HIGH;
        size_t bestIdxR = 0;

        const cv::Mat &dL = mDescriptors.row(iL);

        // Compare descriptor to right keypoints
        for(size_t iC=0; iC<vCandidates.size(); iC++)
        {
            const size_t iR = vCandidates[iC];
            const cv::KeyPoint &kpR = mvKeysRight[iR];

            if(kpR.octave<levelL-1 || kpR.octave>levelL+1)
                continue;

            const float &uR = kpR.pt.x;

            if(uR>=minU && uR<=maxU)
            {
                const cv::Mat &dR = mDescriptorsRight.row(iR);
                const int dist = ORBmatcher::DescriptorDistance(dL,dR);

                if(dist<bestDist)
                {
                    bestDist = dist;
                    bestIdxR = iR;
                }
            }
        }

        // Subpixel match by correlation
        if(bestDist<thOrbDist)
        {
            // coordinates in image pyramid at keypoint scale
            const float uR0 = mvKeysRight[bestIdxR].pt.x;
            const float scaleFactor = mvInvScaleFactors[kpL.octave];
            const float scaleduL = round(kpL.pt.x*scaleFactor);
            const float scaledvL = round(kpL.pt.y*scaleFactor);
            const float scaleduR0 = round(uR0*scaleFactor);

            // sliding window search
            const int w = 5;
            cv::Mat IL = mpORBextractorLeft->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduL-w,scaleduL+w+1);
            //IL.convertTo(IL,CV_32F);
            //IL = IL - IL.at<float>(w,w) *cv::Mat::ones(IL.rows,IL.cols,CV_32F);
            IL.convertTo(IL,CV_16S);
            IL = IL - IL.at<short>(w,w);

            int bestDist = INT_MAX;
            int bestincR = 0;
            const int L = 5;
            vector<float> vDists;
            vDists.resize(2*L+1);

            const float iniu = scaleduR0+L-w;
            const float endu = scaleduR0+L+w+1;
            if(iniu<0 || endu >= mpORBextractorRight->mvImagePyramid[kpL.octave].cols)
                continue;

            for(int incR=-L; incR<=+L; incR++)
            {
                cv::Mat IR = mpORBextractorRight->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduR0+incR-w,scaleduR0+incR+w+1);
                //IR.convertTo(IR,CV_32F);
                //IR = IR - IR.at<float>(w,w) *cv::Mat::ones(IR.rows,IR.cols,CV_32F);
                IR.convertTo(IR,CV_16S);
                IR = IR - IR.at<short>(w,w);

                float dist = cv::norm(IL,IR,cv::NORM_L1);
                if(dist<bestDist)
                {
                    bestDist =  dist;
                    bestincR = incR;
                }

                vDists[L+incR] = dist;
            }

            if(bestincR==-L || bestincR==L)
                continue;

            // Sub-pixel match (Parabola fitting)
            const float dist1 = vDists[L+bestincR-1];
            const float dist2 = vDists[L+bestincR];
            const float dist3 = vDists[L+bestincR+1];

            const float deltaR = (dist1-dist3)/(2.0f*(dist1+dist3-2.0f*dist2));

            if(deltaR<-1 || deltaR>1)
                continue;

            // Re-scaled coordinate
            float bestuR = mvScaleFactors[kpL.octave]*((float)scaleduR0+(float)bestincR+deltaR);

            float disparity = (uL-bestuR);

            if(disparity>=minD && disparity<maxD)
            {
                if(disparity<=0)
                {
                    disparity=0.01;
                    bestuR = uL-0.01;
                }
                mvDepth[iL]=mbf/disparity;
                mvuRight[iL] = bestuR;
                vDistIdx.push_back(pair<int,int>(bestDist,iL));
            }
        }
    }

    sort(vDistIdx.begin(),vDistIdx.end());
    const float median = vDistIdx[vDistIdx.size()/2].first;
    const float thDist = 1.5f*1.4f*median;

    for(int i=vDistIdx.size()-1;i>=0;i--)
    {
        if(vDistIdx[i].first<thDist)
            break;
        else
        {
            mvuRight[vDistIdx[i].second]=-1;
            mvDepth[vDistIdx[i].second]=-1;
        }
    }
}

void Frame::ComputeStereoMatches_Lines()
{
    // Depth, Disparity and the 3D vector that expresses the observed inﬁnite line in the image plane
    mvDepth_l.resize(N_l,pair<float,float>(-1.0f,-1.0f));
    mvDisparity_l.clear();
    mvle_l.clear();
    mvDisparity_l.resize(mvKeys_Line.size(),pair<float,float>(-1,-1));
    mvle_l.resize(mvKeys_Line.size(),Vector3d(0,0,0));

    // Line segments stereo matching
    // --------------------------------------------------------------------------------------------------------------------
    if (mvKeys_Line.empty() || mvKeysRight_Line.empty())
        return;

    std::vector<line_2d> coords;   // line_2d type definition in LineMatcher.h
    coords.reserve(mvKeys_Line.size());
    for (const KeyLine &kl : mvKeys_Line)
        coords.push_back(std::make_pair(std::make_pair(kl.startPointX * inv_width, kl.startPointY * inv_height),
                                        std::make_pair(kl.endPointX * inv_width, kl.endPointY * inv_height))); 

    //Fill in grid & directions
    list<pair<int, int>> line_coords;
    GridStructure grid(FRAME_GRID_ROWS, FRAME_GRID_COLS);
    std::vector<std::pair<double, double>> directions(mvKeysRight_Line.size());
    for (unsigned int idx = 0; idx < mvKeysRight_Line.size(); ++idx) {
        const KeyLine &kl = mvKeysRight_Line[idx];

        std::pair<double, double> &v = directions[idx];
        v = std::make_pair((kl.endPointX - kl.startPointX) * inv_width, (kl.endPointY - kl.startPointY) * inv_height);
        normalize(v);

       getLineCoords(kl.startPointX * inv_width, kl.startPointY * inv_height, kl.endPointX * inv_width, kl.endPointY * inv_height, line_coords);
        for (const std::pair<int, int> &p : line_coords)
            grid.at(p.first, p.second).push_back(idx);
    }

    GridWindow w;
    int size_width = 7;
    int size_height = 2; // You can increase this value for non rectified images
    w.width = std::make_pair(size_width, 0);
    w.height = std::make_pair(size_height, size_height);

    std::vector<int> matches_12;
    LineMatcher::matchGrid(coords, mDescriptors_Line, grid, mDescriptorsRight_Line, directions, w, matches_12);

    // bucle around lmatches
    Mat mDescriptors_Line_aux;
    for (unsigned int i1 = 0; i1 < matches_12.size(); ++i1) {
        const int i2 = matches_12[i1];
        if (i2 < 0) continue;

        // estimate the disparity of the endpoints
        Vector3d sp_l; sp_l << mvKeys_Line[i1].startPointX, mvKeys_Line[i1].startPointY, 1.0;
        Vector3d ep_l; ep_l << mvKeys_Line[i1].endPointX,   mvKeys_Line[i1].endPointY,   1.0;
        Vector3d sp_r; sp_r << mvKeysRight_Line[i2].startPointX, mvKeysRight_Line[i2].startPointY, 1.0;
        Vector3d ep_r; ep_r << mvKeysRight_Line[i2].endPointX,   mvKeysRight_Line[i2].endPointY,   1.0;
        Vector3d le_r; le_r << sp_r.cross(ep_r);

        double overlap = lineSegmentOverlapStereo( sp_l(1), ep_l(1), sp_r(1), ep_r(1) );

        double disp_s, disp_e;
        sp_r << ( sp_r(0)*( sp_l(1) - ep_r(1) ) + ep_r(0)*( sp_r(1) - sp_l(1) ) ) / ( sp_r(1)-ep_r(1) ) , sp_l(1) ,  1.0;
        ep_r << ( sp_r(0)*( ep_l(1) - ep_r(1) ) + ep_r(0)*( sp_r(1) - ep_l(1) ) ) / ( sp_r(1)-ep_r(1) ) , ep_l(1) ,  1.0;
        filterLineSegmentDisparity( sp_l.head(2), ep_l.head(2), sp_r.head(2), ep_r.head(2), disp_s, disp_e );

        // check minimal disparity
        int minDisp = 1;
        float lineHorizTh = 0.1;
        float stereoOverlapTh = 0.75;
        if( disp_s >= minDisp && disp_e >= minDisp
            && std::abs( sp_l(1)-ep_l(1) ) > lineHorizTh
            && std::abs( sp_r(1)-ep_r(1) ) > lineHorizTh
            && overlap > stereoOverlapTh )
        {
            mvDisparity_l[i1] = make_pair(disp_s,disp_e);
            mvDepth_l[i1] = pair<float,float>(mbf/float(disp_s), mbf/float(disp_e));
        }
    }
    for (int i=0; i < N_l; i++) {
        Vector3d sp_lun; sp_lun << mvKeysUn_Line[i].startPointX, mvKeysUn_Line[i].startPointY, 1.0;
        Vector3d ep_lun; ep_lun << mvKeysUn_Line[i].endPointX,   mvKeysUn_Line[i].endPointY,   1.0;
        Vector3d le_l; le_l << sp_lun.cross(ep_lun); le_l = le_l / std::sqrt( le_l(0)*le_l(0) + le_l(1)*le_l(1) );
        mvle_l[i] = le_l;
    }
}

double Frame::lineSegmentOverlapStereo(double spl_obs, double epl_obs, double spl_proj, double epl_proj)
{

    double overlap = 1.f;
    float lineHorizTh = 0.1;

    if( fabs( epl_obs - spl_obs ) > lineHorizTh ) // normal lines (verticals included)
    {
        double sln    = min(spl_obs,  epl_obs);
        double eln    = max(spl_obs,  epl_obs);
        double spn    = min(spl_proj, epl_proj);
        double epn    = max(spl_proj, epl_proj);

        double length = eln-spn;

        if ( (epn < sln) || (spn > eln) )
            overlap = 0.f;
        else{
            if ( (epn>eln) && (spn<sln) )
                overlap = eln-sln;
            else
                overlap = min(eln,epn) - max(sln,spn);
        }

        if(length>0.01f)
            overlap = overlap / length;
        else
            overlap = 0.f;

        if( overlap > 1.f )
            overlap = 1.f;

    }

    return overlap;
}

void Frame::filterLineSegmentDisparity( Vector2d spl, Vector2d epl, Vector2d spr, Vector2d epr, double &disp_s, double &disp_e )
{
    disp_s = spl(0) - spr(0);
    disp_e = epl(0) - epr(0);
    // if they are too different, ignore them
    float lsMinDispRatio = 0.7;
    if(  min( disp_s, disp_e ) / max( disp_s, disp_e ) < lsMinDispRatio )
    {
        disp_s = -1.0;
        disp_e = -1.0;
    }
}


void Frame::ComputeStereoFromRGBD(const cv::Mat &imDepth)
{
    mvuRight = vector<float>(N,-1);
    mvDepth = vector<float>(N,-1);

    for(int i=0; i<N; i++)
    {
        const cv::KeyPoint &kp = mvKeys[i];
        const cv::KeyPoint &kpU = mvKeysUn[i];

        const float &v = kp.pt.y;
        const float &u = kp.pt.x;

        const float d = imDepth.at<float>(v,u);

        if(d>0)
        {
            mvDepth[i] = d;
            mvuRight[i] = kpU.pt.x-mbf/d;
        }
    }
}

cv::Mat Frame::UnprojectStereo(const int &i)
{
    const float z = mvDepth[i];
    if(z>0)
    {
        const float u = mvKeysUn[i].pt.x;
        const float v = mvKeysUn[i].pt.y;
        const float x = (u-cx)*z*invfx;
        const float y = (v-cy)*z*invfy;
        cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);
        return mRwc*x3Dc+mOw;
    }
    else
        return cv::Mat();
}

Eigen::Vector3d Frame::UnprojectStereoLines(const double &u, const double &v, const double &z)
{
    // convert the point in image to the point in world coordinate.
    Eigen::Vector3d P;      // point in camera coordinate
    P(0) = (u-cx)*z*invfx;
    P(1) = (v-cy)*z*invfy;
    P(2) = z;
    return Converter::toMatrix3d(mRwc)*P+Converter::toVector3d(mOw);
}

Eigen::Vector2d Frame::projection(const Eigen::Vector3d &P )
{
    // convert the 3Dpoint in camera coordinate to the point in image.
    Vector2d uv_unit;
    uv_unit(0) = cx + fx * P(0) / P(2);
    uv_unit(1) = cy + fy * P(1) / P(2);
    return uv_unit;
}

bool Frame::imuIsPreintegrated()
{
    unique_lock<std::mutex> lock(*mpMutexImu);
    return mbImuPreintegrated;
}

void Frame::setIntegrated()
{
    unique_lock<std::mutex> lock(*mpMutexImu);
    mbImuPreintegrated = true;
}

Frame::Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor* extractorLeft, ORBextractor* extractorRight, ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth, GeometricCamera* pCamera, GeometricCamera* pCamera2, cv::Mat& Tlr,Frame* pPrevF, const IMU::Calib &ImuCalib)
        :mpcpi(NULL), mpORBvocabulary(voc),mpORBextractorLeft(extractorLeft),mpORBextractorRight(extractorRight), mTimeStamp(timeStamp), mK(K.clone()), mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth),
         mImuCalib(ImuCalib), mpImuPreintegrated(NULL), mpPrevFrame(pPrevF),mpImuPreintegratedFrame(NULL), mpReferenceKF(static_cast<KeyFrame*>(NULL)), mbImuPreintegrated(false), mpCamera(pCamera), mpCamera2(pCamera2), mTlr(Tlr)
{
    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    imgLeft = imLeft.clone();
    imgRight = imRight.clone();
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    // Frame ID
    mnId=nNextId++;

    // Scale Level Info
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // ORB extraction
    thread threadLeft(&Frame::ExtractORB,this,0,imLeft,static_cast<KannalaBrandt8*>(mpCamera)->mvLappingArea[0],static_cast<KannalaBrandt8*>(mpCamera)->mvLappingArea[1]);
    thread threadRight(&Frame::ExtractORB,this,1,imRight,static_cast<KannalaBrandt8*>(mpCamera2)->mvLappingArea[0],static_cast<KannalaBrandt8*>(mpCamera2)->mvLappingArea[1]);
    threadLeft.join();
    threadRight.join();
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

    Nleft = mvKeys.size();
    Nright = mvKeysRight.size();
    N = Nleft + Nright;

    if(N == 0)
        return;

    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        ComputeImageBounds(imLeft);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/(mnMaxY-mnMinY);

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    mb = mbf / fx;

    mRlr = mTlr.rowRange(0,3).colRange(0,3);
    mtlr = mTlr.col(3);

    cv::Mat Rrl = mTlr.rowRange(0,3).colRange(0,3).t();
    cv::Mat trl = Rrl * (-1 * mTlr.col(3));

    cv::hconcat(Rrl,trl,mTrl);

    ComputeStereoFishEyeMatches();
    std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();

    //Put all descriptors in the same matrix
    cv::vconcat(mDescriptors,mDescriptorsRight,mDescriptors);

    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(nullptr));
    mvbOutlier = vector<bool>(N,false);

    AssignFeaturesToGrid();
    std::chrono::steady_clock::time_point t4 = std::chrono::steady_clock::now();

    mpMutexImu = new std::mutex();

    UndistortKeyPoints();
    std::chrono::steady_clock::time_point t5 = std::chrono::steady_clock::now();

    double t_read = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t1 - t0).count();
    double t_orbextract = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t2 - t1).count();
    double t_stereomatches = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t3 - t2).count();
    double t_assign = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t4 - t3).count();
    double t_undistort = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t5 - t4).count();

    /*cout << "Reading time: " << t_read << endl;
    cout << "Extraction time: " << t_orbextract << endl;
    cout << "Matching time: " << t_stereomatches << endl;
    cout << "Assignment time: " << t_assign << endl;
    cout << "Undistortion time: " << t_undistort << endl;*/

}

void Frame::ComputeStereoFishEyeMatches() {
    //Speed it up by matching keypoints in the lapping area
    vector<cv::KeyPoint> stereoLeft(mvKeys.begin() + monoLeft, mvKeys.end());
    vector<cv::KeyPoint> stereoRight(mvKeysRight.begin() + monoRight, mvKeysRight.end());

    cv::Mat stereoDescLeft = mDescriptors.rowRange(monoLeft, mDescriptors.rows);
    cv::Mat stereoDescRight = mDescriptorsRight.rowRange(monoRight, mDescriptorsRight.rows);

    mvLeftToRightMatch = vector<int>(Nleft,-1);
    mvRightToLeftMatch = vector<int>(Nright,-1);
    mvDepth = vector<float>(Nleft,-1.0f);
    mvuRight = vector<float>(Nleft,-1);
    mvStereo3Dpoints = vector<cv::Mat>(Nleft);
    mnCloseMPs = 0;

    //Perform a brute force between Keypoint in the left and right image
    vector<vector<cv::DMatch>> matches;

    BFmatcher.knnMatch(stereoDescLeft,stereoDescRight,matches,2);

    int nMatches = 0;
    int descMatches = 0;

    //Check matches using Lowe's ratio
    for(vector<vector<cv::DMatch>>::iterator it = matches.begin(); it != matches.end(); ++it){
        if((*it).size() >= 2 && (*it)[0].distance < (*it)[1].distance * 0.7){
            //For every good match, check parallax and reprojection error to discard spurious matches
            cv::Mat p3D;
            descMatches++;
            float sigma1 = mvLevelSigma2[mvKeys[(*it)[0].queryIdx + monoLeft].octave], sigma2 = mvLevelSigma2[mvKeysRight[(*it)[0].trainIdx + monoRight].octave];
            float depth = static_cast<KannalaBrandt8*>(mpCamera)->TriangulateMatches(mpCamera2,mvKeys[(*it)[0].queryIdx + monoLeft],mvKeysRight[(*it)[0].trainIdx + monoRight],mRlr,mtlr,sigma1,sigma2,p3D);
            if(depth > 0.0001f){
                mvLeftToRightMatch[(*it)[0].queryIdx + monoLeft] = (*it)[0].trainIdx + monoRight;
                mvRightToLeftMatch[(*it)[0].trainIdx + monoRight] = (*it)[0].queryIdx + monoLeft;
                mvStereo3Dpoints[(*it)[0].queryIdx + monoLeft] = p3D.clone();
                mvDepth[(*it)[0].queryIdx + monoLeft] = depth;
                nMatches++;
            }
        }
    }
}

bool Frame::isInFrustumChecks(MapPoint *pMP, float viewingCosLimit, bool bRight) {
    // 3D in absolute coordinates
    cv::Mat P = pMP->GetWorldPos();

    cv::Mat mR, mt, twc;
    if(bRight){
        cv::Mat Rrl = mTrl.colRange(0,3).rowRange(0,3);
        cv::Mat trl = mTrl.col(3);
        mR = Rrl * mRcw;
        mt = Rrl * mtcw + trl;
        twc = mRwc * mTlr.rowRange(0,3).col(3) + mOw;
    }
    else{
        mR = mRcw;
        mt = mtcw;
        twc = mOw;
    }

    // 3D in camera coordinates
    cv::Mat Pc = mR*P+mt;
    const float Pc_dist = cv::norm(Pc);
    const float &PcZ = Pc.at<float>(2);

    // Check positive depth
    if(PcZ<0.0f)
        return false;

    // Project in image and check it is not outside
    cv::Point2f uv;
    if(bRight) uv = mpCamera2->project(Pc);
    else uv = mpCamera->project(Pc);

    if(uv.x<mnMinX || uv.x>mnMaxX)
        return false;
    if(uv.y<mnMinY || uv.y>mnMaxY)
        return false;

    // Check distance is in the scale invariance region of the MapPoint
    const float maxDistance = pMP->GetMaxDistanceInvariance();
    const float minDistance = pMP->GetMinDistanceInvariance();
    const cv::Mat PO = P-twc;
    const float dist = cv::norm(PO);

    if(dist<minDistance || dist>maxDistance)
        return false;

    // Check viewing angle
    cv::Mat Pn = pMP->GetNormal();

    const float viewCos = PO.dot(Pn)/dist;

    if(viewCos<viewingCosLimit)
        return false;

    // Predict scale in the image
    const int nPredictedLevel = pMP->PredictScale(dist,this);

    if(bRight){
        pMP->mTrackProjXR = uv.x;
        pMP->mTrackProjYR = uv.y;
        pMP->mnTrackScaleLevelR= nPredictedLevel;
        pMP->mTrackViewCosR = viewCos;
        pMP->mTrackDepthR = Pc_dist;
    }
    else{
        pMP->mTrackProjX = uv.x;
        pMP->mTrackProjY = uv.y;
        pMP->mnTrackScaleLevel= nPredictedLevel;
        pMP->mTrackViewCos = viewCos;
        pMP->mTrackDepth = Pc_dist;
    }

    return true;
}

cv::Mat Frame::UnprojectStereoFishEye(const int &i){
    return mRwc*mvStereo3Dpoints[i]+mOw;
}

} //namespace ORB_SLAM
