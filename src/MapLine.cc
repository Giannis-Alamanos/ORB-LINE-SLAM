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

#include "MapLine.h"
#include "ORBmatcher.h"
#include "Converter.h"

#include<mutex>

namespace ORB_SLAM3
{

long unsigned int MapLine::nNextId=0;

mutex MapLine::mGlobalMutex;

MapLine::MapLine(const Eigen::Vector3d &sP, const Eigen::Vector3d &eP, Map* pMap):
    mnFirstKFid(-1), mnFirstFrame(0), nObs(0), mnTrackReferenceForFrame(0),mnLastFrameSeen(0), mnCorrectedByKF(0), mnCorrectedReference(0),
    mpRefKF(static_cast<KeyFrame*>(NULL)), mnVisible(1), mnFound(1), mnBALocalForKF(0),
    mbBad(false), mpReplaced(static_cast<MapLine*>(NULL)), mpMap(pMap)
{
    mWorldPos_sP = sP;
    mWorldPos_eP = eP;
    mNormalVector = cv::Mat::zeros(3,1,CV_32F);

    // MapLines can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexLineCreation);
}

MapLine::MapLine(const Eigen::Vector3d &sP, const Eigen::Vector3d &eP, KeyFrame* pRefKF, Map* pMap):
    mnFirstKFid(pRefKF->mnId), mnFirstFrame(pRefKF->mnFrameId), nObs(0), mnTrackReferenceForFrame(0),mnLastFrameSeen(0), mnCorrectedByKF(0), mnCorrectedReference(0),
    mpRefKF(pRefKF), mnVisible(1), mnFound(1), mnBALocalForKF(0),
    mbBad(false), mpReplaced(static_cast<MapLine*>(NULL)), mpMap(pMap)
{
    mWorldPos_sP = sP;
    mWorldPos_eP = eP;
    mNormalVector = cv::Mat::zeros(3,1,CV_32F);

    // MapLines can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexLineCreation);
    mnId=nNextId++;
}

MapLine::MapLine(const Eigen::Vector3d &sP, const Eigen::Vector3d &eP,  Map* pMap, Frame* pFrame, const int &idxF):
    mnFirstKFid(-1), mnFirstFrame(pFrame->mnId), nObs(0), mnTrackReferenceForFrame(0),mnLastFrameSeen(0), mnCorrectedByKF(0), mnCorrectedReference(0),
    mpRefKF(static_cast<KeyFrame*>(NULL)), mnVisible(1), mnFound(1), mnBALocalForKF(0),
    mbBad(false), mpReplaced(static_cast<MapLine*>(NULL)), mpMap(pMap)
{
    mWorldPos_sP = sP;
    mWorldPos_eP = eP;
    Eigen::Vector3d MidPoint = (sP+eP)/2;
    cv::Mat Ow = pFrame->GetCameraCenter();
    cv::Mat mWorldPosMid = Converter::toCvMat(MidPoint);
    mNormalVector = mWorldPosMid - Ow;
    mNormalVector = mNormalVector/cv::norm(mNormalVector);

    cv::Mat PC = mWorldPosMid - Ow;
    const float dist = cv::norm(PC);
    const int level = pFrame->mvKeysUn_Line[idxF].octave;
    const float levelScaleFactor =  pFrame->mvScaleFactors_l[level];
    const int nLevels = pFrame->mnScaleLevels_l;

    mfMaxDistance = dist*levelScaleFactor;
    mfMinDistance = mfMaxDistance/pFrame->mvScaleFactors_l[nLevels-1]; 

    pFrame->mDescriptors_Line.row(idxF).copyTo(mDescriptor);

    // MapLines can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexLineCreation);
    mnId=nNextId++;
}

void MapLine::SetWorldPos(const Eigen::Vector3d &sP, const Eigen::Vector3d &eP)
{
    unique_lock<mutex> lock2(mGlobalMutex);
    unique_lock<mutex> lock(mMutexPos);
    mWorldPos_sP = sP;
    mWorldPos_eP = eP;
}

Vector6d MapLine::GetWorldPos()
{
    unique_lock<mutex> lock(mMutexPos);
    Vector6d sep;
    sep.head(3) = mWorldPos_sP;
    sep.tail(3) = mWorldPos_eP;
    return sep;
}

Map* MapLine::GetMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mpMap;
}

KeyFrame* MapLine::GetReferenceKeyFrame()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mpRefKF;
}

void MapLine::AddObservation(KeyFrame* pKF, size_t idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    if(mObservations.count(pKF))
        return;
    mObservations[pKF]=idx;

    if(pKF->mvDepth_l[idx].first>=0 && pKF->mvDepth_l[idx].second>=0)
        nObs+=2;
    else
        nObs++;
}

void MapLine::EraseObservation(KeyFrame* pKF)
{
    bool bBad=false;
    {
        unique_lock<mutex> lock(mMutexFeatures);
        if(mObservations.count(pKF))
        {
            int idx = mObservations[pKF];
            if(pKF->mvDepth_l[idx].first>=0 && pKF->mvDepth_l[idx].second>=0)
                nObs-=2;
            else
                nObs--;

            mObservations.erase(pKF);

            if(mpRefKF==pKF)
                mpRefKF=mObservations.begin()->first;

            // If only 2 observations or less, discard point
            if(nObs<=2)
                bBad=true;
        }
    }

    if(bBad)
        SetBadFlag();
}

map<KeyFrame*, size_t> MapLine::GetObservations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mObservations;
} 

int MapLine::Observations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return nObs;
}

void MapLine::SetBadFlag()
{
    map<KeyFrame*,size_t> obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        mbBad=true;
        obs = mObservations;
        mObservations.clear();
    }
    for(map<KeyFrame*,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;
        pKF->EraseMapLineMatch(mit->second);
    }

    mpMap->EraseMapLine(this);
}

MapLine* MapLine::GetReplaced()
{
    unique_lock<mutex> lock1(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    return mpReplaced;
}

void MapLine::Replace(MapLine* pML)
{
    if(pML->mnId==this->mnId)
        return;

    int nvisible, nfound;
    map<KeyFrame*,size_t> obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        obs=mObservations;
        mObservations.clear();
        mbBad=true;
        nvisible = mnVisible;
        nfound = mnFound;
        mpReplaced = pML;
    }

    for(map<KeyFrame*,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        // Replace measurement in keyframe
        KeyFrame* pKF = mit->first;

        if(!pML->IsInKeyFrame(pKF))
        {
            pKF->ReplaceMapLineMatch(mit->second, pML);
            pML->AddObservation(pKF,mit->second);
        }
        else
        {
            pKF->EraseMapLineMatch(mit->second);
        }
    }
    pML->IncreaseFound(nfound);
    pML->IncreaseVisible(nvisible);
    pML->ComputeDistinctiveDescriptors();

    mpMap->EraseMapLine(this);
}

bool MapLine::isBad()
{
    unique_lock<mutex> lock1(mMutexFeatures,std::defer_lock);
    unique_lock<mutex> lock2(mMutexPos,std::defer_lock);
    lock(lock1, lock2);

    return mbBad;
}

void MapLine::IncreaseVisible(int n)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mnVisible+=n;
}

void MapLine::IncreaseFound(int n)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mnFound+=n;
}

float MapLine::GetFoundRatio()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return static_cast<float>(mnFound)/mnVisible;
}

void MapLine::ComputeDistinctiveDescriptors()
{
    // Retrieve all observed descriptors
    vector<cv::Mat> vDescriptors;

    map<KeyFrame*,size_t> observations;

    {
        unique_lock<mutex> lock1(mMutexFeatures);
        if(mbBad)
            return;
        observations=mObservations;
    }

    if(observations.empty())
        return;

    vDescriptors.reserve(observations.size());

    for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;

        if(!pKF->isBad())
            vDescriptors.push_back(pKF->mDescriptors_l.row(mit->second));
    }

    if(vDescriptors.empty())
        return;

    // Compute distances between them
    const size_t N = vDescriptors.size();

    float Distances[N][N];
    for(size_t i=0;i<N;i++)
    {
        Distances[i][i]=0;
        for(size_t j=i+1;j<N;j++)
        {
            int distij = ORBmatcher::DescriptorDistance(vDescriptors[i],vDescriptors[j]);
            Distances[i][j]=distij;
            Distances[j][i]=distij;
        }
    }

    // Take the descriptor with least median distance to the rest
    int BestMedian = INT_MAX;
    int BestIdx = 0;
    for(size_t i=0;i<N;i++)
    {
        vector<int> vDists(Distances[i],Distances[i]+N);
        sort(vDists.begin(),vDists.end());
        int median = vDists[0.5*(N-1)];

        if(median<BestMedian)
        {
            BestMedian = median;
            BestIdx = i;
        }
    }

    {
        unique_lock<mutex> lock(mMutexFeatures);
        mDescriptor = vDescriptors[BestIdx].clone();
    }
}

cv::Mat MapLine::GetDescriptor()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mDescriptor.clone();
}

void MapLine::UpdateNormalAndDepth()
{
    map<KeyFrame*,size_t> observations;
    KeyFrame* pRefKF;
    cv::Mat Pos;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        if(mbBad)
            return;
        observations=mObservations;
        pRefKF=mpRefKF;
        Eigen::Vector3d MidPoint = (mWorldPos_sP+mWorldPos_eP)/2;
        Pos = Converter::toCvMat(MidPoint);
    }

    if(observations.empty())
        return;

    cv::Mat normal = cv::Mat::zeros(3,1,CV_32F);
    int n=0;
    for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;
        cv::Mat Owi = pKF->GetCameraCenter();
        cv::Mat normali = Pos - Owi;
        normal = normal + normali/cv::norm(normali);
        n++;
    }

    cv::Mat PC = Pos - pRefKF->GetCameraCenter();
    const float dist = cv::norm(PC);
    const int level = pRefKF->mvKeysUn_Line[observations[pRefKF]].octave;
    const float levelScaleFactor =  pRefKF->mvScaleFactors_l[level];
    const int nLevels = pRefKF->mnScaleLevels_l;

    {
        unique_lock<mutex> lock3(mMutexPos);
        mfMaxDistance = dist*levelScaleFactor;
        mfMinDistance = mfMaxDistance/pRefKF->mvScaleFactors_l[nLevels-1];
        mNormalVector = normal/n;
    }
}

float MapLine::GetMinDistanceInvariance()
{
    unique_lock<mutex> lock(mMutexPos);
    return 0.8f*mfMinDistance;
}

float MapLine::GetMaxDistanceInvariance()
{
    unique_lock<mutex> lock(mMutexPos);
    return 1.2f*mfMaxDistance;
}

cv::Mat MapLine::GetNormal()
{
    unique_lock<mutex> lock(mMutexPos);
    return mNormalVector.clone();
}

int MapLine::GetIndexInKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexFeatures);
    if(mObservations.count(pKF))
        return mObservations[pKF];
    else
        return -1;
}

bool MapLine::IsInKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexFeatures);
    return (mObservations.count(pKF));
}

KeyFrame* MapLine::SetReferenceKeyFrame(KeyFrame* RFKF)
{
    return mpRefKF = RFKF;
}

void MapLine::UpdateMap(Map* pMap)
{
    unique_lock<mutex> lock(mMutexMap);
    mpMap = pMap;
}

void MapLine::SetNormalVector(cv::Mat& normal)
{
    unique_lock<mutex> lock3(mMutexPos);
    mNormalVector = normal;
}

} //namespace ORB_SLAM
