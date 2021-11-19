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

#ifndef ORB_SLAM3_OPTIMIZABLETYPES_H
#define ORB_SLAM3_OPTIMIZABLETYPES_H

#include "Thirdparty/g2o/g2o/core/base_unary_edge.h"
#include <Thirdparty/g2o/g2o/types/types_six_dof_expmap.h>
#include <Thirdparty/g2o/g2o/types/sim3.h>

#include <Eigen/Geometry>
#include <include/CameraModels/GeometricCamera.h>


namespace ORB_SLAM3 {
class  EdgeSE3ProjectXYZOnlyPose: public  g2o::BaseUnaryEdge<2, Eigen::Vector2d, g2o::VertexSE3Expmap>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeSE3ProjectXYZOnlyPose(){}

    bool read(std::istream& is);

    bool write(std::ostream& os) const;

    void computeError()  {
        const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
        Eigen::Vector2d obs(_measurement);
        _error = obs-pCamera->project(v1->estimate().map(Xw));
    }

    bool isDepthPositive() {
        const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
        return (v1->estimate().map(Xw))(2)>0.0;
    }


    virtual void linearizeOplus();

    Eigen::Vector3d Xw;
    GeometricCamera* pCamera;
};

class  EdgeSE3ProjectXYZOnlyPoseToBody: public  g2o::BaseUnaryEdge<2, Eigen::Vector2d, g2o::VertexSE3Expmap>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeSE3ProjectXYZOnlyPoseToBody(){}

    bool read(std::istream& is);

    bool write(std::ostream& os) const;

    void computeError()  {
        const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
        Eigen::Vector2d obs(_measurement);
        _error = obs-pCamera->project((mTrl * v1->estimate()).map(Xw));
    }

    bool isDepthPositive() {
        const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
        return ((mTrl * v1->estimate()).map(Xw))(2)>0.0;
    }


    virtual void linearizeOplus();

    Eigen::Vector3d Xw;
    GeometricCamera* pCamera;

    g2o::SE3Quat mTrl;
};

class  EdgeSE3ProjectXYZ: public  g2o::BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSBAPointXYZ, g2o::VertexSE3Expmap>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeSE3ProjectXYZ();

    bool read(std::istream& is);

    bool write(std::ostream& os) const;

    void computeError()  {
        const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
        const g2o::VertexSBAPointXYZ* v2 = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);
        Eigen::Vector2d obs(_measurement);
        _error = obs-pCamera->project(v1->estimate().map(v2->estimate()));
    }

    bool isDepthPositive() {
        const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
        const g2o::VertexSBAPointXYZ* v2 = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);
        return ((v1->estimate().map(v2->estimate()))(2)>0.0);
    }

    virtual void linearizeOplus();

    GeometricCamera* pCamera;
};

class  EdgeSE3ProjectXYZToBody: public  g2o::BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSBAPointXYZ, g2o::VertexSE3Expmap>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeSE3ProjectXYZToBody();

    bool read(std::istream& is);

    bool write(std::ostream& os) const;

    void computeError()  {
        const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
        const g2o::VertexSBAPointXYZ* v2 = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);
        Eigen::Vector2d obs(_measurement);
        _error = obs-pCamera->project((mTrl * v1->estimate()).map(v2->estimate()));
    }

    bool isDepthPositive() {
        const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
        const g2o::VertexSBAPointXYZ* v2 = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);
        return ((mTrl * v1->estimate()).map(v2->estimate()))(2)>0.0;
    }

    virtual void linearizeOplus();

    GeometricCamera* pCamera;
    g2o::SE3Quat mTrl;
};

class VertexSim3Expmap : public g2o::BaseVertex<7, g2o::Sim3>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexSim3Expmap();
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    virtual void setToOriginImpl() {
        _estimate = g2o::Sim3();
    }

    virtual void oplusImpl(const double* update_)
    {
        Eigen::Map<g2o::Vector7d> update(const_cast<double*>(update_));

        if (_fix_scale)
            update[6] = 0;

        g2o::Sim3 s(update);
        setEstimate(s*estimate());
    }

    GeometricCamera* pCamera1, *pCamera2;

    bool _fix_scale;
};


class EdgeSim3ProjectXYZ : public  g2o::BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSBAPointXYZ, ORB_SLAM3::VertexSim3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeSim3ProjectXYZ();
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    void computeError()
    {
        const ORB_SLAM3::VertexSim3Expmap* v1 = static_cast<const ORB_SLAM3::VertexSim3Expmap*>(_vertices[1]);
        const g2o::VertexSBAPointXYZ* v2 = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);

        Eigen::Vector2d obs(_measurement);
        _error = obs-v1->pCamera1->project(v1->estimate().map(v2->estimate()));
    }

    // virtual void linearizeOplus();

};

class EdgeInverseSim3ProjectXYZ : public  g2o::BaseBinaryEdge<2, Eigen::Vector2d,  g2o::VertexSBAPointXYZ, VertexSim3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeInverseSim3ProjectXYZ();
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    void computeError()
    {
        const ORB_SLAM3::VertexSim3Expmap* v1 = static_cast<const ORB_SLAM3::VertexSim3Expmap*>(_vertices[1]);
        const g2o::VertexSBAPointXYZ* v2 = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);

        Eigen::Vector2d obs(_measurement);
        _error = obs-v1->pCamera2->project((v1->estimate().inverse().map(v2->estimate())));
    }

    // virtual void linearizeOplus();

};

class  EdgeLineSE3ProjectXYZOnlyPose: public  g2o::BaseUnaryEdge<2, Eigen::Vector3d, g2o::VertexSE3Expmap>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeLineSE3ProjectXYZOnlyPose(){}

    bool read(std::istream& is);

    bool write(std::ostream& os) const;

    void computeError()  {
        const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
        Eigen::Vector3d obs(_measurement);
        Eigen::Matrix<double,2,3> Matrix23d; 
        Matrix23d(0,0) = cam_project(v1->estimate().map(Xw_s))(0); 
        Matrix23d(0,1) = cam_project(v1->estimate().map(Xw_s))(1);
        Matrix23d(0,2) = 1.0;
        Matrix23d(1,0) = cam_project(v1->estimate().map(Xw_e))(0); 
        Matrix23d(1,1) = cam_project(v1->estimate().map(Xw_e))(1);
        Matrix23d(1,2) = 1.0;
        _error = Matrix23d * obs;


    }

    bool isDepthPositive() {
        const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
        return (v1->estimate().map(Xw_s))(2)>0.0 && (v1->estimate().map(Xw_e))(2)>0.0;
    }


    virtual void linearizeOplus();

    Eigen::Vector2d cam_project(const Eigen::Vector3d & trans_xyz) const;

    Eigen::Vector3d Xw_s;
    Eigen::Vector3d Xw_e;
    Eigen::Vector3d obs_temp;
    double fx, fy, cx, cy;
};

class  EdgeLineAngleSE3ProjectXYZOnlyPose: public  g2o::BaseUnaryEdge<2, Eigen::Matrix<double,4,1>, g2o::VertexSE3Expmap>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeLineAngleSE3ProjectXYZOnlyPose(){}

    bool read(std::istream& is);

    bool write(std::ostream& os) const;

    void computeError()  {
        const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
        Eigen::Matrix<double,4,1> obs(_measurement);
        Eigen::Vector2d P; 
        Eigen::Vector2d Q; 
        Eigen::Vector2d QP_pr; 
        Eigen::Vector2d QP; 
        Eigen::Vector2d PQ_pr; 

        P << obs(0), obs(1);
        Q << obs(2), obs(3);
        QP_pr = cam_project(v1->estimate().map(Xw_s)) - Q; 
        QP = P - Q;
        PQ_pr = cam_project(v1->estimate().map(Xw_e)) - P; 

       _error << (QP_pr(0)*QP(0)+QP_pr(1)*QP(1))/(QP_pr.norm()*QP.norm()) - 1, (PQ_pr(0)*QP(0)+PQ_pr(1)*QP(1))/(PQ_pr.norm()*QP.norm()) + 1;
    }

    bool isDepthPositive() {
        const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
        return (v1->estimate().map(Xw_s))(2)>0.0 && (v1->estimate().map(Xw_e))(2)>0.0;
    }


    virtual void linearizeOplus();

    Eigen::Vector2d cam_project(const Eigen::Vector3d & trans_xyz) const;

    Eigen::Vector3d Xw_s;
    Eigen::Vector3d Xw_e;
    Eigen::Matrix<double,4,1> obs_temp;
    double fx, fy, cx, cy;
};

class  EdgeLineWithAngleSE3ProjectXYZOnlyPose: public  g2o::BaseUnaryEdge<4, Eigen::Matrix<double,7,1>, g2o::VertexSE3Expmap>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeLineWithAngleSE3ProjectXYZOnlyPose(){}

    bool read(std::istream& is);

    bool write(std::ostream& os) const;

    void computeError()  {
        const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
        Eigen::Matrix<double,7,1> obs(_measurement);
        Eigen::Vector2d P; 
        Eigen::Vector2d Q; 
        Eigen::Vector2d QP_pr; 
        Eigen::Vector2d QP; 
        Eigen::Vector2d PQ_pr; 


        P << obs(0), obs(1);
        Q << obs(2), obs(3);
        QP_pr = cam_project(v1->estimate().map(Xw_s)) - Q; 
        QP = P - Q;
        PQ_pr = cam_project(v1->estimate().map(Xw_e)) - P; 

       _error << obs(4)*cam_project(v1->estimate().map(Xw_s))(0)+obs(5)*cam_project(v1->estimate().map(Xw_s))(1)+obs(6), obs(4)*cam_project(v1->estimate().map(Xw_e))(0)+obs(5)*cam_project(v1->estimate().map(Xw_e))(1)+obs(6), (QP_pr(0)*QP(0)+QP_pr(1)*QP(1))/(QP_pr.norm()*QP.norm()) - 1, (PQ_pr(0)*QP(0)+PQ_pr(1)*QP(1))/(PQ_pr.norm()*QP.norm()) + 1;
    }

    bool isDepthPositive() {
        const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
        return (v1->estimate().map(Xw_s))(2)>0.0 && (v1->estimate().map(Xw_e))(2)>0.0;
    }


    virtual void linearizeOplus();

    Eigen::Vector2d cam_project(const Eigen::Vector3d & trans_xyz) const;

    Eigen::Vector3d Xw_s;
    Eigen::Vector3d Xw_e;
    Eigen::Matrix<double,7,1> obs_temp;
    double fx, fy, cx, cy;
};

class  EdgeLineSE3ProjectXYZ: public  g2o::BaseBinaryEdge<2, Eigen::Vector3d, g2o::VertexSBALineXYZ, g2o::VertexSE3Expmap>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeLineSE3ProjectXYZ();

    bool read(std::istream& is);

    bool write(std::ostream& os) const;

    void computeError()  {
        const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
        const g2o::VertexSBALineXYZ* v2 = static_cast<const g2o::VertexSBALineXYZ*>(_vertices[0]);
        Eigen::Vector3d obs(_measurement);
        Eigen::Matrix<double,2,3> Matrix23d; 
        Matrix23d(0,0) = cam_project(v1->estimate().map(v2->estimate().head(3)))(0); 
        Matrix23d(0,1) = cam_project(v1->estimate().map(v2->estimate().head(3)))(1);
        Matrix23d(0,2) = 1.0;
        Matrix23d(1,0) = cam_project(v1->estimate().map(v2->estimate().tail(3)))(0); 
        Matrix23d(1,1) = cam_project(v1->estimate().map(v2->estimate().tail(3)))(1);
        Matrix23d(1,2) = 1.0;
        _error = Matrix23d * obs;


    }

    bool isDepthPositive() {
        const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
        const g2o::VertexSBALineXYZ* v2 = static_cast<const g2o::VertexSBALineXYZ*>(_vertices[0]);
        return (v1->estimate().map(v2->estimate().head(3)))(2)>0.0 && (v1->estimate().map(v2->estimate().tail(3)))(2)>0.0;
    }


    virtual void linearizeOplus();

    Eigen::Vector2d cam_project(const Eigen::Vector3d & trans_xyz) const;

    Eigen::Vector3d obs_temp;
    double fx, fy, cx, cy;
};

class  EdgeLineAngleSE3ProjectXYZ: public  g2o::BaseBinaryEdge<2, Eigen::Matrix<double,4,1>, g2o::VertexSBALineXYZ, g2o::VertexSE3Expmap>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeLineAngleSE3ProjectXYZ();

    bool read(std::istream& is);

    bool write(std::ostream& os) const;

    void computeError()  {
        const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
        const g2o::VertexSBALineXYZ* v2 = static_cast<const g2o::VertexSBALineXYZ*>(_vertices[0]);
        Eigen::Matrix<double,4,1> obs(_measurement);
        Eigen::Vector2d P; 
        Eigen::Vector2d Q; 
        Eigen::Vector2d QP_pr; 
        Eigen::Vector2d QP; 
        Eigen::Vector2d PQ_pr; 


        P << obs(0), obs(1);
        Q << obs(2), obs(3);
        QP_pr = cam_project(v1->estimate().map(v2->estimate().head(3))) - Q; 
        QP = P - Q;
        PQ_pr = cam_project(v1->estimate().map(v2->estimate().tail(3))) - P; 

       _error << (QP_pr(0)*QP(0)+QP_pr(1)*QP(1))/(QP_pr.norm()*QP.norm()) - 1, (PQ_pr(0)*QP(0)+PQ_pr(1)*QP(1))/(PQ_pr.norm()*QP.norm()) + 1;
    }

    bool isDepthPositive() {
        const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
        const g2o::VertexSBALineXYZ* v2 = static_cast<const g2o::VertexSBALineXYZ*>(_vertices[0]);
        return (v1->estimate().map(v2->estimate().head(3)))(2)>0.0 && (v1->estimate().map(v2->estimate().tail(3)))(2)>0.0;
    }


    virtual void linearizeOplus();

    Eigen::Vector2d cam_project(const Eigen::Vector3d & trans_xyz) const;

    Eigen::Matrix<double,4,1> obs_temp;
    double fx, fy, cx, cy;
};

class  EdgeLineWithAngleSE3ProjectXYZ: public  g2o::BaseBinaryEdge<4, Eigen::Matrix<double,7,1>, g2o::VertexSBALineXYZ, g2o::VertexSE3Expmap>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeLineWithAngleSE3ProjectXYZ();

    bool read(std::istream& is);

    bool write(std::ostream& os) const;

    void computeError()  {
        const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
        const g2o::VertexSBALineXYZ* v2 = static_cast<const g2o::VertexSBALineXYZ*>(_vertices[0]);
        Eigen::Matrix<double,7,1> obs(_measurement);
        Eigen::Vector2d P; 
        Eigen::Vector2d Q; 
        Eigen::Vector2d QP_pr; 
        Eigen::Vector2d QP; 
        Eigen::Vector2d PQ_pr; 


        P << obs(0), obs(1);
        Q << obs(2), obs(3);
        QP_pr = cam_project(v1->estimate().map(v2->estimate().head(3))) - Q; 
        QP = P - Q;
        PQ_pr = cam_project(v1->estimate().map(v2->estimate().tail(3))) - P; 

       _error << obs(4)*cam_project(v1->estimate().map(v2->estimate().head(3)))(0)+obs(5)*cam_project(v1->estimate().map(v2->estimate().head(3)))(1)+obs(6), obs(4)*cam_project(v1->estimate().map(v2->estimate().tail(3)))(0)+obs(5)*cam_project(v1->estimate().map(v2->estimate().tail(3)))(1)+obs(6), (QP_pr(0)*QP(0)+QP_pr(1)*QP(1))/(QP_pr.norm()*QP.norm()) - 1, (PQ_pr(0)*QP(0)+PQ_pr(1)*QP(1))/(PQ_pr.norm()*QP.norm()) + 1;
    }

    bool isDepthPositive() {
        const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
        const g2o::VertexSBALineXYZ* v2 = static_cast<const g2o::VertexSBALineXYZ*>(_vertices[0]);
        return (v1->estimate().map(v2->estimate().head(3)))(2)>0.0 && (v1->estimate().map(v2->estimate().tail(3)))(2)>0.0;
    }


    virtual void linearizeOplus();

    Eigen::Vector2d cam_project(const Eigen::Vector3d & trans_xyz) const;

    Eigen::Matrix<double,7,1> obs_temp;
    double fx, fy, cx, cy;
};

}

#endif //ORB_SLAM3_OPTIMIZABLETYPES_H
