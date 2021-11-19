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

#include "OptimizableTypes.h"

namespace ORB_SLAM3 {
    bool EdgeSE3ProjectXYZOnlyPose::read(std::istream& is){
        for (int i=0; i<2; i++){
            is >> _measurement[i];
        }
        for (int i=0; i<2; i++)
            for (int j=i; j<2; j++) {
                is >> information()(i,j);
                if (i!=j)
                    information()(j,i)=information()(i,j);
            }
        return true;
    }

    bool EdgeSE3ProjectXYZOnlyPose::write(std::ostream& os) const {

        for (int i=0; i<2; i++){
            os << measurement()[i] << " ";
        }

        for (int i=0; i<2; i++)
            for (int j=i; j<2; j++){
                os << " " <<  information()(i,j);
            }
        return os.good();
    }


    void EdgeSE3ProjectXYZOnlyPose::linearizeOplus() {
        g2o::VertexSE3Expmap * vi = static_cast<g2o::VertexSE3Expmap *>(_vertices[0]);
        Eigen::Vector3d xyz_trans = vi->estimate().map(Xw);

        double x = xyz_trans[0];
        double y = xyz_trans[1];
        double z = xyz_trans[2];

        Eigen::Matrix<double,3,6> SE3deriv;
        SE3deriv << 0.f, z,   -y, 1.f, 0.f, 0.f,
                     -z , 0.f, x, 0.f, 1.f, 0.f,
                     y ,  -x , 0.f, 0.f, 0.f, 1.f;

        _jacobianOplusXi = -pCamera->projectJac(xyz_trans) * SE3deriv;
    }

    bool EdgeSE3ProjectXYZOnlyPoseToBody::read(std::istream& is){
        for (int i=0; i<2; i++){
            is >> _measurement[i];
        }
        for (int i=0; i<2; i++)
            for (int j=i; j<2; j++) {
                is >> information()(i,j);
                if (i!=j)
                    information()(j,i)=information()(i,j);
            }
        return true;
    }

    bool EdgeSE3ProjectXYZOnlyPoseToBody::write(std::ostream& os) const {

        for (int i=0; i<2; i++){
            os << measurement()[i] << " ";
        }

        for (int i=0; i<2; i++)
            for (int j=i; j<2; j++){
                os << " " <<  information()(i,j);
            }
        return os.good();
    }

    void EdgeSE3ProjectXYZOnlyPoseToBody::linearizeOplus() {
        g2o::VertexSE3Expmap * vi = static_cast<g2o::VertexSE3Expmap *>(_vertices[0]);
        g2o::SE3Quat T_lw(vi->estimate());
        Eigen::Vector3d X_l = T_lw.map(Xw);
        Eigen::Vector3d X_r = mTrl.map(T_lw.map(Xw));

        double x_w = X_l[0];
        double y_w = X_l[1];
        double z_w = X_l[2];

        Eigen::Matrix<double,3,6> SE3deriv;
        SE3deriv << 0.f, z_w,   -y_w, 1.f, 0.f, 0.f,
                -z_w , 0.f, x_w, 0.f, 1.f, 0.f,
                y_w ,  -x_w , 0.f, 0.f, 0.f, 1.f;

        _jacobianOplusXi = -pCamera->projectJac(X_r) * mTrl.rotation().toRotationMatrix() * SE3deriv;
    }

    EdgeSE3ProjectXYZ::EdgeSE3ProjectXYZ() : BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSBAPointXYZ, g2o::VertexSE3Expmap>() {
    }

    bool EdgeSE3ProjectXYZ::read(std::istream& is){
        for (int i=0; i<2; i++){
            is >> _measurement[i];
        }
        for (int i=0; i<2; i++)
            for (int j=i; j<2; j++) {
                is >> information()(i,j);
                if (i!=j)
                    information()(j,i)=information()(i,j);
            }
        return true;
    }

    bool EdgeSE3ProjectXYZ::write(std::ostream& os) const {

        for (int i=0; i<2; i++){
            os << measurement()[i] << " ";
        }

        for (int i=0; i<2; i++)
            for (int j=i; j<2; j++){
                os << " " <<  information()(i,j);
            }
        return os.good();
    }


    void EdgeSE3ProjectXYZ::linearizeOplus() {
        g2o::VertexSE3Expmap * vj = static_cast<g2o::VertexSE3Expmap *>(_vertices[1]);
        g2o::SE3Quat T(vj->estimate());
        g2o::VertexSBAPointXYZ* vi = static_cast<g2o::VertexSBAPointXYZ*>(_vertices[0]);
        Eigen::Vector3d xyz = vi->estimate();
        Eigen::Vector3d xyz_trans = T.map(xyz);

        double x = xyz_trans[0];
        double y = xyz_trans[1];
        double z = xyz_trans[2];

        auto projectJac = -pCamera->projectJac(xyz_trans);

        _jacobianOplusXi =  projectJac * T.rotation().toRotationMatrix();

        Eigen::Matrix<double,3,6> SE3deriv;
        SE3deriv << 0.f, z,   -y, 1.f, 0.f, 0.f,
                -z , 0.f, x, 0.f, 1.f, 0.f,
                y ,  -x , 0.f, 0.f, 0.f, 1.f;

        _jacobianOplusXj = projectJac * SE3deriv;
    }

    EdgeSE3ProjectXYZToBody::EdgeSE3ProjectXYZToBody() : BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSBAPointXYZ, g2o::VertexSE3Expmap>() {
    }

    bool EdgeSE3ProjectXYZToBody::read(std::istream& is){
        for (int i=0; i<2; i++){
            is >> _measurement[i];
        }
        for (int i=0; i<2; i++)
            for (int j=i; j<2; j++) {
                is >> information()(i,j);
                if (i!=j)
                    information()(j,i)=information()(i,j);
            }
        return true;
    }

    bool EdgeSE3ProjectXYZToBody::write(std::ostream& os) const {

        for (int i=0; i<2; i++){
            os << measurement()[i] << " ";
        }

        for (int i=0; i<2; i++)
            for (int j=i; j<2; j++){
                os << " " <<  information()(i,j);
            }
        return os.good();
    }


    void EdgeSE3ProjectXYZToBody::linearizeOplus() {
        g2o::VertexSE3Expmap * vj = static_cast<g2o::VertexSE3Expmap *>(_vertices[1]);
        g2o::SE3Quat T_lw(vj->estimate());
        g2o::SE3Quat T_rw = mTrl * T_lw;
        g2o::VertexSBAPointXYZ* vi = static_cast<g2o::VertexSBAPointXYZ*>(_vertices[0]);
        Eigen::Vector3d X_w = vi->estimate();
        Eigen::Vector3d X_l = T_lw.map(X_w);
        Eigen::Vector3d X_r = mTrl.map(T_lw.map(X_w));

        _jacobianOplusXi =  -pCamera->projectJac(X_r) * T_rw.rotation().toRotationMatrix();

        double x = X_l[0];
        double y = X_l[1];
        double z = X_l[2];

        Eigen::Matrix<double,3,6> SE3deriv;
        SE3deriv << 0.f, z,   -y, 1.f, 0.f, 0.f,
                -z , 0.f, x, 0.f, 1.f, 0.f,
                y ,  -x , 0.f, 0.f, 0.f, 1.f;

        _jacobianOplusXj = -pCamera->projectJac(X_r) * mTrl.rotation().toRotationMatrix() * SE3deriv;
    }


    VertexSim3Expmap::VertexSim3Expmap() : BaseVertex<7, g2o::Sim3>()
    {
        _marginalized=false;
        _fix_scale = false;
    }

    bool VertexSim3Expmap::read(std::istream& is)
    {
        g2o::Vector7d cam2world;
        for (int i=0; i<6; i++){
            is >> cam2world[i];
        }
        is >> cam2world[6];

        float nextParam;
        for(size_t i = 0; i < pCamera1->size(); i++){
            is >> nextParam;
            pCamera1->setParameter(nextParam,i);
        }

        for(size_t i = 0; i < pCamera2->size(); i++){
            is >> nextParam;
            pCamera2->setParameter(nextParam,i);
        }

        setEstimate(g2o::Sim3(cam2world).inverse());
        return true;
    }

    bool VertexSim3Expmap::write(std::ostream& os) const
    {
        g2o::Sim3 cam2world(estimate().inverse());
        g2o::Vector7d lv=cam2world.log();
        for (int i=0; i<7; i++){
            os << lv[i] << " ";
        }

        for(size_t i = 0; i < pCamera1->size(); i++){
            os << pCamera1->getParameter(i) << " ";
        }

        for(size_t i = 0; i < pCamera2->size(); i++){
            os << pCamera2->getParameter(i) << " ";
        }

        return os.good();
    }

    EdgeSim3ProjectXYZ::EdgeSim3ProjectXYZ() :
            g2o::BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSBAPointXYZ, VertexSim3Expmap>()
    {
    }

    bool EdgeSim3ProjectXYZ::read(std::istream& is)
    {
        for (int i=0; i<2; i++)
        {
            is >> _measurement[i];
        }

        for (int i=0; i<2; i++)
            for (int j=i; j<2; j++) {
                is >> information()(i,j);
                if (i!=j)
                    information()(j,i)=information()(i,j);
            }
        return true;
    }

    bool EdgeSim3ProjectXYZ::write(std::ostream& os) const
    {
        for (int i=0; i<2; i++){
            os  << _measurement[i] << " ";
        }

        for (int i=0; i<2; i++)
            for (int j=i; j<2; j++){
                os << " " <<  information()(i,j);
            }
        return os.good();
    }

    EdgeInverseSim3ProjectXYZ::EdgeInverseSim3ProjectXYZ() :
            g2o::BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSBAPointXYZ, VertexSim3Expmap>()
    {
    }

    bool EdgeInverseSim3ProjectXYZ::read(std::istream& is)
    {
        for (int i=0; i<2; i++)
        {
            is >> _measurement[i];
        }

        for (int i=0; i<2; i++)
            for (int j=i; j<2; j++) {
                is >> information()(i,j);
                if (i!=j)
                    information()(j,i)=information()(i,j);
            }
        return true;
    }

    bool EdgeInverseSim3ProjectXYZ::write(std::ostream& os) const
    {
        for (int i=0; i<2; i++){
            os  << _measurement[i] << " ";
        }

        for (int i=0; i<2; i++)
            for (int j=i; j<2; j++){
                os << " " <<  information()(i,j);
            }
        return os.good();
    }

    bool EdgeLineSE3ProjectXYZOnlyPose::read(std::istream& is){
        for (int i=0; i<3; i++){
            is >> _measurement[i];
        }
        for (int i=0; i<2; i++)
            for (int j=i; j<2; j++) {
                is >> information()(i,j);
                if (i!=j)
                    information()(j,i)=information()(i,j);
            }
        return true;
    }

    bool EdgeLineSE3ProjectXYZOnlyPose::write(std::ostream& os) const {
        for (int i=0; i<3; i++){
            os << measurement()[i] << " ";
        }

        for (int i=0; i<2; i++)
            for (int j=i; j<2; j++){
                os << " " <<  information()(i,j);
            }
        return os.good();
    }

    Eigen::Vector2d EdgeLineSE3ProjectXYZOnlyPose::cam_project(const Eigen::Vector3d & trans_xyz) const{
    const float invz = 1.0f/trans_xyz[2];
    Eigen::Vector2d res;
    res[0] = trans_xyz[0]*invz*fx + cx;
    res[1] = trans_xyz[1]*invz*fy + cy;
    return res;
    }


    void EdgeLineSE3ProjectXYZOnlyPose::linearizeOplus() {
        g2o::VertexSE3Expmap * vi = static_cast<g2o::VertexSE3Expmap *>(_vertices[0]);
        Eigen::Vector3d xyz_trans_s = vi->estimate().map(Xw_s);
        Eigen::Vector3d xyz_trans_e = vi->estimate().map(Xw_e);

        double x_s = xyz_trans_s[0];
        double y_s = xyz_trans_s[1];
        double invz_s = 1.0/xyz_trans_s[2];
        double invz_s_2 = invz_s*invz_s;

        double x_e = xyz_trans_e[0];
        double y_e = xyz_trans_e[1];
        double invz_e = 1.0/xyz_trans_e[2];
        double invz_e_2 = invz_e*invz_e;

        double l0 = obs_temp(0);
        double l1 = obs_temp(1);

        _jacobianOplusXi(0,0) = -fx*x_s*y_s*invz_s_2*l0-fy*(1+y_s*y_s*invz_s_2)*l1;
        _jacobianOplusXi(0,1) = fx*(1+x_s*x_s*invz_s_2)*l0+fy*x_s*y_s*invz_s_2*l1;
        _jacobianOplusXi(0,2) = -fx*y_s*invz_s*l0+fy*x_s*invz_s*l1; 
        _jacobianOplusXi(0,3) = fx*invz_s*l0;
        _jacobianOplusXi(0,4) = fy*invz_s*l1;
        _jacobianOplusXi(0,5) = (-fx*x_s*l0-fy*y_s*l1)*invz_s_2;

        _jacobianOplusXi(1,0) = -fx*x_e*y_e*invz_e_2*l0-fy*(1+y_e*y_e*invz_e_2)*l1;
        _jacobianOplusXi(1,1) = fx*(1+x_e*x_e*invz_e_2)*l0+fy*x_e*y_e*invz_e_2*l1;
        _jacobianOplusXi(1,2) = -fx*y_e*invz_e*l0+fy*x_e*invz_e*l1;  
        _jacobianOplusXi(1,3) = fx*invz_e*l0;
        _jacobianOplusXi(1,4) = fy*invz_e*l1;
        _jacobianOplusXi(1,5) = (-fx*x_e*l0-fy*y_e*l1)*invz_e_2;
    }

    bool EdgeLineAngleSE3ProjectXYZOnlyPose::read(std::istream& is){
        for (int i=0; i<4; i++){
            is >> _measurement[i];
        }
        for (int i=0; i<2; i++)
            for (int j=i; j<2; j++) {
                is >> information()(i,j);
                if (i!=j)
                    information()(j,i)=information()(i,j);
            }
        return true;
    }

    bool EdgeLineAngleSE3ProjectXYZOnlyPose::write(std::ostream& os) const {
        for (int i=0; i<4; i++){
            os << measurement()[i] << " ";
        }

        for (int i=0; i<2; i++)
            for (int j=i; j<2; j++){
                os << " " <<  information()(i,j);
            }
        return os.good();
    }

    Eigen::Vector2d EdgeLineAngleSE3ProjectXYZOnlyPose::cam_project(const Eigen::Vector3d & trans_xyz) const{
    const float invz = 1.0f/trans_xyz[2];
    Eigen::Vector2d res;
    res[0] = trans_xyz[0]*invz*fx + cx;
    res[1] = trans_xyz[1]*invz*fy + cy;
    return res;
    }


    void EdgeLineAngleSE3ProjectXYZOnlyPose::linearizeOplus() {
        g2o::VertexSE3Expmap * vi = static_cast<g2o::VertexSE3Expmap *>(_vertices[0]);
        Eigen::Vector3d xyz_trans_s = vi->estimate().map(Xw_s);
        Eigen::Vector3d xyz_trans_e = vi->estimate().map(Xw_e);
        Eigen::Vector2d P_pr = cam_project(vi->estimate().map(Xw_s));
        Eigen::Vector2d Q_pr = cam_project(vi->estimate().map(Xw_e));

        double x_s = xyz_trans_s[0];
        double y_s = xyz_trans_s[1];
        double z_s = xyz_trans_s[2];
        double invz_s = 1.0/xyz_trans_s[2];
        double invz_s_2 = invz_s*invz_s;

        double x_e = xyz_trans_e[0];
        double y_e = xyz_trans_e[1];
        double z_e = xyz_trans_e[2];
        double invz_e = 1.0/xyz_trans_e[2];
        double invz_e_2 = invz_e*invz_e;

        double xp = obs_temp(0);
        double yp = obs_temp(1);
        double xq = obs_temp(2);
        double yq = obs_temp(3);

        Eigen::Vector2d QP;
        QP << xp-xq, yp-yq;
        Eigen::Vector2d QP_pr;
        QP_pr << P_pr(0)-xq, P_pr(1)-yq;
        Eigen::Vector2d PQ_pr;
        PQ_pr << Q_pr(0)-xp, Q_pr(1)-yp;
        Eigen::Vector2d Der_start;
        Eigen::Vector2d Der_end;

        Der_start[0] = ((xp-xq)*QP.norm()*QP_pr.norm()-((xp-xq)*(P_pr(0)-xq)+(yp-yq)*(P_pr(1)-yq))*QP.norm()*(P_pr(0)-xq)/QP_pr.norm())/(QP.norm()*QP.norm()*QP_pr.norm()*QP_pr.norm());
        Der_start[1] = ((yp-yq)*QP.norm()*QP_pr.norm()-((xp-xq)*(P_pr(0)-xq)+(yp-yq)*(P_pr(1)-yq))*QP.norm()*(P_pr(1)-yq)/QP_pr.norm())/(QP.norm()*QP.norm()*QP_pr.norm()*QP_pr.norm());

        Eigen::Matrix<double,2,3> ProjJac_s;

        ProjJac_s << fx*invz_s, 0.f, -fx*x_s*invz_s_2,
                    0.f, fy*invz_s, -fx*y_s*invz_s_2;

        Eigen::Matrix<double,1,3> Jac_s = Der_start.transpose()*ProjJac_s;

        Der_end[0] = ((xp-xq)*QP.norm()*PQ_pr.norm()-((xp-xq)*(Q_pr(0)-xp)+(yp-yq)*(Q_pr(1)-yp))*QP.norm()*(Q_pr(0)-xp)/PQ_pr.norm())/(QP.norm()*QP.norm()*QP_pr.norm()*QP_pr.norm());
        Der_end[1] = ((yp-yq)*QP.norm()*PQ_pr.norm()-((xp-xq)*(Q_pr(0)-xp)+(yp-yq)*(Q_pr(1)-yp))*QP.norm()*(Q_pr(1)-yp)/PQ_pr.norm())/(QP.norm()*QP.norm()*QP_pr.norm()*QP_pr.norm());

        Eigen::Matrix<double,2,3> ProjJac_e;

        ProjJac_e << fx*invz_e, 0.f, -fx*x_e*invz_e_2,
                    0.f, fy*invz_e, -fx*y_e*invz_e_2;

        Eigen::Matrix<double,1,3> Jac_e = Der_end.transpose()*ProjJac_e;
        
        Eigen::Matrix<double,3,6> SE3deriv_s;
        SE3deriv_s << 0.f, z_s, -y_s, 1.f, 0.f, 0.f,
                      -z_s, 0.f, x_s, 0.f, 1.f, 0.f,
                      y_s, -x_s, 0.f, 0.f, 0.f, 1.f;

        Eigen::Matrix<double,3,6> SE3deriv_e;
        SE3deriv_e << 0.f, z_e, -y_e, 1.f, 0.f, 0.f,
                      -z_e, 0.f, x_e, 0.f, 1.f, 0.f,
                      y_e, -x_e, 0.f, 0.f, 0.f, 1.f;

        _jacobianOplusXi << Jac_s*SE3deriv_s,
                            Jac_e*SE3deriv_e;
    }

    bool EdgeLineWithAngleSE3ProjectXYZOnlyPose::read(std::istream& is){
        for (int i=0; i<7; i++){
            is >> _measurement[i];
        }
        for (int i=0; i<4; i++)
            for (int j=i; j<4; j++) {
                is >> information()(i,j);
                if (i!=j)
                    information()(j,i)=information()(i,j);
            }
        return true;
    }

    bool EdgeLineWithAngleSE3ProjectXYZOnlyPose::write(std::ostream& os) const {
        for (int i=0; i<7; i++){
            os << measurement()[i] << " ";
        }

        for (int i=0; i<4; i++)
            for (int j=i; j<4; j++){
                os << " " <<  information()(i,j);
            }
        return os.good();
    }

    Eigen::Vector2d EdgeLineWithAngleSE3ProjectXYZOnlyPose::cam_project(const Eigen::Vector3d & trans_xyz) const{
    const float invz = 1.0f/trans_xyz[2];
    Eigen::Vector2d res;
    res[0] = trans_xyz[0]*invz*fx + cx;
    res[1] = trans_xyz[1]*invz*fy + cy;
    return res;
    }

    void EdgeLineWithAngleSE3ProjectXYZOnlyPose::linearizeOplus() {
        g2o::VertexSE3Expmap * vi = static_cast<g2o::VertexSE3Expmap *>(_vertices[0]);
        Eigen::Vector3d xyz_trans_s = vi->estimate().map(Xw_s);
        Eigen::Vector3d xyz_trans_e = vi->estimate().map(Xw_e);
        Eigen::Vector2d P_pr = cam_project(vi->estimate().map(Xw_s));
        Eigen::Vector2d Q_pr = cam_project(vi->estimate().map(Xw_e));

        double x_s = xyz_trans_s[0];
        double y_s = xyz_trans_s[1];
        double z_s = xyz_trans_s[2];
        double invz_s = 1.0/xyz_trans_s[2];
        double invz_s_2 = invz_s*invz_s;

        double x_e = xyz_trans_e[0];
        double y_e = xyz_trans_e[1];
        double z_e = xyz_trans_e[2];
        double invz_e = 1.0/xyz_trans_e[2];
        double invz_e_2 = invz_e*invz_e;

        double xp = obs_temp(0);
        double yp = obs_temp(1);
        double xq = obs_temp(2);
        double yq = obs_temp(3);
        double l0 = obs_temp(4);
        double l1 = obs_temp(5);

        Eigen::Vector2d QP;
        QP << xp-xq, yp-yq;
        Eigen::Vector2d QP_pr;
        QP_pr << P_pr(0)-xq, P_pr(1)-yq;
        Eigen::Vector2d PQ_pr;
        PQ_pr << Q_pr(0)-xp, Q_pr(1)-yp;
        Eigen::Vector2d Der_start;
        Eigen::Vector2d Der_end;

        Der_start[0] = ((xp-xq)*QP.norm()*QP_pr.norm()-((xp-xq)*(P_pr(0)-xq)+(yp-yq)*(P_pr(1)-yq))*QP.norm()*(P_pr(0)-xq)/QP_pr.norm())/(QP.norm()*QP.norm()*QP_pr.norm()*QP_pr.norm());
        Der_start[1] = ((yp-yq)*QP.norm()*QP_pr.norm()-((xp-xq)*(P_pr(0)-xq)+(yp-yq)*(P_pr(1)-yq))*QP.norm()*(P_pr(1)-yq)/QP_pr.norm())/(QP.norm()*QP.norm()*QP_pr.norm()*QP_pr.norm());

        Eigen::Matrix<double,2,3> ProjJac_s;

        ProjJac_s << fx*invz_s, 0.f, -fx*x_s*invz_s_2,
                    0.f, fy*invz_s, -fx*y_s*invz_s_2;

        Eigen::Matrix<double,1,3> Jac_s = Der_start.transpose()*ProjJac_s;

        Der_end[0] = ((xp-xq)*QP.norm()*PQ_pr.norm()-((xp-xq)*(Q_pr(0)-xp)+(yp-yq)*(Q_pr(1)-yp))*QP.norm()*(Q_pr(0)-xp)/PQ_pr.norm())/(QP.norm()*QP.norm()*QP_pr.norm()*QP_pr.norm());
        Der_end[1] = ((yp-yq)*QP.norm()*PQ_pr.norm()-((xp-xq)*(Q_pr(0)-xp)+(yp-yq)*(Q_pr(1)-yp))*QP.norm()*(Q_pr(1)-yp)/PQ_pr.norm())/(QP.norm()*QP.norm()*QP_pr.norm()*QP_pr.norm());

        Eigen::Matrix<double,2,3> ProjJac_e;

        ProjJac_e << fx*invz_e, 0.f, -fx*x_e*invz_e_2,
                    0.f, fy*invz_e, -fx*y_e*invz_e_2;

        Eigen::Matrix<double,1,3> Jac_e = Der_end.transpose()*ProjJac_e;
        
        Eigen::Matrix<double,3,6> SE3deriv_s;
        SE3deriv_s << 0.f, z_s, -y_s, 1.f, 0.f, 0.f,
                      -z_s, 0.f, x_s, 0.f, 1.f, 0.f,
                      y_s, -x_s, 0.f, 0.f, 0.f, 1.f;

        Eigen::Matrix<double,3,6> SE3deriv_e;
        SE3deriv_e << 0.f, z_e, -y_e, 1.f, 0.f, 0.f,
                      -z_e, 0.f, x_e, 0.f, 1.f, 0.f,
                      y_e, -x_e, 0.f, 0.f, 0.f, 1.f;

        Eigen::Matrix<double,1,6> JacOplusS;
        Eigen::Matrix<double,1,6> JacOplusE;
       
        JacOplusS(0,0) = -fx*x_s*y_s*invz_s_2*l0-fy*(1+y_s*y_s*invz_s_2)*l1;
        JacOplusS(0,1) = fx*(1+x_s*x_s*invz_s_2)*l0+fy*x_s*y_s*invz_s_2*l1;
        JacOplusS(0,2) = -fx*y_s*invz_s*l0+fy*x_s*invz_s*l1; 
        JacOplusS(0,3) = fx*invz_s*l0;
        JacOplusS(0,4) = fy*invz_s*l1;
        JacOplusS(0,5) = (-fx*x_s*l0-fy*y_s*l1)*invz_s_2;

        JacOplusE(0,0) = -fx*x_e*y_e*invz_e_2*l0-fy*(1+y_e*y_e*invz_e_2)*l1;
        JacOplusE(0,1) = fx*(1+x_e*x_e*invz_e_2)*l0+fy*x_e*y_e*invz_e_2*l1;
        JacOplusE(0,2) = -fx*y_e*invz_e*l0+fy*x_e*invz_e*l1;  
        JacOplusE(0,3) = fx*invz_e*l0;
        JacOplusE(0,4) = fy*invz_e*l1;
        JacOplusE(0,5) = (-fx*x_e*l0-fy*y_e*l1)*invz_e_2;

        _jacobianOplusXi << JacOplusS,
                            JacOplusE,
                            Jac_s*SE3deriv_s,
                            Jac_e*SE3deriv_e;
    }

    EdgeLineSE3ProjectXYZ::EdgeLineSE3ProjectXYZ() : 
    g2o::BaseBinaryEdge<2, Eigen::Vector3d, g2o::VertexSBALineXYZ, g2o::VertexSE3Expmap>() 
    {
    } 

    bool EdgeLineSE3ProjectXYZ::read(std::istream& is){
        for (int i=0; i<3; i++){
            is >> _measurement[i];
        }
        for (int i=0; i<2; i++)
            for (int j=i; j<2; j++) {
                is >> information()(i,j);
                if (i!=j)
                    information()(j,i)=information()(i,j);
            }
        return true;
    }

    bool EdgeLineSE3ProjectXYZ::write(std::ostream& os) const {
        for (int i=0; i<3; i++){
            os << measurement()[i] << " ";
        }

        for (int i=0; i<2; i++)
            for (int j=i; j<2; j++){
                os << " " <<  information()(i,j);
            }
        return os.good();
    }

    Eigen::Vector2d EdgeLineSE3ProjectXYZ::cam_project(const Eigen::Vector3d & trans_xyz) const{
    const float invz = 1.0f/trans_xyz[2];
    Eigen::Vector2d res;
    res[0] = trans_xyz[0]*invz*fx + cx;
    res[1] = trans_xyz[1]*invz*fy + cy;
    return res;
    }

    void EdgeLineSE3ProjectXYZ::linearizeOplus() {
        g2o::VertexSE3Expmap * vj= static_cast<g2o::VertexSE3Expmap *>(_vertices[1]);
        g2o::SE3Quat T(vj->estimate());
        g2o::VertexSBALineXYZ* vi = static_cast<g2o::VertexSBALineXYZ*>(_vertices[0]);
        Eigen::Vector3d xyz_s = vi->estimate().head(3);
        Eigen::Vector3d xyz_trans_s = T.map(xyz_s);
        Eigen::Vector3d xyz_e = vi->estimate().tail(3);
        Eigen::Vector3d xyz_trans_e = T.map(xyz_e); 

        const Eigen::Matrix3d R =  T.rotation().toRotationMatrix();

        double x_s = xyz_trans_s[0];
        double y_s = xyz_trans_s[1];
        double invz_s = 1.0/xyz_trans_s[2];
        double invz_s_2 = invz_s*invz_s;

        double x_e = xyz_trans_e[0];
        double y_e = xyz_trans_e[1];
        double invz_e = 1.0/xyz_trans_e[2];
        double invz_e_2 = invz_e*invz_e;

        double l0 = obs_temp(0);
        double l1 = obs_temp(1);

        _jacobianOplusXi(0,0) = fx*l0*invz_s*R(0,0)+fy*l1*invz_s*R(1,0)-(fx*x_s*l0*invz_s_2+fy*y_s*l1*invz_s_2)*R(2,0);
        _jacobianOplusXi(0,1) = fx*l0*invz_s*R(0,1)+fy*l1*invz_s*R(1,1)-(fx*x_s*l0*invz_s_2+fy*y_s*l1*invz_s_2)*R(2,1);
        _jacobianOplusXi(0,2) = fx*l0*invz_s*R(0,2)+fy*l1*invz_s*R(1,2)-(fx*x_s*l0*invz_s_2+fy*y_s*l1*invz_s_2)*R(2,2);

        _jacobianOplusXi(1,0) = fx*l0*invz_e*R(0,0)+fy*l1*invz_e*R(1,0)-(fx*x_e*l0*invz_e_2+fy*y_e*l1*invz_e_2)*R(2,0);
        _jacobianOplusXi(1,1) = fx*l0*invz_e*R(0,1)+fy*l1*invz_e*R(1,1)-(fx*x_e*l0*invz_e_2+fy*y_e*l1*invz_e_2)*R(2,1);
        _jacobianOplusXi(1,2) = fx*l0*invz_e*R(0,2)+fy*l1*invz_e*R(1,2)-(fx*x_e*l0*invz_e_2+fy*y_e*l1*invz_e_2)*R(2,2);

        _jacobianOplusXj(0,0) = -fx*x_s*y_s*invz_s_2*l0-fy*(1+y_s*y_s*invz_s_2)*l1;
        _jacobianOplusXj(0,1) = fx*(1+x_s*x_s*invz_s_2)*l0+fy*x_s*y_s*invz_s_2*l1;
        _jacobianOplusXj(0,2) = -fx*y_s*invz_s*l0+fy*x_s*invz_s*l1; 
        _jacobianOplusXj(0,3) = fx*invz_s*l0;
        _jacobianOplusXj(0,4) = fy*invz_s*l1;
        _jacobianOplusXj(0,5) = (-fx*x_s*l0-fy*y_s*l1)*invz_s_2;

        _jacobianOplusXj(1,0) = -fx*x_e*y_e*invz_e_2*l0-fy*(1+y_e*y_e*invz_e_2)*l1;
        _jacobianOplusXj(1,1) = fx*(1+x_e*x_e*invz_e_2)*l0+fy*x_e*y_e*invz_e_2*l1;
        _jacobianOplusXj(1,2) = -fx*y_e*invz_e*l0+fy*x_e*invz_e*l1;  
        _jacobianOplusXj(1,3) = fx*invz_e*l0;
        _jacobianOplusXj(1,4) = fy*invz_e*l1;
        _jacobianOplusXj(1,5) = (-fx*x_e*l0-fy*y_e*l1)*invz_e_2; 
    }

    EdgeLineAngleSE3ProjectXYZ::EdgeLineAngleSE3ProjectXYZ() : 
    g2o::BaseBinaryEdge<2, Eigen::Matrix<double,4,1>, g2o::VertexSBALineXYZ, g2o::VertexSE3Expmap>() 
    {
    } 

    bool EdgeLineAngleSE3ProjectXYZ::read(std::istream& is){
        for (int i=0; i<4; i++){
            is >> _measurement[i];
        }
        for (int i=0; i<2; i++)
            for (int j=i; j<2; j++) {
                is >> information()(i,j);
                if (i!=j)
                    information()(j,i)=information()(i,j);
            }
        return true;
    }

    bool EdgeLineAngleSE3ProjectXYZ::write(std::ostream& os) const {
        for (int i=0; i<4; i++){
            os << measurement()[i] << " ";
        }

        for (int i=0; i<2; i++)
            for (int j=i; j<2; j++){
                os << " " <<  information()(i,j);
            }
        return os.good();
    }

    Eigen::Vector2d EdgeLineAngleSE3ProjectXYZ::cam_project(const Eigen::Vector3d & trans_xyz) const{
    const float invz = 1.0f/trans_xyz[2];
    Eigen::Vector2d res;
    res[0] = trans_xyz[0]*invz*fx + cx;
    res[1] = trans_xyz[1]*invz*fy + cy;
    return res;
    }

    void EdgeLineAngleSE3ProjectXYZ::linearizeOplus() {
        g2o::VertexSE3Expmap * vj= static_cast<g2o::VertexSE3Expmap *>(_vertices[1]);
        g2o::SE3Quat T(vj->estimate());
        g2o::VertexSBALineXYZ* vi = static_cast<g2o::VertexSBALineXYZ*>(_vertices[0]);
        Eigen::Vector3d xyz_s = vi->estimate().head(3);
        Eigen::Vector3d xyz_trans_s = T.map(xyz_s);
        Eigen::Vector3d xyz_e = vi->estimate().tail(3);
        Eigen::Vector3d xyz_trans_e = T.map(xyz_e); 
        Eigen::Vector2d P_pr = cam_project(xyz_trans_s);
        Eigen::Vector2d Q_pr = cam_project(xyz_trans_e);

        const Eigen::Matrix3d R =  T.rotation().toRotationMatrix();

        double x_s = xyz_trans_s[0];
        double y_s = xyz_trans_s[1];
        double z_s = xyz_trans_s[2];
        double invz_s = 1.0/xyz_trans_s[2];
        double invz_s_2 = invz_s*invz_s;

        double x_e = xyz_trans_e[0];
        double y_e = xyz_trans_e[1];
        double z_e = xyz_trans_e[2];
        double invz_e = 1.0/xyz_trans_e[2];
        double invz_e_2 = invz_e*invz_e;

        double xp = obs_temp(0);
        double yp = obs_temp(1);
        double xq = obs_temp(2);
        double yq = obs_temp(3);

        Eigen::Vector2d QP;
        QP << xp-xq, yp-yq;
        Eigen::Vector2d QP_pr;
        QP_pr << P_pr(0)-xq, P_pr(1)-yq;
        Eigen::Vector2d PQ_pr;
        PQ_pr << Q_pr(0)-xp, Q_pr(1)-yp;
        Eigen::Vector2d Der_start;
        Eigen::Vector2d Der_end;

        Der_start[0] = ((xp-xq)*QP.norm()*QP_pr.norm()-((xp-xq)*(P_pr(0)-xq)+(yp-yq)*(P_pr(1)-yq))*QP.norm()*(P_pr(0)-xq)/QP_pr.norm())/(QP.norm()*QP.norm()*QP_pr.norm()*QP_pr.norm());
        Der_start[1] = ((yp-yq)*QP.norm()*QP_pr.norm()-((xp-xq)*(P_pr(0)-xq)+(yp-yq)*(P_pr(1)-yq))*QP.norm()*(P_pr(1)-yq)/QP_pr.norm())/(QP.norm()*QP.norm()*QP_pr.norm()*QP_pr.norm());

        Eigen::Matrix<double,2,3> ProjJac_s;

        ProjJac_s << fx*invz_s, 0.f, -fx*x_s*invz_s_2,
                    0.f, fy*invz_s, -fx*y_s*invz_s_2;

        Eigen::Matrix<double,1,3> Jac_s = Der_start.transpose()*ProjJac_s;

        Der_end[0] = ((xp-xq)*QP.norm()*PQ_pr.norm()-((xp-xq)*(Q_pr(0)-xp)+(yp-yq)*(Q_pr(1)-yp))*QP.norm()*(Q_pr(0)-xp)/PQ_pr.norm())/(QP.norm()*QP.norm()*QP_pr.norm()*QP_pr.norm());
        Der_end[1] = ((yp-yq)*QP.norm()*PQ_pr.norm()-((xp-xq)*(Q_pr(0)-xp)+(yp-yq)*(Q_pr(1)-yp))*QP.norm()*(Q_pr(1)-yp)/PQ_pr.norm())/(QP.norm()*QP.norm()*QP_pr.norm()*QP_pr.norm());

        Eigen::Matrix<double,2,3> ProjJac_e;

        ProjJac_e << fx*invz_e, 0.f, -fx*x_e*invz_e_2,
                    0.f, fy*invz_e, -fx*y_e*invz_e_2;

        Eigen::Matrix<double,1,3> Jac_e = Der_end.transpose()*ProjJac_e;
        
        Eigen::Matrix<double,3,6> SE3deriv_s;
        SE3deriv_s << 0.f, z_s, -y_s, 1.f, 0.f, 0.f,
                      -z_s, 0.f, x_s, 0.f, 1.f, 0.f,
                      y_s, -x_s, 0.f, 0.f, 0.f, 1.f;

        Eigen::Matrix<double,3,6> SE3deriv_e;
        SE3deriv_e << 0.f, z_e, -y_e, 1.f, 0.f, 0.f,
                      -z_e, 0.f, x_e, 0.f, 1.f, 0.f,
                      y_e, -x_e, 0.f, 0.f, 0.f, 1.f;

        _jacobianOplusXi << Jac_s*R,
                            Jac_e*R;

        _jacobianOplusXj << Jac_s*SE3deriv_s,
                            Jac_e*SE3deriv_e;
    }

    EdgeLineWithAngleSE3ProjectXYZ::EdgeLineWithAngleSE3ProjectXYZ() : 
    g2o::BaseBinaryEdge<4, Eigen::Matrix<double,7,1>, g2o::VertexSBALineXYZ, g2o::VertexSE3Expmap>() 
    {
    } 

    bool EdgeLineWithAngleSE3ProjectXYZ::read(std::istream& is){
        for (int i=0; i<7; i++){
            is >> _measurement[i];
        }
        for (int i=0; i<4; i++)
            for (int j=i; j<4; j++) {
                is >> information()(i,j);
                if (i!=j)
                    information()(j,i)=information()(i,j);
            }
        return true;
    }

    bool EdgeLineWithAngleSE3ProjectXYZ::write(std::ostream& os) const {
        for (int i=0; i<7; i++){
            os << measurement()[i] << " ";
        }

        for (int i=0; i<4; i++)
            for (int j=i; j<4; j++){
                os << " " <<  information()(i,j);
            }
        return os.good();
    }

    Eigen::Vector2d EdgeLineWithAngleSE3ProjectXYZ::cam_project(const Eigen::Vector3d & trans_xyz) const{
    const float invz = 1.0f/trans_xyz[2];
    Eigen::Vector2d res;
    res[0] = trans_xyz[0]*invz*fx + cx;
    res[1] = trans_xyz[1]*invz*fy + cy;
    return res;
    }

    void EdgeLineWithAngleSE3ProjectXYZ::linearizeOplus() {
        g2o::VertexSE3Expmap * vj= static_cast<g2o::VertexSE3Expmap *>(_vertices[1]);
        g2o::SE3Quat T(vj->estimate());
        g2o::VertexSBALineXYZ* vi = static_cast<g2o::VertexSBALineXYZ*>(_vertices[0]);
        Eigen::Vector3d xyz_s = vi->estimate().head(3);
        Eigen::Vector3d xyz_trans_s = T.map(xyz_s);
        Eigen::Vector3d xyz_e = vi->estimate().tail(3);
        Eigen::Vector3d xyz_trans_e = T.map(xyz_e); 
        Eigen::Vector2d P_pr = cam_project(xyz_trans_s);
        Eigen::Vector2d Q_pr = cam_project(xyz_trans_e);

        const Eigen::Matrix3d R =  T.rotation().toRotationMatrix();

        double x_s = xyz_trans_s[0];
        double y_s = xyz_trans_s[1];
        double z_s = xyz_trans_s[2];
        double invz_s = 1.0/xyz_trans_s[2];
        double invz_s_2 = invz_s*invz_s;

        double x_e = xyz_trans_e[0];
        double y_e = xyz_trans_e[1];
        double z_e = xyz_trans_e[2];
        double invz_e = 1.0/xyz_trans_e[2];
        double invz_e_2 = invz_e*invz_e;

        double xp = obs_temp(0);
        double yp = obs_temp(1);
        double xq = obs_temp(2);
        double yq = obs_temp(3);
        double l0 = obs_temp(4);
        double l1 = obs_temp(5);

        Eigen::Vector2d QP;
        QP << xp-xq, yp-yq;
        Eigen::Vector2d QP_pr;
        QP_pr << P_pr(0)-xq, P_pr(1)-yq;
        Eigen::Vector2d PQ_pr;
        PQ_pr << Q_pr(0)-xp, Q_pr(1)-yp;
        Eigen::Vector2d Der_start;
        Eigen::Vector2d Der_end;

        Der_start[0] = ((xp-xq)*QP.norm()*QP_pr.norm()-((xp-xq)*(P_pr(0)-xq)+(yp-yq)*(P_pr(1)-yq))*QP.norm()*(P_pr(0)-xq)/QP_pr.norm())/(QP.norm()*QP.norm()*QP_pr.norm()*QP_pr.norm());
        Der_start[1] = ((yp-yq)*QP.norm()*QP_pr.norm()-((xp-xq)*(P_pr(0)-xq)+(yp-yq)*(P_pr(1)-yq))*QP.norm()*(P_pr(1)-yq)/QP_pr.norm())/(QP.norm()*QP.norm()*QP_pr.norm()*QP_pr.norm());

        Eigen::Matrix<double,2,3> ProjJac_s;

        ProjJac_s << fx*invz_s, 0.f, -fx*x_s*invz_s_2,
                    0.f, fy*invz_s, -fx*y_s*invz_s_2;

        Eigen::Matrix<double,1,3> Jac_s = Der_start.transpose()*ProjJac_s;

        Der_end[0] = ((xp-xq)*QP.norm()*PQ_pr.norm()-((xp-xq)*(Q_pr(0)-xp)+(yp-yq)*(Q_pr(1)-yp))*QP.norm()*(Q_pr(0)-xp)/PQ_pr.norm())/(QP.norm()*QP.norm()*QP_pr.norm()*QP_pr.norm());
        Der_end[1] = ((yp-yq)*QP.norm()*PQ_pr.norm()-((xp-xq)*(Q_pr(0)-xp)+(yp-yq)*(Q_pr(1)-yp))*QP.norm()*(Q_pr(1)-yp)/PQ_pr.norm())/(QP.norm()*QP.norm()*QP_pr.norm()*QP_pr.norm());

        Eigen::Matrix<double,2,3> ProjJac_e;

        ProjJac_e << fx*invz_e, 0.f, -fx*x_e*invz_e_2,
                    0.f, fy*invz_e, -fx*y_e*invz_e_2;

        Eigen::Matrix<double,1,3> Jac_e = Der_end.transpose()*ProjJac_e;
        
        Eigen::Matrix<double,3,6> SE3deriv_s;
        SE3deriv_s << 0.f, z_s, -y_s, 1.f, 0.f, 0.f,
                      -z_s, 0.f, x_s, 0.f, 1.f, 0.f,
                      y_s, -x_s, 0.f, 0.f, 0.f, 1.f;

        Eigen::Matrix<double,3,6> SE3deriv_e;
        SE3deriv_e << 0.f, z_e, -y_e, 1.f, 0.f, 0.f,
                      -z_e, 0.f, x_e, 0.f, 1.f, 0.f,
                      y_e, -x_e, 0.f, 0.f, 0.f, 1.f;

        
        Eigen::Matrix<double,1,3> JacOplusXis;
        Eigen::Matrix<double,1,3> JacOplusXie;
        Eigen::Matrix<double,1,3> JacOplusXjs;
        Eigen::Matrix<double,1,3> JacOplusXje;

        JacOplusXis(0,0) = fx*l0*invz_s*R(0,0)+fy*l1*invz_s*R(1,0)-(fx*x_s*l0*invz_s_2+fy*y_s*l1*invz_s_2)*R(2,0);
        JacOplusXis(0,1) = fx*l0*invz_s*R(0,1)+fy*l1*invz_s*R(1,1)-(fx*x_s*l0*invz_s_2+fy*y_s*l1*invz_s_2)*R(2,1);
        JacOplusXis(0,2) = fx*l0*invz_s*R(0,2)+fy*l1*invz_s*R(1,2)-(fx*x_s*l0*invz_s_2+fy*y_s*l1*invz_s_2)*R(2,2);

        JacOplusXie(0,0) = fx*l0*invz_e*R(0,0)+fy*l1*invz_e*R(1,0)-(fx*x_e*l0*invz_e_2+fy*y_e*l1*invz_e_2)*R(2,0);
        JacOplusXie(0,1) = fx*l0*invz_e*R(0,1)+fy*l1*invz_e*R(1,1)-(fx*x_e*l0*invz_e_2+fy*y_e*l1*invz_e_2)*R(2,1);
        JacOplusXie(0,2) = fx*l0*invz_e*R(0,2)+fy*l1*invz_e*R(1,2)-(fx*x_e*l0*invz_e_2+fy*y_e*l1*invz_e_2)*R(2,2);

        JacOplusXjs(0,0) = -fx*x_s*y_s*invz_s_2*l0-fy*(1+y_s*y_s*invz_s_2)*l1;
        JacOplusXjs(0,1) = fx*(1+x_s*x_s*invz_s_2)*l0+fy*x_s*y_s*invz_s_2*l1;
        JacOplusXjs(0,2) = -fx*y_s*invz_s*l0+fy*x_s*invz_s*l1; 
        JacOplusXjs(0,3) = fx*invz_s*l0;
        JacOplusXjs(0,4) = fy*invz_s*l1;
        JacOplusXjs(0,5) = (-fx*x_s*l0-fy*y_s*l1)*invz_s_2;

        JacOplusXje(0,0) = -fx*x_e*y_e*invz_e_2*l0-fy*(1+y_e*y_e*invz_e_2)*l1;
        JacOplusXje(0,1) = fx*(1+x_e*x_e*invz_e_2)*l0+fy*x_e*y_e*invz_e_2*l1;
        JacOplusXje(0,2) = -fx*y_e*invz_e*l0+fy*x_e*invz_e*l1;  
        JacOplusXje(0,3) = fx*invz_e*l0;
        JacOplusXje(0,4) = fy*invz_e*l1;
        JacOplusXje(0,5) = (-fx*x_e*l0-fy*y_e*l1)*invz_e_2; 

        _jacobianOplusXi << JacOplusXis,
                            JacOplusXie,
                            Jac_s*R,
                            Jac_e*R;

        _jacobianOplusXj << JacOplusXjs,
                            JacOplusXje,
                            Jac_s*SE3deriv_s,
                            Jac_e*SE3deriv_e;
    }

}
