/*
 *  Originally from Kangaroo
 *  Steve Lovegrove
 *
 */


#pragma once

#include <Eigen/Eigen>
#include <Mvlpp/Mvl.h>

class CameraModelPyramid : public mvl::CameraModel
{
    public:

        inline CameraModelPyramid() : mvl::CameraModel()
        {
        }

        inline CameraModelPyramid(const std::string& filename)
            : mvl::CameraModel(filename)
        {
            PopulatePyramid();
        }

        inline bool Read(const std::string& filename)
        {
            bool ret = mvl::CameraModel::Read(filename);
            if( ret ) {
                PopulatePyramid();
                return true;
            } else {
                return false;
            }
        }

        inline const Eigen::Matrix<double,3,3>& K(size_t i = 0) const
        {
            return m_K[i];
        }

        inline const Eigen::Matrix<double,3,3>& Kinv(size_t i = 0) const
        {
            return m_Kinv[i];
        }

        inline void PopulatePyramid(int max_levels = 10)
        {
            m_K.clear();
            m_Kinv.clear();
            unsigned level = 0;
            unsigned w = mvl::CameraModel::Width();
            unsigned h = mvl::CameraModel::Height();
            Eigen::Matrix3d K = mvl::CameraModel::K();

            while(level <= max_levels && w > 0 && h > 0)
            {
                m_K.push_back(K);
                m_Kinv.push_back(MakeKinv(K));
                level++;
                w = w/2;
                h = h/2;
                const Eigen::Matrix3d nk = ScaleK(K, 0.5);
                K = nk;
            }
        }


    private:

        inline Eigen::Matrix3d MakeKinv(const Eigen::Matrix3d& K)
        {
            Eigen::Matrix3d Kinv = Eigen::Matrix3d::Identity();
            Kinv << 1.0/K(0,0), 0, - K(0,2) / K(0,0),
                    0, 1.0/K(1,1), - K(1,2) / K(1,1),
                    0,0,1;
            return Kinv;
        }

        inline Eigen::Matrix3d ScaleK(const Eigen::Matrix3d& K, double imageScale)
        {
            Eigen::Matrix3d rK = K;
            rK(0,0) *= imageScale;
            rK(1,1) *= imageScale;
            rK(0,2) = imageScale * (K(0,2)+0.5) - 0.5;
            rK(1,2) = imageScale * (K(1,2)+0.5) - 0.5;
            return rK;
        }


    protected:
        std::vector<Eigen::Matrix3d> m_K;
        std::vector<Eigen::Matrix3d> m_Kinv;
};

