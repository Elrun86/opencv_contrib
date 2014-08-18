//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                          License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2014, OpenCV Foundation, all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
// Author: Tolga Birdal


/**
 * @file icp.hpp
 *
 * @brief     Implementation of ICP (Iterative Closest Point) Algorithm
  * @author    Tolga Birdal
 */

#ifndef __OPENCV_ICP_HPP__
#define __OPENCV_ICP_HPP__

#include <opencv2/core/version.hpp>
#if CV_MAJOR_VERSION > 2
#include <opencv2/core/utility.hpp>
#else
#include <opencv2/core.hpp>
#endif

#include "surface_matching/pose_3d.hpp"
#include <vector>

namespace cv
{
namespace ppf_match_3d
{
/**
* @class ICP
* @brief This class implements a very efficient and robust variant of the iterative closest point (ICP) algorithm.
* The task is to register a 3D model (or point cloud) against a set of noisy target data. The variants are put together
* by myself after certain tests. The task is to be able to match partial, noisy point clouds in cluttered scenes, quickly.
* You will find that my emphasis is on the performance, while retaining the accuracy.
 The main contributions come from:
 1. Picky ICP:
 http://www5.informatik.uni-erlangen.de/Forschung/Publikationen/2003/Zinsser03-ARI.pdf
 2. Efficient variants of the ICP Algorithm:
 http://docs.happycoders.org/orgadoc/graphics/imaging/fasticp_paper.pdf
 3. Geometrically Stable Sampling for the ICP Algorithm: https://graphics.stanford.edu/papers/stabicp/stabicp.pdf
 4. Multi-resolution registration:
 http://www.cvl.iis.u-tokyo.ac.jp/~oishi/Papers/Alignment/Jost_MultiResolutionICP_3DIM03.pdf
 5. Linearization of Point-to-Plane metric by Kok Lim Low:
 https://www.comp.nus.edu.sg/~lowkl/publications/lowk_point-to-plane_icp_techrep.pdf
 Note that the test function requires Gabriel Peyr�'s read_ply m-file in here:
 http://www.mathworks.com/matlabcentral/fileexchange/5355-toolbox-graph/content/toolbox_graph/read_ply.m
* Typical Use:
*/
class CV_EXPORTS ICP
{
    public:
    
        enum ICP_SAMPLING_TYPE
        {
            ICP_SAMPLING_TYPE_UNIFORM, ICP_SAMPLING_TYPE_GELFAND
        };
        
        /*!
        Default constructor
         `  */
        ICP()
        {
            m_tolerence = 0.005f;
            m_rejectionScale = 2.5f;
            m_maxItereations = 250;
            m_numLevels = 6;
            m_sampleType = ICP_SAMPLING_TYPE_UNIFORM;
            m_numNeighborsCorr = 1;
        }
        
        virtual ~ICP() { }
        
        /**
         *  \brief Brief
         *  @param [in] tolerence tolerence parameter controls the accuracy of registration at each iteration of ICP.
         *  @param [in] rejectionScale Robust outlier rejection is applied for robustness. This value actually corresponds to the standard deviation coefficient. Points with rejectionScale * \sigma are ignored during registration.
         *  @param [in] numLevels Number of pyramid levels to proceed. Deep pyramids increase speed but decrease accuracy. Too coarse pyramids might have computational overhead on top of the inaccurate registrtaion. This parameter should be chosen to optimize a balance. Typical values range from 4 to 10.
         *  @param [in] sampleType Currently this parameter is ignored and only uniform sampling is applied. Leave it as 0.
         *  @param [in] numMaxCorr Currently this parameter is ignored and only PickyICP is applied. Leave it as 1.
         *  \return
         *
         *  \details Constructor
         */
        ICP(const int iterations, const float tolerence=0.05, const float rejectionScale=2.5, const int numLevels=6, const ICP_SAMPLING_TYPE sampleType = ICP_SAMPLING_TYPE_UNIFORM, const int numMaxCorr=1)
        {
            m_tolerence = tolerence;
            m_numNeighborsCorr = numMaxCorr;
            m_rejectionScale = rejectionScale;
            m_maxItereations = iterations;
            m_numLevels = numLevels;
            m_sampleType = sampleType;
        };
        
        /**
         *  \brief Perform registration
         *
         *  @param [in] srcPC The input point cloud for the model. Expected to have the normals (Nx6). Currently,
         *  CV_32F is the only supported data type.
         *  @param [in] dstPC The input point cloud for the scene. It is assumed that the model is registered on the scene. Scene remains static. Expected to have the normals (Nx6). Currently, CV_32F is the only supported data type.
         *  @param [out] residual The output registration error.
         *  \return On successful termination, the function returns 0.
         *
         *  \details It is assumed that the model is registered on the scene. Scene remains static, while the model transforms. The output poses transform the models onto the scene. Because of the point to plane minimization, the scene is expected to have the normals available. Expected to have the normals (Nx6).
         */
        int registerModelToScene(const Mat& srcPC, const Mat& dstPC, double& residual, double pose[16]);
        
        /**
         *  \brief Perform registration with multiple initial poses
         *
         *  @param [in] srcPC The input point cloud for the model. Expected to have the normals (Nx6). Currently,
         *  CV_32F is the only supported data type.
         *  @param [in] dstPC The input point cloud for the scene. Currently, CV_32F is the only supported data type.
         *  @param [out] poses List output of poses. For more detailed information check out Pose3D.
         *  \return On successful termination, the function returns 0.
         *
         *  \details It is assumed that the model is registered on the scene. Scene remains static, while the model transforms. The output poses transform the models onto the scene. Because of the point to plane minimization, the scene is expected to have the normals available. Expected to have the normals (Nx6).
         */
        int registerModelToScene(const Mat& srcPC, const Mat& dstPC, std::vector<Pose3D*>& poses);
        
    private:
        float m_tolerence;
        int m_maxItereations;
        float m_rejectionScale;
        int m_numNeighborsCorr;
        int m_numLevels;
        int m_sampleType;
        
};

} // namespace ppf_match_3d

} // namespace cv

#endif
