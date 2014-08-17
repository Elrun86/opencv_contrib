/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
// Copyright (C) 2014, Tolga Birdal, all rights reserved.
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
// In no event shall the contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

#ifndef __OPENCV_ICP_HPP__
#define __OPENCV_ICP_HPP__

#include <opencv2/core/version.hpp>
#if CV_MAJOR_VERSION > 2
#include <opencv2/core/utility.hpp>
#else
#include <opencv2/core/core.hpp>
#endif

#include "pose_3d.hpp"
#include "c_utils.hpp"
#include <vector>

using namespace std;

namespace cv 
{
	namespace ppf_match_3d 
	{
		/**
		* @class PPF3DDetector
		* @brief Class, allowing ICP Registration of 3D Shapes
		* Typical Use:
		*
		*   // Train a model
		*   ppf_match_3d::ICP icp(200, 0.005, 3, 8);
		*	// Register for all selected poses. Writes back on results ( vector < Pose3D* > )
		* 	icp.registerModelToScene(pc, pcTest, results); 
		*
		* @author Tolga Birdal
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
    `		*/
			ICP() 
			{
				Tolerence = 0.05;
				RejectionScale = 2.5;
				MaxIterations = 250;
				NumLevels = 6;
				SampleType = 0;
				NumNeighborsCorr = 1;
			}

			~ICP() { }

			/**
			 *  @brief Brief
			 *  @param [in] tolerence Tolerence parameter controls the accuracy of registration at each iteration of ICP.
			 *  @param [in] rejectionScale Robust outlier rejection is applied for robustness. This value actually corresponds to the standard deviation coefficient. Points with rejectionScale * \sigma are ignored during registration.
			 *  @param [in] numLevels Number of pyramid levels to proceed. Deep pyramids increase speed but decrease accuracy. Too coarse pyramids might have computational overhead on top of the inaccurate registrtaion. This parameter should be chosen to optimize a balance. Typical values range from 4 to 10.
			 *  @param [in] sampleType Currently this parameter is ignored and only uniform sampling is applied. Leave it as 0.
			 *  @param [in] numMaxCorr Currently this parameter is ignored and only PickyICP is applied. Leave it as 1.
			 *  \return
			 *  
			 *  \details Constructor
			 */
			ICP(const int iterations, const float tolerence=0.05, const float rejectionScale=2.5, const int numLevels=6, const int sampleType=0, const int numMaxCorr=1) 
			{
				Tolerence = tolerence;
				NumNeighborsCorr = numMaxCorr;
				RejectionScale = rejectionScale;
				MaxIterations = iterations;
				NumLevels = numLevels;
				SampleType = sampleType;
			};

			/**
			 *  @brief Brief
			 *  
			 *  \return Return_Description
			 *  
			 *  \details Details
			 */void operator()(InputArray SrcPC, InputArray DstPC, double& Residual, double Pose[16]) const;
			 
			/**
			 *  @brief Brief
			 *  
			 *  @param [in] SrcPC The input point cloud for the model. Expected to have the normals (Nx6). Currently,
			 *  CV_32F is the only supported data type.
			 *  @param [in] DstPC The input point cloud for the scene. It is assumed that the model is registered on the scene. Scene remains static. Expected to have the normals (Nx6). Currently, CV_32F is the only supported data type.
			 *  @param [out] Residual The output registration error. 
			 *  \return On successful termination, the function returns 0.
			 *  
			 *  \details It is assumed that the model is registered on the scene. Scene remains static, while the model transforms. The output poses transform the models onto the scene. Because of the point to plane minimization, the scene is expected to have the normals available. Expected to have the normals (Nx6).
			 */ 
			int registerModelToScene(const Mat& SrcPC, const Mat& DstPC, double& Residual, double Pose[16]);
			
			/**
			 *  @brief Brief
			 *  
			 *  @param [in] SrcPC The input point cloud for the model. Expected to have the normals (Nx6). Currently,
			 *  CV_32F is the only supported data type.
			 *  @param [in] DstPC The input point cloud for the scene. Currently, CV_32F is the only supported data type.
			 *  @param [out] Poses List output of poses. For more detailed information check out Pose3D.
			 *  \return On successful termination, the function returns 0.
			 *  
			 *  \details It is assumed that the model is registered on the scene. Scene remains static, while the model transforms. The output poses transform the models onto the scene. Because of the point to plane minimization, the scene is expected to have the normals available. Expected to have the normals (Nx6).
			 */ 
			int registerModelToScene(const Mat& SrcPC, const Mat& DstPC, std::vector<Pose3D*>& Poses);

		private:
			float Tolerence;
			int MaxIterations;
			float RejectionScale;
			int NumNeighborsCorr;
			int NumLevels;
			int SampleType;


			

		};
	}
}

#endif