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

#include "ppf_match_3d.hpp"
#include <iostream>
#include "icp.hpp"

using namespace std;
using namespace cv;
using namespace ppf_match_3d;

static void help(std::string errorMessage)
{
    std::cout<<"Program init error : "<<errorMessage<<std::endl;
    std::cout<<"\nUsage : ppf_matching [input model file] [input scene file]"<<std::endl;    
    std::cout<<"\nPlease start again with new parameters"<<std::endl;    
}

int main(int argc, char** argv)
{
    // welcome message    
    std::cout<< "****************************************************"<<std::endl;    
    std::cout<< "* Surface Matching demonstration : demonstrates the use of surface matching"
             " using point pair features."<<std::endl;
    std::cout<< "* The sample loads a model and a scene, where the model lies in a different"
             " pose than the training."<<std::endl;
    std::cout<< "* The sample loads a model and a scene, where the model lies in a different"
             " pose than the training. It then "<<std::endl;
    std::cout<< "****************************************************"<<std::endl;    
    
    if (argc < 3)    
    {    
        help("Not enough input arguments");        
        exit(1);        
    }
    
    string modelFileName = (string)argv[1];
    string sceneFileName = (string)argv[2];
    
    Mat pc = loadPLYSimple(modelFileName.c_str(), 1);
    
    // Now train the model
    printf("Training...");
    int64 tick1 = cv::getTickCount();
    ppf_match_3d::PPF3DDetector detector(0.03, 0.05);
    detector.trainModel(pc);
    int64 tick2 = cv::getTickCount();
    printf("\nTraining complete in %f ms.\nLoading model...", (double)(tick2-tick1)/ cv::getTickFrequency());
    
    // Read the scene
    Mat pcTest = loadPLYSimple(sceneFileName.c_str(), 1);
    
    // Match the model to the scene and get the pose
    printf("\nStarting matching...");
    vector < Pose3D* > results;
    tick1 = cv::getTickCount();
    detector.match(pcTest, results, 1.0/10.0, 0.05);
    tick2 = cv::getTickCount();
    printf("\nPPF Elapsed Time %f sec", (double)(tick2-tick1)/ cv::getTickFrequency());
    
    // Get only first N results
    int N = 2;
    vector<Pose3D*>::const_iterator first = results.begin();
    vector<Pose3D*>::const_iterator last = results.begin() + N;
    vector<Pose3D*> resultsSub(first, last);
    
    // Create an instance of ICP
    ICP icp(200, 0.001f, 2.5f, 8);
    float residualOutput = 0;
    int64 t1 = cv::getTickCount();
    
    // Register for all selected poses
    printf("\nPerforming ICP on %d poses...", N);
    icp.registerModelToScene(pc, pcTest, resultsSub);
    int64 t2 = cv::getTickCount();
    
    printf("\nElapsed Time on ICP: %f\nEstimated Poses:\n"", (double)(t2-t1)/cv::getTickFrequency());
    
    // debug first five poses
    for (size_t i=0; i<resultsSub.size(); i++)
    {
        Pose3D* pose = resultsSub[i];        
        printf("Pose Result %d:\n", i);
        pose->printPose();
    }
    
    return 0;
}

