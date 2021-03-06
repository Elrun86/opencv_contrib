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

#include "precomp.hpp"
#include "ppf_match_3d.hpp"
#include "hash_murmur.hpp"

#if defined (T_OPENMP)
#include<omp.h>
#endif


namespace cv
{
namespace ppf_match_3d
{
// routines for assisting sort
static int qsortPoseCmp (const void * a, const void * b)
{
    Pose3D* pose1 = *(Pose3D**)a;
    Pose3D* pose2 = *(Pose3D**)b;
    return ( pose2->numVotes - pose1->numVotes );
}

static int sortPoseClusters (const PoseCluster3D* a, const PoseCluster3D* b)
{
    return ( a->numVotes > b->numVotes );
}

// simple hashing
static int hashPPFSimple(const double f[4], const double AngleStep, const double DistanceStep)
{
    const unsigned char d1 = (unsigned char) (floor ((double)f[0] / (double)AngleStep));
    const unsigned char d2 = (unsigned char) (floor ((double)f[1] / (double)AngleStep));
    const unsigned char d3 = (unsigned char) (floor ((double)f[2] / (double)AngleStep));
    const unsigned char d4 = (unsigned char) (floor ((double)f[3] / (double)DistanceStep));
    
    int hashKey = (d1 | (d2<<8) | (d3<<16) | (d4<<24));
    return hashKey;
}

// quantize ppf and hash it for proper indexing
static int hashPPF(const double f[4], const double AngleStep, const double DistanceStep)
{
    const int d1 = (int) (floor ((double)f[0] / (double)AngleStep));
    const int d2 = (int) (floor ((double)f[1] / (double)AngleStep));
    const int d3 = (int) (floor ((double)f[2] / (double)AngleStep));
    const int d4 = (int) (floor ((double)f[3] / (double)DistanceStep));
    int key[4]={d1,d2,d3,d4};
    int hashKey=0;
    
    hashMurmurx86(key, 4*sizeof(int), 42, &hashKey);
    
    return hashKey;
}

static size_t hashMurmur(unsigned int key)
{
    size_t hashKey=0;
    hashMurmurx86((void*)&key, 4, 42, &hashKey);
    return hashKey;
}

static double computeAlpha(const double p1[4], const double n1[4], const double p2[4])
{
    double Tmg[3], mpt[3], row2[3], row3[3], alpha;
    
    computeTransformRTyz(p1, n1, row2, row3, Tmg);
    
    // checked row2, row3: They are correct
    
    mpt[1] = Tmg[1] + row2[0] * p2[0] + row2[1] * p2[1] + row2[2] * p2[2];
    mpt[2] = Tmg[2] + row3[0] * p2[0] + row3[1] * p2[1] + row3[2] * p2[2];
    
    alpha=atan2(-mpt[2], mpt[1]);
    
    if ( alpha != alpha)
    {
        return 0;
    }
    
    if (sin(alpha)*mpt[2]<0.0)
        alpha=-alpha;
        
    return (-alpha);
}

PPF3DDetector::PPF3DDetector()
{
    samplingStepRelative = 0.05;
    distanceStepRelative = 0.05;
    SceneSampleStep = 1/0.04;
    angleStepRelative = 30;
    angleStepRadians = (360.0/angleStepRelative)*PI/180.0;
    angle_step = angleStepRadians;
    trained = false;
    
    SetSearchParams();
}

PPF3DDetector::PPF3DDetector(const double RelativeSamplingStep, const double RelativeDistanceStep, const double NumAngles)
{
    samplingStepRelative = RelativeSamplingStep;
    distanceStepRelative = RelativeDistanceStep;
    angleStepRelative = NumAngles;
    angleStepRadians = (360.0/angleStepRelative)*PI/180.0;
    //SceneSampleStep = 1.0/RelativeSceneSampleStep;
    angle_step = angleStepRadians;
    trained = false;
    
    SetSearchParams();
}

void PPF3DDetector::SetSearchParams(const int numPoses, const double positionThreshold, const double rotationThreshold, const double minMatchScore, const bool useWeightedClustering)
{
    NumPoses=numPoses;
    
    if (positionThreshold<0)
        PositionThreshold = samplingStepRelative;
    else
        PositionThreshold = positionThreshold;
        
    if (rotationThreshold<0)
        RotationThreshold = ((360/angle_step) / 180.0 * M_PI);
    else
        RotationThreshold = rotationThreshold;
        
    UseWeightedAvg = useWeightedClustering;
    MinMatchScore = minMatchScore;
}

// compute per point PPF as in paper
void PPF3DDetector::computePPFFeatures( const double p1[4], const double n1[4],
                                        const double p2[4], const double n2[4],
                                        double f[4])
{
    /*
        Vectors will be defined as of length 4 instead of 3, because of:
        - Further SIMD vectorization
        - Cache alignment
        */
    
    double d[4] = {p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2], 0};
    double c[4];
    
    double norm = TNorm3(d);
    f[3] = norm;
    
    if (norm)
    {
        d[0] /= f[3];
        d[1] /= f[3];
        d[2] /= f[3];
    }
    else
    {
        // TODO: Handle this
        f[0] = 0;
        f[1] = 0;
        f[2] = 0;
        return ;
    }
    
    /*
    Tolga Birdal's note:
    Issues of numerical stability is of concern here.
    Bertram's suggestion: atan2(a dot b, |axb|)
    My correction :
    I guess it should be: angle = atan2(norm(cross(a,b)), dot(a,b))
    The macro is implemented accordingly.
    TAngle3 actually outputs in range [0, pi] as
    Bertram suggests
    */
    
    f[0] = TAngle3(n1, d);
    f[1] = TAngle3(n2, d);
    f[2] = TAngle3(n1, n2);
}

void PPF3DDetector::clearTrainingModels()
{
    if (this->hash_nodes)
    {
        free(this->hash_nodes);
        this->hash_nodes=0;
    }
    
    if (this->hash_table)
    {
        hashtableDestroy(this->hash_table);
        this->hash_table=0;
    }
}

PPF3DDetector::~PPF3DDetector()
{
    clearTrainingModels();
}

// TODO: Check all step sizes to be positive
int PPF3DDetector::trainModel(const Mat &PC)
{
    CV_Assert(PC.type() == CV_32F || PC.type() == CV_32FC1);
    
    const int numPoints = PC.rows;
    
    // compute bbox
    float xRange[2], yRange[2], zRange[2];
    computeBboxStd(PC, xRange, yRange, zRange);
    
    // compute sampling step from diameter of bbox
    float dx = xRange[1] - xRange[0];
    float dy = yRange[1] - yRange[0];
    float dz = zRange[1] - zRange[0];
    float diameter = sqrt ( dx * dx + dy * dy + dz * dz );
    
    float distanceStep = diameter * samplingStepRelative;
    
    Mat sampled = samplePCByQuantization(PC, xRange, yRange, zRange, samplingStepRelative,0);
    
    int size = sampled.rows*sampled.rows;
    
    hashtable_int* hashTable = hashtableCreate(size, NULL);
    
    int numPPF = sampled.rows*sampled.rows;
    PPF = Mat(numPPF, T_PPF_LENGTH, CV_32FC1);
    int ppfStep = PPF.step;
    int sampledStep = sampled.step;
    
    // TODO: Maybe I could sample 1/5th of them here. Check the performance later.
    int numRefPoints = sampled.rows;
    
    // pre-allocate the hash nodes
    hash_nodes = (THash*)calloc(numRefPoints*numRefPoints, sizeof(THash));
    
    // TODO : This can easily be parallelized. But we have to lock hashtable_insert.
    // I realized that performance drops when this loop is parallelized (unordered
    // inserts into the hashtable
    // But it is still there to be investigated. For now, I leave this unparallelized
    // since this is just a training part.
    for (int i=0; i<numRefPoints; i++)
    {
        float* f1 = (float*)(&sampled.data[i * sampledStep]);
        const double p1[4] = {f1[0], f1[1], f1[2], 0};
        const double n1[4] = {f1[3], f1[4], f1[5], 0};
        
        //printf("///////////////////// NEW REFERENCE ////////////////////////\n");
        for (int j=0; j<numRefPoints; j++)
        {
            // cannnot compute the ppf with myself
            if (i!=j)
            {
                float* f2 = (float*)(&sampled.data[j * sampledStep]);
                const double p2[4] = {f2[0], f2[1], f2[2], 0};
                const double n2[4] = {f2[3], f2[4], f2[5], 0};
                
                double f[4]={0};
                computePPFFeatures(p1, n1, p2, n2, f);
                unsigned int hashValue = hashPPF(f, angleStepRadians, distanceStep);
                double alpha = computeAlpha(p1, n1, p2);
                unsigned int corrInd = i*numRefPoints+j;
                unsigned int ppfInd = corrInd*ppfStep;
                
                THash* hashNode = &hash_nodes[i*numRefPoints+j];
                hashNode->id = hashValue;
                hashNode->i = i;
                hashNode->ppfInd = ppfInd;
                
                hashtableInsertHashed(hashTable, hashValue, (void*)hashNode);
                
                float* ppfRow = (float*)(&(PPF.data[ ppfInd ]));
                ppfRow[0] = f[0];
                ppfRow[1] = f[1];
                ppfRow[2] = f[2];
                ppfRow[3] = f[3];
                ppfRow[4] = (float)alpha;
            }
        }
    }
    
    angle_step = angleStepRadians;
    distance_step = distanceStep;
    hash_table = hashTable;
    sampled_step = sampledStep;
    ppf_step = ppfStep;
    num_ref_points = numRefPoints;
    //samplingStepRelative = sampling_step_relative;
    sampledPC = sampled;
    trained = true;
    
    return 0;
}



///////////////////////// MATCHING ////////////////////////////////////////


bool PPF3DDetector::matchPose(const Pose3D& sourcePose, const Pose3D& targetPose)
{
    // translational difference
    const double* Pose = sourcePose.Pose;
    const double* PoseT = targetPose.Pose;
    double dv[3] = {targetPose.t[0]-sourcePose.t[0], targetPose.t[1]-sourcePose.t[1], targetPose.t[2]-sourcePose.t[2]};
    double dNorm = sqrt(dv[0]*dv[0]+dv[1]*dv[1]+dv[2]*dv[2]);
    
    const double phi = fabs ( sourcePose.angle - targetPose.angle );
    
    return (phi<this->RotationThreshold && dNorm < this->PositionThreshold);
}

int PPF3DDetector::clusterPoses(Pose3D** poseList, int NumPoses, vector < Pose3D* >& finalPoses)
{
    vector<PoseCluster3D*> poseClusters;
    poseClusters.clear();
    
    finalPoses.clear();
    
    // sort the poses for stability
    qsort(poseList, NumPoses, sizeof(Pose3D*), qsortPoseCmp);
    
    for (int i=0; i<NumPoses; i++)
    {
        Pose3D* pose = poseList[i];
        bool assigned = false;
        
        // search all clusters
        for (int j=0; j<poseClusters.size() && !assigned; j++)
        {
            const Pose3D* poseCenter = poseClusters[j]->poseList[0];
            if (matchPose(*pose, *poseCenter))
            {
                poseClusters[j]->addPose(pose);
                assigned = true;
            }
        }
        
        if (!assigned)
        {
            poseClusters.push_back ( new PoseCluster3D(pose));
        }
    }
    
    // sort the clusters so that we could output multiple hypothesis
    std::sort (poseClusters.begin(), poseClusters.end(), sortPoseClusters);
    
    finalPoses.resize(poseClusters.size());
    
    // TODO: Use MinMatchScore
    
    if (UseWeightedAvg)
    {
#if defined T_OPENMP
#pragma omp parallel for
#endif
        // uses weighting by the number of votes
        for (int i=0; i<poseClusters.size(); i++)
        {
            // We could only average the quaternions. So I will make use of them here
            double qAvg[4]={0}, tAvg[3]={0}, R[9]={0}, Pose[16]={0};
            
            // Perform the final averaging
            PoseCluster3D* curCluster = poseClusters[i];
            vector<Pose3D*> curPoses = curCluster->poseList;
            const int curSize = curPoses.size();
            int numTotalVotes = 0;
            
            for (int j=0; j<curSize; j++)
                numTotalVotes += curPoses[j]->numVotes;
                
            double wSum=0;
            
            for (int j=0; j<curSize; j++)
            {
                const double w = (double)curPoses[j]->numVotes / (double)numTotalVotes;
                
                qAvg[0]+= w*curPoses[j]->q[0];
                qAvg[1]+= w*curPoses[j]->q[1];
                qAvg[2]+= w*curPoses[j]->q[2];
                qAvg[3]+= w*curPoses[j]->q[3];
                
                tAvg[0]+= w*curPoses[j]->t[0];
                tAvg[1]+= w*curPoses[j]->t[1];
                tAvg[2]+= w*curPoses[j]->t[2];
                wSum+=w;
            }
            
            tAvg[0]/=wSum;
            tAvg[1]/=wSum;
            tAvg[2]/=wSum;
            
            qAvg[0]/=wSum;
            qAvg[1]/=wSum;
            qAvg[2]/=wSum;
            qAvg[3]/=wSum;
            
            curPoses[0]->updatePoseQuat(qAvg, tAvg);
            curPoses[0]->numVotes=curCluster->numVotes;
            
            finalPoses[i]=curPoses[0]->clone();
            
            delete poseClusters[i];
        }
    }
    else
    {
#if defined T_OPENMP
#pragma omp parallel for
#endif
        for (int i=0; i<poseClusters.size(); i++)
        {
            // We could only average the quaternions. So I will make use of them here
            double qAvg[4]={0}, tAvg[3]={0}, R[9]={0}, Pose[16]={0};
            
            // Perform the final averaging
            PoseCluster3D* curCluster = poseClusters[i];
            vector<Pose3D*> curPoses = curCluster->poseList;
            const int curSize = curPoses.size();
            
            for (int j=0; j<curSize; j++)
            {
                qAvg[0]+= curPoses[j]->q[0];
                qAvg[1]+= curPoses[j]->q[1];
                qAvg[2]+= curPoses[j]->q[2];
                qAvg[3]+= curPoses[j]->q[3];
                
                tAvg[0]+= curPoses[j]->t[0];
                tAvg[1]+= curPoses[j]->t[1];
                tAvg[2]+= curPoses[j]->t[2];
            }
            
            tAvg[0]/=(double)curSize;
            tAvg[1]/=(double)curSize;
            tAvg[2]/=(double)curSize;
            
            qAvg[0]/=(double)curSize;
            qAvg[1]/=(double)curSize;
            qAvg[2]/=(double)curSize;
            qAvg[3]/=(double)curSize;
            
            curPoses[0]->updatePoseQuat(qAvg, tAvg);
            curPoses[0]->numVotes=curCluster->numVotes;
            
            finalPoses[i]=curPoses[0]->clone();
            
            // we won't need this
            delete poseClusters[i];
        }
    }
    
    poseClusters.clear();
    
    return 0;
}

void PPF3DDetector::match(const Mat& pc, vector < Pose3D* >& results, const double RelativeSceneSampleStep, const double RelativeSceneDistance)
{
    if (!trained)
    {
        throw cv::Exception(cv::Error::StsError, "The model is not trained. Cannot match without training", __FUNCTION__, __FILE__, __LINE__);
    }
    
    CV_Assert(pc.type() == CV_32F || pc.type() == CV_32FC1);
    
    SceneSampleStep = 1.0/RelativeSceneSampleStep;
    
    int i;
    int numNeighbors = 1000;
    int numAngles = (int) (floor (2 * M_PI / angle_step));
    float angleStepRadians = angle_step;
    float distanceStep = distance_step;
    int sampledStep = sampled_step;
    int ppfStep = ppf_step;
    int numRefPoints = num_ref_points;
    unsigned int n = num_ref_points;
    Pose3D** poseList;
    int sceneSamplingStep = SceneSampleStep, c = 0;
    
    // compute bbox
    float xRange[2], yRange[2], zRange[2];
    computeBboxStd(pc, xRange, yRange, zRange);
    
    // sample the point cloud
    float dx = xRange[1] - xRange[0];
    float dy = yRange[1] - yRange[0];
    float dz = zRange[1] - zRange[0];
    float diameter = sqrt ( dx * dx + dy * dy + dz * dz );
    float distanceSampleStep = diameter * RelativeSceneDistance;
    Mat sampled = samplePCByQuantization(pc, xRange, yRange, zRange, RelativeSceneDistance,1);
    
    // allocate the accumulator : Moved this to the inside of the loop
    /*#if !defined (T_OPENMP)
       unsigned int* accumulator = (unsigned int*)calloc(numAngles*n, sizeof(unsigned int));
    #endif*/
    
    poseList = (Pose3D**)calloc((sampled.rows/sceneSamplingStep)+4, sizeof(Pose3D*));
    
#if defined T_OPENMP
#pragma omp parallel for
#endif
    for (i = 0; i < sampled.rows; i += sceneSamplingStep)
    {
        unsigned int refIndMax = 0, alphaIndMax = 0;
        unsigned int maxVotes = 0;
        
        int j;
        
        float* f1 = (float*)(&sampled.data[i * sampled.step]);
        const double p1[4] = {f1[0], f1[1], f1[2], 0};
        const double n1[4] = {f1[3], f1[4], f1[5], 0};
        double p1t[4];
        double *row1, *row2, *row3, tsg[3]={0}, Rsg[9]={0}, RInv[9]={0};
        
        unsigned int* accumulator = (unsigned int*)calloc(numAngles*n, sizeof(unsigned int));
        computeTransformRT(p1, n1, Rsg, tsg);
        row1=&Rsg[0];
        row2=&Rsg[3];
        row3=&Rsg[6];
        
        // Tolga Birdal's notice:
        // As a later update, we might want to look into a local neighborhood only
        // To do this, simply search the local neighborhood by radius look up
        // and collect the neighbors to compute the relative pose
        
        for (j = 0; j < sampled.rows; j ++)
        {
            if (i!=j)
            {
                float* f2 = (float*)(&sampled.data[j * sampled.step]);
                const double p2[4] = {f2[0], f2[1], f2[2], 0};
                const double n2[4] = {f2[3], f2[4], f2[5], 0};
                double p2t[4], alpha_scene;
                
                double f[4]={0};
                computePPFFeatures(p1, n1, p2, n2, f);
                unsigned int hashValue = hashPPF(f, angleStepRadians, distanceStep);
                
                // we don't need to call this here, as we already estimate the tsg from scene reference point
                // double alpha = computeAlpha(p1, n1, p2);
                p2t[1] = tsg[1] + row2[0] * p2[0] + row2[1] * p2[1] + row2[2] * p2[2];
                p2t[2] = tsg[2] + row3[0] * p2[0] + row3[1] * p2[1] + row3[2] * p2[2];
                
                alpha_scene=atan2(-p2t[2], p2t[1]);
                
                if ( alpha_scene != alpha_scene)
                {
                    continue;
                }
                
                if (sin(alpha_scene)*p2t[2]<0.0)
                    alpha_scene=-alpha_scene;
                    
                alpha_scene=-alpha_scene;
                
                hashnode_i* node = hashtableGetBucketHashed(hash_table, (hashValue));
                
                while (node)
                {
                    THash* tData = (THash*) node->data;
                    int corrI = (int)tData->i;
                    int ppfInd = (int)tData->ppfInd;
                    float* ppfCorrScene = (float*)(&PPF.data[ppfInd]);
                    double alpha_model = (double)ppfCorrScene[T_PPF_LENGTH-1];
                    double alpha = alpha_model - alpha_scene;
                    
                    /*  Tolga Birdal's note: Map alpha to the indices:
                    atan2 generates results in (-pi pi]
                    That's why alpha should be in range [-2pi 2pi]
                    So the quantization would be :
                    numAngles * (alpha+2pi)/(4pi)
                    */
                    
                    //printf("%f\n", alpha);
                    int alpha_index = (int)(numAngles*(alpha + 2*PI) / (4*PI));
                    
                    unsigned int accIndex = corrI * numAngles + alpha_index;
                    
                    accumulator[accIndex]++;
                    node = node->next;
                }
            }
        }
        
        // Maximize the accumulator
        for (int k = 0; k < n; k++)
        {
            for (int j = 0; j < numAngles; j++)
            {
                const unsigned int accInd = k*numAngles + j;
                const unsigned int accVal = accumulator[ accInd ];
                if (accVal > maxVotes)
                {
                    maxVotes = accVal;
                    refIndMax = k;
                    alphaIndMax = j;
                }
                
#if !defined (T_OPENMP)
                accumulator[accInd ] = 0;
#endif
            }
        }
        
        // invert Tsg : Luckily rotation is orthogonal: Inverse = Transpose.
        // We are not required to invert.
        double tInv[3], tmg[3], Rmg[9], Ralpha[9];
        matrixTranspose33(Rsg, RInv);
        matrixProduct331(RInv, tsg, tInv);
        
        double TsgInv[16] = { RInv[0], RInv[1], RInv[2], -tInv[0],
                              RInv[3], RInv[4], RInv[5], -tInv[1],
                              RInv[6], RInv[7], RInv[8], -tInv[2],
                              0, 0, 0, 1
                            };
                            
        // TODO : Compute pose
        const float* fMax = (float*)(&sampledPC.data[refIndMax * sampledPC.step]);
        const double pMax[4] = {fMax[0], fMax[1], fMax[2], 1};
        const double nMax[4] = {fMax[3], fMax[4], fMax[5], 1};
        double pose[4][4];
        
        computeTransformRT(pMax, nMax, Rmg, tmg);
        row1=&Rsg[0];
        row2=&Rsg[3];
        row3=&Rsg[6];
        
        double Tmg[16] = { Rmg[0], Rmg[1], Rmg[2], tmg[0],
                           Rmg[3], Rmg[4], Rmg[5], tmg[1],
                           Rmg[6], Rmg[7], Rmg[8], tmg[2],
                           0, 0, 0, 1
                         };
                         
        // convert alpha_index to alpha
        int alpha_index = alphaIndMax;
        double alpha = (alpha_index*(4*PI))/numAngles-2*PI;
        
        // Equation 2:
        double Talpha[16]={0};
        getUnitXRotation_44(alpha, Talpha);
        
        double Temp[16]={0};
        double Pose[16]={0};
        matrixProduct44(Talpha, Tmg, Temp);
        matrixProduct44(TsgInv, Temp, Pose);
        
        Pose3D *ppf = new Pose3D(alpha, refIndMax, maxVotes);
        
        ppf->updatePose(Pose);
        
        poseList[i/sceneSamplingStep] = ppf;
        
#if defined (T_OPENMP)
        free(accumulator);
#endif
    }
    
    // TODO : Make the parameters relative if not arguments.
    double MinMatchScore = 0.5;
    
    int numPosesAdded = sampled.rows/sceneSamplingStep;
    
    clusterPoses(poseList, numPosesAdded, results);
    
    // free up the used space
    sampled.release();
    
    for (int i=0; i<numPosesAdded; i++)
    {
        Pose3D* pose = poseList[i];
        delete pose;
        poseList[i]=0;
    }
    
    free(poseList);
    /*#if !defined (T_OPENMP)
       free(accumulator);
    #endif*/
}

}

}