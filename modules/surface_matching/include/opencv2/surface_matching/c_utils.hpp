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

#ifndef __OPENCV_PPF_UTILS_HPP_
#define __OPENCV_PPF_UTILS_HPP_

#include <math.h>
#include <stdio.h>

namespace cv
{
namespace ppf_match_3d
{

#define EPS  1.192092896e-07F        /* smallest such that 1.0+FLT_EPSILON != 1.0 */

//#define T_OPENMP // define this if OpenMP is desired

#ifndef PI
#ifdef  M_PI
#define PI               M_PI
#define PI_F             3.1415926535897932384626433832795f
#else
#define PI               3.1415926535897932384626433832795
#define PI_F             3.1415926535897932384626433832795f
#endif
#endif

#ifndef M_PI
#define M_PI             PI
#endif

#ifndef MIN
#define MIN(a,b)  ((a) > (b) ? (b) : (a))
#endif

#ifndef MAX
#define MAX(a,b)  ((a) < (b) ? (b) : (a))
#endif

static inline double TNorm3(const double v[])
{
    return (sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]));
}

static inline double TNormalize3(double v[])
{
    double normTemp=TNorm3(v);
    if (normTemp>0)
    {
        v[0]/=normTemp;
        v[1]/=normTemp;
        v[2]/=normTemp;
    }
}

static inline double TDot3(const double a[3], const double b[3])
{
    return  ((a[0])*(b[0])+(a[1])*(b[1])+(a[2])*(b[2]));
}

static inline void TCross(const double a[], const double b[], double c[])
{
    c[0] = (a[1])*(b[2])-(a[2])*(b[1]);
    c[1] = (a[2])*(b[0])-(a[0])*(b[2]);
    c[2] = (a[0])*(b[1])-(a[1])*(b[0]);
}

static inline double TAngle3(const double a[3], const double b[3])
{
    double c[3];
    TCross(a,b,c);
    return (atan2(TNorm3(c), TDot3(a, b)));
}

static inline void matrixProduct33(double *A, double *B, double *R)
{
    R[0] = A[0] * B[0] + A[1] * B[3] + A[2] * B[6];
    R[1] = A[0] * B[1] + A[1] * B[4] + A[2] * B[7];
    R[2] = A[0] * B[2] + A[1] * B[5] + A[2] * B[8];
    
    R[3] = A[3] * B[0] + A[4] * B[3] + A[5] * B[6];
    R[4] = A[3] * B[1] + A[4] * B[4] + A[5] * B[7];
    R[5] = A[3] * B[2] + A[4] * B[5] + A[5] * B[8];
    
    R[6] = A[6] * B[0] + A[7] * B[3] + A[8] * B[6];
    R[7] = A[6] * B[1] + A[7] * B[4] + A[8] * B[7];
    R[8] = A[6] * B[2] + A[7] * B[5] + A[8] * B[8];
}

// A is a vector
static inline void matrixProduct133(double *A, double *B, double *R)
{
    R[0] = A[0] * B[0] + A[1] * B[3] + A[2] * B[6];
    R[1] = A[0] * B[1] + A[1] * B[4] + A[2] * B[7];
    R[2] = A[0] * B[2] + A[1] * B[5] + A[2] * B[8];
}

static inline void matrixProduct331(const double A[9], const double b[3], double r[3])
{
    r[0] = A[0] * b[0] + A[1] * b[1] + A[2] * b[2];
    r[1] = A[3] * b[0] + A[4] * b[1] + A[5] * b[2];
    r[2] = A[6] * b[0] + A[7] * b[1] + A[8] * b[2];
}

static inline void matrixTranspose33(double *A, double *At)
{
    At[0] = A[0];
    At[4] = A[4];
    At[8] = A[8];
    At[1] = A[3];
    At[2] = A[6];
    At[3] = A[1];
    At[5] = A[7];
    At[6] = A[2];
    At[7] = A[5];
}

static inline void matrixProduct44(const double A[16], const double B[16], double R[16])
{
    R[0] = A[0] * B[0] + A[1] * B[4] + A[2] * B[8] + A[3] * B[12];
    R[1] = A[0] * B[1] + A[1] * B[5] + A[2] * B[9] + A[3] * B[13];
    R[2] = A[0] * B[2] + A[1] * B[6] + A[2] * B[10] + A[3] * B[14];
    R[3] = A[0] * B[3] + A[1] * B[7] + A[2] * B[11] + A[3] * B[15];
    
    R[4] = A[4] * B[0] + A[5] * B[4] + A[6] * B[8] + A[7] * B[12];
    R[5] = A[4] * B[1] + A[5] * B[5] + A[6] * B[9] + A[7] * B[13];
    R[6] = A[4] * B[2] + A[5] * B[6] + A[6] * B[10] + A[7] * B[14];
    R[7] = A[4] * B[3] + A[5] * B[7] + A[6] * B[11] + A[7] * B[15];
    
    R[8] = A[8] * B[0] + A[9] * B[4] + A[10] * B[8] + A[11] * B[12];
    R[9] = A[8] * B[1] + A[9] * B[5] + A[10] * B[9] + A[11] * B[13];
    R[10] = A[8] * B[2] + A[9] * B[6] + A[10] * B[10] + A[11] * B[14];
    R[11] = A[8] * B[3] + A[9] * B[7] + A[10] * B[11] + A[11] * B[15];
    
    R[12] = A[12] * B[0] + A[13] * B[4] + A[14] * B[8] + A[15] * B[12];
    R[13] = A[12] * B[1] + A[13] * B[5] + A[14] * B[9] + A[15] * B[13];
    R[14] = A[12] * B[2] + A[13] * B[6] + A[14] * B[10] + A[15] * B[14];
    R[15] = A[12] * B[3] + A[13] * B[7] + A[14] * B[11] + A[15] * B[15];
}

static inline void matrixProduct441(const double A[16], const double B[4], double R[4])
{
    R[0] = A[0] * B[0] + A[1] * B[1] + A[2] * B[2] + A[3] * B[3];
    R[1] = A[4] * B[0] + A[5] * B[1] + A[6] * B[2] + A[7] * B[3];
    R[2] = A[8] * B[0] + A[9] * B[1] + A[10] * B[2] + A[11] * B[3];
    R[3] = A[12] * B[0] + A[13] * B[1] + A[14] * B[2] + A[15] * B[3];
}

static inline void matrixPrint(double *A, int m, int n)
{
    int i, j;
    
    for (i = 0; i < m; i++)
    {
        printf("  ");
        for (j = 0; j < n; j++)
        {
            printf(" %0.6f ", A[i * n + j]);
        }
        printf("\n");
    }
}

static inline void matrixIdentity(int n, double *A)
{
    int i;
    
    for (i = 0; i < n*n; i++)
    {
        A[i] = 0.0;
    }
    
    for (i = 0; i < n; i++)
    {
        A[i * n + i] = 1.0;
    }
}

static inline void rtToPose(const double R[9], const double t[3], double Pose[16])
{
    Pose[0]=R[0];
    Pose[1]=R[1];
    Pose[2]=R[2];
    Pose[4]=R[3];
    Pose[5]=R[4];
    Pose[6]=R[5];
    Pose[8]=R[6];
    Pose[9]=R[7];
    Pose[10]=R[8];
    Pose[3]=t[0];
    Pose[7]=t[1];
    Pose[11]=t[2];
    
    Pose[15] = 1;
}


static inline void poseToRT(const double Pose[16], double R[9], double t[3])
{
    R[0] = Pose[0];
    R[1] = Pose[1];
    R[2] = Pose[2];
    R[3] = Pose[4];
    R[4] = Pose[5];
    R[5] = Pose[6];
    R[6] = Pose[8];
    R[7] = Pose[9];
    R[8] = Pose[10];
    
    t[0]=Pose[3];
    t[1]=Pose[7];
    t[2]=Pose[11];
}

static inline void poseToR(const double Pose[16], double R[9])
{
    R[0] = Pose[0];
    R[1] = Pose[1];
    R[2] = Pose[2];
    R[3] = Pose[4];
    R[4] = Pose[5];
    R[5] = Pose[6];
    R[6] = Pose[8];
    R[7] = Pose[9];
    R[8] = Pose[10];
}

/**
 *  \brief Axis angle to rotation but only compute y and z components
 */
static inline void aaToRyz(double angle, const double r[3], double row2[3], double row3[3])
{
    const double sinA=sin(angle);
    const double cosA=cos(angle);
    const double cos1A=(1-cosA);
    
    row2[0] =  0.f;
    row2[1] = cosA;
    row2[2] =  0.f;
    row3[0] =  0.f;
    row3[1] =  0.f;
    row3[2] = cosA;
    
    row2[0] +=  r[2] * sinA;
    row2[2] += -r[0] * sinA;
    row3[0] += -r[1] * sinA;
    row3[1] +=  r[0] * sinA;
    
    row2[0] += r[1] * r[0] * cos1A;
    row2[1] += r[1] * r[1] * cos1A;
    row2[2] += r[1] * r[2] * cos1A;
    row3[0] += r[2] * r[0] * cos1A;
    row3[1] += r[2] * r[1] * cos1A;
    row3[2] += r[2] * r[2] * cos1A;
}

/**
 *  \brief Axis angle to rotation
 */
static inline void aaToR(double angle, const double r[3], double R[9])
{
    const double sinA=sin(angle);
    const double cosA=cos(angle);
    const double cos1A=(1-cosA);
    double *row1 = &R[0];
    double *row2 = &R[3];
    double *row3 = &R[6];
    
    row1[0] =  cosA;
    row1[1] = 0.0f;
    row1[2] =  0.f;
    row2[0] =  0.f;
    row2[1] = cosA;
    row2[2] =  0.f;
    row3[0] =  0.f;
    row3[1] =  0.f;
    row3[2] = cosA;
    
    row1[1] += -r[2] * sinA;
    row1[2] +=  r[1] * sinA;
    row2[0] +=  r[2] * sinA;
    row2[2] += -r[0] * sinA;
    row3[0] += -r[1] * sinA;
    row3[1] +=  r[0] * sinA;
    
    row1[0] += r[0] * r[0] * cos1A;
    row1[1] += r[0] * r[1] * cos1A;
    row1[2] += r[0] * r[2] * cos1A;
    row2[0] += r[1] * r[0] * cos1A;
    row2[1] += r[1] * r[1] * cos1A;
    row2[2] += r[1] * r[2] * cos1A;
    row3[0] += r[2] * r[0] * cos1A;
    row3[1] += r[2] * r[1] * cos1A;
    row3[2] += r[2] * r[2] * cos1A;
}

/**
 *  \brief Compute a rotation in order to rotate around X direction
 */
static inline void getUnitXRotation(double angle, double R[9])
{
    const double sinA=sin(angle);
    const double cosA=cos(angle);
    double *row1 = &R[0];
    double *row2 = &R[3];
    double *row3 = &R[6];
    
    row1[0] =  1;
    row1[1] = 0.0f;
    row1[2] =  0.f;
    row2[0] =  0.f;
    row2[1] = cosA;
    row2[2] =  -sinA;
    row3[0] =  0.f;
    row3[1] =  sinA;
    row3[2] = cosA;
}
/**
 *  \brief Compute a transformation in order to rotate around X direction
 */
static inline void getUnitXRotation_44(double angle, double T[16])
{
    const double sinA=sin(angle);
    const double cosA=cos(angle);
    double *row1 = &T[0];
    double *row2 = &T[4];
    double *row3 = &T[8];
    
    row1[0] =  1;
    row1[1] = 0.0f;
    row1[2] =  0.f;
    row2[0] =  0.f;
    row2[1] = cosA;
    row2[2] =  -sinA;
    row3[0] =  0.f;
    row3[1] =  sinA;
    row3[2] = cosA;
    
    row1[3]=0;
    row2[3]=0;
    row3[3]=0;
    T[3]=0;
    T[7]=0;
    T[11]=0;
    T[15] = 1;
}

/**
 *  \brief Compute the yz components of the transformation needed to rotate n1 onto x axis and p1 to origin
 */
static inline void computeTransformRTyz(const double p1[4], const double n1[4], double row2[3], double row3[3], double t[3])
{
    // dot product with x axis
    double angle=acos( n1[0] );
    
    // cross product with x axis
    double axis[3]={0, n1[2], -n1[1]};
    double axisNorm;
    
    // we try to project on the ground plane but it's already parallel
    if (n1[1]==0 && n1[2]==0)
    {
        axis[1]=1;
        axis[2]=0;
    }
    else
    {
        axisNorm=sqrt(axis[2]*axis[2]+axis[1]*axis[1]);
        
        if (axisNorm>EPS)
        {
            axis[1]/=axisNorm;
            axis[2]/=axisNorm;
        }
    }
    
    aaToRyz(angle, axis, row2, row3);
    
    t[1] = row2[0] * (-p1[0]) + row2[1] * (-p1[1]) + row2[2] * (-p1[2]);
    t[2] = row3[0] * (-p1[0]) + row3[1] * (-p1[1]) + row3[2] * (-p1[2]);
}

/**
 *  \brief Compute the transformation needed to rotate n1 onto x axis and p1 to origin
 */
static inline void computeTransformRT(const double p1[4], const double n1[4], double R[9], double t[3])
{
    // dot product with x axis
    double angle=acos( n1[0] );
    
    // cross product with x axis
    double axis[3]={0, n1[2], -n1[1]};
    double axisNorm;
    double *row1, *row2, *row3;
    
    // we try to project on the ground plane but it's already parallel
    if (n1[1]==0 && n1[2]==0)
    {
        axis[1]=1;
        axis[2]=0;
    }
    else
    {
        axisNorm=sqrt(axis[2]*axis[2]+axis[1]*axis[1]);
        
        if (axisNorm>EPS)
        {
            axis[1]/=axisNorm;
            axis[2]/=axisNorm;
        }
    }
    
    aaToR(angle, axis, R);
    row1 = &R[0];
    row2 = &R[3];
    row3 = &R[6];
    
    t[0] = row1[0] * (-p1[0]) + row1[1] * (-p1[1]) + row1[2] * (-p1[2]);
    t[1] = row2[0] * (-p1[0]) + row2[1] * (-p1[1]) + row2[2] * (-p1[2]);
    t[2] = row3[0] * (-p1[0]) + row3[1] * (-p1[1]) + row3[2] * (-p1[2]);
}

/**
 *  \brief Flip a normal to the viewing direction
 *  
 *  \param [in] point Scene point
 *  \param [in] vp_x X component of view direction
 *  \param [in] vp_y Y component of view direction
 *  \param [in] vp_z Z component of view direction
 *  \param [in] nx X component of normal
 *  \param [in] ny Y component of normal
 *  \param [in] nz Z component of normal
 */
static inline void flipNormalViewpoint(const float* point, double vp_x, double vp_y, double vp_z, double *nx, double *ny, double *nz)
{
    double cos_theta;
    
    // See if we need to flip any plane normals
    vp_x -= (double)point[0];
    vp_y -= (double)point[1];
    vp_z -= (double)point[2];
    
    // Dot product between the (viewpoint - point) and the plane normal
    cos_theta = (vp_x * (*nx) + vp_y * (*ny) + vp_z * (*nz));
    
    // Flip the plane normal
    if (cos_theta < 0)
    {
        (*nx) *= -1;
        (*ny) *= -1;
        (*nz) *= -1;
    }
}
/**
 *  \brief Flip a normal to the viewing direction
 *  
 *  \param [in] point Scene point
 *  \param [in] vp_x X component of view direction
 *  \param [in] vp_y Y component of view direction
 *  \param [in] vp_z Z component of view direction
 *  \param [in] nx X component of normal
 *  \param [in] ny Y component of normal
 *  \param [in] nz Z component of normal
 */
static inline void flipNormalViewpoint_32f(const float* point, float vp_x, float vp_y, float vp_z, float *nx, float *ny, float *nz)
{
    float cos_theta;
    
    // See if we need to flip any plane normals
    vp_x -= (float)point[0];
    vp_y -= (float)point[1];
    vp_z -= (float)point[2];
    
    // Dot product between the (viewpoint - point) and the plane normal
    cos_theta = (vp_x * (*nx) + vp_y * (*ny) + vp_z * (*nz));
    
    // Flip the plane normal
    if (cos_theta < 0)
    {
        (*nx) *= -1;
        (*ny) *= -1;
        (*nz) *= -1;
    }
}

/**
 *  \brief Convert a rotation matrix to axis angle representation
 *  
 *  \param [in] R Rotation matrix
 *  \param [in] axis Axis vector
 *  \param [in] angle Angle in radians
 */
static inline void dcmToAA(double *R, double *axis, double *angle)
{
    double d1 = R[7] - R[5];
    double d2 = R[2] - R[6];
    double d3 = R[3] - R[1];
    
    double norm = sqrt(d1 * d1 + d2 * d2 + d3 * d3);
    double x = (R[7] - R[5]) / norm;
    double y = (R[2] - R[6]) / norm;
    double z = (R[3] - R[1]) / norm;
    
    *angle = acos((R[0] + R[4] + R[8] - 1.0) * 0.5);
    
    axis[0] = x;
    axis[1] = y;
    axis[2] = z;
}

/**
 *  \brief Convert axis angle representation to rotation matrix
 *  
 *  \param [in] axis Axis Vector
 *  \param [in] angle Angle (In radians)
 *  \param [in] R 3x3 Rotation matrix
 */
 static inline void aaToDCM(double *axis, double angle, double *R)
{
    double ident[9]={1,0,0,0,1,0,0,0,1};
    double n[9] = { 0.0, -axis[2], axis[1],
                    axis[2], 0.0, -axis[0],
                    -axis[1], axis[0], 0.0
                  };
                  
    double nsq[9];
    double c, s;
    int i;
    
    //c = 1-cos(angle);
    c = cos(angle);
    s = sin(angle);
    
    matrixProduct33(n, n, nsq);
    
    for (i = 0; i < 9; i++)
    {
        const double sni = n[i]*s;
        const double cnsqi = nsq[i]*(c);
        R[i]=ident[i]+sni+cnsqi;
    }
    
    // The below code is the matrix based implemntation of the above
    // double nsq[9], sn[9], cnsq[9], tmp[9];
    //matrix_scale(3, 3, n, s, sn);
    //matrix_scale(3, 3, nsq, (1 - c), cnsq);
    //matrix_sum(3, 3, 3, 3, ident, sn, tmp);
    //matrix_sum(3, 3, 3, 3, tmp, cnsq, R);
}

/**
 *  \brief Convert a discrete cosine matrix to quaternion
 *  
 *  \param [in] R Rotation Matrix
 *  \param [in] q Quaternion
 */
static inline void dcmToQuat(double *R, double *q)
{
    double n4; // the norm of quaternion multiplied by 4
    double tr = R[0] + R[4] + R[8]; // trace of martix
    double factor;
    
    if (tr > 0.0)
    {
        q[1] = R[5] - R[7];
        q[2] = R[6] - R[2];
        q[3] = R[1] - R[3];
        q[0] = tr + 1.0;
        n4 = q[0];
    }
    else
        if ((R[0] > R[4]) && (R[0] > R[8]))
        {
            q[1] = 1.0 + R[0] - R[4] - R[8];
            q[2] = R[3] + R[1];
            q[3] = R[6] + R[2];
            q[0] = R[5] - R[7];
            n4 = q[1];
        }
        else
            if (R[4] > R[8])
            {
                q[1] = R[3] + R[1];
                q[2] = 1.0 + R[4] - R[0] - R[8];
                q[3] = R[7] + R[5];
                q[0] = R[6] - R[2];
                n4 = q[2];
            }
            else
            {
                q[1] = R[6] + R[2];
                q[2] = R[7] + R[5];
                q[3] = 1.0 + R[8] - R[0] - R[4];
                q[0] = R[1] - R[3];
                n4 = q[3];
            }
            
    factor = 0.5 / sqrt(n4);
    q[0] *= factor;
    q[1] *= factor;
    q[2] *= factor;
    q[3] *= factor;
}

/**
 *  \brief Convert quaternion to a discrete cosine matrix
 *  
 *  \param [in] q Quaternion (w is at first element)
 *  \param [in] R Rotation Matrix
 *  
 */
 static inline void quatToDCM(double *q, double *R)
{
    double sqw = q[0] * q[0];
    double sqx = q[1] * q[1];
    double sqy = q[2] * q[2];
    double sqz = q[3] * q[3];
    
    double tmp1, tmp2;
    
    R[0] =  sqx - sqy - sqz + sqw; // since sqw + sqx + sqy + sqz = 1
    R[4] = -sqx + sqy - sqz + sqw;
    R[8] = -sqx - sqy + sqz + sqw;
    
    tmp1 = q[1] * q[2];
    tmp2 = q[3] * q[0];
    
    R[1] = 2.0 * (tmp1 + tmp2);
    R[3] = 2.0 * (tmp1 - tmp2);
    
    tmp1 = q[1] * q[3];
    tmp2 = q[2] * q[0];
    
    R[2] = 2.0 * (tmp1 - tmp2);
    R[6] = 2.0 * (tmp1 + tmp2);
    
    tmp1 = q[2] * q[3];
    tmp2 = q[1] * q[0];
    
    R[5] = 2.0 * (tmp1 + tmp2);
    R[7] = 2.0 * (tmp1 - tmp2);
}


static inline void eigenLowest33(const double C[3][3], double A[3])
{
    const double a = C[0][0];
    const double b = C[0][1];
    const double c = C[0][2];
    const double d = C[1][1];
    const double e = C[1][2];
    const double f = C[2][2];
    const double t2 = c*c;
    const double t3 = e*e;
    const double t4 = b*t2;
    const double t5 = c*d*e;
    const double t34 = b*t3;
    const double t35 = a*c*e;
    const double t6 = t4+t5-t34-t35;
    const double t7 = 1.0/t6;
    const double t8 = a+d+f;
    const double t9 = b*b;
    const double t23 = a*d;
    const double t24 = a*f;
    const double t25 = d*f;
    const double t10 = t2+t3+t9-t23-t24-t25;
    const double t11 = t8*t10*(1.0/6.0);
    const double t12 = t8*t8;
    const double t20 = t8*t12*(1.0/2.7E1);
    const double t21 = b*c*e;
    const double t22 = a*d*f*(1.0/2.0);
    const double t26 = a*t3*(1.0/2.0);
    const double t27 = d*t2*(1.0/2.0);
    const double t28 = f*t9*(1.0/2.0);
    const double t13 = t11+t20+t21+t22-t26-t27-t28;
    const double t14 = t9*(1.0/3.0);
    const double t15 = t2*(1.0/3.0);
    const double t16 = t3*(1.0/3.0);
    const double t17 = t12*(1.0/9.0);
    const double t30 = a*d*(1.0/3.0);
    const double t31 = a*f*(1.0/3.0);
    const double t32 = d*f*(1.0/3.0);
    const double t18 = t14+t15+t16+t17-t30-t31-t32;
    const double t19 = t18*t18;
    const double t29 = t11+t20+t21+t22-t26-t27-t28;
    const double t36 = a*(1.0/3.0);
    const double t37 = d*(1.0/3.0);
    const double t38 = f*(1.0/3.0);
    const double t39 = t8*(t2+t3+t9-t23-t24-t25)*(1.0/6.0);
    const double t41 = t18*t19;
    const double t33 = t36+t37+t38+t18*1.0/pow(t11+t20+t21+t22-a*t3*(1.0/2.0)-d*t2*(1.0/2.0)-f*t9*(1.0/2.0)+sqrt(-t41+t13*t13),1.0/3.0)+pow(t20+t21+t22-t26-t27-t28+t39+sqrt(-t41+t29*t29),1.0/3.0);
    const double t40 = t11+t20+t21+t22-t26-t27-t28;
    const double t42 = t11+t20+t21+t22-t26-t27-t28;
    const double t43 = e*t2;
    const double t60 = b*c*f;
    const double t61 = d*e*f;
    const double t44 = t43-t60-t61+e*t3;
    const double t45 = t7*t44;
    const double t46 = t11+t20+t21+t22-t26-t27-t28;
    const double t47 = t11+t20+t21+t22-t26-t27-t28;
    const double t48 = t11+t20+t21+t22-t26-t27-t28;
    const double t49 = t11+t20+t21+t22-t26-t27-t28;
    const double t57 = sqrt(3.0);
    const double t50 = t36+t37+t38-t18*1.0/pow(t20+t21+t22-t26-t27-t28+t39+sqrt(-t41+t46*t46),1.0/3.0)*(1.0/2.0)-t57*(t18*1.0/pow(t20+t21+t22-t26-t27-t28+t39+sqrt(-t41+t48*t48),1.0/3.0)-pow(t20+t21+t22-t26-t27-t28+t39+sqrt(-t41+t49*t49),1.0/3.0))*5.0E-1*sqrt(-1.0)-pow(t20+t21+t22-t26-t27-t28+t39+sqrt(-t41+t47*t47),1.0/3.0)*(1.0/2.0);
    const double t51 = b*c;
    const double t52 = d*e;
    const double t53 = e*f;
    const double t54 = t51+t52+t53;
    const double t55 = t11+t20+t21+t22-t26-t27-t28;
    const double t56 = t11+t20+t21+t22-t26-t27-t28;
    const double t58 = t11+t20+t21+t22-t26-t27-t28;
    const double t59 = t11+t20+t21+t22-t26-t27-t28;
    const double t62 = t11+t20+t21+t22-t26-t27-t28;
    const double t63 = t11+t20+t21+t22-t26-t27-t28;
    const double t64 = t11+t20+t21+t22-t26-t27-t28;
    const double t65 = t11+t20+t21+t22-t26-t27-t28;
    const double t66 = t36+t37+t38-t18*1.0/pow(t20+t21+t22-t26-t27-t28+t39+sqrt(-t41+t62*t62),1.0/3.0)*(1.0/2.0)+t57*(t18*1.0/pow(t20+t21+t22-t26-t27-t28+t39+sqrt(-t41+t64*t64),1.0/3.0)-pow(t20+t21+t22-t26-t27-t28+t39+sqrt(-t41+t65*t65),1.0/3.0))*5.0E-1*sqrt(-1.0)-pow(t20+t21+t22-t26-t27-t28+t39+sqrt(-t41+t63*t63),1.0/3.0)*(1.0/2.0);
    const double t67 = t11+t20+t21+t22-t26-t27-t28;
    const double t68 = t11+t20+t21+t22-t26-t27-t28;
    const double t69 = t11+t20+t21+t22-t26-t27-t28;
    const double t70 = t11+t20+t21+t22-t26-t27-t28;
    const double t71 = t11+t20+t21+t22-t26-t27-t28;
    const double t72 = t11+t20+t21+t22-t26-t27-t28;
    const double t73 = t36+t37+t38+t18*1.0/pow(t20+t21+t22-t26-t27-t28+t39+sqrt(-t41+t71*t71),1.0/3.0)+pow(t20+t21+t22-t26-t27-t28+t39+sqrt(-t41+t72*t72),1.0/3.0);
    const double t74 = t11+t20+t21+t22-t26-t27-t28;
    const double t75 = t11+t20+t21+t22-t26-t27-t28;
    const double t76 = c*t3;
    const double t91 = a*c*f;
    const double t92 = b*e*f;
    const double t77 = t76-t91-t92+c*t2;
    const double t78 = t11+t20+t21+t22-t26-t27-t28;
    const double t79 = t11+t20+t21+t22-t26-t27-t28;
    const double t80 = t11+t20+t21+t22-t26-t27-t28;
    const double t81 = t11+t20+t21+t22-t26-t27-t28;
    const double t82 = t36+t37+t38-t18*1.0/pow(t20+t21+t22-t26-t27-t28+t39+sqrt(-t41+t78*t78),1.0/3.0)*(1.0/2.0)-t57*(t18*1.0/pow(t20+t21+t22-t26-t27-t28+t39+sqrt(-t41+t80*t80),1.0/3.0)-pow(t20+t21+t22-t26-t27-t28+t39+sqrt(-t41+t81*t81),1.0/3.0))*5.0E-1*sqrt(-1.0)-pow(t20+t21+t22-t26-t27-t28+t39+sqrt(-t41+t79*t79),1.0/3.0)*(1.0/2.0);
    const double t83 = a*c;
    const double t84 = b*e;
    const double t85 = c*f;
    const double t86 = t83+t84+t85;
    const double t87 = t11+t20+t21+t22-t26-t27-t28;
    const double t88 = t11+t20+t21+t22-t26-t27-t28;
    const double t89 = t11+t20+t21+t22-t26-t27-t28;
    const double t90 = t11+t20+t21+t22-t26-t27-t28;
    const double t93 = t11+t20+t21+t22-t26-t27-t28;
    const double t94 = t11+t20+t21+t22-t26-t27-t28;
    const double t95 = t11+t20+t21+t22-t26-t27-t28;
    const double t96 = t11+t20+t21+t22-t26-t27-t28;
    const double t97 = t36+t37+t38-t18*1.0/pow(t20+t21+t22-t26-t27-t28+t39+sqrt(-t41+t93*t93),1.0/3.0)*(1.0/2.0)+t57*(t18*1.0/pow(t20+t21+t22-t26-t27-t28+t39+sqrt(-t41+t95*t95),1.0/3.0)-pow(t20+t21+t22-t26-t27-t28+t39+sqrt(-t41+t96*t96),1.0/3.0))*5.0E-1*sqrt(-1.0)-pow(t20+t21+t22-t26-t27-t28+t39+sqrt(-t41+t94*t94),1.0/3.0)*(1.0/2.0);
    const double t98 = t11+t20+t21+t22-t26-t27-t28;
    const double t99 = t11+t20+t21+t22-t26-t27-t28;
    const double t100 = t11+t20+t21+t22-t26-t27-t28;
    const double t101 = t11+t20+t21+t22-t26-t27-t28;
    //A0[0][0] = t45-e*t7*(t33*t33)+t7*t54*(t36+t37+t38+t18*1.0/pow(t20+t21+t22-t26-t27-t28+t39+sqrt(-t41+t40*t40),1.0/3.0)+pow(t20+t21+t22-t26-t27-t28+t39+sqrt(-t41+t42*t42),1.0/3.0));
    //A0[0][1] = t45-e*t7*(t50*t50)+t7*t54*(t36+t37+t38-t18*1.0/pow(t20+t21+t22-t26-t27-t28+t39+sqrt(-t41+t55*t55),1.0/3.0)*(1.0/2.0)-t57*(t18*1.0/pow(t20+t21+t22-t26-t27-t28+t39+sqrt(-t41+t58*t58),1.0/3.0)-pow(t20+t21+t22-t26-t27-t28+t39+sqrt(-t41+t59*t59),1.0/3.0))*5.0E-1*sqrt(-1.0)-pow(t20+t21+t22-t26-t27-t28+t39+sqrt(-t41+t56*t56),1.0/3.0)*(1.0/2.0));
    A[0] = t45-e*t7*(t66*t66)+t7*t54*(t36+t37+t38-t18*1.0/pow(t20+t21+t22-t26-t27-t28+t39+sqrt(-t41+t67*t67),1.0/3.0)*(1.0/2.0)+t57*(t18*1.0/pow(t20+t21+t22-t26-t27-t28+t39+sqrt(-t41+t69*t69),1.0/3.0)-pow(t20+t21+t22-t26-t27-t28+t39+sqrt(-t41+t70*t70),1.0/3.0))*5.0E-1*sqrt(-1.0)-pow(t20+t21+t22-t26-t27-t28+t39+sqrt(-t41+t68*t68),1.0/3.0)*(1.0/2.0));
    //A0[1][0] = -t7*t77+c*t7*(t73*t73)-t7*t86*(t36+t37+t38+t18*1.0/pow(t20+t21+t22-t26-t27-t28+t39+sqrt(-t41+t74*t74),1.0/3.0)+pow(t20+t21+t22-t26-t27-t28+t39+sqrt(-t41+t75*t75),1.0/3.0));
    //A0[1][1] = -t7*t77+c*t7*(t82*t82)-t7*t86*(t36+t37+t38-t18*1.0/pow(t20+t21+t22-t26-t27-t28+t39+sqrt(-t41+t87*t87),1.0/3.0)*(1.0/2.0)-t57*(t18*1.0/pow(t20+t21+t22-t26-t27-t28+t39+sqrt(-t41+t89*t89),1.0/3.0)-pow(t20+t21+t22-t26-t27-t28+t39+sqrt(-t41+t90*t90),1.0/3.0))*5.0E-1*sqrt(-1.0)-pow(t20+t21+t22-t26-t27-t28+t39+sqrt(-t41+t88*t88),1.0/3.0)*(1.0/2.0));
    A[1] = -t7*t77+c*t7*(t97*t97)-t7*t86*(t36+t37+t38-t18*1.0/pow(t20+t21+t22-t26-t27-t28+t39+sqrt(-t41+t98*t98),1.0/3.0)*(1.0/2.0)+t57*(t18*1.0/pow(t20+t21+t22-t26-t27-t28+t39+sqrt(-t41+t100*t100),1.0/3.0)-pow(t20+t21+t22-t26-t27-t28+t39+sqrt(-t41+t101*t101),1.0/3.0))*5.0E-1*sqrt(-1.0)-pow(t20+t21+t22-t26-t27-t28+t39+sqrt(-t41+t99*t99),1.0/3.0)*(1.0/2.0));
    //A0[2][0] = 1.0;
    //A0[2][1] = 1.0;
    A[2] = 1.0;
}
}
}

#endif