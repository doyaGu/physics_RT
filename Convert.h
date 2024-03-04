#ifndef PHYSICS_RT_CONVERT_H
#define PHYSICS_RT_CONVERT_H

#include "VxVector.h"
#include "VxQuaternion.h"
#include "VxMatrix.h"

#include "ivu_linear.hxx"

inline void VxConvertVector(const IVP_U_Point &in, VxVector &out)
{
    out.x = (float)in.k[0];
    out.y = (float)in.k[1];
    out.z = (float)in.k[2];
}

inline void VxConvertVector(const IVP_U_Float_Point &in, VxVector &out)
{
    out.x = in.k[0];
    out.y = in.k[1];
    out.z = in.k[2];
}

inline void VxConvertVector(const VxVector &in, IVP_U_Point &out)
{
    out.k[0] = in.x;
    out.k[1] = in.y;
    out.k[2] = in.z;
}

inline void VxConvertVector(const VxVector &in, IVP_U_Float_Point &out)
{
    out.k[0] = in.x;
    out.k[1] = in.y;
    out.k[2] = in.z;
}

inline void VxConvertQuaternion(const IVP_U_Quat &in, VxQuaternion &out)
{
    out.x = (float)in.x;
    out.y = (float)in.y;
    out.z = (float)in.z;
    out.w = (float)in.w;
}

inline void VxConvertQuaternion(const VxQuaternion &in, IVP_U_Quat &out)
{
    out.x = in.x;
    out.y = in.y;
    out.z = in.z;
    out.w = in.w;
}

inline void VxConvertMatrix(const IVP_U_Matrix &in, VxMatrix &out)
{
    // Transpose
    for (int i = 3; i >= 0; --i)
        for (int j = 3; j >= 0; --j)
            out[j][i] = (float)in.get_elem(i, j);
    out[0][3] = 0.0f;
    out[1][3] = 0.0f;
    out[2][3] = 0.0f;

    out[3][0] = (float)in.vv.k[0];
    out[3][1] = (float)in.vv.k[1];
    out[3][2] = (float)in.vv.k[2];
    out[3][3] = 1.0f;
}

inline void VxConvertMatrix(const VxMatrix &in, IVP_U_Matrix &out)
{
    // Transpose
    out.set_elem(0, 0, in[0][0]);
    out.set_elem(0, 1, in[1][0]);
    out.set_elem(0, 2, in[2][0]);
    out.set_elem(1, 0, in[0][1]);
    out.set_elem(1, 1, in[1][1]);
    out.set_elem(1, 2, in[2][1]);
    out.set_elem(2, 0, in[0][2]);
    out.set_elem(2, 1, in[1][2]);
    out.set_elem(2, 2, in[2][2]);

    out.vv.k[0] = in[3][0];
    out.vv.k[1] = in[3][1];
    out.vv.k[2] = in[3][2];
}

inline void VxConvertMatrix(const VxMatrix &in, IVP_U_Matrix3 &out)
{
    // Transpose
    out.set_elem(0, 0, in[0][0]);
    out.set_elem(0, 1, in[1][0]);
    out.set_elem(0, 2, in[2][0]);
    out.set_elem(1, 0, in[0][1]);
    out.set_elem(1, 1, in[1][1]);
    out.set_elem(1, 2, in[2][1]);
    out.set_elem(2, 0, in[0][2]);
    out.set_elem(2, 1, in[1][2]);
    out.set_elem(2, 2, in[2][2]);
}

#endif // PHYSICS_RT_CONVERT_H
