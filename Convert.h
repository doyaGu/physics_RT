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
    out[0][0] = (float)in.get_elem(0, 0);
    out[0][1] = (float)in.get_elem(0, 1);
    out[0][2] = (float)in.get_elem(0, 2);
    out[0][3] = (float)in.vv.k[0];

    out[1][0] = (float)in.get_elem(1, 0);
    out[1][1] = (float)in.get_elem(1, 1);
    out[1][2] = (float)in.get_elem(1, 2);
    out[1][3] = (float)in.vv.k[1];

    out[2][0] = (float)in.get_elem(2, 0);
    out[2][1] = (float)in.get_elem(2, 1);
    out[2][2] = (float)in.get_elem(2, 2);
    out[2][3] = (float)in.vv.k[2];

    out[3][0] = out[3][1] = out[3][2] = 0.0f;
    out[3][3] = 1.0f;
}

inline void VxConvertMatrix(const VxMatrix &in, IVP_U_Matrix &out)
{
    out.set_elem(0, 0, in[0][0]);
    out.set_elem(0, 1, in[0][1]);
    out.set_elem(0, 2, in[0][2]);
    out.set_elem(1, 0, in[1][0]);
    out.set_elem(1, 1, in[1][1]);
    out.set_elem(1, 2, in[1][2]);
    out.set_elem(2, 0, in[2][0]);
    out.set_elem(2, 1, in[2][1]);
    out.set_elem(2, 2, in[2][2]);
    out.vv.k[0] = in[0][3];
    out.vv.k[1] = in[1][3];
    out.vv.k[2] = in[2][3];
}

inline void VxConvertMatrix(const VxMatrix &in, IVP_U_Matrix3 &out)
{
    out.set_elem(0, 0, in[0][0]);
    out.set_elem(0, 1, in[0][1]);
    out.set_elem(0, 2, in[0][2]);
    out.set_elem(1, 0, in[1][0]);
    out.set_elem(1, 1, in[1][1]);
    out.set_elem(1, 2, in[1][2]);
    out.set_elem(2, 0, in[2][0]);
    out.set_elem(2, 1, in[2][1]);
    out.set_elem(2, 2, in[2][2]);
}

#endif // PHYSICS_RT_CONVERT_H
