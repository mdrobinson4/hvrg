using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace QuaternionOps
{
    public class QuaternionOps
    {
        double m00, m11, m22, m10, m01, m20, m02, m21, m12;
        public Quaternion quatToMatrix(Quaternion q)
        {
            double sqw = q.w * q.w;
            double sqx = q.x * q.x;
            double sqy = q.y * q.y;
            double sqz = q.z * q.z;

            // invs (inverse square length) is only required if quaternion is not already normalised
            double invs = 1 / (sqx + sqy + sqz + sqw);
            m00 = (sqx - sqy - sqz + sqw) * invs; // since sqw + sqx + sqy + sqz =1/invs*invs
            m11 = (-sqx + sqy - sqz + sqw) * invs;
            m22 = (-sqx - sqy + sqz + sqw) * invs;

            double tmp1 = q.x * q.y;
            double tmp2 = q.z * q.w;
            m10 = 2.0 * (tmp1 + tmp2) * invs;
            m01 = 2.0 * (tmp1 - tmp2) * invs;

            tmp1 = q.x * q.z;
            tmp2 = q.y * q.w;
            m20 = 2.0 * (tmp1 - tmp2) * invs;
            m02 = 2.0 * (tmp1 + tmp2) * invs;
            tmp1 = q.y * q.z;
            tmp2 = q.x * q.w;
            m21 = 2.0 * (tmp1 + tmp2) * invs;
            m12 = 2.0 * (tmp1 - tmp2) * invs;
            return q;
        }
    }
}

