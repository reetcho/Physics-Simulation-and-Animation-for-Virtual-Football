using UnityEngine;
using UnityQuat = UnityEngine.Quaternion;
using SysQuat = System.Numerics.Quaternion;

public static class QuaternionUtils
{

    public static SysQuat ToNumerics(UnityQuat q)
    {
        return new SysQuat(q.x, q.y, q.z, q.w);
    }

    public static UnityQuat ToUnity(SysQuat q)
    {
        return new UnityQuat(q.X, q.Y, q.Z, q.W);
    }

    public static SysQuat QuaternionDerivative(SysQuat quaternion, Vector3 angularVelocity)
    {
        SysQuat angularVelocityQuaternion = new SysQuat(angularVelocity.x, angularVelocity.y, angularVelocity.z, 0f);
        
        SysQuat product = SysQuat.Multiply(angularVelocityQuaternion, quaternion);
        
        SysQuat quaternionDerivative = SysQuat.Multiply(product, 0.5f);
        
        return quaternionDerivative;
    }

}