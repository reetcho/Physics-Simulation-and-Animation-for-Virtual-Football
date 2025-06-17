using UnityEngine;
using UnityQuaternion = UnityEngine.Quaternion;
using SystemQuaternion = System.Numerics.Quaternion;

public static class QuaternionUtils
{

    public static SystemQuaternion ToNumerics(UnityQuaternion q)
    {
        return new SystemQuaternion(q.x, q.y, q.z, q.w);
    }

    public static UnityQuaternion ToUnity(SystemQuaternion q)
    {
        return new UnityQuaternion(q.X, q.Y, q.Z, q.W);
    }

    public static SystemQuaternion QuaternionDerivative(SystemQuaternion q, Vector3 angularVelocity)
    {
        var omega = new SystemQuaternion(angularVelocity.x, angularVelocity.y, angularVelocity.z, 0f);
        var dq = SystemQuaternion.Multiply(omega, SystemQuaternion.Multiply(q, 0.5f));
        return dq;
    }

}