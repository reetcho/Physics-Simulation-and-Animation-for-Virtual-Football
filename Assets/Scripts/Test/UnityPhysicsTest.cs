#if UNITY_EDITOR

using UnityEngine;
using UnityEditor;

public class UnityPhysicsTest : MonoBehaviour
{
    [SerializeField] private Rigidbody testSphere;
    [SerializeField] private Vector3 velocity;
    [SerializeField] private Vector3 angularVelocity;

    public void Shoot()
    {
        testSphere.AddForce(velocity, ForceMode.VelocityChange);
        testSphere.AddTorque(angularVelocity, ForceMode.VelocityChange);
    }
    
    
}

[CustomEditor(typeof(UnityPhysicsTest))]
public class UnityPhysicsTestEditor : Editor
{
    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();

        UnityPhysicsTest test = (UnityPhysicsTest)target;
        if (GUILayout.Button("Shoot"))
        {
            test.Shoot();
        }
    }
}

#endif