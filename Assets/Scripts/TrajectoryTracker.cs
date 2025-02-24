using System.Collections.Generic;
using UnityEngine;

public class TrajectoryTracker : MonoBehaviour
{
    private float sphereSize;
    public List<Vector3> trajectoryPoints = new List<Vector3>();
    public List<Vector3> bouncedPoints = new List<Vector3>();
    public List<Vector3> stopSlidingPoint= new List<Vector3>();

    void Start()
    {
        sphereSize = transform.lossyScale.x/2;
    }
    
    void OnDrawGizmos()
    {
        Gizmos.color = Color.red;

        foreach (Vector3 point in trajectoryPoints)
        {
            Gizmos.DrawSphere(point, sphereSize/4); 
        }
        
        Gizmos.color = Color.blue;
        foreach (Vector3 point in bouncedPoints)
        {
            Gizmos.DrawSphere(point, sphereSize/4); 
        }
        
        Gizmos.color = Color.green;

        foreach (Vector3 point in stopSlidingPoint)
        {
            Gizmos.DrawSphere(point + sphereSize * Vector3.up, sphereSize/2); 
        }
    }
}
