using System.Collections.Generic;
using UnityEngine;

public class TrajectoryTracker : MonoBehaviour
{
    private float recordInterval = 0.1f; 
    private float sphereSize;
    private List<Vector3> trajectoryPoints = new List<Vector3>();
    public List<Vector3> bouncedPoints = new List<Vector3>();

    private float timer = 0f;

    void Start()
    {
        sphereSize = transform.lossyScale.x/2;
    }
    void Update()
    {
        timer += Time.deltaTime;
        if (timer >= recordInterval)
        {
            trajectoryPoints.Add(transform.position);
            timer = 0f; 
        }
    }

    void OnDrawGizmos()
    {
        Gizmos.color = Color.red;

        foreach (Vector3 point in trajectoryPoints)
        {
            Gizmos.DrawSphere(point, sphereSize/2); 
        }
        
        Gizmos.color = Color.blue;
        foreach (Vector3 point in bouncedPoints)
        {
            Gizmos.DrawSphere(point, sphereSize); 
        }
    }
}
