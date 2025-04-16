using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Serialization;

public class RotateBarrierTowardsBall : MonoBehaviour
{

    [FormerlySerializedAs("ball")] [SerializeField] private BallPhysics ballPhysics;
    [SerializeField] private GameObject target;
    [SerializeField] private GameObject goal;
    [SerializeField] private float distanceFromBall;

    // Update is called once per frame
    void Update()
    {
        if(ballPhysics.ballValues.state == BallState.Stopped && !ballPhysics.frameByFrame)
        {
            Vector3 direction = ballPhysics.transform.position - transform.position;
            direction.y = 0;
            transform.rotation = Quaternion.Euler(0, 90f, 0) * Quaternion.LookRotation(direction, Vector3.up);
            Vector3 position = ballPhysics.transform.position + distanceFromBall * ((target.transform.position + goal.transform.position) / 2 - ballPhysics.transform.position).normalized;
            transform.position = new Vector3(position.x, transform.position.y, position.z);
        }
    }
}
