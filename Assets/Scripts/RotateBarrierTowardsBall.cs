using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RotateBarrierTowardsBall : MonoBehaviour
{

    [SerializeField] private Ball ball;
    [SerializeField] private GameObject target;
    [SerializeField] private GameObject goal;
    [SerializeField] private float distanceFromBall;

    // Update is called once per frame
    void Update()
    {
        if(ball.state == BallState.Stopped && !ball.frameByFrame)
        {
            Vector3 direction = ball.transform.position - transform.position;
            direction.y = 0;
            transform.rotation = Quaternion.Euler(0, 90f, 0) * Quaternion.LookRotation(direction, Vector3.up);
            Vector3 position = ball.transform.position + distanceFromBall * ((target.transform.position + goal.transform.position) / 2 - ball.transform.position).normalized;
            transform.position = new Vector3(position.x, transform.position.y, position.z);
        }
    }
}
