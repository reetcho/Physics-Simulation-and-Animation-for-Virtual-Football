using UnityEngine;

public class RotateBarrierTowardsBall : MonoBehaviour
{

    [SerializeField] private BallPhysics ballPhysics;
    [SerializeField] private GameObject ball;
    [SerializeField] private GameObject target;
    [SerializeField] private GameObject goal;
    [SerializeField] private float distanceFromBall;

    void Update()
    {
        if(ballPhysics.ball.state == BallState.Stopped && !ballPhysics.useFrameByFrameMode)
        {
            Vector3 direction = ball.transform.position - transform.position;
            direction.y = 0;
            transform.rotation = Quaternion.Euler(0, 90f, 0) * Quaternion.LookRotation(direction, Vector3.up);
            Vector3 position = ball.transform.position + distanceFromBall * ((target.transform.position + goal.transform.position) / 2 - ball.transform.position).normalized;
            transform.position = new Vector3(position.x, transform.position.y, position.z);
        }
    }
}
