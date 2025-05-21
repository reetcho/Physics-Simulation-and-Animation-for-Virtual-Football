using UnityEngine;

public class Frame
{
    public Vector3 Position { get; private set; }
    public Quaternion Orientation { get; private set; }
    public Vector3 Velocity { get; private set; }
    public Vector3 AngularVelocity { get; private set; }
    public BallState State { get; private set; }
    public float TimeToCompute { get; private set; }


    public Frame(Vector3 position, Quaternion orientation,Vector3 velocity, Vector3 angularVelocity, BallState state, float timeToCompute)
    {
        Position = position;
        Orientation = orientation;
        Velocity = velocity;
        AngularVelocity = angularVelocity;
        State = state;
        TimeToCompute = timeToCompute;
    }
}