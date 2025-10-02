using UnityEngine;

public class Frames
{
    public Vector3 Position { get; private set; }
    public Quaternion Orientation { get; private set; }
    public Vector3 Velocity { get; private set; }
    public Vector3 AngularVelocity { get; private set; }
    public BallStates State { get; private set; }
    public double TimeToCompute { get; private set; }


    public Frames(Vector3 position, Quaternion orientation,Vector3 velocity, Vector3 angularVelocity, BallStates state, double timeToCompute)
    {
        Position = position;
        Orientation = orientation;
        Velocity = velocity;
        AngularVelocity = angularVelocity;
        State = state;
        TimeToCompute = timeToCompute;
    }
}