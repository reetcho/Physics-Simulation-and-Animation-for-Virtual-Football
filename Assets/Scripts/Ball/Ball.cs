using UnityEngine;
using Quaternion = UnityEngine.Quaternion;
using Vector3 = UnityEngine.Vector3;

public class Ball : MonoBehaviour
{
    public Vector3 position;
    public Vector3 velocity;
    public Quaternion rotation;
    public Vector3 angularVelocity;
    public BallState state;
    
    [Header("Ball properties")]
    public float radius;
    public float mass;
    [Range(0, 1)] public float coefficientOfVerticalRestitution;
    [Range(-1, 1)] public float coefficientOfHorizontalRestitution;
    //TODO commentare
    [Range(-1, 1)] public float coefficientRotationRestitutionYaxis;
    
    [Header("Friction")]
    [Range(0, 1)] public float coefficientOfSlidingFriction;
    [Range(0, 1)] public float coefficientOfRollingFriction; 
    //TODO
    [Range(0,1)] public float coefficientOfVerticalAxisSpinningFriction;
    
    public float inertialMomentum; //assumption of thin sphere -> 2/3 * mass * radius^2
    public float dragCoefficient;
    public bool useCustomDragCoefficient;
    public float customDragCoefficient;
    
    //TODO SPIEGARE CHE COSA SONO vc e vs
    public float vc;
    public float vs;
    public float liftCoefficient;


    private const float Gravity = 9.81f;
    public Vector3 resetPosition;

    private void Awake()
    {
        transform.localScale = new Vector3(radius, radius, radius)*2;
        inertialMomentum = 2f/3f * mass * radius * radius;
        
        resetPosition = transform.position;
        position = resetPosition;
        rotation = transform.rotation;
    }

    private void FixedUpdate()
    {
        transform.position = position;
        transform.rotation = rotation;
    }

    public void Reset()
    {
        resetPosition.y = radius;
        position = resetPosition;
        velocity = Vector3.zero;
        rotation = Quaternion.identity;
        angularVelocity = Vector3.zero;
        state = BallState.Stopped;
    }
    
    public void ResetValues()
    {
        velocity = Vector3.zero;
        angularVelocity = Vector3.zero;
        state = BallState.Stopped;
    }
    
    public float CalculatePotentialEnergy()
    {
        if(state != BallState.Bouncing)
            return 0f;
        
        return mass * Gravity * (position.y - radius);
    }

    public float CalculateKineticEnergy()
    {
        return 0.5f * mass * velocity.magnitude * velocity.magnitude;
    }

    public float CalculateRotationalEnergy()
    {
        return 0.5f * inertialMomentum * angularVelocity.magnitude * angularVelocity.magnitude;
    }

    public float CalculateTotalEnergy()
    {
        float kineticEnergy = CalculateKineticEnergy();
        float potentialEnergy = CalculatePotentialEnergy();
        float rotationalEnergy = CalculateRotationalEnergy();
        
        return kineticEnergy + potentialEnergy + rotationalEnergy;
    }
    
}

public enum BallState
{
    Bouncing,
    Sliding,
    Rolling,
    Stopped
}