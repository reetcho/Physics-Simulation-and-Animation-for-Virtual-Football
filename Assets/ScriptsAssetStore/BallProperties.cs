using UnityEngine;
using Quaternion = UnityEngine.Quaternion;
using Vector3 = UnityEngine.Vector3;

public class BallProperties : MonoBehaviour
{
    #region Variables
        public Vector3 position;
        public Vector3 velocity;
        public Quaternion orientation;
        public Vector3 angularVelocity;
        public BallStates state;
        
        [Header("Ball properties")]
        [Range(0.001f, 100)] public float radius = 0.11f;
        public float mass = 0.43f;
        public float inertialMomentumConstant = 2f/3f; //value for a hollow sphere 
        public float InertialMomentum { get; private set; } //assumption of thin sphere -> 2/3 * mass * radius^2
        
        [Header("Restitution coefficients")]
        [Range(0, 1)] public float coefficientOfVerticalRestitution = 0.7f;
        [Range(-1, 1)] public float coefficientOfHorizontalRestitution = -0.75f;
        [Range(0, 1)] public float coefficientOfVerticalAxisSpinRestitution = 0.95f; //percentage of angular velocity around y-axis that is preserved after the ball bounces on the ground
        
        [Header("Ground friction forces")]
        [Range(0, 1)] public float coefficientOfSlidingFriction = 0.25f;
        [Range(0, 1)] public float coefficientOfRollingFriction = 0.15f; 
        [Range(0, 1)] public float coefficientOfVerticalAxisSpinningDamping = 0.04f; //friction that acts on the spin about the y-axis when the ball is rolling/sliding
        
        [Header("Aerodynamic forces")]
        public float dragCoefficient = 0.4f;
        public float liftCoefficient = 0.3f;
        public float spinDecayCoefficient = 0.01f;
        
        [Header("Other")]
        public Vector3 resetPosition;
        
                
        public Frames[] Frames { get; private set; } = new Frames[1000];
        public int NewestFrameIndex { get; private set; }
        public int OldestFrameIndex { get; private set; }
    
    #endregion 
        
    #region Unity
        private void Awake()
        {
            InertialMomentum = inertialMomentumConstant * mass * radius * radius;
            
            resetPosition = transform.position;
            position = resetPosition;
            orientation = transform.rotation;
            radius = transform.localScale.x / 2;
            transform.localScale = new Vector3(radius, radius, radius) * 2;
        }
        
        private void LateUpdate()
        {
            transform.position = position;
            transform.rotation = orientation;
        }

        private void FixedUpdate()
        {
            CheckIsMoving();
        }

        private void Update()
        {
            transform.localScale = new Vector3(radius, radius, radius) * 2;
            
            if (transform.position.y - radius < 0 && state == BallStates.Stopped)
            {
                position.y = radius;
                transform.position = position;
            }
        }
        
    #endregion

    #region Energy
        private float CalculatePotentialEnergy()
        {
            if(state != BallStates.Bouncing)
                return 0f;
            
            return mass * Simulation.Gravity * (position.y - radius);
        }

        private float CalculateKineticEnergy()
        {
            return 0.5f * mass * velocity.magnitude * velocity.magnitude;
        }

        private float CalculateRotationalEnergy()
        {
            return 0.5f * InertialMomentum * angularVelocity.magnitude * angularVelocity.magnitude;
        }

        private float CalculateTotalEnergy()
        {
            float kineticEnergy = CalculateKineticEnergy();
            float potentialEnergy = CalculatePotentialEnergy();
            float rotationalEnergy = CalculateRotationalEnergy();
            
            return kineticEnergy + potentialEnergy + rotationalEnergy;
        }
        
    #endregion
    
    #region Utils
        private void CheckIsMoving()
        {
            if (CalculateTotalEnergy() < 1e-4f)
            {
                ResetValues();
            }
        }
        public void Reset()
        {
            position = resetPosition;
            velocity = Vector3.zero;
            orientation = Quaternion.identity;
            angularVelocity = Vector3.zero;
            state = BallStates.Stopped;
        }
        private void ResetValues()
        {
            velocity = Vector3.zero;
            angularVelocity = Vector3.zero;
            state = BallStates.Stopped;
        }
    
    #endregion
    
    
    public void AddTrajectoryFrame(double timeToComputeFrame = 0f)
    {
        if (state != BallStates.Stopped)
        {
            Frames frames = new Frames(position, orientation, velocity, angularVelocity, state, timeToComputeFrame);
            Frames[NewestFrameIndex] = frames;
            NewestFrameIndex++;
                
            if(NewestFrameIndex >= Frames.Length)
                NewestFrameIndex = 0;
                
            if(NewestFrameIndex == OldestFrameIndex)
                OldestFrameIndex++;
               
            if(OldestFrameIndex >= Frames.Length)
                OldestFrameIndex = 0;
        }
    }
    
    void OnDrawGizmos()
    {
        DrawTrajectory();
    }
    private void DrawTrajectory()
    {
        if(Frames.Length > 0)
        {
            foreach (Frames frame in Frames)
            {
                if(frame != null)
                {
                    switch (frame.State)
                    {
                        case BallStates.Bouncing:
                            Gizmos.color = Color.red;
                            break;
                        case BallStates.Sliding:
                            Gizmos.color = Color.blue;
                            break;
                        case BallStates.Rolling:
                            Gizmos.color = Color.green;
                            break;
                    }
                    Gizmos.DrawSphere(frame.Position, radius / 4);
                }
            }
        }
    }

}

public enum BallStates
{
    Bouncing,
    Sliding,
    Rolling,
    Stopped
}