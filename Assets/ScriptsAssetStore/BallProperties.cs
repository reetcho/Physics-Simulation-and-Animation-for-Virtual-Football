using UnityEngine;
using UnityEngine.Serialization;
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
        public float radius = 0.11f;
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
    
    #endregion 
        
    #region Unity
        private void Awake()
        {
            transform.localScale = new Vector3(radius, radius, radius)*2;
            InertialMomentum = inertialMomentumConstant * mass * radius * radius;
            
            resetPosition = transform.position;
            position = resetPosition;
            orientation = transform.rotation;
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
        
    #endregion

    #region Energy
        public float CalculatePotentialEnergy()
        {
            if(state != BallStates.Bouncing)
                return 0f;
            
            return mass * Simulation.Gravity * (position.y - radius);
        }

        public float CalculateKineticEnergy()
        {
            return 0.5f * mass * velocity.magnitude * velocity.magnitude;
        }

        public float CalculateRotationalEnergy()
        {
            return 0.5f * InertialMomentum * angularVelocity.magnitude * angularVelocity.magnitude;
        }

        public float CalculateTotalEnergy()
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
}

public enum BallStates
{
    Bouncing,
    Sliding,
    Rolling,
    Stopped
}