using System;
using UnityEngine;
using UnityEngine.Serialization;
using Quaternion = UnityEngine.Quaternion;
using Vector3 = UnityEngine.Vector3;

public class Ball : MonoBehaviour
{
    #region Variables
        public Vector3 position;
        public Vector3 velocity;
        public Quaternion orientation;
        public Vector3 angularVelocity;
        public BallState state;
        
        [Header("Ball properties")]
        public float radius;
        public float mass;
        public float inertialMomentumConstant = 2f/3f; //hollow sphere 
        public float InertialMomentum { get; private set; } //assumption of thin sphere -> 2/3 * mass * radius^2
        
        [Header("Restitution coefficients")]
        [Range(0, 1)] public float coefficientOfVerticalRestitution;
        [Range(-1, 1)] public float coefficientOfHorizontalRestitution;
        [Range(0, 1)] public float coefficientOfVerticalAxisSpinRestitution; //percentage of angular velocity around y-axis that is preserved after the ball bounces on the ground
        
        [Header("Ground friction forces")]
        [Range(0, 1)] public float coefficientOfSlidingFriction;
        [Range(0, 1)] public float coefficientOfRollingFriction; 
        [FormerlySerializedAs("coefficientOfVerticalAxisSpinningFriction")] [Range(0, 1)] public float coefficientOfVerticalAxisSpinningDamping; //friction that acts on the spin about the y-axis when the ball is rolling/sliding
        
        [Header("Aerodynamic forces")]
        public float dragCoefficient;
        public float liftCoefficient;

        //drag critical speed from laminar to turbulent
        public float speedOfTransitionFromLaminarToTurbulentDrag = 12.19f;
        //drag transition sharpness speed
        public float dragTransitionSlope = 1.309f;
        
        public bool useConstantCoefficients;
        public float constantDragCoefficient;
        public float constantLiftCoefficient;
        public float constantSpinDecayCoefficient;
        
        [Header("Other")]
        public Vector3 resetPosition;
        public float maximumSpinValue = 30f;
        
        public Frame[] Frames { get; private set; } = new Frame[1000];
        public int NewestFrameIndex { get; private set; }
        public int OldestFrameIndex { get; private set; }
    
    #endregion 
        
    #region Unity
        private void Awake()
        {
            transform.localScale = new Vector3(radius, radius, radius)*2;
            InertialMomentum = inertialMomentumConstant * mass * radius * radius;
            
            resetPosition = transform.position;
            position = resetPosition;
            orientation = transform.rotation;

            Reset();
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
        
        void OnDrawGizmos()
        {
            DrawTrajectory();
        }
        
    #endregion

    #region Energy
        public float CalculatePotentialEnergy()
        {
            if(state != BallState.Bouncing)
                return 0f;
            
            return mass * BallPhysics.Gravity * (position.y - radius);
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
            state = BallState.Stopped;
                
            Frames = new Frame[1000];
            NewestFrameIndex = 0;
            OldestFrameIndex = 0;
        }
        private void ResetValues()
        {
            //TODO remove this
            /*
            position = new Vector3(0, radius, 0);
            velocity = new Vector3(20, 10, 0);
            angularVelocity = new Vector3(0, 15, 10);
            state = BallState.Bouncing;
            */
            
            velocity = Vector3.zero;
            angularVelocity = Vector3.zero;
            state = BallState.Stopped;
        }
        
        public void AddTrajectoryFrame(double timeToComputeFrame = 0f)
        {
            if (state != BallState.Stopped)
            {
                Frame frame = new Frame(position, orientation, velocity, angularVelocity, state, timeToComputeFrame);
                Frames[NewestFrameIndex] = frame;
                NewestFrameIndex++;
                
                if(NewestFrameIndex >= Frames.Length)
                    NewestFrameIndex = 0;
                
                if(NewestFrameIndex == OldestFrameIndex)
                    OldestFrameIndex++;
               
                if(OldestFrameIndex >= Frames.Length)
                    OldestFrameIndex = 0;
            }
        }
        private void DrawTrajectory()
        {
            if(Frames.Length > 0)
            {
                foreach (Frame frame in Frames)
                {
                    if(frame != null)
                    {
                        switch (frame.State)
                        {
                            case BallState.Bouncing:
                                Gizmos.color = Color.red;
                                break;
                            case BallState.Sliding:
                                Gizmos.color = Color.blue;
                                break;
                            case BallState.Rolling:
                                Gizmos.color = Color.green;
                                break;
                        }
                        Gizmos.DrawSphere(frame.Position, radius / 4);
                    }
                }
            }
        }
    
    #endregion
}

public enum BallState
{
    Bouncing,
    Sliding,
    Rolling,
    Stopped
}