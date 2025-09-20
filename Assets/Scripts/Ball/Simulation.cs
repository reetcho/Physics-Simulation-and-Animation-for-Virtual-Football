using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
#if UNITY_EDITOR
using UnityEditor;
#endif
using UnityEngine;
using UnityEngine.Profiling;
using UnityEngine.Serialization;
using Debug = UnityEngine.Debug;
using Quaternion = UnityEngine.Quaternion;
using SysQuat = System.Numerics.Quaternion;
using Vector2 = UnityEngine.Vector2;
using Vector3 = UnityEngine.Vector3;

public class Simulation : MonoBehaviour
{
    #region Variables
        public Ball ball;
        
        [Header("Constants")]
        [SerializeField] private float airDensity = 1.2f;
        [SerializeField] private float dynamicViscosityOfAir = 1.81f * 1e-5f;
        public static readonly float Gravity = 9.81f; 
        private const float TimeStepThreshold = 1e-9f;
        private float _kinematicViscosityOfAir;
        private Vector3 _gravitationalForce;
        private Vector3 _buoyantForce;
        
        [Header("Simulation")]
        [SerializeField] private IntegrationMethod integrationMethod;
        [SerializeField] private float framePerSecond;
        
        [Header("Targeted shot")]
        [SerializeField] private int computationFrameRate;
        [SerializeField] private float constraintMaximumError;
        [SerializeField] private int maxIterations;
        [Range(0, 10)][SerializeField] private float maxErrorInCentimeters;
        [Range(0, 90)] [SerializeField] private float elevationAngleMaximumLimit;
        [Range(0, 90)] [SerializeField] private float elevationAngleMinimumLimit;
        [SerializeField] private bool adjustSpeed;
        [SerializeField] private float maxSpeedIncrease = 1.5f;
        //[SerializeField] private float maxSpeedDecrease = 0.5f;
        [Range(1, 100)] [SerializeField] private int speedCorrectionSteps = 2;
        [FormerlySerializedAs("useMethdoB")] [SerializeField] private bool useMethodB;

        [Header("Frame History")] 
        public bool useFrameByFrameMode;
        public bool interpolateFrames;
        public int FrameCount { private get; set; }
        public float ContinuousFrameCount { private get; set; }
        
        [Header("Other")]
        [SerializeField] private bool useAerodynamicForces;
        [SerializeField] private GameObject target;
        [SerializeField] private GameObject closestPointToTargetPlaceholder;
        [SerializeField] private GameObject constraint;
        [SerializeField] private GameObject closestPointToConstraintPlaceholder;
        public InteractionController interactionController;
        private readonly List<List<Vector3>> _precalculationTrajectories = new List<List<Vector3>>();
        private List<Vector3> _currentTrajectory;
        private double _timeToComputeFrame;
        private TestData _testData;
        private bool _testRunning;
        
    #endregion

    #region Unity
        void Awake()
        {
            Application.targetFrameRate = -1;
            Time.fixedDeltaTime = 1/framePerSecond;
            
            _gravitationalForce = Gravity * ball.mass * Vector3.down;
            _buoyantForce = 4f / 3f * Mathf.PI * Mathf.Pow(ball.radius, 3) * airDensity * Gravity * Vector3.up;
            _kinematicViscosityOfAir = dynamicViscosityOfAir * airDensity;
        }

        void Update()
        {
            Time.fixedDeltaTime = 1/framePerSecond;
            if(useAerodynamicForces)
            {
                _buoyantForce = 4f / 3f * Mathf.PI * Mathf.Pow(ball.radius, 3) * airDensity * Gravity * Vector3.up;
                _kinematicViscosityOfAir = dynamicViscosityOfAir * airDensity;
            }
            else
            {
                _buoyantForce = Vector3.zero;
                _kinematicViscosityOfAir = 0f;
            }

        }
        
        void FixedUpdate()
        {
            if (useFrameByFrameMode)
            {
                HandleFrameByFrame();
                return;
            }
            
            if (ball.state == BallState.Stopped)
                return;
            
            ComputeNewFrame(Time.fixedDeltaTime, ball);

            ball.AddTrajectoryFrame(_timeToComputeFrame);
        }
#if UNITY_EDITOR     
        void OnDrawGizmos()
        {
            DisplayComputedTrajectories();
        }
#endif
        
    #endregion
    
    #region Simulation
        public void ComputeNewFrame(float deltaTime, Ball targetBall)
        {
            //given the current state of the ball, the motion is integrated and the new state of the ball after delta time is returned
            Stopwatch stopwatch = Stopwatch.StartNew();
            Profiler.BeginSample("IntegrateMotion");
            (Vector3 newPosition, SysQuat newOrientation, Vector3 newVelocity, Vector3 newAngularVelocity) = IntegrateMotion(deltaTime, targetBall.position, QuaternionUtils.ToNumerics(targetBall.orientation), targetBall.velocity, targetBall.angularVelocity, targetBall.state);
            Profiler.EndSample();
            stopwatch.Stop();
            _timeToComputeFrame = stopwatch.Elapsed.TotalMilliseconds;
            
            //checks if there was a collision and if so handles it
            if (newPosition.y - targetBall.radius < 0f)
                (newPosition, newOrientation, newVelocity, newAngularVelocity, targetBall.state) = HandleGroundCollisionAndBounce(deltaTime, targetBall.position, QuaternionUtils.ToNumerics(targetBall.orientation), targetBall.velocity, targetBall.angularVelocity, targetBall.state);

            Vector3 previousVelocity = targetBall.velocity;
            Vector3 previousAngularVelocity = targetBall.angularVelocity;
            
            //updates the state of the ball
            targetBall.position = newPosition;
            targetBall.orientation = QuaternionUtils.ToUnity(newOrientation);
            targetBall.velocity = newVelocity;
            targetBall.angularVelocity = newAngularVelocity;
            
            //checks if the ball is a pure rolling state
            if(targetBall.state == BallState.Sliding)
                CheckPureRolling(targetBall, previousVelocity, previousAngularVelocity);
            
        }
        
        //this method chooses one of the four integration method to compute the new state of the ball after delta time
        private (Vector3, SysQuat, Vector3, Vector3) IntegrateMotion(float deltaTime, Vector3 initialPosition, SysQuat initialOrientation, Vector3 initialVelocity, Vector3 initialAngularVelocity, BallState state)
        {
            switch (integrationMethod)
            {
                case IntegrationMethod.ExplicitEuler:
                    return ExplicitEuler(deltaTime, initialPosition, initialOrientation, initialVelocity,initialAngularVelocity, state);
                case IntegrationMethod.SemiImplicitEuler:
                    return SemiImplicitEuler(deltaTime, initialPosition, initialOrientation, initialVelocity,initialAngularVelocity, state);
                case IntegrationMethod.Rk2:
                    return Runge_Kutta_2(deltaTime, initialPosition, initialOrientation, initialVelocity,initialAngularVelocity, state);
                case IntegrationMethod.Rk4:
                    return Runge_Kutta_4(deltaTime, initialPosition, initialOrientation, initialVelocity,initialAngularVelocity, state);
            }
                
            return (Vector3.zero, SysQuat.Identity, Vector3.zero, Vector3.zero);
        }
        
        //this method checks the pure rolling condition (HEURISTIC)
        private void CheckPureRolling(Ball targetBall, Vector3 previousVelocity, Vector3 previousAngularVelocity)
        {
            //inclination in of the ball axis against the up-axis in radians
            float currentTheta = (90f - Vector3.Angle(targetBall.angularVelocity.normalized, Vector3.up)) * Mathf.Deg2Rad;
            float previousTheta = (90f - Vector3.Angle(previousAngularVelocity.normalized, Vector3.up)) * Mathf.Deg2Rad;
            
            //compute the velocity of the contact point of the current and the previous frames
            float currentContactPointSpeed = (targetBall.velocity + Vector3.Cross(ball.radius * Mathf.Cos(currentTheta) * Vector3.up, targetBall.angularVelocity)).magnitude;
            
            float previousContactPointSpeed = (previousVelocity + Vector3.Cross(ball.radius * Mathf.Cos(previousTheta) * Vector3.up, previousAngularVelocity)).magnitude;
            
            //the state is set to rolling when the current contact point velocity becomes greater than the previous, meaning that it reached the minimum
            //this is done in this way because after this point the contact point velocity oscillating between -epsilon and +epsilon, resulting in an infinite cycle due to numerical error (HEURISTIC)
            if(currentContactPointSpeed > previousContactPointSpeed || currentContactPointSpeed < 1e-2)
            {
                ball.angularVelocity = -Vector3.Cross(targetBall.velocity, Vector3.up) / ball.radius + Vector3.up * targetBall.angularVelocity.y;
                targetBall.state = BallState.Rolling;
            }
        }
        
        #region Integration Methods
        
            private (Vector3, SysQuat, Vector3, Vector3) ExplicitEuler(float deltaTime,Vector3 pCurrent, SysQuat qCurrent, Vector3 vCurrent, Vector3 wCurrent, BallState state)
            {
                //acceleration and angular acceleration at current state
                (Vector3 aCurrent, Vector3 alphaCurrent) = CalculateAccelerationAndAngularAcceleration(vCurrent, wCurrent, state);

                //velocity update
                Vector3 vNext = vCurrent + deltaTime * aCurrent;
                
                //angular velocity update
                Vector3 wNext = wCurrent + deltaTime * alphaCurrent;

                //position update
                Vector3 pNext = pCurrent + deltaTime * vCurrent;
                
                //orientation update
                SysQuat qDerivative = QuaternionUtils.QuaternionDerivative(qCurrent, wCurrent);
                SysQuat qDelta = SysQuat.Multiply(qDerivative, deltaTime);
                SysQuat qNext = SysQuat.Add(qCurrent, qDelta);
                qNext = SysQuat.Normalize(qNext);

                return (pNext, qNext, vNext, wNext);
            }
            
            private (Vector3, SysQuat, Vector3, Vector3) SemiImplicitEuler(float deltaTime, Vector3 pCurrent, SysQuat qCurrent, Vector3 vCurrent, Vector3 wCurrent, BallState state)
            {
                //acceleration and angular acceleration at current state
                (Vector3 aCurrent, Vector3 alphaCurrent) = CalculateAccelerationAndAngularAcceleration(vCurrent, wCurrent, state);

                //velocity update
                Vector3 vNext = vCurrent + deltaTime * aCurrent;
                
                //angular velocity update
                Vector3 wNext = wCurrent + deltaTime * alphaCurrent;

                //position update
                Vector3 pNext = pCurrent + deltaTime * vNext;
                
                //orientation update
                SysQuat qDerivative = QuaternionUtils.QuaternionDerivative(qCurrent, wNext);
                SysQuat qDelta = SysQuat.Multiply(qDerivative, deltaTime);
                SysQuat qNext = SysQuat.Add(qCurrent, qDelta);
                qNext = SysQuat.Normalize(qNext);

                return (pNext, qNext, vNext, wNext);
            }
            
            private (Vector3, SysQuat, Vector3, Vector3) Runge_Kutta_2(float deltaTime, Vector3 pCurrent, SysQuat qCurrent, Vector3 vCurrent, Vector3 wCurrent, BallState state)
            {
                //k1 = f(t_n, y_n)
                K k1 = ComputeK(vCurrent, wCurrent, qCurrent, state);
                
                //k2 = f(t_n + h/2, y_n + k1 * h/2)
                K k2 = ComputeK(vCurrent + deltaTime * k1.V / 2f, wCurrent + deltaTime * k1.W / 2f, SysQuat.Add(qCurrent, SysQuat.Multiply(k1.Q, deltaTime / 2f)), state);

                //position update
                Vector3 pNext = pCurrent + deltaTime * k2.P;
                
                //velocity update
                Vector3 vNext = vCurrent + deltaTime * k2.V;
                
                //angular velocity update
                Vector3 wNext = wCurrent + deltaTime * k2.W;

                //orientation update
                SysQuat qDelta = SysQuat.Multiply(k2.Q, deltaTime);
                SysQuat qNext = SysQuat.Add(qCurrent, qDelta);
                qNext = SysQuat.Normalize(qNext);

                return (pNext, qNext, vNext, wNext);
            }

            
            private (Vector3, SysQuat, Vector3, Vector3) Runge_Kutta_4(float deltaTime,Vector3 pCurrent, SysQuat qCurrent, Vector3 vCurrent, Vector3 wCurrent, BallState state)
            {
                //k1 = f(t_n, y_n)
                K k1 = ComputeK(vCurrent, wCurrent, qCurrent, state);
                
                //k2 = f(t_n + h/2, y_n + k1 * h/2)
                K k2 = ComputeK(vCurrent + deltaTime * k1.V / 2f, wCurrent + deltaTime * k1.W / 2f, SysQuat.Add(qCurrent, SysQuat.Multiply(k1.Q, deltaTime / 2f)), state);
                    
                //k3 = f(t_n + h/2, y_n + k2 * h/2)
                K k3 = ComputeK(vCurrent + deltaTime * k2.V / 2f, wCurrent + deltaTime * k2.W / 2f, SysQuat.Add(qCurrent, SysQuat.Multiply(k2.Q, deltaTime / 2f)), state);
                
                //k4 = f(t_n + h, y_n + k3 * h)
                K k4 = ComputeK(vCurrent + deltaTime * k3.V, wCurrent + deltaTime * k3.W, SysQuat.Add(qCurrent, SysQuat.Multiply(k3.Q, deltaTime)), state);
                
                //velocity update
                Vector3 vNext = vCurrent + deltaTime * (k1.V + 2*k2.V + 2*k3.V + k4.V) / 6;

                //position update
                Vector3 pNext = pCurrent + deltaTime * (k1.P + 2*k2.P + 2*k3.P + k4.P) / 6;
                
                //angular velocity update
                Vector3 wNext = wCurrent + deltaTime * (k1.W + 2*k2.W + 2*k3.W + k4.W) / 6;

                //orientation update
                SysQuat weightedSum = SysQuat.Add(k1.Q, SysQuat.Multiply(k2.Q, 2f));
                weightedSum = SysQuat.Add(weightedSum, SysQuat.Multiply(k3.Q, 2f));
                weightedSum = SysQuat.Add(weightedSum, k4.Q);
                SysQuat deltaQ = SysQuat.Multiply(weightedSum, deltaTime / 6f);
                SysQuat qNext = SysQuat.Add(qCurrent, deltaQ);
                qNext = SysQuat.Normalize(qNext);
                
                return (pNext, qNext, vNext, wNext);
            }
            
            private enum IntegrationMethod
            {
                ExplicitEuler,
                SemiImplicitEuler,
                Rk2,
                Rk4
            }

        #endregion
        
        #region Collision Handling
        
            //this function handles the collision with the ground and returns the new state of the ball after the collision
            private (Vector3, SysQuat, Vector3, Vector3, BallState) HandleGroundCollisionAndBounce(float deltaTime, Vector3 initialPosition, SysQuat initialOrientation, Vector3 initialVelocity, Vector3 initialAngularVelocity, BallState state)
            {
                //finds moment of collision
                float collisionDeltaTime = RecursiveCollisionDetection(deltaTime / 2, deltaTime / 2, initialPosition, initialOrientation, initialVelocity, initialAngularVelocity, state);
                
                //updates position, orientation, velocity and angular velocity after the collision
                (Vector3 positionAfterCollision, SysQuat orientationAfterCollision, Vector3 velocityAfterCollision, Vector3 angularVelocityAfterCollision) = ComputePostCollisionVelocities(collisionDeltaTime, initialPosition, initialOrientation, initialVelocity, initialAngularVelocity, state);
                
                //sets the new state to sliding if the vertical component of the velocity is small enough
                if (velocityAfterCollision.y < 1e-1)
                {
                    velocityAfterCollision.y = 0f;
                    positionAfterCollision.y = ball.radius;
                    state = BallState.Sliding;
                }
                
                return (positionAfterCollision, orientationAfterCollision, velocityAfterCollision, angularVelocityAfterCollision, state);
            }
            
            //this method is called recursively to binary search the moment of collision with the ground
            private float RecursiveCollisionDetection(float deltaTime, float step, Vector3 pCurrent, SysQuat qCurrent, Vector3 vCurrent, Vector3 wCurrent, BallState state)
            {
                //predict new position
                (Vector3 newPosition, _, _, _) = IntegrateMotion(deltaTime, pCurrent, qCurrent, vCurrent, wCurrent, state);
    
                //adjust delta time: go back if below ground, forward if above
                float newDeltaTime = newPosition.y - ball.radius < 0f ? deltaTime - step / 2 : deltaTime + step / 2;
    
                //stop if step is below precision threshold
                if (step < TimeStepThreshold)
                    return newDeltaTime;
    
                //recurse with new delta time and halved step size
                return RecursiveCollisionDetection(newDeltaTime, step/2, pCurrent, qCurrent, vCurrent, wCurrent, state);
            }
            private (Vector3, SysQuat, Vector3, Vector3) ComputePostCollisionVelocities(float collisionDeltaTime, Vector3 initialPosition, SysQuat initialOrientation, Vector3 initialVelocity, Vector3 initialAngularVelocity, BallState state)
            {
                //here the movement is integrated until the moment the ball collides with the ground
                (Vector3 collisionPoint, SysQuat collisionOrientation, Vector3 velocityBeforeCollision, Vector3 angularVelocityBeforeCollision) = IntegrateMotion(collisionDeltaTime, initialPosition, initialOrientation, initialVelocity, initialAngularVelocity, state);
                
                Vector3 velocityAfterCollision = new Vector3();
                Vector3 angularVelocityAfterCollision = new Vector3();
                
                //the new velocity after the collision is computed
                velocityAfterCollision.x = velocityBeforeCollision.x * (1 - ball.inertialMomentumConstant * ball.coefficientOfHorizontalRestitution) / 
                    (ball.inertialMomentumConstant + 1) - ball.inertialMomentumConstant * (1 + ball.coefficientOfHorizontalRestitution) / (ball.inertialMomentumConstant + 1) * ball.radius * angularVelocityBeforeCollision.z;
                
                velocityAfterCollision.y = -ball.coefficientOfVerticalRestitution * velocityBeforeCollision.y;
                
                velocityAfterCollision.z = velocityBeforeCollision.z * (1 - ball.inertialMomentumConstant * ball.coefficientOfHorizontalRestitution) / 
                    (ball.inertialMomentumConstant + 1) + ball.inertialMomentumConstant * (1 + ball.coefficientOfHorizontalRestitution) / (ball.inertialMomentumConstant + 1) * ball.radius * angularVelocityBeforeCollision.x;

                //the new angular velocity after the collision is computed
                angularVelocityAfterCollision.x = (ball.inertialMomentumConstant-ball.coefficientOfHorizontalRestitution)/(ball.inertialMomentumConstant+1) * 
                    angularVelocityBeforeCollision.x + (ball.coefficientOfHorizontalRestitution+1)/(ball.inertialMomentumConstant+1) * velocityBeforeCollision.z/ball.radius;
                
                angularVelocityAfterCollision.y = angularVelocityBeforeCollision.y * ball.coefficientOfVerticalAxisSpinRestitution;
                
                angularVelocityAfterCollision.z = (ball.inertialMomentumConstant-ball.coefficientOfHorizontalRestitution)/(ball.inertialMomentumConstant+1) * 
                    angularVelocityBeforeCollision.z - (ball.coefficientOfHorizontalRestitution+1)/(ball.inertialMomentumConstant+1) * velocityBeforeCollision.x/ball.radius;
                
                //calculate the remaining time until the end of the frame and the  is integrated using it
                float remainingTime = Time.fixedDeltaTime - collisionDeltaTime;
                return IntegrateMotion(remainingTime, collisionPoint, collisionOrientation, velocityAfterCollision, angularVelocityAfterCollision, state);
            }
            
        #endregion
        
        #region Forces And Torques
            
            //this method computes the force caused by the air resistance
            private Vector3 CalculateDragForce(Vector3 velocity, Vector3 angularVelocity)
            {
                if(!useAerodynamicForces || velocity.magnitude < 1e-3)
                    return Vector3.zero;
                
                Vector3 dragForce;
                
                if(ball.useConstantCoefficients)
                {
                    //use constant linear drag coefficient
                    dragForce = -0.5f * airDensity * ball.constantDragCoefficient * Mathf.PI * Mathf.Pow(ball.radius, 2) * velocity.magnitude * velocity;
                }
                else
                //use empirical model
                {
                    //spin rate parameter
                    float s = angularVelocity.magnitude * ball.radius / velocity.magnitude;

                    if (velocity.magnitude > ball.speedOfTransitionFromLaminarToTurbulentDrag && s > 0.05f)
                        //if the ball is spinning fast enough and moving faster than critical speed
                        ball.dragCoefficient = 0.4127f * Mathf.Pow(s, 0.3056f);
                    else
                        //otherwise use the drag coefficient model depending on Reynolds numbers
                        ball.dragCoefficient = 0.155f + 0.346f / (1 + Mathf.Exp((velocity.magnitude - ball.speedOfTransitionFromLaminarToTurbulentDrag) / ball.dragTransitionSlope));

                    dragForce = -0.5f * airDensity * ball.dragCoefficient * Mathf.PI * Mathf.Pow(ball.radius, 2) * velocity.magnitude * velocity;
                }
                
                return dragForce;
            }
            
            
            private Vector3 CalculateMagnusForce(Vector3 velocity, Vector3 angularVelocity, BallState state)
            {
                if(!useAerodynamicForces || velocity.magnitude < 1e-3 || angularVelocity.magnitude < 1e-3 || state != BallState.Bouncing)
                    return Vector3.zero;
                
                Vector3 magnusForce;

                if (ball.useConstantCoefficients)
                {
                    //use constant linear drag coefficient
                    magnusForce = ball.constantDragCoefficient * Mathf.PI * airDensity * Mathf.Pow(ball.radius, 3) * Vector3.Cross(angularVelocity, velocity);
                }
                else
                //use empirical model
                {
                    //direction cross product
                    Vector3 direction = Vector3.Cross(angularVelocity.normalized, velocity.normalized);
                    
                    //spin rate parameter, how strong the spin is compared to the speed
                    float s = angularVelocity.magnitude * ball.radius / velocity.magnitude;
                    
                    float cdRe0 = 0.155f + 0.346f / (1 + Mathf.Exp(-ball.speedOfTransitionFromLaminarToTurbulentDrag) / ball.dragTransitionSlope);
                    float cdReRef = 0.155f;
                    
                    float dragCoefficient;
                    if (velocity.magnitude > ball.speedOfTransitionFromLaminarToTurbulentDrag && s > 0.05f)
                        //if the ball is spinning fast enough and moving faster than critical speed
                        dragCoefficient = 0.4127f * Mathf.Pow(s, 0.3056f);
                    else
                        //otherwise use the drag coefficient model depending on Reynolds numbers
                        dragCoefficient = 0.155f + 0.346f / (1 + Mathf.Exp((velocity.magnitude - ball.speedOfTransitionFromLaminarToTurbulentDrag) / ball.dragTransitionSlope));


                    ball.liftCoefficient = 1.15f * Mathf.Pow(s, 0.83f) * (cdRe0 -  dragCoefficient) / (cdRe0 - cdReRef);
                    
                    magnusForce = 0.5f * airDensity * ball.liftCoefficient * Mathf.PI * Mathf.Pow(ball.radius, 2) * Mathf.Pow(velocity.magnitude, 2) * direction;
                }
                
                return magnusForce;
            }
            
            //this method computes this friction force with the ground, given the velocity, the angular velocity and the current state of the ball
            private Vector3 CalculateFrictionForce(Vector3 velocity, Vector3 angularVelocity, BallState state)
            {
                if(state == BallState.Bouncing)
                    return Vector3.zero;
                
                Vector3 frictionForce;
                
                float normalForce = (_gravitationalForce + _buoyantForce).magnitude;
                
                //if the ball is sliding, use the sliding friction formula
                if(state == BallState.Sliding) 
                {
                    //inclination in of the ball axis against the up-axis in radians
                    float theta = (90f - Vector3.Angle(angularVelocity.normalized, Vector3.up)) * Mathf.Deg2Rad;

                    //compute direction of the velocity of the contact point
                    Vector3 collisionPointVelocity = (velocity + Vector3.Cross(ball.radius * Mathf.Cos(theta) * Vector3.up, angularVelocity)).normalized;
                    
                    frictionForce = -ball.coefficientOfSlidingFriction * normalForce * collisionPointVelocity;
                }
                else
                {
                    //otherwise rolling friction formula
                    frictionForce = -ball.coefficientOfRollingFriction * normalForce * velocity.normalized;
                }
                

                if (velocity.magnitude < 1e-3)
                {
                    frictionForce *= velocity.magnitude;
                }
            
                return frictionForce;
            }

            private Vector3 CalculateNormalForce(BallState state)
            {
                return state != BallState.Bouncing ? - _gravitationalForce - _buoyantForce : Vector3.zero;
            }
            
            //this method computes the result of the sum of all forces acting on the ball
            private Vector3 CalculateTotalForce(Vector3 velocity, Vector3 angularVelocity, BallState state)
            {
                Vector3 dragForce = CalculateDragForce(velocity, angularVelocity);
                Vector3 magnusForce = CalculateMagnusForce(velocity, angularVelocity, state);
                Vector3 normalForce = CalculateNormalForce(state);
                Vector3 frictionForce = CalculateFrictionForce(velocity, angularVelocity, state);
                
                //sum all forces
                Vector3 totalForce = _gravitationalForce + _buoyantForce + dragForce + magnusForce + normalForce + frictionForce;

                return totalForce;
            }
            
            //this method computes the torque acting on the ball created by the friction with the ground, given velocity, angular velocity and the state of the ball
            private Vector3 CalculateFrictionTorque(Vector3 velocity, Vector3 angularVelocity, Vector3 acceleration, BallState state)
            { 
                if(state == BallState.Bouncing)
                    return Vector3.zero;
                
                float normalForce = (_gravitationalForce + _buoyantForce).magnitude;
                
                Vector3 frictionTorque = new Vector3(0, - ball.coefficientOfVerticalAxisSpinningDamping * Mathf.Sign(angularVelocity.y), 0);
                
                //if the ball is rolling, the contact point is not moving with respect to the ground, therefore the is no other friction component acting on the ball
                if (state == BallState.Rolling)
                {
                    frictionTorque += Vector3.Cross(acceleration / ball.radius, Vector3.down) * ball.InertialMomentum;
                }
                else
                {
                    //inclination in of the ball axis against the up-axis in radians
                    float theta = (90f - Vector3.Angle(angularVelocity.normalized, Vector3.up)) * Mathf.Deg2Rad;

                    //compute direction of the velocity of the contact point
                    Vector3 frictionDirection = velocity + Vector3.Cross(ball.radius * Mathf.Cos(theta) * Vector3.up, angularVelocity).normalized;

                    //compute the axis on which the torque acts
                    Vector3 axisOfTorque = Vector3.Cross(Vector3.up, frictionDirection).normalized;

                    Vector3 angularAcceleration = ball.coefficientOfSlidingFriction * normalForce * ball.radius * axisOfTorque / ball.InertialMomentum;

                    frictionTorque += angularAcceleration * ball.InertialMomentum;
                }

                if (angularVelocity.y < 1)
                {
                    frictionTorque.y *= Mathf.Abs(angularVelocity.y);
                }
                
                return frictionTorque;
            }
            
            private Vector3 CalculateSpinDegradation(Vector3 velocity, Vector3 angularVelocity, BallState state)
            {
                if(state != BallState.Bouncing || angularVelocity.magnitude < 1e-6 || !useAerodynamicForces)
                    return Vector3.zero;
    
                float totalForce;

                if (ball.useConstantCoefficients)
                { 
                    //use constant coefficient
                    totalForce = - 0.5f * airDensity * Mathf.PI * Mathf.Pow(ball.radius, 2) * Mathf.Pow(velocity.magnitude, 2) * ball.constantSpinDecayCoefficient; 
                }
                else
                {
                    //use empirical formula
                    
                    //calculate the relative velocity of both side of the side
                    float utP = Mathf.Abs(Vector3.Cross(velocity, angularVelocity.normalized).magnitude + ball.radius / 2 * angularVelocity.magnitude);
                    float utM = Mathf.Abs(Vector3.Cross(velocity, angularVelocity.normalized).magnitude - ball.radius / 2 * angularVelocity.magnitude);
        
                    //coefficient of each side
                    float cfP = 0.0074f * Mathf.Pow(utP * airDensity * ball.radius * 2 / _kinematicViscosityOfAir, -1f / 5f);
                    float cfM = 0.0074f * Mathf.Pow(utM * airDensity * ball.radius * 2 / _kinematicViscosityOfAir, -1f / 5f);
        
                    totalForce = - airDensity * Mathf.PI * Mathf.Pow(ball.radius, 2) * (Mathf.Pow(utP, 2) * cfP + Mathf.Pow(utM, 2) * cfM);
                }
    
                Vector3 spinDegradationTorque = totalForce * ball.radius / 2 * angularVelocity.normalized;

                return spinDegradationTorque;
            }
            
            //this method computes the result of the sum of all torques acting on the ball
            private Vector3 CalculateTotalTorque(Vector3 velocity, Vector3 angularVelocity, Vector3 acceleration, BallState state)
            {
                Vector3 frictionTorque = CalculateFrictionTorque(velocity, angularVelocity, acceleration, state);
                Vector3 spinDegradationTorque = CalculateSpinDegradation(velocity, angularVelocity, state);
                Vector3 totalTorque = frictionTorque + spinDegradationTorque;
                
                return totalTorque;
            }

            private (Vector3, Vector3) CalculateAccelerationAndAngularAcceleration(Vector3 velocity, Vector3 angularVelocity, BallState state)
            {
                Vector3 acceleration = CalculateTotalForce(velocity, angularVelocity, state) / ball.mass;
                Vector3 angularAcceleration = CalculateTotalTorque(velocity, angularVelocity, acceleration, state) / ball.InertialMomentum;
                
                return (acceleration, angularAcceleration);
            }
            
        #endregion
    
    #endregion
    
    #region Trajectory Prediction
       private (Vector3, Vector3) InitialVelocityGuess(Vector3 initialPosition, Vector3 targetPosition, float speed, float verticalAngle, Vector3 spin)
        {
            Vector3 direction = new Vector3(targetPosition.x - initialPosition.x, 0, targetPosition.z - initialPosition.z);
            
            direction = ElevateVector(direction, verticalAngle).normalized;
            
            Vector3 bestVelocityEstimation = direction * speed;
            Vector3 alignedSpin = AlignSpinWithVelocityDirection(bestVelocityEstimation, spin);
            
            return (bestVelocityEstimation, alignedSpin);
        }
       
        private (Vector3, Vector3) PredictInitialValuesMethodA(float timeStep, Vector3 targetPosition, Vector3 initialPosition, float speed, Vector3 spin, float minVerticalAngle, float maxVerticalAngle, int maximumIterations, bool constraintShot = false, Vector3 constraintPosition = default, bool testing = false)
        { 
            int iterationCounter = 1;
            
            float distanceFromTarget = Vector3.Distance(initialPosition, targetPosition);
            
            Stopwatch stopwatch = Stopwatch.StartNew();
            
            (Vector3 bestVelocityEstimation, Vector3 alignedSpin) = InitialVelocityGuess(initialPosition, targetPosition, speed, maxVerticalAngle, spin);
            (Vector3 predictedPosition,_) = EstimateTrajectoryClosestPoint(timeStep, initialPosition, SysQuat.Identity, bestVelocityEstimation, alignedSpin, targetPosition, constraintShot, constraintPosition);
            
            bestVelocityEstimation = RotateVelocityTowardTarget(initialPosition, predictedPosition, targetPosition, bestVelocityEstimation);
            bestVelocityEstimation = ElevateVector(bestVelocityEstimation, maxVerticalAngle);
            alignedSpin = AlignSpinWithVelocityDirection(bestVelocityEstimation, spin);
            (predictedPosition,_) = EstimateTrajectoryClosestPoint(timeStep, initialPosition, SysQuat.Identity, bestVelocityEstimation, alignedSpin, targetPosition, constraintShot, constraintPosition);
            
            bestVelocityEstimation = RotateVelocityTowardTarget(initialPosition, predictedPosition, targetPosition, bestVelocityEstimation);
            bestVelocityEstimation = ElevateVector(bestVelocityEstimation, maxVerticalAngle);
            alignedSpin = AlignSpinWithVelocityDirection(bestVelocityEstimation, spin);
            (predictedPosition,_) = EstimateTrajectoryClosestPoint(timeStep, initialPosition, SysQuat.Identity, bestVelocityEstimation, alignedSpin, targetPosition, constraintShot, constraintPosition);
            
            float errorInCentimeters = Vector3.Distance(predictedPosition, targetPosition) * 1e2f;
            
            if (adjustSpeed && predictedPosition.y < targetPosition.y)
            {                
                int correctionCounter = 0;
                while (correctionCounter < speedCorrectionSteps && predictedPosition.y < targetPosition.y)
                {
                    iterationCounter++;
                    
                    bestVelocityEstimation *= Mathf.Pow(maxSpeedIncrease, 1f/speedCorrectionSteps);
                    bestVelocityEstimation = ElevateVector(bestVelocityEstimation, maxVerticalAngle);
                    (predictedPosition,_) = EstimateTrajectoryClosestPoint(timeStep, initialPosition, SysQuat.Identity, bestVelocityEstimation, alignedSpin, targetPosition, constraintShot, constraintPosition);
                    correctionCounter++;
                }
                
                bestVelocityEstimation = bestVelocityEstimation.normalized * bestVelocityEstimation.magnitude;
                (predictedPosition,_) = EstimateTrajectoryClosestPoint(timeStep, initialPosition, SysQuat.Identity, bestVelocityEstimation, alignedSpin, targetPosition, constraintShot, constraintPosition);
                errorInCentimeters = Vector3.Distance(predictedPosition, targetPosition) * 1e2f;
            }
            
            Vector3 minElevatedVelocity = ElevateVector(bestVelocityEstimation, minVerticalAngle);
            (Vector3 minElevationImpactPoint,_) = EstimateTrajectoryClosestPoint(timeStep, initialPosition, SysQuat.Identity, minElevatedVelocity, alignedSpin, targetPosition, constraintShot, constraintPosition);
            _precalculationTrajectories.RemoveAt(_precalculationTrajectories.Count-1);
            
            if(predictedPosition.y > targetPosition.y && minElevationImpactPoint.y < targetPosition.y)
            {
                float previousError = errorInCentimeters;
                
                //keeps iterating until either the error gets smaller than the maximum tolerated error or when the iteration counter reaches the maximum
                while (errorInCentimeters > maxErrorInCentimeters && iterationCounter < maximumIterations)
                {
                    iterationCounter++;

                    Vector3 currentVelocityEstimation = RotateVelocityTowardTarget(initialPosition, predictedPosition, targetPosition, bestVelocityEstimation);
                    alignedSpin = AlignSpinWithVelocityDirection(currentVelocityEstimation, spin);

                    //compute the closest point to the target with current rotation
                    (predictedPosition, _) = EstimateTrajectoryClosestPoint(timeStep, initialPosition, SysQuat.Identity, currentVelocityEstimation, alignedSpin, targetPosition, constraintShot, constraintPosition);

                    errorInCentimeters = Vector3.Distance(predictedPosition, targetPosition) * 1e2f;
                    
                    bestVelocityEstimation = currentVelocityEstimation;
                    
                    if(Mathf.Abs(errorInCentimeters - previousError) < 1e-3f)
                        break;

                    previousError = errorInCentimeters;
                }
            }            
            else
            {
                if (minElevationImpactPoint.y > targetPosition.y)
                    bestVelocityEstimation = minElevatedVelocity;
            }
            
            stopwatch.Stop();
            
            if(!constraintShot)
            {
                EstimateTrajectoryClosestPoint(timeStep, initialPosition, SysQuat.Identity, bestVelocityEstimation, alignedSpin, targetPosition, false, constraintPosition);
                _precalculationTrajectories.RemoveAt(_precalculationTrajectories.Count - 1);

                (predictedPosition, _) = EstimateTrajectoryPathToTargetAccurate(1 / framePerSecond, initialPosition, SysQuat.Identity, bestVelocityEstimation, alignedSpin, targetPosition, false, constraintPosition);
                closestPointToTargetPlaceholder.transform.position = predictedPosition;
                errorInCentimeters = Vector3.Distance(predictedPosition, targetPosition) * 1e2f;

                DebugAndTest(false, stopwatch, iterationCounter, errorInCentimeters, testing, distanceFromTarget, bestVelocityEstimation, alignedSpin);
            }
            
            return (bestVelocityEstimation, alignedSpin);
        }
            
        private (Vector3, Vector3) PredictInitialValuesMethodB(float timeStep, Vector3 targetPosition, Vector3 initialPosition, float speed, Vector3 spin, float minVerticalAngle, float maxVerticalAngle, int maximumIterations, bool constraintShot = false, Vector3 constraintPosition = default, bool testing = false)
        { 
            int iterationCounter = 1;
            
            float distanceFromTarget = Vector3.Distance(initialPosition, targetPosition);
            
            Stopwatch stopwatch = Stopwatch.StartNew();
            
            (Vector3 bestVelocityEstimation, Vector3 alignedSpin) = InitialVelocityGuess(initialPosition, targetPosition, speed, maxVerticalAngle, spin);
            (Vector3 predictedPosition,_) = EstimateTrajectoryClosestPoint(timeStep, initialPosition, SysQuat.Identity, bestVelocityEstimation, alignedSpin, targetPosition, constraintShot, constraintPosition);
            
            bestVelocityEstimation = RotateVelocityTowardTarget(initialPosition, predictedPosition, targetPosition, bestVelocityEstimation);
            bestVelocityEstimation = ElevateVector(bestVelocityEstimation, maxVerticalAngle);
            alignedSpin = AlignSpinWithVelocityDirection(bestVelocityEstimation, spin);
            (predictedPosition,_) = EstimateTrajectoryClosestPoint(timeStep, initialPosition, SysQuat.Identity, bestVelocityEstimation, alignedSpin, targetPosition, constraintShot, constraintPosition);
            
            bestVelocityEstimation = RotateVelocityTowardTarget(initialPosition, predictedPosition, targetPosition, bestVelocityEstimation);
            bestVelocityEstimation = ElevateVector(bestVelocityEstimation, maxVerticalAngle);
            alignedSpin = AlignSpinWithVelocityDirection(bestVelocityEstimation, spin);
            (predictedPosition,_) = EstimateTrajectoryClosestPoint(timeStep, initialPosition, SysQuat.Identity, bestVelocityEstimation, alignedSpin, targetPosition, constraintShot, constraintPosition);

            float errorInCentimeters = Vector3.Distance(predictedPosition, targetPosition) * 1e2f;
            
            if (adjustSpeed && predictedPosition.y < targetPosition.y)
            {                
                int correctionCounter = 0;
                while (correctionCounter < speedCorrectionSteps && predictedPosition.y < targetPosition.y)
                {
                    iterationCounter++;
                    
                    bestVelocityEstimation *= Mathf.Pow(maxSpeedIncrease, 1f/speedCorrectionSteps);
                    bestVelocityEstimation = ElevateVector(bestVelocityEstimation, maxVerticalAngle);
                    (predictedPosition,_) = EstimateTrajectoryClosestPoint(timeStep, initialPosition, SysQuat.Identity, bestVelocityEstimation, alignedSpin, targetPosition, constraintShot, constraintPosition);
                    correctionCounter++;
                }
                
                bestVelocityEstimation = bestVelocityEstimation.normalized * bestVelocityEstimation.magnitude;
                (predictedPosition,_) = EstimateTrajectoryClosestPoint(timeStep, initialPosition, SysQuat.Identity, bestVelocityEstimation, alignedSpin, targetPosition, constraintShot, constraintPosition);
                errorInCentimeters = Vector3.Distance(predictedPosition, targetPosition) * 1e2f;
            }
            
            
            Vector3 minElevatedVelocity = ElevateVector(bestVelocityEstimation, minVerticalAngle);
            (Vector3 minElevationImpactPoint,_) = EstimateTrajectoryClosestPoint(timeStep, initialPosition, SysQuat.Identity, minElevatedVelocity, alignedSpin, targetPosition, constraintShot, constraintPosition);
            
            if(predictedPosition.y > targetPosition.y && minElevationImpactPoint.y < targetPosition.y)
            {
                float elevationAngleLowLimit = minVerticalAngle;
                float elevationAngleHighLimit = maxVerticalAngle;
                
                //keeps iterating until either the error gets smaller than the maximum tolerated error or when the iteration counter reaches the maximum
                while (errorInCentimeters > maxErrorInCentimeters && iterationCounter < maximumIterations)
                {
                    iterationCounter++;
                    
                    float elevationAngle = (elevationAngleLowLimit + elevationAngleHighLimit) / 2;

                    Vector3 currentVelocityEstimation = RotateVelocityTowardTarget(initialPosition, predictedPosition, targetPosition, bestVelocityEstimation);
                    currentVelocityEstimation = ElevateVector(currentVelocityEstimation, elevationAngle);
                    alignedSpin = AlignSpinWithVelocityDirection(currentVelocityEstimation, spin);

                    //compute the closest point to the target with current rotation
                    (predictedPosition, _) = EstimateTrajectoryClosestPoint(timeStep, initialPosition, SysQuat.Identity, currentVelocityEstimation, alignedSpin, targetPosition, constraintShot, constraintPosition);

                    errorInCentimeters = Vector3.Distance(predictedPosition, targetPosition) * 1e2f;
                    
                    if (predictedPosition.y < targetPosition.y)
                        elevationAngleLowLimit = elevationAngle;
                    else
                        elevationAngleHighLimit = elevationAngle;
                    
                    bestVelocityEstimation = currentVelocityEstimation;
                }
            }            
            else
            {
                if (minElevationImpactPoint.y > targetPosition.y)
                    bestVelocityEstimation = minElevatedVelocity;
            }
            
            stopwatch.Stop();
            
            EstimateTrajectoryClosestPoint(timeStep, initialPosition, SysQuat.Identity, bestVelocityEstimation, alignedSpin, targetPosition, constraintShot, constraintPosition);
            
            (predictedPosition, _) = EstimateTrajectoryPathToTargetAccurate(1/framePerSecond, initialPosition, SysQuat.Identity, bestVelocityEstimation, alignedSpin, targetPosition, constraintShot, constraintPosition);
            closestPointToTargetPlaceholder.transform.position = predictedPosition;
            errorInCentimeters = Vector3.Distance(predictedPosition, targetPosition) * 1e2f;
            
            DebugAndTest(constraintShot, stopwatch, iterationCounter, errorInCentimeters, testing, distanceFromTarget, bestVelocityEstimation, alignedSpin);
            
            return (bestVelocityEstimation, alignedSpin);
        }
        
        private (Vector3, Vector3)  PredictInitialValuesTwoConstraints(float deltaTime, Vector3 targetPosition, Vector3 constraintPosition, Vector3 initialPosition, float speed, int maximumIterations, bool testing = false)
        { 
            Stopwatch stopwatch = Stopwatch.StartNew();
            
            float distanceFromTarget = Vector3.Distance(initialPosition, targetPosition);
            int iterationCounter = 1;

            //this rotation is needed to have the displacement of the shot in the coordinate system of the XY plane
            //in this way the displacement can be computed from any direction
            Quaternion displacementXYPlaneRot = Quaternion.FromToRotation(new Vector3(initialPosition.x - targetPosition.x, 0, initialPosition.z - targetPosition.z), Vector3.left);
            
            //first velocity guess, pointing from the initial position to the constraint with no spin
            Vector3 bestEstimatedVelocity = (constraintPosition - initialPosition).normalized * speed;
            
            (Vector3 noSpinPredictedTargetPosition,_) = EstimateTrajectoryClosestPoint(deltaTime, initialPosition, SysQuat.Identity, bestEstimatedVelocity, Vector3.zero, targetPosition, true, constraintPosition);

            //finds displacement of the previous shot from the target
            Vector3 targetDisplacement = displacementXYPlaneRot * (noSpinPredictedTargetPosition - targetPosition).normalized;
            
            //sets both horizontal and vertical spin magnitude to the possible maximum values
            float horizontalSpinMagnitude = ball.maximumSpinValue;
            float verticalSpinMagnitude = ball.maximumSpinValue;
            
            //choose the direction of the spin based on the sign of the displacement from the target of the previous shot and set the magnitude to the maximum possible value
            Vector3 spin = new Vector3(0, Math.Sign(targetDisplacement.z) * horizontalSpinMagnitude, -Math.Sign(targetDisplacement.y) * verticalSpinMagnitude);
            
            //align spin with the velocity
            Vector3 bestEstimatedSpin = AlignSpinWithVelocityDirection(bestEstimatedVelocity, spin);
            
            //compute the trajectory with maximum spin and save the closest target position and closest constraint position
            (Vector3 bestSpinPredictedTargetPosition,Vector3 bestSpinPredictedConstraintPosition) = EstimateTrajectoryClosestPoint(deltaTime, initialPosition, SysQuat.Identity, bestEstimatedVelocity, bestEstimatedSpin, targetPosition, true, constraintPosition);
            
            //find target error and constraint error of the trajectory
            float targetError = Vector3.Distance(bestSpinPredictedTargetPosition, targetPosition) * 1e2f;
            float constraintError = Vector3.Distance(bestSpinPredictedConstraintPosition, constraintPosition) * 1e2f;

            float previousTargetError = float.PositiveInfinity;
            
            //the cycle stops when either the iteration counter reaches the maximum or when both the target and the constraint errors are below their maximum value
            while (iterationCounter < maximumIterations && (targetError > maxErrorInCentimeters || constraintError > constraintMaximumError))
            {
                iterationCounter++;
                
                //HORIZONTAL//
                
                //computes trajectory having no horizontal spin component
                Vector3 noHorizontalSpin = bestEstimatedSpin;
                noHorizontalSpin.y = 0f;
                (Vector3 noHorizontalSpinPredictedTargetPosition,_) = EstimateTrajectoryClosestPoint(deltaTime, initialPosition, SysQuat.Identity, bestEstimatedVelocity, noHorizontalSpin, targetPosition, true, constraintPosition);
                
                //compute the target displacement of the shot without horizontal spin
                targetDisplacement = displacementXYPlaneRot * (noHorizontalSpinPredictedTargetPosition - targetPosition).normalized;
                
                //finds the new horizontal spin proportionally to the displacement of the predicted target of the shot with no horizontal spin from both the actual target position and the predicted position of the previous best found spin
                float d1 = Mathf.Abs((displacementXYPlaneRot * (bestSpinPredictedTargetPosition - noHorizontalSpinPredictedTargetPosition)).z);
                float d2 = Mathf.Abs((displacementXYPlaneRot * (targetPosition - noHorizontalSpinPredictedTargetPosition)).z); 
                horizontalSpinMagnitude = Mathf.Min(horizontalSpinMagnitude * d2 / d1, ball.maximumSpinValue);
                spin.y = Math.Sign(targetDisplacement.z) * horizontalSpinMagnitude;
                Vector3 newEstimatedSpin = bestEstimatedSpin;
                newEstimatedSpin.y = spin.y;
                
                //compute trajectory with the new-found horizontal spin and align spin with the velocity
                (_, Vector3 predictedConstraintPosition) = EstimateTrajectoryClosestPoint(deltaTime, initialPosition, SysQuat.Identity, bestEstimatedVelocity, newEstimatedSpin, targetPosition, true, constraintPosition);
                Vector3 newEstimatedVelocity = RotateVelocityTowardTarget(initialPosition, predictedConstraintPosition, constraintPosition, bestEstimatedVelocity);
                newEstimatedSpin = AlignSpinWithVelocityDirection(newEstimatedVelocity, spin);

                //compute new trajectory to find new predicted target position
                (bestSpinPredictedTargetPosition,_) = EstimateTrajectoryClosestPoint(deltaTime, initialPosition, SysQuat.Identity, newEstimatedVelocity, newEstimatedSpin, targetPosition, true, constraintPosition);
                
                //VERTICAL//
                
                //computes trajectory having no vertical spin component
                Vector3 noVerticalSpin = spin;
                noVerticalSpin.z = 0f;
                (Vector3 noVerticalSpinPredictedTargetPosition,_) = EstimateTrajectoryClosestPoint(deltaTime, initialPosition, SysQuat.Identity, newEstimatedVelocity, noVerticalSpin, targetPosition, true, constraintPosition);

                //compute the target displacement of the shot without vertical spin
                targetDisplacement = displacementXYPlaneRot * (noVerticalSpinPredictedTargetPosition - targetPosition).normalized;

                //finds the new vertical spin proportionally to the displacement of the predicted target of the shot with no vertical spin from both the actual target position and the predicted position of the previous best found spin
                d1 = bestSpinPredictedTargetPosition.y - noVerticalSpinPredictedTargetPosition.y;
                d2 = targetPosition.y - noVerticalSpinPredictedTargetPosition.y;
                verticalSpinMagnitude = Mathf.Min(verticalSpinMagnitude * Math.Abs(d2 / d1), ball.maximumSpinValue);
                spin.z = -Math.Sign(targetDisplacement.y) * verticalSpinMagnitude;
                newEstimatedSpin = AlignSpinWithVelocityDirection(newEstimatedVelocity, spin);
                
                //computes trajectory with the new-found vertical spin and align spin with the velocity
                (_, predictedConstraintPosition) = EstimateTrajectoryClosestPoint(deltaTime, initialPosition, SysQuat.Identity, newEstimatedVelocity, newEstimatedSpin, targetPosition, true, constraintPosition);
                newEstimatedVelocity  = RotateVelocityTowardTarget(initialPosition, predictedConstraintPosition, constraintPosition, newEstimatedVelocity);
                newEstimatedSpin = AlignSpinWithVelocityDirection(newEstimatedVelocity, spin);
                
                (bestSpinPredictedTargetPosition,bestSpinPredictedConstraintPosition) = EstimateTrajectoryClosestPoint(deltaTime, initialPosition, SysQuat.Identity, newEstimatedVelocity, newEstimatedSpin, targetPosition, true, constraintPosition);
                _precalculationTrajectories.Add(_currentTrajectory);
                
                //finds distance from target and from constraint of the trajectory
                targetError = Vector3.Distance(bestSpinPredictedTargetPosition, targetPosition) * 1e2f;
                constraintError = Vector3.Distance(bestSpinPredictedConstraintPosition, constraintPosition) * 1e2f;
                
                
                //if the target error doesn't improve because the maximum possible spin for one of the two components reaches the maximum value, then the prediction is interrupted
                if ((Mathf.Abs(targetError - previousTargetError ) < 1e-3 || targetError > previousTargetError) && (Mathf.Abs(spin.y) - ball.maximumSpinValue < 1e-3f || Mathf.Abs(spin.z) - ball.maximumSpinValue < 1e-3f))
                {
                    iterationCounter++;
                        
                    //use the predictor with no constraint with the found spin to have the trajectory prioritizing the target but passing in the closest possible point to the constraint
                    (newEstimatedVelocity, newEstimatedSpin) = PredictInitialValuesMethodA(deltaTime, targetPosition, initialPosition, speed, spin, elevationAngleMinimumLimit, elevationAngleMaximumLimit, 10, true, bestEstimatedVelocity);

                    //update best velocity estimation and best spin estimation
                    bestEstimatedVelocity = newEstimatedVelocity;
                    bestEstimatedSpin = newEstimatedSpin;
                    
                    break;
                }
                
                //update best velocity estimation
                bestEstimatedVelocity = newEstimatedVelocity;
                bestEstimatedSpin = newEstimatedSpin;
                
                //update previous error
                previousTargetError = targetError;
            }
            
            stopwatch.Stop();
            
            //debugging predicted position
            (bestSpinPredictedTargetPosition,bestSpinPredictedConstraintPosition) = EstimateTrajectoryClosestPoint(deltaTime, initialPosition, SysQuat.Identity, bestEstimatedVelocity, bestEstimatedSpin, targetPosition, true, constraintPosition);
            _precalculationTrajectories.Add(_currentTrajectory);
            
            //finds distance from target and from constraint of the trajectory
            targetError = Vector3.Distance(bestSpinPredictedTargetPosition, targetPosition) * 1e2f;
            constraintError = Vector3.Distance(bestSpinPredictedConstraintPosition, constraintPosition) * 1e2f;

            closestPointToTargetPlaceholder.transform.position = bestSpinPredictedTargetPosition;
            closestPointToConstraintPlaceholder.transform.position = bestSpinPredictedConstraintPosition;

            DebugAndTest(false, stopwatch, iterationCounter, targetError, testing, distanceFromTarget, bestEstimatedVelocity, bestEstimatedSpin, constraintError);
            
            return (bestEstimatedVelocity,bestEstimatedSpin);
        }
        
        //given the initial conditions of the ball, the method will compute the trajectory from the initialPosition until the closest position to the target
        //the method returns the closest point of the trajectory to the target and the closest point to the constraint
        private (Vector3,Vector3) EstimateTrajectoryClosestPoint(float deltaTime, Vector3 initialPosition, SysQuat initialOrientation, Vector3 initialVelocity,  Vector3 initialAngularVelocity, Vector3 targetPosition, bool constraintShot, Vector3 constraintPosition = default)
        {
            _currentTrajectory = new List<Vector3>();
            
            bool stopSimulation = false;
            bool foundClosestConstraintPoint = false;

            //initialize useful state variables
            //previous state variables are needed to go back in time when the ball passes to the closest point 
            Vector3 currentPosition = initialPosition;
            SysQuat currentOrientation = initialOrientation;
            Vector3 currentVelocity = initialVelocity;
            Vector3 currentAngularVelocity = initialAngularVelocity;

            Vector3 targetClosestPoint = currentPosition;
            Vector3 constraintClosestPoint = currentPosition;
            
            while(!stopSimulation)
            {
                //integrate the next step on the motion
                (Vector3 newPosition, SysQuat newOrientation, Vector3 newVelocity, Vector3 newAngularVelocity) = IntegrateMotion(deltaTime, currentPosition,  currentOrientation, currentVelocity, currentAngularVelocity, BallState.Bouncing);
                _currentTrajectory.Add(currentPosition);
                
                // Calculate signed distances to the plane
                Vector3 planeNormal = initialPosition - targetPosition;
                planeNormal.y = 0;
                float dCurrent = Vector3.Dot(currentPosition - targetPosition, planeNormal);
                float dNew = Vector3.Dot(newPosition - targetPosition, planeNormal);
                
                targetClosestPoint = currentPosition;
                // Check if there's a sign change (crossed the plane)
                if (dCurrent * dNew < 0)
                {
                    stopSimulation = true;
                    targetClosestPoint = GetSegmentPlaneIntersection(currentPosition, newPosition,targetPosition, planeNormal);
                }
                
                if (currentPosition.y - ball.radius < 0)
                {
                    stopSimulation = true;
                    targetClosestPoint = currentPosition;
                }
                
                //in case of shot with constraint find the closest point of trajectory to the constraint
                if(constraintShot && !foundClosestConstraintPoint)
                {
                    // Calculate signed distances to the plane
                    Vector3 planeNormal2 = initialPosition -  constraintPosition;
                    planeNormal2.y = 0;
                    float dCurrent2 = Vector3.Dot(currentPosition - constraintPosition, planeNormal2);
                    float dNew2 = Vector3.Dot(newPosition - constraintPosition, planeNormal2);

                    // Check if there's a sign change (crossed the plane)
                    if (dCurrent2 * dNew2 < 0)
                    {
                        constraintClosestPoint = GetSegmentPlaneIntersection(currentPosition, newPosition,constraintPosition, planeNormal2);
                        foundClosestConstraintPoint = true;
                    }
                }
                
                currentPosition = newPosition;
                currentOrientation = newOrientation;
                currentVelocity = newVelocity;
                currentAngularVelocity = newAngularVelocity;
            }
            
            if (!constraintShot)
                _precalculationTrajectories.Add(_currentTrajectory);
            
            return (targetClosestPoint, constraintClosestPoint);
        }
        
        private (Vector3,Vector3) EstimateTrajectoryPathToTargetAccurate(float deltaTime, Vector3 initialPosition, SysQuat initialOrientation, Vector3 initialVelocity,  Vector3 initialAngularVelocity, Vector3 targetPosition, bool constraintShot, Vector3 constraintPosition = default)
        {
            bool stopSimulation = false;
            bool foundClosestConstraintPoint = false;

            //initialize useful state variables
            //previous state variables are needed to go back in time when the ball passes to the closest point 
            Vector3 currentPosition = initialPosition;
            Vector3 previousPosition = initialPosition;
            SysQuat currentOrientation = initialOrientation;
            SysQuat previousOrientation = initialOrientation;
            Vector3 currentVelocity = initialVelocity;
            Vector3 previousVelocity = initialVelocity;
            Vector3 currentAngularVelocity = initialAngularVelocity;
            Vector3 previousAngularVelocity = initialAngularVelocity;

            Vector3 targetClosestPoint = currentPosition;
            Vector3 constraintClosestPoint = currentPosition;
            
            float previousTargetError = Vector3.Distance(currentPosition, targetPosition);
            float previousConstraintError = Vector3.Distance(currentPosition, constraintPosition);
            while(!stopSimulation)
            {
                //integrate the next step on the motion
                (Vector3 newPosition, SysQuat newOrientation, Vector3 newVelocity, Vector3 newAngularVelocity) = IntegrateMotion(deltaTime, currentPosition,  currentOrientation, currentVelocity, currentAngularVelocity, BallState.Bouncing);
                
                //compute the new distance from the target
                float targetError = Vector3.Distance(currentPosition, targetPosition);
                float newTargetError = Vector3.Distance(newPosition, targetPosition);
                
                //if the error at frame n is < than the error at frame n-1 and < than the error at frame n+1, then the closest point is between n-1 and n+1
                if (targetError <= previousTargetError && targetError <= newTargetError)
                {
                    stopSimulation = true;
                    
                    //if the error at frame n-1 is < than the error at frame n+1, then the closest point is between n-1 and n, otherwise between n and n+1
                    targetClosestPoint = previousTargetError <= newTargetError 
                        ? RecursiveClosestPointSearch(deltaTime / 2, deltaTime / 2, previousPosition, previousOrientation, previousVelocity, previousAngularVelocity, BallState.Bouncing, targetPosition, previousTargetError, targetError)
                        : RecursiveClosestPointSearch(deltaTime / 2, deltaTime / 2, currentPosition, currentOrientation, currentVelocity, currentAngularVelocity, BallState.Bouncing, targetPosition, targetError, newTargetError);
                }

                //update previous error
                previousTargetError = targetError;

                //in case of shot with constraint find the closest point of trajectory to the constraint
                if(constraintShot && !foundClosestConstraintPoint)
                {
                    //compute the new distance from the target
                    float constraintError = Vector3.Distance(currentPosition, constraintPosition);
                    float newConstraintError = Vector3.Distance(newPosition, constraintPosition);


                    //if the error at frame n-1 is < than the error at frame n+1, then the closest point is between n-1 and n, otherwise between n and n+1
                    if (constraintError <= previousConstraintError && constraintError <= newConstraintError)
                    {
                        
                        //if the error at frame n-1 is < than the error at frame n+1, then the closest point is between n-1 and n, otherwise between n and n+1
                        constraintClosestPoint = previousConstraintError <= newConstraintError
                            ? RecursiveClosestPointSearch(deltaTime / 2, deltaTime / 4, previousPosition, previousOrientation, previousVelocity, previousAngularVelocity, BallState.Bouncing, constraintPosition, previousConstraintError, constraintError)
                            : RecursiveClosestPointSearch(deltaTime / 2, deltaTime / 4, currentPosition, currentOrientation, currentVelocity, currentAngularVelocity, BallState.Bouncing, constraintPosition, constraintError, newConstraintError);
                        
                        foundClosestConstraintPoint = true;
                    }

                    //update previous constraint error
                    previousConstraintError = constraintError;
                }
                
                //update previous state variables
                previousPosition = currentPosition;
                previousVelocity = currentVelocity;
                previousAngularVelocity = currentAngularVelocity;
                previousOrientation = currentOrientation;
                
                //update current state variables
                currentVelocity = newVelocity;
                currentPosition = newPosition;
                currentAngularVelocity = newAngularVelocity;
                currentOrientation = newOrientation;
            }
            
            return (targetClosestPoint, constraintClosestPoint);
        }
        
    #endregion
    
    #region Kick Ball
        //this method is used to set the velocity and the angular velocity of the ball, given direction, speed and spin tp apply
        public void Kick(Vector3 direction, float speed, Vector3 spin)
        {
            _precalculationTrajectories.Clear();
                
            ball.velocity = direction * speed;
            ball.angularVelocity = AlignSpinWithVelocityDirection(ball.velocity, spin);
            ball.state = ball.velocity.y > 0 ? BallState.Bouncing : BallState.Sliding;
                
            ball.AddTrajectoryFrame();
        }
        
        //this method finds the optimal velocity for the ball to reach the target and sets velocity and angular velocity equal to the found values, given speed and spin
        public void TargetedKick(float speed, Vector3 spin)
        {
            _precalculationTrajectories.Clear();

            if (useMethodB)
                (ball.velocity, ball.angularVelocity) = PredictInitialValuesMethodB(1f/computationFrameRate, target.transform.position, ball.position, speed, spin, elevationAngleMinimumLimit, elevationAngleMaximumLimit, maxIterations);
            else
                (ball.velocity, ball.angularVelocity) = PredictInitialValuesMethodA(1f/computationFrameRate, target.transform.position, ball.position, speed, spin, elevationAngleMinimumLimit, elevationAngleMaximumLimit, maxIterations);
            
            ball.state = BallState.Bouncing;
                
            ball.AddTrajectoryFrame();
        }
        
        //this method finds the optimal velocity adn angular velocity for the ball to reach the target passing through a constraint and sets velocity and angular velocity equal to the found values, given speed
        public void TargetedKickWithConstraint(float speed)
        {
            _precalculationTrajectories.Clear();
            
            (ball.velocity, ball.angularVelocity) = PredictInitialValuesTwoConstraints(1f/computationFrameRate, target.transform.position, constraint.transform.position, ball.position, speed, maxIterations);
            ball.state = BallState.Bouncing;

            ball.AddTrajectoryFrame();
        }
        
    #endregion
    
    #region Testing
        
        //if no other test is running, starts the coroutine to run a new test
        public void StartTestForTargetedKick(String description, String fileName, int testSteps, Vector3 testStartPosition, Vector3 testEndPosition, float speed, Vector3 spin)
        {
            if (_testRunning)
                return;

            _testRunning = true;

            StartCoroutine(RunCoroutineTestForTargetedKick(description, fileName, testSteps, testStartPosition, testEndPosition, speed, spin, target.transform.position));
        }
        
        //method called in the coroutines that runs the test
        private IEnumerator RunCoroutineTestForTargetedKick(String description, String fileName, int testSteps, Vector3 testStartPosition, Vector3 testEndPosition, float speed, Vector3 spin, Vector3 targetPosition)
        {
            _testData = new TestData();
            int currentTestStep = 0;

            //each iteration of the cycle the optimal shot from the initial position to the target is computed
            //at each step the position is moved gradually from the start point to the end point
            while(currentTestStep < testSteps)
            {
                _precalculationTrajectories.Clear();
                
                Vector3 stepDirection = (testEndPosition - testStartPosition) / testSteps;
                Vector3 testCurrentPosition = testStartPosition + stepDirection * currentTestStep;
                
                if(useMethodB)
                {
                    PredictInitialValuesMethodB(1f / computationFrameRate, targetPosition,
                        testCurrentPosition, speed, spin, elevationAngleMinimumLimit, elevationAngleMaximumLimit, maxIterations, false, default, true);
                }
                else
                {
                    PredictInitialValuesMethodA(1f / computationFrameRate, targetPosition,
                        testCurrentPosition, speed, spin, elevationAngleMinimumLimit, elevationAngleMaximumLimit, maxIterations, false, default, true);
                }

                currentTestStep++;
                yield return null;
            }

            //save test data
            _testData.description = description;
            _testData.SaveToJson(fileName);
            _testRunning = false;
        }

        //if no other test is running, starts the coroutine to run a new test
        public void StartTestForTargetedKickWithConstraint(String description, String fileName, int testSteps, float minInitialSpeed, float maxInitialSpeed)
        {
            if (_testRunning)
                return;

            _testRunning = true;
            StartCoroutine(RunCoroutineTestForTargetedKickWithConstraint(description, fileName, testSteps, minInitialSpeed, maxInitialSpeed, target.transform.position, constraint.transform.position));
        }
        
        //method called in the coroutines that runs the test
        private IEnumerator RunCoroutineTestForTargetedKickWithConstraint(String description, String fileName, int testSteps, float minInitialSpeed, float maxInitialSpeed, Vector3 targetPosition, Vector3 constraintPosition)
        {
            _testData = new TestData();
            int currentTestStep = 0;
            Vector3 initialPosition = ball.position;
            float testInitialSpeed = minInitialSpeed;
            float speedIncrement = (maxInitialSpeed - minInitialSpeed) / testSteps;

            //each iteration of the cycle the optimal shot from the initial position to the target and the constraint is computed
            //at each step the speed is increased gradually from the start speed to the end speed
            while (currentTestStep < testSteps)
            {
                testInitialSpeed += speedIncrement;

                int multiIterationCounter = 0;
                while(multiIterationCounter < 9)
                {
                    _precalculationTrajectories.Clear();
                    PredictInitialValuesTwoConstraints(1f / computationFrameRate, targetPosition, constraintPosition, initialPosition, testInitialSpeed, maxIterations, true);
                    multiIterationCounter++;
                }
                
                currentTestStep++;

                yield return null;
            }

            //save test data
            _testData.description = description;
            _testData.SaveToJson(fileName);
            _testRunning = false;
        }

        public void StopTest()
        {
            if (!_testRunning)
                return;

            _testRunning = false;
            StopAllCoroutines();
        }
        
    #endregion
    
    #region Utils
        
        //reset ball and trajectory gizmos
        public void Reset()
        {
            _precalculationTrajectories.Clear();
            ball.Reset();
        }
        
        private Vector3 AlignSpinWithVelocityDirection(Vector3 velocity, Vector3 spin)
        {
            //horizontal direction of velocity
            Vector3 direction = new Vector3(velocity.x, 0f, velocity.z).normalized;

            //direction to align spin
            Vector3 alignedSpinDirection = Vector3.Cross(Vector3.up, direction);

            //horizontal magnitude of spin
            float horizontalSpinMagnitude = -spin.z;

            //rescale aligned direction to match original spin magnitude
            return alignedSpinDirection * horizontalSpinMagnitude + new Vector3(0f, spin.y, 0f);
        }
        
        Vector3 GetSegmentPlaneIntersection(Vector3 p0, Vector3 p1, Vector3 planePoint, Vector3 planeNormal)
        {
            Vector3 segment = p1 - p0;
            
            float denominator = Vector3.Dot(planeNormal, segment);

            float t = Vector3.Dot(planeNormal, planePoint - p0) / denominator;
            
            return p0 + t * segment;
        }
        
        private Vector3 RotateVelocityTowardTarget(Vector3 initialPosition, Vector3 guessPosition, Vector3 targetPosition, Vector3 velocity)
        {
            Vector3 uGuess = guessPosition - initialPosition;
            Vector3 uTarget = targetPosition - initialPosition;
            
            //compute rotation axis, perpendicular to the plane oh which uGuess and uTarget lie
            Vector3 rotationAxis = Vector3.Cross(uGuess, uTarget).normalized;
            
            //compute angle between uGuess and uTarget
            float theta = Vector3.SignedAngle(uGuess, uTarget, rotationAxis);
            
            //compute the rotation needed to align uGuess with uTarget
            Quaternion rotation = Quaternion.AngleAxis(theta, rotationAxis);

            //apply rotation to velocity
            Vector3 rotatedVelocity = rotation * velocity;

            return rotatedVelocity;
        }

        private Vector3 ElevateVector(Vector3 vector, float elevationAngle)
        {
            Vector3 dirHor = new Vector3(vector.x, 0f, vector.z).normalized;
            Vector3 rotationAxis = Vector3.Cross(dirHor, Vector3.up);
            Quaternion rotation = Quaternion.AngleAxis(elevationAngle, rotationAxis);
            Vector3 rotatedVector = rotation * dirHor;
            Vector3 rescaledVector = rotatedVector * vector.magnitude;
            
            return rescaledVector;
        }
        
        private Vector3 RecursiveClosestPointSearch(float deltaTime, float step, Vector3 position, SysQuat orientation, Vector3 velocity, Vector3 angularVelocity, BallState state, Vector3 targetPosition, float previousFrameDistance, float nextFrameDistance)
        {
            (Vector3 newPosition, _, _, _) = IntegrateMotion(deltaTime, position, orientation, velocity, angularVelocity, state);
            float newDistance = Vector3.Distance(newPosition, targetPosition);

            bool goingBackward = previousFrameDistance < nextFrameDistance;
            float newDeltaTime = deltaTime + (goingBackward ? -step / 2f : step / 2f);

            if (step < TimeStepThreshold)
            {
                (Vector3 closestPoint, _, _, _) = IntegrateMotion(newDeltaTime, position, orientation, velocity, angularVelocity, state);
                return closestPoint;
            }

            return RecursiveClosestPointSearch(newDeltaTime, step / 2f, position, orientation, velocity, angularVelocity, state, targetPosition, goingBackward ? previousFrameDistance : newDistance, goingBackward ? newDistance : nextFrameDistance);
        }

        private void DebugAndTest(bool constraintShot, Stopwatch stopwatch, int iterationCounter, float error, bool testing, float distanceFromTarget = default, Vector3 velocity = default, Vector3 spin = default, float constraintError = default)
        {
            if (!constraintShot)
            {
                //debugging results
                double computationTimeInMilliseconds = stopwatch.Elapsed.TotalMilliseconds;

                Debug.Log("Prediction duration: " + computationTimeInMilliseconds.ToString("F3") + " ms with " +
                          iterationCounter + " iterations");
                Debug.Log("Error: " + error.ToString("F1") + " cm");

                interactionController.ShowComputationTime(computationTimeInMilliseconds, error);

                //saving the data for the test
                if (testing && iterationCounter > 1)
                {
                    _testData.errors.Add(error);
                    _testData.distances.Add(distanceFromTarget);
                    _testData.iterations.Add(iterationCounter);
                    _testData.initialSpeeds.Add(velocity.magnitude);
                    float elevationAngle = Mathf.Atan2(velocity.y, new Vector2(velocity.x, velocity.z).magnitude) * Mathf.Rad2Deg;
                    _testData.elevationAngle.Add(elevationAngle);
                    _testData.constraintErrors.Add(constraintError);
                    _testData.zSpin.Add(spin.z);
                    _testData.ySpin.Add(spin.y);
                    _testData.computationTime.Add(computationTimeInMilliseconds);
                }
            }
        }

        private class K
        {
            public Vector3 V, W, P;
            public SysQuat Q;
        }
        
        private K ComputeK(Vector3 velocity, Vector3 angularVelocity, SysQuat orientation, BallState state)
        {
            K k = new K();
                
            (k.V, k.W) = CalculateAccelerationAndAngularAcceleration(velocity, angularVelocity, state);

            k.P = velocity;

            k.Q = QuaternionUtils.QuaternionDerivative(orientation, angularVelocity);

            return k;
        }
        
        //handles frame by frame mode
        private void HandleFrameByFrame()
        {
            if (!interpolateFrames)
            {
                if (FrameCount < ball.Frames.Length && ball.Frames.Length > 0)
                {
                    ball.position = ball.Frames[FrameCount].Position;
                    ball.orientation = ball.Frames[FrameCount].Orientation;
                }
            }
            else
            {
                int currentIndex = FrameCount;
                int nextIndex = FrameCount + 1;
                    
                float t = Mathf.Clamp01(ContinuousFrameCount - currentIndex);

                if (nextIndex > ball.Frames.Length - 1)
                {
                    nextIndex = ball.OldestFrameIndex != 0 ? 0 : currentIndex;
                }

                if (nextIndex >= ball.NewestFrameIndex && ball.OldestFrameIndex == 0)
                {
                    nextIndex = currentIndex;
                }

                Vector3 interpolatedPos = Vector3.Lerp(ball.Frames[currentIndex].Position, ball.Frames[nextIndex].Position, t);
                Quaternion interpolatedRot = Quaternion.Slerp(ball.Frames[currentIndex].Orientation, ball.Frames[nextIndex].Orientation, t);

                ball.position = interpolatedPos;
                ball.orientation = interpolatedRot;
            }
        }
    
#if UNITY_EDITOR
        //display predicted trajectories gizmos
        private void DisplayComputedTrajectories()
        {
            if (!Application.isPlaying)
                return;
        
            Color[] colorArray = new Color[_precalculationTrajectories.Count];
            for (int i = 1; i <= _precalculationTrajectories.Count; i++)
            {
                float t = (float)i / _precalculationTrajectories.Count;
                colorArray[i-1] = Color.Lerp(Color.green, Color.blue, t);
            }

            for (int i = 0; i < _precalculationTrajectories.Count; i++)
            {

                Handles.color = colorArray[i]; 

                Vector3[] linePoints = _precalculationTrajectories[i].ToArray();

                Handles.DrawAAPolyLine(7.5f, linePoints);
            }
        }
#endif
    #endregion
    
    #region Getters
        public Vector3 TotalForces()
        {
            if(useFrameByFrameMode)
            {
                return CalculateTotalForce(ball.Frames[FrameCount].Velocity, ball.Frames[FrameCount].AngularVelocity, ball.Frames[FrameCount].State);
            }
            return CalculateTotalForce(ball.velocity, ball.angularVelocity, ball.state);
        }

        public Vector3 DragForce()
        {
            if(useFrameByFrameMode)
            {
                return CalculateDragForce(ball.Frames[FrameCount].Velocity, ball.Frames[FrameCount].AngularVelocity);
            }
            return CalculateDragForce(ball.velocity, ball.angularVelocity);
        }

        public Vector3 MagnusForce()
        {
            if(useFrameByFrameMode)
            {
                return CalculateMagnusForce(ball.Frames[FrameCount].Velocity, ball.Frames[FrameCount].AngularVelocity, ball.Frames[FrameCount].State);
            }
            return CalculateMagnusForce(ball.velocity, ball.angularVelocity, ball.state);
        }
    

        public Vector3 FrictionForce()
        {
            if(useFrameByFrameMode)
            {
                return CalculateFrictionForce(ball.Frames[FrameCount].Velocity, ball.Frames[FrameCount].AngularVelocity, ball.Frames[FrameCount].State);
            }
            return CalculateFrictionForce(ball.velocity, ball.angularVelocity, ball.state);
        }

        public Vector3 FrictionTorque()
        {
            if(useFrameByFrameMode)
            {
                return CalculateFrictionTorque(ball.Frames[FrameCount].Velocity, ball.Frames[FrameCount].AngularVelocity, CalculateTotalForce(ball.Frames[FrameCount].Velocity, ball.Frames[FrameCount].AngularVelocity, ball.Frames[FrameCount].State), ball.Frames[FrameCount].State);
            }
            return CalculateFrictionTorque(ball.velocity, ball.angularVelocity, CalculateTotalForce(ball.velocity, ball.angularVelocity, ball.state), ball.state);
        }
        
        public Vector3 DragTorque()
        {
            if(useFrameByFrameMode)
            {
                return CalculateSpinDegradation(ball.Frames[FrameCount].Velocity, ball.Frames[FrameCount].AngularVelocity, ball.Frames[FrameCount].State);
            }
            return CalculateSpinDegradation(ball.velocity, ball.angularVelocity, ball.state);
        }
        
        public Vector3 TotalTorque()
        {
            if(useFrameByFrameMode)
            {
                return CalculateTotalTorque(ball.Frames[FrameCount].Velocity, ball.Frames[FrameCount].AngularVelocity, CalculateTotalForce(ball.Frames[FrameCount].Velocity, ball.Frames[FrameCount].AngularVelocity, ball.Frames[FrameCount].State), ball.Frames[FrameCount].State);
            }
            return CalculateTotalTorque(ball.velocity, ball.angularVelocity, CalculateTotalForce(ball.velocity, ball.angularVelocity, ball.state), ball.state);
        }

        public double TimeToCompute()
        {
            if(useFrameByFrameMode)
            {
                return ball.Frames[FrameCount].TimeToCompute;
            }
            return _timeToComputeFrame;
        }
        
         public Vector3 Position()
        {
            if (useFrameByFrameMode)
            {
                return ball.Frames[FrameCount].Position;
            }
            return ball.position;
        }
        
        public Vector3 Velocity()
        {
            if(useFrameByFrameMode)
            {
                return ball.Frames[FrameCount].Velocity;
            }
            return ball.velocity;
        }
        
        public Vector3 AngularVelocity()
        {
            if(useFrameByFrameMode)
            {
                return ball.Frames[FrameCount].AngularVelocity;
            }
            return ball.angularVelocity;
        }

        public Vector3 Acceleration()
        {
            if(useFrameByFrameMode)
            {
                return CalculateTotalForce(ball.Frames[FrameCount].Velocity, ball.Frames[FrameCount].AngularVelocity, ball.Frames[FrameCount].State)/ball.mass;
            }
            
            return CalculateTotalForce(ball.velocity, ball.angularVelocity, ball.state)/ball.mass;
        }
        
        public float PotentialEnergy()
        {
            if(useFrameByFrameMode)
            {
                Vector3 savedPos = ball.position;
                Quaternion savedRot = ball.orientation;
                Vector3 savedVel = ball.velocity;
                Vector3 savedAng = ball.angularVelocity;
                ball.position = ball.Frames[FrameCount].Position;
                ball.orientation = ball.Frames[FrameCount].Orientation;
                ball.velocity = ball.Frames[FrameCount].Velocity;
                ball.angularVelocity = ball.Frames[FrameCount].AngularVelocity;
                float potentialEnergy = ball.CalculatePotentialEnergy();
                ball.position = savedPos;
                ball.orientation = savedRot;
                ball.velocity = savedVel;
                ball.angularVelocity = savedAng;
                return potentialEnergy;
            }
            return ball.CalculatePotentialEnergy();
        }

        public float KineticEnergy()
        {
            if(useFrameByFrameMode)
            {
                Vector3 savedPos = ball.position;
                Quaternion savedRot = ball.orientation;
                Vector3 savedVel = ball.velocity;
                Vector3 savedAng = ball.angularVelocity;
                ball.position = ball.Frames[FrameCount].Position;
                ball.orientation = ball.Frames[FrameCount].Orientation;
                ball.velocity = ball.Frames[FrameCount].Velocity;
                ball.angularVelocity = ball.Frames[FrameCount].AngularVelocity;
                float kineticEnergy = ball.CalculateKineticEnergy();
                ball.position = savedPos;
                ball.orientation = savedRot;
                ball.velocity = savedVel;
                ball.angularVelocity = savedAng;
                return kineticEnergy;
            }
            return ball.CalculateKineticEnergy();
        }

        public float RotationalEnergy()
        {
            if(useFrameByFrameMode)
            {
                Vector3 savedPos = ball.position;
                Quaternion savedRot = ball.orientation;
                Vector3 savedVel = ball.velocity;
                Vector3 savedAng = ball.angularVelocity;
                ball.position = ball.Frames[FrameCount].Position;
                ball.orientation = ball.Frames[FrameCount].Orientation;
                ball.velocity = ball.Frames[FrameCount].Velocity;
                ball.angularVelocity = ball.Frames[FrameCount].AngularVelocity;
                float rotationalEnergy = ball.CalculateRotationalEnergy();
                ball.position = savedPos;
                ball.orientation = savedRot;
                ball.velocity = savedVel;
                ball.angularVelocity = savedAng;
                return rotationalEnergy;
            }
            return ball.CalculateRotationalEnergy();
        }

        public float TotalEnergy()
        {
            if(useFrameByFrameMode)
            {
                Vector3 savedPos = ball.position;
                Quaternion savedRot = ball.orientation;
                Vector3 savedVel = ball.velocity;
                Vector3 savedAng = ball.angularVelocity;
                ball.position = ball.Frames[FrameCount].Position;
                ball.orientation = ball.Frames[FrameCount].Orientation;
                ball.velocity = ball.Frames[FrameCount].Velocity;
                ball.angularVelocity = ball.Frames[FrameCount].AngularVelocity;
                float totalEnergy = ball.CalculateTotalEnergy();
                ball.position = savedPos;
                ball.orientation = savedRot;
                ball.velocity = savedVel;
                ball.angularVelocity = savedAng;
                return totalEnergy;
            }
            return ball.CalculateTotalEnergy();
        }
    
    #endregion
}




