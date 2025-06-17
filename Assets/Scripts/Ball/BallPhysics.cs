using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using UnityEditor;
using UnityEngine;
using Debug = UnityEngine.Debug;
using Quaternion = UnityEngine.Quaternion;
using SysQuat = System.Numerics.Quaternion;
using Random = System.Random;
using Vector2 = UnityEngine.Vector2;
using Vector3 = UnityEngine.Vector3;

public class BallPhysics : MonoBehaviour
{
    #region Variables
        public Ball ball;
        
        [Header("Constants")]
        [SerializeField] private float airDensity = 1.2f;
        [SerializeField] private float dynamicViscosityOfAir = 1.81f * 1e-5f;
        public static readonly float Gravity = 9.81f; 
        private const float CollisionThreshold = 1e-6f;
        private Vector3 _gravitationalForce;
        private Vector3 _buoyantForce;
        
        [Header("Simulation")]
        [SerializeField] private IntegrationMethod integrationMethod;
        [SerializeField] private float framePerSecond;
        
        [Header("Targeted shot")]
        [SerializeField] private AnimationCurve targetMaximumErrorCurve;
        [SerializeField] private float constraintMaximumError;
        [SerializeField] private int maxIterations;

        [Header("Frame History")] 
        public bool useFrameByFrameMode;
        public bool interpolateFrames;
        public int FrameCount { private get; set; }
        public float ContinuousFrameCount { private get; set; }
        
        [Header("Other")]
        [SerializeField] private GameObject target;
        [SerializeField] private GameObject closestPointToTargetPlaceholder;
        [SerializeField] private GameObject constraint;
        [SerializeField] private GameObject closestPointToConstraintPlaceholder;
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
        }

        void Update()
        {
            Time.fixedDeltaTime = 1/framePerSecond;
            _gravitationalForce = Gravity * ball.mass * Vector3.down;
            _buoyantForce = 4f / 3f * Mathf.PI * Mathf.Pow(ball.radius, 3) * airDensity * Gravity * Vector3.up;
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
        void OnDrawGizmos()
        {
            DisplayComputedTrajectories();
        }
        
    #endregion
    
    #region Simulation
        public void ComputeNewFrame(float deltaTime, Ball targetBall)
        {
            //given the current state of the ball, the motion is integrated and the new state of the ball after delta time is returned
            Stopwatch stopwatch = Stopwatch.StartNew();
            (Vector3 newPosition, Quaternion newOrientation, Vector3 newVelocity, Vector3 newAngularVelocity) = IntegrateMotion(deltaTime, targetBall.position, targetBall.orientation, targetBall.velocity, targetBall.angularVelocity, targetBall.state);
            stopwatch.Stop();
            _timeToComputeFrame = stopwatch.Elapsed.TotalMilliseconds;


            //checks if there was a collision and if so handles it
            if (newPosition.y - targetBall.radius < 0f)
            {
                (newPosition, newOrientation, newVelocity, newAngularVelocity, targetBall.state) = HandleGroundCollisionAndBounce(deltaTime, targetBall.position, targetBall.orientation, targetBall.velocity, targetBall.angularVelocity, targetBall.state);
            }

            Vector3 previousVelocity = targetBall.velocity;
            Vector3 previousAngularVelocity = targetBall.angularVelocity;
            
            //updates the state of the ball
            targetBall.position = newPosition;
            targetBall.orientation = newOrientation;
            targetBall.velocity = newVelocity;
            targetBall.angularVelocity = newAngularVelocity;
            
            //checks if the ball is a pure rolling state
            if(ball.state == BallState.Sliding)
                targetBall.state = CheckPureRolling(targetBall.state, newVelocity, newAngularVelocity, previousVelocity, previousAngularVelocity);
        }
        
        //this method chooses one of the four integration method to compute the new state of the ball after delta time
        private (Vector3, Quaternion, Vector3, Vector3) IntegrateMotion(float deltaTime, Vector3 initialPosition, Quaternion initialOrientation, Vector3 initialVelocity, Vector3 initialAngularVelocity, BallState state)
        {
            switch (integrationMethod)
            {
                case IntegrationMethod.ExplicitEuler:
                    return ExplicitEuler(deltaTime, initialPosition, initialOrientation, initialVelocity,initialAngularVelocity, state);
                case IntegrationMethod.SemiImplicitEuler:
                    return SemiImplicitEuler(deltaTime, initialPosition, initialOrientation, initialVelocity,initialAngularVelocity, state);
                case IntegrationMethod.VelocityVerlet:
                    return VelocityVerlet(deltaTime, initialPosition, initialOrientation, initialVelocity,initialAngularVelocity, state);
                case IntegrationMethod.Rk2:
                    return Runge_Kutta_2(deltaTime, initialPosition, initialOrientation, initialVelocity,initialAngularVelocity, state);
                case IntegrationMethod.Rk4:
                    return Runge_Kutta_4(deltaTime, initialPosition, initialOrientation, initialVelocity,initialAngularVelocity, state);
            }
                
            return (Vector3.zero, Quaternion.identity, Vector3.zero, Vector3.zero);
        }
        
        //this method checks the pure rolling condition (HEURISTIC)
        private BallState CheckPureRolling(BallState state, Vector3 currentVelocity, Vector3 currentAngularVelocity, Vector3 previousVelocity, Vector3 previousAngularVelocity)
        {
            //inclination in of the ball axis against the up-axis in radians
            float currentTheta = (90f - Vector3.Angle(currentAngularVelocity.normalized, Vector3.up)) * Mathf.Deg2Rad;
            float previousTheta = (90f - Vector3.Angle(previousAngularVelocity.normalized, Vector3.up)) * Mathf.Deg2Rad;
            
            //compute the velocity of the contact point of the current and the previous frames
            float currentContactPointSpeed = (currentVelocity + Vector3.Cross(ball.radius * Mathf.Cos(currentTheta) * Vector3.up, currentAngularVelocity)).magnitude;
            
            float previousContactPointSpeed = (previousVelocity + Vector3.Cross(ball.radius * Mathf.Cos(previousTheta) * Vector3.up, previousAngularVelocity)).magnitude;
            
            //the state is set to rolling when the current contact point velocity becomes greater than the previous, meaning that it reached the minimum
            //this is done in this way because after this point the contact point velocity oscillating between -epsilon and +epsilon, resulting in an infinite cycle due to numerical error (HEURISTIC)
            if(currentContactPointSpeed > previousContactPointSpeed || currentContactPointSpeed < 1e-3)
            {
                ball.angularVelocity = -Vector3.Cross(currentVelocity, Vector3.up) / ball.radius + Vector3.up * currentAngularVelocity.y;
                state = BallState.Rolling;
            }
            
            return state;
        }
        
        #region Integration Methods
        
            private (Vector3, Quaternion, Vector3, Vector3) ExplicitEuler(float deltaTime,Vector3 initialPosition, Quaternion initialOrientation, Vector3 initialVelocity, Vector3 initialAngularVelocity, BallState state)
            {
                SysQuat numericsInitialOrientation = QuaternionUtils.ToNumerics(initialOrientation);
                
                //computes the acceleration/angular acceleration based on current state
                (Vector3 acceleration, Vector3 angularAcceleration) = CalculateAccelerationAndAngularAcceleration(initialVelocity, initialAngularVelocity, state);
                        
                //full step velocity/angular velocity update
                Vector3 newVelocity = initialVelocity + deltaTime * acceleration;
                Vector3 newAngularVelocity = initialAngularVelocity + deltaTime * angularAcceleration;
                        
                //full step position/orientation update
                Vector3 newPosition = initialPosition + deltaTime * initialVelocity;
                Quaternion newOrientation = QuaternionUtils.ToUnity(SysQuat.Add(numericsInitialOrientation, QuaternionUtils.QuaternionDerivative(numericsInitialOrientation, initialAngularVelocity * deltaTime)));

                return (newPosition, newOrientation, newVelocity, newAngularVelocity);
            }
            
            private (Vector3, Quaternion, Vector3, Vector3) SemiImplicitEuler(float deltaTime,Vector3 initialPosition, Quaternion initialOrientation, Vector3 initialVelocity, Vector3 initialAngularVelocity, BallState state)
            {
                SysQuat numericsInitialOrientation = QuaternionUtils.ToNumerics(initialOrientation);
                
                //computes the acceleration/angular acceleration based on current state
                (Vector3 acceleration, Vector3 angularAcceleration) = CalculateAccelerationAndAngularAcceleration(initialVelocity, initialAngularVelocity, state);
                    
                //full step velocity/angular velocity update
                Vector3 newVelocity = initialVelocity + deltaTime * acceleration;
                Vector3 newAngularVelocity = initialAngularVelocity + deltaTime * angularAcceleration;
                    
                //full step position/orientation update
                Vector3 newPosition = initialPosition + deltaTime * newVelocity;
                Quaternion newOrientation = QuaternionUtils.ToUnity(SysQuat.Add(numericsInitialOrientation, QuaternionUtils.QuaternionDerivative(numericsInitialOrientation, newAngularVelocity * deltaTime)));
                
                return (newPosition, newOrientation, newVelocity, newAngularVelocity);
            }
            
            private (Vector3, Quaternion, Vector3, Vector3) VelocityVerlet(float deltaTime,Vector3 initialPosition, Quaternion initialOrientation, Vector3 initialVelocity, Vector3 initialAngularVelocity, BallState state) 
            {
                SysQuat numericsInitialOrientation = QuaternionUtils.ToNumerics(initialOrientation);
                
                 //computes the acceleration/angular acceleration based on current state
                 (Vector3 acceleration, Vector3 angularAcceleration) = CalculateAccelerationAndAngularAcceleration(initialVelocity, initialAngularVelocity, state);

                 //first half-step velocity/angular velocity update
                 Vector3 newVelocity = initialVelocity + deltaTime/2 * acceleration;
                 Vector3 newAngularVelocity = initialAngularVelocity + deltaTime/2 * angularAcceleration;

                 //full step position/orientation update
                 Vector3 newPosition = initialPosition + deltaTime * newVelocity;
                 Quaternion newOrientation = QuaternionUtils.ToUnity(SysQuat.Add(numericsInitialOrientation, QuaternionUtils.QuaternionDerivative(numericsInitialOrientation, newAngularVelocity * deltaTime)));
                
                 //the acceleration/angular acceleration based on new half-step state
                 (acceleration, angularAcceleration) = CalculateAccelerationAndAngularAcceleration(newVelocity, newAngularVelocity, state);

                 //second half-step velocity/angular velocity update
                 newVelocity += deltaTime/2 * acceleration;
                 newAngularVelocity += deltaTime/2 * angularAcceleration;

                 return (newPosition, newOrientation, newVelocity, newAngularVelocity);
            }
            
            private (Vector3, Quaternion, Vector3, Vector3) Runge_Kutta_2(float deltaTime,Vector3 initialPosition, Quaternion initialOrientation, Vector3 initialVelocity, Vector3 initialAngularVelocity, BallState state)
            {
                SysQuat numericsInitialOrientation = QuaternionUtils.ToNumerics(initialOrientation);
                
                //K1
                (Vector3 k1V, Vector3 k1W) = CalculateAccelerationAndAngularAcceleration(initialVelocity, initialAngularVelocity, state);
                SysQuat k1T = QuaternionUtils.QuaternionDerivative(numericsInitialOrientation, initialAngularVelocity);
                //K1P not computed since it doesn't influence the result neither directly nor indirectly through K2
                
                //K2
                (Vector3 k2V, Vector3 k2W) = CalculateAccelerationAndAngularAcceleration(initialVelocity + deltaTime * k1V / 2, initialAngularVelocity + deltaTime * k1W / 2, state);
                Vector3 k2P = initialVelocity + k1V * deltaTime / 2;
                SysQuat k2T = QuaternionUtils.QuaternionDerivative(SysQuat.Add(numericsInitialOrientation, SysQuat.Multiply(k1T, deltaTime/2)), initialAngularVelocity + deltaTime * k1W / 2);
           
                //update velocity and position
                Vector3 newPosition = initialPosition + deltaTime * k2P;
                Vector3 newVelocity = initialVelocity + deltaTime * k2V;
                
                //update angular velocity and orientation 
                Vector3 newAngularVelocity = initialAngularVelocity + deltaTime * k2W;
                Quaternion newOrientation = QuaternionUtils.ToUnity(SysQuat.Normalize(SysQuat.Add(numericsInitialOrientation, SysQuat.Multiply(k2T, deltaTime))));
               
                
                return (newPosition, newOrientation, newVelocity, newAngularVelocity);
            }
            
            private (Vector3, Quaternion, Vector3, Vector3) Runge_Kutta_4(float deltaTime,Vector3 initialPosition, Quaternion initialOrientation, Vector3 initialVelocity, Vector3 initialAngularVelocity, BallState state)
            {
                SysQuat numericsInitialOrientation = QuaternionUtils.ToNumerics(initialOrientation);
                
                //K1
                (Vector3 k1V, Vector3 k1W) = CalculateAccelerationAndAngularAcceleration(initialVelocity, initialAngularVelocity, state);
                Vector3 k1P = initialVelocity;
                SysQuat k1T = QuaternionUtils.QuaternionDerivative(numericsInitialOrientation, initialAngularVelocity);
                
                
                
                //K2
                (Vector3 k2V, Vector3 k2W) = CalculateAccelerationAndAngularAcceleration(initialVelocity + deltaTime * k1V / 2, initialAngularVelocity + deltaTime * k1W / 2, state);
                Vector3 k2P = initialVelocity + deltaTime * k1V/2;
                SysQuat k2T = QuaternionUtils.QuaternionDerivative(SysQuat.Add(numericsInitialOrientation, SysQuat.Multiply(k1T, deltaTime/2)), initialAngularVelocity + deltaTime * k1W / 2);

                
                
                //K3
                (Vector3 k3V, Vector3 k3W) = CalculateAccelerationAndAngularAcceleration(initialVelocity + deltaTime * k2V/2, initialAngularVelocity + deltaTime * k2W/2, state);
                Vector3 k3P = initialVelocity + deltaTime * k2V/2;
                SysQuat k3T = QuaternionUtils.QuaternionDerivative(SysQuat.Add(numericsInitialOrientation, SysQuat.Multiply(k2T, deltaTime/2)), initialAngularVelocity + deltaTime * k2W / 2);

                
                
                //K4
                ( Vector3 k4V, Vector3 k4W) = CalculateAccelerationAndAngularAcceleration(initialVelocity + deltaTime * k3V, initialAngularVelocity + deltaTime * k3W, state);
                Vector3 k4P = initialVelocity + deltaTime * k3V;
                SysQuat k4T = QuaternionUtils.QuaternionDerivative(SysQuat.Add(numericsInitialOrientation, SysQuat.Multiply(k3T, deltaTime)), initialAngularVelocity + deltaTime * k3W);

                
                
                
                //update velocity and position using the weighted average of k1, k2, k3 and k4
                Vector3 newPosition = initialPosition + deltaTime * (k1P + 2*k2P + 2*k3P + k4P) / 6;
                Vector3 newVelocity = initialVelocity + deltaTime * (k1V + 2*k2V + 2*k3V + k4V) / 6;
                
                //update angular velocity and orientation using the weighted average of k1, k2, k3 and k4
                Vector3 newAngularVelocity = initialAngularVelocity + deltaTime * (k1W + 2*k2W + 2*k3W + k4W) / 6;

                SysQuat kSum = k1T;
                kSum = SysQuat.Add(kSum, SysQuat.Multiply(k2T, 2));
                kSum = SysQuat.Add(kSum, SysQuat.Multiply(k3T, 2));
                kSum = SysQuat.Add(kSum, k4T);
                Quaternion newOrientation = QuaternionUtils.ToUnity(SysQuat.Add(numericsInitialOrientation, SysQuat.Multiply(kSum, deltaTime/6)));
                
                return (newPosition, newOrientation, newVelocity, newAngularVelocity);
            }
            
            private enum IntegrationMethod
            {
                ExplicitEuler,
                SemiImplicitEuler,
                VelocityVerlet,
                Rk2,
                Rk4
            }

        #endregion
        
        #region Collision Handling
        
            //this function handles the collision with the ground and returns the new state of the ball after the collision
            private (Vector3, Quaternion, Vector3, Vector3, BallState) HandleGroundCollisionAndBounce(float deltaTime, Vector3 initialPosition, Quaternion initialOrientation, Vector3 initialVelocity, Vector3 initialAngularVelocity, BallState state)
            {
                //finds moment of collision
                float collisionDeltaTime = RecursiveCollisionDetection(deltaTime / 2, deltaTime / 2, initialPosition, initialOrientation, initialVelocity, initialAngularVelocity, state);
                
                //updates position, orientation, velocity and angular velocity after the collision
                (Vector3 positionAfterCollision, Quaternion orientationAfterCollision, Vector3 velocityAfterCollision, Vector3 angularVelocityAfterCollision) = ComputePostCollisionVelocities(collisionDeltaTime, initialPosition, initialOrientation, initialVelocity, initialAngularVelocity, state);
                
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
            private float RecursiveCollisionDetection(float deltaTime, float step, Vector3 initialPosition, Quaternion initialOrientation, Vector3 initialVelocity, Vector3 initialAngularVelocity, BallState state)
            {
                //integrate the motion with the current iteration delta time
                (Vector3 integratedPosition, _, _, _) = IntegrateMotion(deltaTime, initialPosition, initialOrientation, initialVelocity, initialAngularVelocity, state);
                
                //the new delta time is determined based on whether the ball collides with the ground or not given the current collision delta time guess
                //if the ball has collided, then the new delta time needs to go back of half the current step, otherwise it has to go further of half step
                float newDeltaTime = integratedPosition.y - ball.radius < 0f ? deltaTime - step / 2 : deltaTime + step / 2;
                
                //if the current delta time step is small enough it means a good guess of the exact moment of collision was obtained
                if (step < CollisionThreshold)
                    return newDeltaTime;
                
                //otherwise a recursive call with the new guess of the delta time and the halved step is made
                return RecursiveCollisionDetection(newDeltaTime, step/2, initialPosition, initialOrientation, initialVelocity, initialAngularVelocity, state);
            }
            private (Vector3, Quaternion, Vector3, Vector3) ComputePostCollisionVelocities(float collisionDeltaTime, Vector3 initialPosition, Quaternion initialOrientation, Vector3 initialVelocity, Vector3 initialAngularVelocity, BallState state)
            {
                //here the movement is integrated until the moment the ball collides with the ground
                (Vector3 collisionPoint, Quaternion collisionOrientation, Vector3 velocityBeforeCollision, Vector3 angularVelocityBeforeCollision) = IntegrateMotion(collisionDeltaTime, initialPosition, initialOrientation, initialVelocity, initialAngularVelocity, state);

                Vector3 velocityAfterCollision = new Vector3();
                Vector3 angularVelocityAfterCollision = new Vector3();
                
                //the new velocity after the collision is computed
                velocityAfterCollision.x = velocityBeforeCollision.x * (1 - ball.inertialMomentumConstant * ball.coefficientOfHorizontalRestitution)  / (ball.inertialMomentumConstant + 1) - ball.inertialMomentumConstant * (1 + ball.coefficientOfHorizontalRestitution) / (ball.inertialMomentumConstant + 1) * ball.radius * angularVelocityBeforeCollision.z;
                velocityAfterCollision.y = -ball.coefficientOfVerticalRestitution * velocityBeforeCollision.y;
                velocityAfterCollision.z = velocityBeforeCollision.z * (1 - ball.inertialMomentumConstant * ball.coefficientOfHorizontalRestitution) / (ball.inertialMomentumConstant + 1) + ball.inertialMomentumConstant * (1 + ball.coefficientOfHorizontalRestitution) / (ball.inertialMomentumConstant + 1) * ball.radius * angularVelocityBeforeCollision.x;

                //the new angular velocity after the collision is computed
                angularVelocityAfterCollision.x = (ball.inertialMomentumConstant-ball.coefficientOfHorizontalRestitution)/(ball.inertialMomentumConstant+1) * angularVelocityBeforeCollision.x + (ball.coefficientOfHorizontalRestitution+1)/(ball.inertialMomentumConstant+1) * velocityBeforeCollision.z/ball.radius;
                angularVelocityAfterCollision.y = angularVelocityBeforeCollision.y * ball.coefficientOfVerticalAxisSpinRestitution;
                angularVelocityAfterCollision.z = (ball.inertialMomentumConstant-ball.coefficientOfHorizontalRestitution)/(ball.inertialMomentumConstant+1) * angularVelocityBeforeCollision.z - (ball.coefficientOfHorizontalRestitution+1)/(ball.inertialMomentumConstant+1) * velocityBeforeCollision.x/ball.radius;
                
                //calculate the remaining time until the end of the frame and the  is integrated using it
                float remainingTime = Time.fixedDeltaTime - collisionDeltaTime;
                return IntegrateMotion(remainingTime, collisionPoint, collisionOrientation, velocityAfterCollision, angularVelocityAfterCollision, state);
            }
            
        #endregion
        
        #region Forces And Torques
            
            //this method computes the force caused by the air resistance
            private Vector3 ComputeAirResistanceForce(Vector3 velocity, Vector3 angularVelocity)
            {
                // this part computes the drag force based on the paper "Flight and bounce of spinning sports balls"
                if(!ball.useConstantCoefficients)
                {
                    //spin rate parameter, how strong the spin is compared to the speed
                    float s = angularVelocity.magnitude * ball.radius / velocity.magnitude;

                    //if the ball is spinning fast enough and moving faster than critical speed
                    //use the formula where drag decreases with spin rate
                    if (velocity.magnitude > ball.speedOfTransitionFromLaminarToTurbulentDrag && s > 0.05f)
                        //TODO explain values
                        ball.dragCoefficient = 0.4127f * Mathf.Pow(s, 0.3056f);
                    //otherwise use the drag coefficient model depending on Reynolds numbers
                    else
                        //TODO explain values
                        ball.dragCoefficient = 0.155f + 0.346f / (1 + Mathf.Exp((velocity.magnitude - ball.speedOfTransitionFromLaminarToTurbulentDrag) / ball.dragTransitionSlope));
                    
                    return - 0.5f * airDensity * ball.dragCoefficient * Mathf.PI * Mathf.Pow(ball.radius, 2) * velocity.magnitude * velocity;
                }
                
                //otherwise use constant linear drag coefficient
                return - 0.5f * airDensity * ball.constantDragCoefficient * Mathf.PI * Mathf.Pow(ball.radius, 2) * velocity.magnitude * velocity;
            }
            
            
            private Vector3 ComputeMagnusForce(Vector3 velocity, Vector3 angularVelocity, BallState state)
            {
                
                //direction cross product
                Vector3 direction = Vector3.Cross(angularVelocity.normalized, velocity.normalized);

                Vector3 magnusForce;
                
                if(!ball.useConstantCoefficients)
                {
                    //spin rate parameter, how strong the spin is compared to the speed
                    float s = angularVelocity.magnitude * ball.radius / velocity.magnitude;
                    
                    float cdRe0 = 0.155f + 0.346f / (1 + Mathf.Exp(-ball.speedOfTransitionFromLaminarToTurbulentDrag) / ball.dragTransitionSlope);
                    float cdReRef = 0.155f;

                    ball.liftCoefficient = 1.15f * Mathf.Pow(s, 0.83f) * (cdRe0 -  ball.dragCoefficient) / (cdRe0 - cdReRef);


                    magnusForce = 0.5f * airDensity * ball.liftCoefficient * Mathf.PI * Mathf.Pow(ball.radius, 2) * Mathf.Pow(velocity.magnitude, 2) * direction;

                    if (state != BallState.Bouncing)
                        magnusForce.y = 0f;

                    return magnusForce;
                }
                
                //otherwise use constant linear drag coefficient
                magnusForce =  0.5f * airDensity * ball.constantLiftCoefficient * Mathf.PI * Mathf.Pow(ball.radius, 2) * Mathf.Pow(velocity.magnitude, 2) * direction;
                
                if (state != BallState.Bouncing)
                    magnusForce.y = 0f;

                return magnusForce;
            }
            
            //this method computes this friction force with the ground, given the velocity, the angular velocity and the current state of the ball
            private Vector3 CalculateFrictionForce(Vector3 velocity, Vector3 angularVelocity, BallState state)
            {
                if(state == BallState.Bouncing)
                    return Vector3.zero;
                
                Vector3 frictionForce = Vector3.zero;
                
                float normalForce = (- _gravitationalForce - _buoyantForce).magnitude;
                
                //if the ball is sliding, use the sliding friction formula
                if(state == BallState.Sliding)
                {
                    //inclination in of the ball axis against the up-axis in radians
                    float theta = (90f - Vector3.Angle(angularVelocity.normalized, Vector3.up)) * Mathf.Deg2Rad;
                    
                    //compute direction of the velocity of the contact point
                    Vector3 frictionDirection = (velocity + Vector3.Cross(ball.radius * Mathf.Cos(theta) * Vector3.up, angularVelocity)).normalized;
                    
                    frictionForce += -ball.coefficientOfSlidingFriction * normalForce * frictionDirection;
                    return frictionForce;
                }
                
                //otherwise, rolling friction formula
                frictionForce += -ball.coefficientOfRollingFriction * normalForce * velocity.normalized;
                return frictionForce;
                
            }
            
            //this method computes the result of the sum of all forces acting on the ball
            private Vector3 CalculateTotalForces(Vector3 velocity, Vector3 angularVelocity, BallState state)
            {
                Vector3 dragForce = ComputeAirResistanceForce(velocity, angularVelocity);
                Vector3 magnusForce = ComputeMagnusForce(velocity, angularVelocity, state);
                Vector3 normalForce = state != BallState.Bouncing ? - _gravitationalForce - _buoyantForce : Vector3.zero;
                Vector3 frictionForce = CalculateFrictionForce(velocity, angularVelocity, state);
                
                //sum all forces
                Vector3 totalForces = _gravitationalForce + _buoyantForce + dragForce + magnusForce + normalForce + frictionForce;

                return totalForces;
            }
            
            //this method computes the torque acting on the ball created by the friction with the ground, given velocity, angular velocity and the state of the ball
            private Vector3 CalculateFrictionTorque(Vector3 velocity, Vector3 angularVelocity, BallState state)
            { 
                if(state == BallState.Bouncing)
                    return Vector3.zero;
                
                float normalForce = (- _gravitationalForce - _buoyantForce).magnitude;
                
                Vector3 frictionTorque = Vector3.zero;
                
                frictionTorque = new Vector3(0, - ball.coefficientOfVerticalAxisSpinningFriction * angularVelocity.y, 0);
                
                //if the ball is rolling, the contact point is not moving with respect to the ground, therefore the is no other friction component acting on the ball
                if (state == BallState.Rolling)
                    return frictionTorque;
                
                //inclination in of the ball axis against the up-axis in radians
                float theta = (90f - Vector3.Angle(angularVelocity.normalized, Vector3.up)) * Mathf.Deg2Rad;
                    
                //compute direction of the velocity of the contact point
                Vector3 frictionDirection = velocity + Vector3.Cross(ball.radius * Mathf.Cos(theta) * Vector3.up, angularVelocity).normalized;
                Vector3 direction = Vector3.Cross(Vector3.up, frictionDirection).normalized;
                
                Vector3 angularAcceleration = ball.coefficientOfSlidingFriction * normalForce * ball.radius * direction / ball.InertialMomentum;

                frictionTorque += angularAcceleration * ball.InertialMomentum;
                
                return frictionTorque;
            }
            
            private Vector3 CalculateDragTorque(Vector3 velocity, Vector3 angularVelocity, BallState state)
            {
                if(state != BallState.Bouncing || angularVelocity.magnitude < 1e-3)
                    return Vector3.zero;
                
                float utP = Vector3.Cross(velocity, angularVelocity.normalized).magnitude + ball.radius/2 * angularVelocity.magnitude;
                float utM = Vector3.Cross(velocity, angularVelocity.normalized).magnitude - ball.radius/2 * angularVelocity.magnitude;

                float kinematicViscosityOfAir = dynamicViscosityOfAir * airDensity;
                
                float cfP = 0.0074f * Mathf.Pow(utP * airDensity * ball.radius * 2 / kinematicViscosityOfAir, -1f / 5f);
                float cfM = 0.0074f * Mathf.Pow(utM * airDensity * ball.radius * 2 / kinematicViscosityOfAir, -1f / 5f);
                
                //this was added to handle cases when cfP and cfM are too small, therefore become NaN
                if(float.IsNaN(cfP) || float.IsNaN(cfM))
                    return Vector3.zero;
                
                float totalForce = airDensity * Mathf.PI * Mathf.Pow(ball.radius, 2) * (Mathf.Pow(utP, 2) * cfP + Mathf.Pow(utM, 2) * cfM);
                
                return - totalForce * ball.radius / 2 * angularVelocity.normalized;
            }
            
            
            //this method computes the result of the sum of all torques acting on the ball
            private Vector3 CalculateTotalTorque(Vector3 velocity, Vector3 angularVelocity, BallState state)
            {
                Vector3 frictionTorque = CalculateFrictionTorque(velocity, angularVelocity, state);
                Vector3 airFrictionTorque = CalculateDragTorque(velocity, angularVelocity, state);
                Vector3 totalTorque = frictionTorque + airFrictionTorque;
                
                return totalTorque;
            }

            private (Vector3, Vector3) CalculateAccelerationAndAngularAcceleration(Vector3 velocity, Vector3 angularVelocity, BallState state)
            {
                Vector3 acceleration = CalculateTotalForces(velocity, angularVelocity, state) / ball.mass;
                Vector3 angularAcceleration = CalculateTotalTorque(velocity, angularVelocity, state) / ball.InertialMomentum;
                
                angularAcceleration = state == BallState.Rolling ? Vector3.Cross(acceleration / ball.radius, Vector3.down) + Vector3.up * angularAcceleration.y : angularAcceleration;
                
                return (acceleration, angularAcceleration);
            }
            
        #endregion
    
    #endregion
    
    #region Trajectory Prediction
    
        //This method computes the optimal velocity for the ball to reach a target, given the initial ball position, initial speed(velocity magnitude) and initial spin.
        //Moreover, minVerticalAngle and maxVerticalAngle are used if we want the angle of the velocity over horizontal plane to be clamped between the two values. 
        //constraintShot is only set true when the function is being called in the constraint shot function. This prevents the Debug.Logs to be printed.
        //testing is only set true when the function is being called during the tests. This allows to save the data of the test.
        private (Vector3, Vector3) FindOptimalVelocityForTargetHitGivenInitialSpeedAndSpin(float deltaTime, Vector3 targetPosition, Vector3 initialPosition, float speed, Vector3 spin, float minVerticalAngle, float maxVerticalAngle, int maximumIterations, bool constraintShot = false, Vector3 constraintPosition = default, bool testing = false)
        { 
            Stopwatch stopwatch = Stopwatch.StartNew();
            float distanceFromTarget = Vector3.Distance(initialPosition, targetPosition);
            
            //initial velocity direction guess is toward the target and angle over horizontal plane is the average between minVerticalAngle and maxVerticalAngle
            Vector3 direction = new Vector3(targetPosition.x - initialPosition.x, 0, targetPosition.z - initialPosition.z).normalized;
            direction = new Vector3(direction.x * Mathf.Cos((maxVerticalAngle - (maxVerticalAngle - minVerticalAngle) / 2) * Mathf.Deg2Rad), Mathf.Sin((maxVerticalAngle - (maxVerticalAngle - minVerticalAngle) / 2) * Mathf.Deg2Rad), direction.z * Mathf.Cos((maxVerticalAngle - (maxVerticalAngle - minVerticalAngle) / 2) * Mathf.Deg2Rad));
            Vector3 bestVelocityEstimation = direction * speed;
            
            //align spin with the velocity
            Vector3 alignedSpin = AlignSpinWithVelocityDirection(bestVelocityEstimation, spin);

            //instantiate iteration counter, error and rotation quaternion
            int iterationCounter = 0;
            float error = float.MaxValue;
            Vector3 currentVelocityEstimation = bestVelocityEstimation;
            
            //vector that points from the initial position to the target position
            Vector3 vTarget = (targetPosition - initialPosition).normalized;
            
            //initialize randomizer
            Random randomizer = new Random(180300);
            
            //keeps iterating until either the error gets smaller than the maximum tolerated error or when the iteration counter reaches the maximum
            while (error > targetMaximumErrorCurve.Evaluate(distanceFromTarget) * 1e-3f && iterationCounter < maximumIterations)
            {
                iterationCounter++;
                
                //align spin with the velocity
                alignedSpin = AlignSpinWithVelocityDirection(currentVelocityEstimation, spin);
                
                //compute the closest point to the target with current rotation
                _currentTrajectory = new List<Vector3>();
                (Vector3 predictedPosition,_) = ComputeTrajectoryPathToTarget(deltaTime, initialPosition, Quaternion.identity, currentVelocityEstimation, alignedSpin, targetPosition, constraintShot, constraintPosition);
                closestPointToTargetPlaceholder.transform.position = predictedPosition;
                
                //check angle from horizontal plane
                float upDot = Vector3.Dot(currentVelocityEstimation.normalized, Vector3.up);
                float angleFromHorizontalPlane = 90f - Mathf.Acos(upDot) * Mathf.Rad2Deg;

                float newError = Vector3.Distance(predictedPosition, targetPosition);
                
                //if the angle from horizontal plane is between the minimum and maximum limits, a new iteration will be computed
                if (angleFromHorizontalPlane < maxVerticalAngle && angleFromHorizontalPlane > minVerticalAngle)
                {
                    //if the error doesn't improve, stop the simulation
                    if (Mathf.Abs(error - newError) < 1e-3)
                        break;
                    
                    //new velocity estimation is set equal to the rotated velocity
                    bestVelocityEstimation = currentVelocityEstimation;
                    
                    //error is updated
                    error = newError;

                    //compute new velocity estimation, rotating the previous best estimation in the target direction
                    currentVelocityEstimation = RotateVelocityTowardTarget(initialPosition, predictedPosition, vTarget, bestVelocityEstimation, error, targetMaximumErrorCurve.Evaluate(distanceFromTarget) * 1e-3f, randomizer);

                    if (!constraintShot)
                        _precalculationTrajectories.Add(_currentTrajectory);
                }
                
                //if the angle from horizontal plane is not between the minimum and maximum limits, it means that the predictor can't make a better guess to reach the target and the angle from the horizontal plane will be clamped to either the minimum or the maximum possible value
                else
                {
                    float velocityMagnitude = currentVelocityEstimation.magnitude;
                    
                    //compute projection of the rotated velocity on the horizontal plane
                    Vector2 xzProjection = new Vector2(currentVelocityEstimation.x, currentVelocityEstimation.z);
                    float xzMagnitude = xzProjection.magnitude;

                    //find the new y value in order to have the angle from the horizontal plane either equal to minimum vertical angle or maximum vertical angle
                    float newY = angleFromHorizontalPlane > maxVerticalAngle ? Mathf.Tan(maxVerticalAngle * Mathf.Deg2Rad) * xzMagnitude : Mathf.Tan(minVerticalAngle * Mathf.Deg2Rad) * xzMagnitude;
                    bestVelocityEstimation = new Vector3(currentVelocityEstimation.x, newY, currentVelocityEstimation.z).normalized * velocityMagnitude;
                    
                    //align spin with the velocity
                    alignedSpin = AlignSpinWithVelocityDirection(currentVelocityEstimation, spin);

                    //compute the closest point to the target with current rotation
                    _currentTrajectory = new List<Vector3>();
                    (predictedPosition,_) = ComputeTrajectoryPathToTarget(deltaTime, initialPosition, Quaternion.identity, bestVelocityEstimation, alignedSpin, targetPosition, constraintShot, constraintPosition);
                    closestPointToTargetPlaceholder.transform.position = predictedPosition;
                    
                    //the error between the estimated position and the actual target position is computed
                    error = Vector3.Distance(predictedPosition, targetPosition);

                    if(!constraintShot)
                        _precalculationTrajectories.Add(_currentTrajectory);
                    
                    //the cycle is interrupted because the prediction can't reach a better solution since the angle would be outside the limits
                    break;
                }
            }
            
            stopwatch.Stop();
            
            if(!constraintShot)
            {
                //debugging results
                double computationTimeInMilliseconds = stopwatch.Elapsed.TotalMilliseconds;
                    
                Debug.Log("Prediction duration: " + computationTimeInMilliseconds.ToString("F3") + " ms with " + iterationCounter + " iterations");
                Debug.Log("Error: " + (error * 1e3).ToString("F3") + " mm");

                //saving the data for the test
                if (testing)
                {
                    _testData.errors.Add(error*1e3f);
                    _testData.distances.Add(distanceFromTarget);
                    _testData.iterations.Add(iterationCounter);
                    _testData.initialSpeeds.Add(speed);
                    _testData.constraintErrors.Add(Mathf.Abs(-1));
                    _testData.zSpin.Add(spin.z);
                    _testData.ySpin.Add(spin.y);
                    _testData.computationTime.Add(computationTimeInMilliseconds);
                }
            }

            return (bestVelocityEstimation, alignedSpin);
        }
        
        //This method computes the optimal velocity and spin for the ball to reach a target, passing through a given constraint(if possible) given the initial ball position and initial speed(velocity magnitude).
        //testing is only set true when the function is being called during the tests. This allows to save the data of the test.
        private (Vector3, Vector3)  FindOptimalVelocityForTargetHitWithConstraintGivenInitialSpeed(float deltaTime, Vector3 targetPosition, Vector3 constraintPosition, Vector3 initialPosition, float speed, int maximumIterations, bool testing = false)
        { 
            Stopwatch stopwatch = Stopwatch.StartNew();
            
            float distanceFromTarget = Vector3.Distance(initialPosition, targetPosition);
            int iterationCounter = 0;

            //this rotation is needed to have the displacement of the shot in the coordinate system of the XY plane
            //in this way the displacement can be computed from any direction
            Quaternion displacementXYPlaneRot = Quaternion.FromToRotation(new Vector3(initialPosition.x - targetPosition.x, 0, initialPosition.z - targetPosition.z), Vector3.left);
            
            //first velocity guess, pointing from the initial position to the constraint with no spin
            _currentTrajectory = new List<Vector3>();
            Vector3 bestEstimatedVelocity = (constraintPosition - initialPosition).normalized * speed;
            
            (Vector3 noSpinPredictedTargetPosition,_) = ComputeTrajectoryPathToTarget(deltaTime, initialPosition, Quaternion.identity, bestEstimatedVelocity, Vector3.zero, targetPosition, true, constraintPosition);

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
            (Vector3 bestSpinPredictedTargetPosition,Vector3 bestSpinPredictedConstraintPosition) = ComputeTrajectoryPathToTarget(deltaTime, initialPosition, Quaternion.identity, bestEstimatedVelocity, bestEstimatedSpin, targetPosition, true, constraintPosition);
            
            //find target error and constraint error of the trajectory
            float targetError = (bestSpinPredictedTargetPosition - targetPosition).magnitude;
            float constraintError = (bestSpinPredictedConstraintPosition - constraintPosition).magnitude;

            //vector that points from the initial position to the constraint position
            Vector3 vConstraint = (constraintPosition - initialPosition).normalized;

            float previousTargetError = float.PositiveInfinity;
            
            //the cycle stops when either the iteration counter reaches the maximum or when both the target and the constraint errors are below their maximum value
            while (iterationCounter < maximumIterations && (targetError > targetMaximumErrorCurve.Evaluate(distanceFromTarget) * 1e-3f || constraintError > constraintMaximumError))
            {
                iterationCounter++;
                _currentTrajectory = new List<Vector3>();
                
                //HORIZONTAL//
                
                //computes trajectory having no horizontal spin component
                Vector3 noHorizontalSpin = bestEstimatedSpin;
                noHorizontalSpin.y = 0f;
                (Vector3 noHorizontalSpinPredictedTargetPosition,_) = ComputeTrajectoryPathToTarget(deltaTime, initialPosition, Quaternion.identity, bestEstimatedVelocity, noHorizontalSpin, targetPosition, true, constraintPosition);
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
                (_, Vector3 predictedConstraintPosition) = ComputeTrajectoryPathToTarget(deltaTime, initialPosition, Quaternion.identity, bestEstimatedVelocity, newEstimatedSpin, targetPosition, true, constraintPosition);
                Vector3 newEstimatedVelocity = RotateVelocityTowardConstraint(initialPosition, predictedConstraintPosition, vConstraint, bestEstimatedVelocity);
                newEstimatedSpin = AlignSpinWithVelocityDirection(newEstimatedVelocity, spin);

                //compute new trajectory to find new predicted target position
                (bestSpinPredictedTargetPosition,_) = ComputeTrajectoryPathToTarget(deltaTime, initialPosition, Quaternion.identity, newEstimatedVelocity, newEstimatedSpin, targetPosition, true, constraintPosition);
                
                //VERTICAL//
                
                //computes trajectory having no vertical spin component
                Vector3 noVerticalSpin = spin;
                noVerticalSpin.z = 0f;
                (Vector3 noVerticalSpinPredictedTargetPosition,_) = ComputeTrajectoryPathToTarget(deltaTime, initialPosition, Quaternion.identity, newEstimatedVelocity, noVerticalSpin, targetPosition, true, constraintPosition);

                //compute the target displacement of the shot without vertical spin
                targetDisplacement = displacementXYPlaneRot * (noVerticalSpinPredictedTargetPosition - targetPosition).normalized;

                //finds the new vertical spin proportionally to the displacement of the predicted target of the shot with no vertical spin from both the actual target position and the predicted position of the previous best found spin
                d1 = bestSpinPredictedTargetPosition.y - noVerticalSpinPredictedTargetPosition.y;
                d2 = targetPosition.y - noVerticalSpinPredictedTargetPosition.y;
                verticalSpinMagnitude = Mathf.Min(verticalSpinMagnitude * Math.Abs(d2 / d1), ball.maximumSpinValue);
                spin.z = -Math.Sign(targetDisplacement.y) * verticalSpinMagnitude;
                newEstimatedSpin = AlignSpinWithVelocityDirection(newEstimatedVelocity, spin);
                
                //computes trajectory with the new-found vertical spin and align spin with the velocity
                (_, predictedConstraintPosition) = ComputeTrajectoryPathToTarget(deltaTime, initialPosition, Quaternion.identity, newEstimatedVelocity, newEstimatedSpin, targetPosition, true, constraintPosition);
                
                newEstimatedVelocity = RotateVelocityTowardConstraint(initialPosition, predictedConstraintPosition, vConstraint, newEstimatedVelocity);
                newEstimatedSpin = AlignSpinWithVelocityDirection(newEstimatedVelocity, spin);
                
                //compute new trajectory to find new predicted target position
                _currentTrajectory = new List<Vector3>();
                (bestSpinPredictedTargetPosition,bestSpinPredictedConstraintPosition) = ComputeTrajectoryPathToTarget(deltaTime, initialPosition, Quaternion.identity, newEstimatedVelocity, newEstimatedSpin, targetPosition, true, constraintPosition);
                
                _precalculationTrajectories.Add(_currentTrajectory);
                
                //finds distance from target and from constraint of the trajectory
                targetError = (bestSpinPredictedTargetPosition - targetPosition).magnitude;
                constraintError = (bestSpinPredictedConstraintPosition - constraintPosition).magnitude;
                
                
                //if the target error doesn't improve because the maximum possible spin for one of the two components reaches the maximum value, then the prediction is interrupted
                if ((Mathf.Abs(targetError - previousTargetError ) < 1e-5 || targetError > previousTargetError) && (Mathf.Abs(spin.y) - ball.maximumSpinValue < 1e-6f || Mathf.Abs(spin.z) - ball.maximumSpinValue < 1e-6f))
                {
                    iterationCounter++;
                        
                    //use the predictor with no constraint with the found spin to have the trajectory prioritizing the target but passing in the closest possible point to the constraint
                    (newEstimatedVelocity, newEstimatedSpin) = FindOptimalVelocityForTargetHitGivenInitialSpeedAndSpin(deltaTime, targetPosition, initialPosition, speed, spin, 0, 30, 10, true, constraintPosition);
                   
                    //compute trajectory with the new-found best estimated velocity and spin
                    _currentTrajectory = new List<Vector3>();
                    (bestSpinPredictedTargetPosition,bestSpinPredictedConstraintPosition) = ComputeTrajectoryPathToTarget(deltaTime, initialPosition, Quaternion.identity, newEstimatedVelocity, newEstimatedSpin, targetPosition, true, constraintPosition);
                    _precalculationTrajectories.Add(_currentTrajectory);
                
                    //finds distance from target and from constraint of the trajectory
                    targetError = (bestSpinPredictedTargetPosition - targetPosition).magnitude;
                    constraintError = (bestSpinPredictedConstraintPosition - constraintPosition).magnitude;

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
            double computationTimeInMilliseconds = stopwatch.Elapsed.TotalMilliseconds;
            
            //debugging predicted position
            _currentTrajectory = new List<Vector3>();
            (bestSpinPredictedTargetPosition,bestSpinPredictedConstraintPosition) = ComputeTrajectoryPathToTarget(deltaTime, initialPosition, Quaternion.identity, bestEstimatedVelocity, bestEstimatedSpin, targetPosition, true, constraintPosition);
            closestPointToTargetPlaceholder.transform.position = bestSpinPredictedTargetPosition;
            closestPointToConstraintPlaceholder.transform.position = bestSpinPredictedConstraintPosition;
            
            Debug.Log("Constrained kick prediction duration: " + computationTimeInMilliseconds.ToString("F3") + " ms with " + iterationCounter + " iterations");
            Debug.Log("Constraint error: " + (constraintError*1e3f).ToString("F3") + " mm");
            Debug.Log("Target error: " + (targetError*1e3f).ToString("F3") + " mm");
            
            //saving the data for the test
            if (testing)
            {
                _testData.errors.Add(targetError*1e3f);
                _testData.distances.Add(Vector3.Distance(initialPosition, targetPosition));
                _testData.iterations.Add(iterationCounter);
                _testData.initialSpeeds.Add(speed);
                _testData.constraintErrors.Add(Mathf.Abs(constraintError*1e3f));
                _testData.zSpin.Add(spin.z);
                _testData.ySpin.Add(spin.y);
                _testData.computationTime.Add(computationTimeInMilliseconds);
            }
            
            return (bestEstimatedVelocity,bestEstimatedSpin);
        }
        
        //given the initial conditions of the ball, the method will compute the trajectory from the initialPosition until the closest position to the target
        //the method returns the closest point of the trajectory to the target and the closest point to the constraint
        private (Vector3,Vector3) ComputeTrajectoryPathToTarget(float deltaTime, Vector3 initialPosition, Quaternion initialOrientation, Vector3 initialVelocity,  Vector3 initialAngularVelocity, Vector3 targetPosition, bool constraintShot, Vector3 constraintPosition = default)
        {
            bool stopSimulation = false;
            bool foundClosestConstraintPoint = false;

            //initialize useful state variables
            //previous state variables are needed to go back in time when the ball passes to the closest point 
            Vector3 currentPosition = initialPosition;
            Vector3 previousPosition = initialPosition;
            Quaternion currentOrientation = initialOrientation;
            Quaternion previousOrientation = initialOrientation;
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
                (Vector3 newPosition, Quaternion newOrientation, Vector3 newVelocity, Vector3 newAngularVelocity) = IntegrateMotion(deltaTime, currentPosition,  currentOrientation, currentVelocity, currentAngularVelocity, BallState.Bouncing);
                _currentTrajectory.Add(newPosition);
                
                //compute the new distance from the target
                float targetError = Vector3.Distance(currentPosition, targetPosition);
                float newTargetError = Vector3.Distance(newPosition, targetPosition);
                
                //if the error at frame n is < than the error at frame n-1 and < than the error at frame n+1, then the closest point is between n-1 and n+1
                if (targetError <= previousTargetError && targetError <= newTargetError)
                {
                    stopSimulation = true;
                    
                    //if the error at frame n-1 is < than the error at frame n+1, then the closest point is between n-1 and n, otherwise between n and n+1
                    targetClosestPoint = previousTargetError <= newTargetError 
                        ? BinarySearchClosestPoint(deltaTime / 2, deltaTime / 4, previousPosition, previousOrientation, previousVelocity, previousAngularVelocity, BallState.Bouncing, targetPosition)
                        : BinarySearchClosestPoint(deltaTime / 2, deltaTime / 4, currentPosition, currentOrientation, currentVelocity, currentAngularVelocity, BallState.Bouncing, targetPosition);
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
                            ? BinarySearchClosestPoint(deltaTime / 2, deltaTime / 4, previousPosition, previousOrientation, previousVelocity, previousAngularVelocity, BallState.Bouncing, constraintPosition)
                            : BinarySearchClosestPoint(deltaTime / 2, deltaTime / 4, currentPosition, currentOrientation, currentVelocity, currentAngularVelocity, BallState.Bouncing, constraintPosition);
                        
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
            
            (ball.velocity, ball.angularVelocity) = FindOptimalVelocityForTargetHitGivenInitialSpeedAndSpin(1/framePerSecond, target.transform.position, ball.position, speed, spin, 0, 30, maxIterations);
            ball.state = BallState.Bouncing;
                
            ball.AddTrajectoryFrame();
        }
        
        //this method finds the optimal velocity adn angular velocity for the ball to reach the target passing through a constraint and sets velocity and angular velocity equal to the found values, given speed
        public void TargetedKickWithConstraint(float speed)
        {
            _precalculationTrajectories.Clear();
            
            (ball.velocity, ball.angularVelocity) = FindOptimalVelocityForTargetHitWithConstraintGivenInitialSpeed(1/framePerSecond, target.transform.position, constraint.transform.position, ball.position, speed, maxIterations);
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
                FindOptimalVelocityForTargetHitGivenInitialSpeedAndSpin(1/framePerSecond, targetPosition, testCurrentPosition, speed, spin, 0, 45f, maxIterations, false, default, true);

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
                    FindOptimalVelocityForTargetHitWithConstraintGivenInitialSpeed(1 / framePerSecond, targetPosition, constraintPosition, initialPosition, testInitialSpeed, maxIterations, true);
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
        
        //this method is used to find the rotation needed for the velocity to have the ball passing through the constraint
        private Vector3 RotateVelocityTowardConstraint(Vector3 initialPosition, Vector3 predictedConstraintPosition, Vector3 vConstraint, Vector3 velocity)
        {
            //first it computes the vector pointing from the initial position of the ball in the kick to the predicted closest position to the target in the trajectory
            Vector3 vPredictedConstraint = (predictedConstraintPosition - initialPosition).normalized;
            
            //then the rotation needed to transform the previously found vector into the vector vConstraint(which points from the initial position to the actual constraint position) is computed
            Quaternion rotation = Quaternion.FromToRotation(vPredictedConstraint, vConstraint);
            
            //the rotation is applied to the velocity
            Vector3 rotatedVelocity = rotation * velocity;
           
            return rotatedVelocity;
        }

        //this method is similar to the one above but is more accurate, therefore it is used for the adjustment of the trajectory to the target, but it's also less efficient
        private Vector3 RotateVelocityTowardTarget(Vector3 initialPosition, Vector3 predictedTargetPosition, Vector3 vTarget, Vector3 velocity, float error, float maxError, Random random)
        {
            //vIteration points from the initial position to the estimated target position
            Vector3 vIteration = (predictedTargetPosition - initialPosition).normalized;
                    
            //compute the cosine of the angle between vIteration and vTarget
            float cosTheta = Vector3.Dot(vIteration, vTarget);

            //compute the angle between vIteration and vTarget
            float theta = Mathf.Acos(Mathf.Clamp(cosTheta, -1f, 1f)) * Mathf.Rad2Deg;

            //if the angle is too small and the error is still above the target maximum error I adjust the velocity estimation magnitude, this prevents to have too many iterations in some cases
            if (Mathf.Abs(theta) < 1e-3 && error > maxError)
            {
                velocity *= random.Next(2) == 0 ? 1.0005f : 0.9995f;
            }
                    
            //compute rotation axis, perpendicular to the plane containing vIteration and vTarget
            Vector3 rotationAxis = Vector3.Cross(vIteration, vTarget).normalized;
                    
            //compute the rotation needed to rotate vIteration to vTarget
            Quaternion rotation = Quaternion.AngleAxis(theta, rotationAxis);

            //apply rotation to velocity
            Vector3 rotatedVelocity = rotation * velocity;

            return rotatedVelocity;
        }
        
        //this method finds a rotation that is needed to be applied to the spin in order to make its z component orthogonal to the velocity direction
        private Vector3 AlignSpinWithVelocityDirection(Vector3 velocity, Vector3 spin)
        {
            //find the projection of the direction of the velocity on the xz plane
            Vector3 targetXZ = new Vector3(velocity.x, 0f, velocity.z).normalized;
            
            //find the needed rotation
            Quaternion targetRotation = Quaternion.Euler(0,-90,0) * Quaternion.LookRotation(targetXZ, Vector3.up);
            
            return targetRotation * spin;
        }
        
        //this method calculates the closest position of a ball to a target position
        //using a recursive binary search to find the time of closest approach
        private Vector3 BinarySearchClosestPoint(float deltaTime, float step , Vector3 position, Quaternion orientation, Vector3 velocity, Vector3 angularVelocity, BallState state, Vector3 targetPosition)
        {
            //calculate position at (deltaTime - step) and get its distance to target
            (Vector3 pMinus, _, _, _) = IntegrateMotion(deltaTime - step, position, orientation, velocity, angularVelocity, state);
            float distance1 = Vector3.Distance(pMinus, targetPosition);
            
            //calculate position at (deltaTime + step) and get its distance to target
            (Vector3 pPlus, _, _, _) = IntegrateMotion(deltaTime + step, position, orientation, velocity, angularVelocity, state);
            float distance2 = Vector3.Distance(pPlus, targetPosition);
            
            Vector3 closestPoint;
            
            //compare which distance is closer to the target
            if (distance1 < distance2) 
            {
                //if the step size is smaller than the maximum allowed value, the best estimation is found
                if (step < CollisionThreshold)
                {
                    float correctDeltaTime = deltaTime - step;
                    (closestPoint, _, _, _) = IntegrateMotion(correctDeltaTime, position, orientation, velocity, angularVelocity, state);
                }
                else
                    //recursively search in the negative direction with half the step size
                    closestPoint = BinarySearchClosestPoint(deltaTime - step, step/2, position, orientation, velocity, angularVelocity, state, targetPosition);
            }
            else
            {
                //if the step size is smaller than the maximum allowed value, the best estimation is found
                if (step < CollisionThreshold)
                {
                    float correctDeltaTime = deltaTime + step;
                    (closestPoint, _, _, _) = IntegrateMotion(correctDeltaTime, position, orientation, velocity, angularVelocity, state);
                }
                else
                    //recursively search in the negative direction with half the step size
                    closestPoint = BinarySearchClosestPoint(deltaTime + step, step/2, position, orientation, velocity, angularVelocity, state, targetPosition);
            }

            return closestPoint;
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

    #endregion
    
    #region Getters
        public Vector3 TotalForces()
        {
            if(useFrameByFrameMode)
            {
                return CalculateTotalForces(ball.Frames[FrameCount].Velocity, ball.Frames[FrameCount].AngularVelocity, ball.Frames[FrameCount].State);
            }
            return CalculateTotalForces(ball.velocity, ball.angularVelocity, ball.state);
        }

        public Vector3 DragForce()
        {
            if(useFrameByFrameMode)
            {
                return ComputeAirResistanceForce(ball.Frames[FrameCount].Velocity, ball.Frames[FrameCount].AngularVelocity);
            }
            return ComputeAirResistanceForce(ball.velocity, ball.angularVelocity);
        }

        public Vector3 MagnusForce()
        {
            if(useFrameByFrameMode)
            {
                return ComputeMagnusForce(ball.Frames[FrameCount].Velocity, ball.Frames[FrameCount].AngularVelocity, ball.Frames[FrameCount].State);
            }
            return ComputeMagnusForce(ball.velocity, ball.angularVelocity, ball.state);
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
                return CalculateFrictionTorque(ball.Frames[FrameCount].Velocity, ball.Frames[FrameCount].AngularVelocity, ball.Frames[FrameCount].State);
            }
            return CalculateFrictionTorque(ball.velocity, ball.angularVelocity, ball.state);
        }
        
        public Vector3 DragTorque()
        {
            if(useFrameByFrameMode)
            {
                return CalculateDragTorque(ball.Frames[FrameCount].Velocity, ball.Frames[FrameCount].AngularVelocity, ball.Frames[FrameCount].State);
            }
            return CalculateDragTorque(ball.velocity, ball.angularVelocity, ball.state);
        }
        
        public Vector3 TotalTorque()
        {
            if(useFrameByFrameMode)
            {
                return CalculateTotalTorque(ball.Frames[FrameCount].Velocity, ball.Frames[FrameCount].AngularVelocity, ball.Frames[FrameCount].State);
            }
            return CalculateTotalTorque(ball.velocity, ball.angularVelocity, ball.state);
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
                return CalculateTotalForces(ball.Frames[FrameCount].Velocity, ball.Frames[FrameCount].AngularVelocity, ball.Frames[FrameCount].State)/ball.mass;
            }
            
            return CalculateTotalForces(ball.velocity, ball.angularVelocity, ball.state)/ball.mass;
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




