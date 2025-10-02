using System;
using System.Diagnostics;
using UnityEngine;
using UnityEngine.Serialization;
using Debug = UnityEngine.Debug;
using SysQuat = System.Numerics.Quaternion;
using Vector3 = UnityEngine.Vector3;

public class BallSimulation : MonoBehaviour
{
    #region Variables
        public BallProperties ball;
        
        [Header("Constants")]
        [SerializeField] private float airDensity = 1.2f;
        [SerializeField] private float gravity = 9.81f; 
        
        private const float TimeStepThreshold = 1e-6f;
        private Vector3 _gravitationalForce;
        private Vector3 _buoyantForce;
        
        [Header("Simulation")]
        [SerializeField] private IntegrationMethod integrationMethod;
        [SerializeField] private float framePerSecond = 60;
        
        [Header("Frame History")] 
        public bool useFrameByFrameMode;
        public bool interpolateFrames = true;
        public int FrameCount { private get; set; }
        public float ContinuousFrameCount { private get; set; }
        
    #endregion

    #region Unity
        void Awake()
        {
            Application.targetFrameRate = -1;
            Time.fixedDeltaTime = 1/framePerSecond;
            
            _gravitationalForce = gravity * ball.mass * Vector3.down;
            _buoyantForce = 4f / 3f * Mathf.PI * Mathf.Pow(ball.radius, 3) * airDensity * gravity * Vector3.up;
        }
        
        void FixedUpdate()
        {
            //TODO ELIMINARE
            if (useFrameByFrameMode)
            {
                HandleFrameByFrame();
                return;
            }
            
            if (ball.state == BallStates.Stopped)
                return;
            
            ComputeNewFrame(Time.fixedDeltaTime, ball);
            
            ball.AddTrajectoryFrame(0);
        }

        private void Update()
        {
            Application.targetFrameRate = -1;
            Time.fixedDeltaTime = 1/framePerSecond;
            
            _gravitationalForce = gravity * ball.mass * Vector3.down;
            _buoyantForce = 4f / 3f * Mathf.PI * Mathf.Pow(ball.radius, 3) * airDensity * gravity * Vector3.up;
        }

        #endregion
    
    #region Simulation

    public void ComputeNewFrame(float deltaTime, BallProperties targetBall)
        {
            //given the current state of the ball, the motion is integrated and the new state of the ball after delta time is returned
            Stopwatch stopwatch = Stopwatch.StartNew();
            (Vector3 newPosition, SysQuat newOrientation, Vector3 newVelocity, Vector3 newAngularVelocity) = IntegrateMotion(deltaTime, targetBall.position, QuaternionUtils.ToNumerics(targetBall.orientation), targetBall.velocity, targetBall.angularVelocity, targetBall.state);
            stopwatch.Stop();
            
            //checks if there was a collision and if so handles it
            Collider[] hits = Physics.OverlapSphere(newPosition, ball.radius);
            if (hits.Length > 0)
                (newPosition, newOrientation, newVelocity, newAngularVelocity, targetBall.state) = HandleGroundCollisionAndBounce(deltaTime, targetBall.position, QuaternionUtils.ToNumerics(targetBall.orientation), targetBall.velocity, targetBall.angularVelocity, targetBall.state);

            Vector3 previousVelocity = targetBall.velocity;
            Vector3 previousAngularVelocity = targetBall.angularVelocity;
            
            //updates the state of the ball
            targetBall.position = newPosition;
            targetBall.orientation = QuaternionUtils.ToUnity(newOrientation);
            targetBall.velocity = newVelocity;
            targetBall.angularVelocity = newAngularVelocity;
            
            //checks if the ball is a pure rolling state
            if(targetBall.state == BallStates.Sliding)
                CheckPureRolling(targetBall, previousVelocity, previousAngularVelocity);
            
        }
        
        //this method chooses one of the four integration method to compute the new state of the ball after delta time
        private (Vector3, SysQuat, Vector3, Vector3) IntegrateMotion(float deltaTime, Vector3 initialPosition, SysQuat initialOrientation, Vector3 initialVelocity, Vector3 initialAngularVelocity, BallStates state)
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
        private void CheckPureRolling(BallProperties targetBall, Vector3 previousVelocity, Vector3 previousAngularVelocity)
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
                targetBall.state = BallStates.Rolling;
            }
        }
        
        #region Integration Methods
        
            private (Vector3, SysQuat, Vector3, Vector3) ExplicitEuler(float deltaTime,Vector3 pCurrent, SysQuat qCurrent, Vector3 vCurrent, Vector3 wCurrent, BallStates state)
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
            
            private (Vector3, SysQuat, Vector3, Vector3) SemiImplicitEuler(float deltaTime, Vector3 pCurrent, SysQuat qCurrent, Vector3 vCurrent, Vector3 wCurrent, BallStates state)
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
            
            private (Vector3, SysQuat, Vector3, Vector3) Runge_Kutta_2(float deltaTime, Vector3 pCurrent, SysQuat qCurrent, Vector3 vCurrent, Vector3 wCurrent, BallStates state)
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

            
            private (Vector3, SysQuat, Vector3, Vector3) Runge_Kutta_4(float deltaTime,Vector3 pCurrent, SysQuat qCurrent, Vector3 vCurrent, Vector3 wCurrent, BallStates state)
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
            private (Vector3, SysQuat, Vector3, Vector3, BallStates) HandleGroundCollisionAndBounce(float deltaTime, Vector3 initialPosition, SysQuat initialOrientation, Vector3 initialVelocity, Vector3 initialAngularVelocity, BallStates state)
            {
                //finds moment of collision
                (float collisionDeltaTime, Vector3 collisionPoint, Vector3 collisionPosition) = IterativeCollisionDetection(deltaTime / 2, deltaTime / 2, initialPosition, initialOrientation, initialVelocity, initialAngularVelocity, state);
                
                //updates position, orientation, velocity and angular velocity after the collision
                (Vector3 positionAfterCollision, SysQuat orientationAfterCollision, Vector3 velocityAfterCollision, Vector3 angularVelocityAfterCollision) = ComputePostCollisionVelocities(collisionDeltaTime, initialPosition, initialOrientation, initialVelocity, initialAngularVelocity, state, collisionPoint);
                
                //sets the new state to sliding if the vertical component of the velocity is small enough
                if (Mathf.Abs(Vector3.Dot(Vector3.up, (collisionPosition - collisionPoint).normalized) - 1) < 1e-1 && velocityAfterCollision.y < 1e-1)
                {
                    velocityAfterCollision.y = 0f;
                    positionAfterCollision.y = ball.radius + collisionPoint.y;
                    state = BallStates.Sliding;
                }
                else
                {
                    state = BallStates.Bouncing;
                }
                
                return (positionAfterCollision, orientationAfterCollision, velocityAfterCollision, angularVelocityAfterCollision, state);
            }
            
            //this method is called recursively to binary search the moment of collision with the ground
            private (float, Vector3, Vector3) IterativeCollisionDetection(float deltaTime, float step, Vector3 pCurrent, SysQuat qCurrent, Vector3 vCurrent, Vector3 wCurrent, BallStates state)
            {
                //predict new position
                (Vector3 newPosition, _, _, _) = IntegrateMotion(deltaTime, pCurrent, qCurrent, vCurrent, wCurrent, state);
    
                //adjust delta time: go back if below ground, forward if above
                Collider[] hits = Physics.OverlapSphere(newPosition, ball.radius);
                float newDeltaTime = hits.Length > 0 ? deltaTime - step / 2 : deltaTime + step / 2;
    
                //stop if step is below precision threshold
                if (step < TimeStepThreshold && hits.Length > 0)
                    return (newDeltaTime, hits[0].ClosestPoint(newPosition), newPosition);
    
                //recurse with new delta time and halved step size
                return IterativeCollisionDetection(newDeltaTime, step/2, pCurrent, qCurrent, vCurrent, wCurrent, state);
            }
            
            private (Vector3, SysQuat, Vector3, Vector3) ComputePostCollisionVelocities(float collisionDeltaTime, Vector3 initialPosition, SysQuat initialOrientation, Vector3 initialVelocity, Vector3 initialAngularVelocity, BallStates state, Vector3 contactPoint)
            {
                //here the movement is integrated until the moment the ball collides with the ground
                (Vector3 collisionPosition, SysQuat collisionOrientation, Vector3 velocityBeforeCollision, Vector3 angularVelocityBeforeCollision) = IntegrateMotion(collisionDeltaTime, initialPosition, initialOrientation, initialVelocity, initialAngularVelocity, state);
                
                
                // Step 2: calcola la normale di contatto
                Vector3 n = (collisionPosition - contactPoint).normalized;

                // Step 3: costruisci la matrice di rotazione R che porta n in Vector3.up
                Quaternion R = Quaternion.FromToRotation(n, Vector3.up);

                // Step 4: ruota velocità nel frame “orizzontale”
                velocityBeforeCollision = R * velocityBeforeCollision;
                angularVelocityBeforeCollision = R * angularVelocityBeforeCollision;
                
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
                
                
                velocityAfterCollision = Quaternion.Inverse(R) * velocityAfterCollision;
                angularVelocityAfterCollision = Quaternion.Inverse(R) * angularVelocityAfterCollision;
                
                return IntegrateMotion(remainingTime, collisionPosition, collisionOrientation, velocityAfterCollision, angularVelocityAfterCollision, state);
            }
            
        #endregion
        
        #region Forces And Torques
            
            //this method computes the force caused by the air resistance
            private Vector3 CalculateDragForce(Vector3 velocity)
            {
                if(velocity.magnitude < 1e-3)
                    return Vector3.zero;
                
                //use constant linear drag coefficient
                Vector3 dragForce = -0.5f * airDensity * ball.dragCoefficient * Mathf.PI * Mathf.Pow(ball.radius, 2) * velocity.magnitude * velocity;
                
                return dragForce;
            }
            
            private Vector3 CalculateMagnusForce(Vector3 velocity, Vector3 angularVelocity, BallStates state)
            {
                if(velocity.magnitude < 1e-3 || angularVelocity.magnitude < 1e-3 || state != BallStates.Bouncing)
                    return Vector3.zero;
                
                //use constant linear drag coefficient
                Vector3 magnusForce = ball.liftCoefficient * Mathf.PI * airDensity * Mathf.Pow(ball.radius, 3) * Vector3.Cross(angularVelocity, velocity);
                    
                return magnusForce;
            }
            
            //this method computes this friction force with the ground, given the velocity, the angular velocity and the current state of the ball
            private Vector3 CalculateFrictionForce(Vector3 velocity, Vector3 angularVelocity, BallStates state)
            {
                if(state == BallStates.Bouncing)
                    return Vector3.zero;
                
                Vector3 frictionForce;
                
                float normalForce = (_gravitationalForce + _buoyantForce).magnitude;
                
                //if the ball is sliding, use the sliding friction formula
                if(state == BallStates.Sliding) 
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

            private Vector3 CalculateNormalForce(BallStates state)
            {
                return state != BallStates.Bouncing ? - _gravitationalForce - _buoyantForce : Vector3.zero;
            }
            
            //this method computes the result of the sum of all forces acting on the ball
            private Vector3 CalculateTotalForce(Vector3 velocity, Vector3 angularVelocity, BallStates state)
            {
                Vector3 dragForce = CalculateDragForce(velocity);
                Vector3 magnusForce = CalculateMagnusForce(velocity, angularVelocity, state);
                Vector3 normalForce = CalculateNormalForce(state);
                Vector3 frictionForce = CalculateFrictionForce(velocity, angularVelocity, state);
                
                //sum all forces
                Vector3 totalForce = _gravitationalForce + _buoyantForce + dragForce + magnusForce + normalForce + frictionForce;

                return totalForce;
            }
            
            //this method computes the torque acting on the ball created by the friction with the ground, given velocity, angular velocity and the state of the ball
            private Vector3 CalculateFrictionTorque(Vector3 velocity, Vector3 angularVelocity, Vector3 acceleration, BallStates state)
            { 
                if(state == BallStates.Bouncing)
                    return Vector3.zero;
                
                float normalForce = (_gravitationalForce + _buoyantForce).magnitude;
                
                Vector3 frictionTorque = new Vector3(0, - ball.coefficientOfVerticalAxisSpinningDamping * Mathf.Sign(angularVelocity.y), 0);
                
                //if the ball is rolling, the contact point is not moving with respect to the ground, therefore the is no other friction component acting on the ball
                if (state == BallStates.Rolling)
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
            
            private Vector3 CalculateSpinDegradation(Vector3 velocity, Vector3 angularVelocity, BallStates state)
            {
                if(state != BallStates.Bouncing || angularVelocity.magnitude < 1e-6)
                    return Vector3.zero;
                
                //use constant coefficient
                float totalForce = - 0.5f * airDensity * Mathf.PI * Mathf.Pow(ball.radius, 2) * Mathf.Pow(velocity.magnitude, 2) * ball.spinDecayCoefficient; 
    
                Vector3 spinDegradationTorque = totalForce * ball.radius / 2 * angularVelocity.normalized;

                return spinDegradationTorque;
            }
            
            //this method computes the result of the sum of all torques acting on the ball
            private Vector3 CalculateTotalTorque(Vector3 velocity, Vector3 angularVelocity, Vector3 acceleration, BallStates state)
            {
                Vector3 frictionTorque = CalculateFrictionTorque(velocity, angularVelocity, acceleration, state);
                Vector3 spinDegradationTorque = CalculateSpinDegradation(velocity, angularVelocity, state);
                Vector3 totalTorque = frictionTorque + spinDegradationTorque;
                
                return totalTorque;
            }

            private (Vector3, Vector3) CalculateAccelerationAndAngularAcceleration(Vector3 velocity, Vector3 angularVelocity, BallStates state)
            {
                Vector3 acceleration = CalculateTotalForce(velocity, angularVelocity, state) / ball.mass;
                Vector3 angularAcceleration = CalculateTotalTorque(velocity, angularVelocity, acceleration, state) / ball.InertialMomentum;
                
                return (acceleration, angularAcceleration);
            }
            
        #endregion
    
    #endregion
    
    #region Utils
        
        //reset ball and trajectory gizmos
        public void Reset()
        {
            ball.Reset();
        }
        
        private class K
        {
            public Vector3 V, W, P;
            public SysQuat Q;
        }
        
        private K ComputeK(Vector3 velocity, Vector3 angularVelocity, SysQuat orientation, BallStates state)
        {
            K k = new K();
                
            (k.V, k.W) = CalculateAccelerationAndAngularAcceleration(velocity, angularVelocity, state);

            k.P = velocity;

            k.Q = QuaternionUtils.QuaternionDerivative(orientation, angularVelocity);

            return k;
        }
        
        public void Kick(Vector3 direction, float speed, Vector3 spin)
        {
                
            ball.velocity = direction * speed;
            ball.angularVelocity = AlignSpinWithVelocityDirection(ball.velocity, spin);
            ball.state = ball.velocity.y > 0 ? BallStates.Bouncing : BallStates.Sliding;
                
            ball.AddTrajectoryFrame();
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
        
    #endregion
}




