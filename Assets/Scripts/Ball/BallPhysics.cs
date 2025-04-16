using System;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;
using UnityEngine.UI;
using Quaternion = UnityEngine.Quaternion;
using Vector2 = UnityEngine.Vector2;
using Vector3 = UnityEngine.Vector3;

public class BallPhysics : MonoBehaviour
{
    public Ball ballValues;

    private Vector3 _frictionForce;
    private Vector3 _previousSignOfFriction;
    
    
    
    [Header("Constants")]
    [SerializeField] private float Gravity = 9.81f; 
    [SerializeField] private float airDensity = 1.2f;
    [SerializeField] private float kinematicViscosityOfAir = 1.48f * 1e-5f;
    private const float CollisionDetectionError = 1e-5f;

    
    [Header("Test Initial values")]
    [SerializeField] private Vector3 testInitialSpin;
    [SerializeField] private float testInitialSpeed;
    
    public float maximumSpinValue = 30f;
    
    [Header("Simulation")]
    //TODO mettere un enum
    [SerializeField] private IntegrationMethod integrationMethod;
    //TODO 100 fps
    [SerializeField] private float frameRate;

    [Header("Tracker")]
    //TODO utilizzare verbi per i boolean
    public bool frameByFrame;
    public bool interpolateFrames;
    [Range(0,1000)] public int frameCount;
    public float continuousFrameCount;
    private Queue<Frame> _frames;
    public Frame[] frameArray;
    //TODO
    private float _timeToCompute;
    private float _totalTimeToCompute = 0f;

    [Header("Targeted shot")]
    //TODO eliminare 
    public bool precompute;
    
    [SerializeField] private int precomputationFrameRate;
    [SerializeField] private float precalculationMaxError;
    [SerializeField] private int maxIterations;
    //TODO verbi!!
    public bool barrierShot;
    [SerializeField] private GameObject barrier;
    [SerializeField] private GameObject constraint;

    
    //TODO creare una struttura per questo
    [Header("Testing")]
    [SerializeField] private bool targetKickTesting;
    private SaveSystem saveSystem;
    [SerializeField] private String description;
    [SerializeField] private String fileName;
    [SerializeField] private Vector3 testStartPosition;
    [SerializeField] private Vector3 testEndPosition;
    [SerializeField] private float minInitialSpeed;
    [SerializeField] private float maxInitialSpeed;
    [SerializeField] private int testSteps;
    [SerializeField] private List<Vector3> startPositions;
    [SerializeField] private List<float> distances;
    [SerializeField] private List<float> errors;
    [SerializeField] private List<int> iterations;
    [SerializeField] private List<Vector3> outliersStartPositions;
    [SerializeField] private List<float> outliersError;
    [SerializeField] private List<float> constraintErrors;
    [SerializeField] private List<float> initialSpeeds;
    [SerializeField] private List<float> zSpin;
    [SerializeField] private List<float> ySpin;
    [SerializeField] private List<float> computationTime;
    [SerializeField] private float averageComputationTime = 0f;
    //TODO 
    private int _currentStep = 0;
    private Vector3 _testCurrentPosition;
    private float _totalComputationTime;
    
    [Header("Other")]
    [SerializeField] private GameObject comparisonSphere;
    [SerializeField] private GameObject target;
    [SerializeField] private GameObject placeholder;
    [SerializeField] private GameObject constraintPlaceholder;
    private List<List<Vector3>> _precalculationTrajectories =  new List<List<Vector3>>();
    private List<Vector3> _currentTrajectory;
    
    public void Reset()
    {
        _precalculationTrajectories.Clear();
        
        ballValues.Reset();
        
        _frames.Clear();
    }

    public void Kick(Vector3 direction, float speed, Vector3 spin)
    {
        _precalculationTrajectories.Clear();
        
        ballValues.velocity = direction * speed;
        ballValues.angularVelocity = spin;
        ballValues.state = BallState.Bouncing;
        
        AddTrajectoryFrame();
    }

    public void TargetedKick(float speed, Vector3 spin)
    {
        _precalculationTrajectories.Clear();
        
        Vector3 initialPosition = ballValues.position;
        (ballValues.velocity, ballValues.angularVelocity) = FindInitialVelocity(initialPosition, speed, spin, 0, 45);
        ballValues.state = BallState.Bouncing;
        
        AddTrajectoryFrame();
    }
    
    public void DoubleTargetedKick(float speed)
    {
        _precalculationTrajectories.Clear();
        Vector3 initialPosition = ballValues.position;
        
        //TODO barrier shot deve essere passato in input alla funzione
        barrierShot = true;
        (ballValues.velocity, ballValues.angularVelocity) = FindInitialVelocityWithConstraint(initialPosition, speed);
        ballValues.state = BallState.Bouncing;
        barrierShot = false;

        AddTrajectoryFrame();
    }
   
    void Awake()
    {
        Application.targetFrameRate = -1;
        
        Time.fixedDeltaTime = 1/frameRate;
        
        _frames = new Queue<Frame>();

        Reset();
        
        if (targetKickTesting)
        {
            saveSystem = new SaveSystem();
            startPositions = new List<Vector3>();
            distances = new List<float>();
            errors = new List<float>();
            outliersStartPositions = new List<Vector3>();
            outliersError = new List<float>();
            precompute = false;

            if (barrierShot)
            {
                testInitialSpeed = minInitialSpeed;
            }
        }
        
    }

    void Update()
    {
        if (targetKickTesting)
        {
            if(!precompute)
            {
                _precalculationTrajectories.Clear();
                precompute = true;

                if (barrierShot)
                {
                    testInitialSpeed += (maxInitialSpeed - minInitialSpeed) / testSteps;
                    (ballValues.velocity, ballValues.angularVelocity) = FindInitialVelocityWithConstraint(ballValues.position, testInitialSpeed);
                }
                else
                {
                    Vector3 stepDirection = (testEndPosition - testStartPosition) / testSteps;
                    _testCurrentPosition = testStartPosition + stepDirection * _currentStep;
                    transform.position = _testCurrentPosition;
                    (ballValues.velocity, ballValues.angularVelocity) = FindInitialVelocity(_testCurrentPosition, testInitialSpeed, testInitialSpin,0, 45f);
                }
                precompute = false;

                _currentStep++;

                if (_currentStep > testSteps)
                {
                    targetKickTesting = false;
                    if(barrierShot)
                    {
                        saveSystem.SaveToJson(fileName, description, startPositions, distances, errors, outliersStartPositions, outliersError, averageComputationTime, iterations, computationTime, constraintErrors, initialSpeeds, zSpin, ySpin);
                    }
                    else
                    {
                        saveSystem.SaveToJson(fileName, description, startPositions, distances, errors, outliersStartPositions, outliersError, averageComputationTime, iterations, computationTime);
                    }
                }
                
                ballValues.position = transform.position;
                ballValues.rotation = Quaternion.identity;
                ballValues.state = BallState.Bouncing;
            }
        }
        
        
    }


    void FixedUpdate()
    {
        //handles frame by frame mode
        if (frameByFrame)
        {
            if (!interpolateFrames)
            {
                if (frameCount < _frames.Count && _frames.Count > 0)
                {
                    ballValues.position = frameArray[frameCount].position;
                    ballValues.rotation = frameArray[frameCount].rotation;
                }
            }
            else
            {
                int currentIndex = frameCount;
                int nextIndex = frameCount + 1;
                
                float t = Mathf.Clamp01(continuousFrameCount - currentIndex);

                if (nextIndex > frameArray.Length - 1)
                {
                    nextIndex = currentIndex;
                }

                Vector3 interpolatedPos = Vector3.Lerp(frameArray[currentIndex].position, frameArray[nextIndex].position, t);
                Quaternion interpolatedRot = Quaternion.Slerp(frameArray[currentIndex].rotation, frameArray[nextIndex].rotation, t);

                ballValues.position = interpolatedPos;
                ballValues.rotation = interpolatedRot;
            }

            return;
        }
        
        if (targetKickTesting || ballValues.state == BallState.Stopped)
            return;
        
        //TODO change name of function
        CheckIsMoving();
         
        ComputeFrame(Time.fixedDeltaTime, ballValues);

        AddTrajectoryFrame();
    }

    public void ComputeFrame(float deltaTime, Ball ball)
    {
        float startTime = Time.realtimeSinceStartup;
        
        (Vector3 newPosition, Quaternion newRotation, Vector3 newVelocity, Vector3 newAngularVelocity) = IntegrateMotion(deltaTime, ball.position, ball.rotation, ball.velocity, ball.angularVelocity, ball.state);
        
        //TODO spiegare il nome meglio
        if (newPosition.y - ballValues.radius <= 0f && ball.state == BallState.Bouncing && newVelocity.y < 0)
        {
            (newPosition, newRotation, newVelocity, newAngularVelocity, ball.state) = CheckCollision(deltaTime, ball.position, ball.rotation, ball.velocity, ball.angularVelocity, ball.state);
        }

        ball.state = CheckPureRolling(ball.state, _frictionForce, _previousSignOfFriction);
        
        
        float endTime = Time.realtimeSinceStartup;
        _timeToCompute = endTime - startTime;
        _totalTimeToCompute += _timeToCompute;

        ball.position = newPosition;
        ball.rotation = newRotation;
        ball.velocity = newVelocity;
        ball.angularVelocity = newAngularVelocity;
    }

    private void AddTrajectoryFrame()
    {
        if (ballValues.state != BallState.Stopped && !targetKickTesting)
        {
            Frame frame = new Frame(transform.position, transform.rotation, ballValues.position, ballValues.velocity, CalculateTotalForces(ballValues.velocity, ballValues.angularVelocity, ballValues.state)/ballValues.mass, ballValues.angularVelocity, ballValues.CalculatePotentialEnergy(),
                ballValues.CalculateKineticEnergy(), ballValues.CalculateRotationalEnergy(), ballValues.CalculateTotalEnergy(), CalculateDragForce(ballValues.velocity, ballValues.angularVelocity), CalculateMagnusForce(ballValues.velocity, ballValues.angularVelocity), CalculateNormalForce(ballValues.velocity, ballValues.angularVelocity, ballValues.state),
                CalculateFrictionForce(ballValues.velocity, ballValues.angularVelocity, ballValues.state), CalculateTotalForces(ballValues.velocity, ballValues.angularVelocity, ballValues.state), CalculateTotalTorque(ballValues.velocity, ballValues.angularVelocity, ballValues.state), CalculateTotalTorque(ballValues.velocity, ballValues.angularVelocity, ballValues.state), _timeToCompute);
            
            if (_frames.Count >= 1000)
            {
                _frames.Dequeue(); 
            }

            _frames.Enqueue(frame);
            frameArray = _frames.ToArray();
            
            GetComponent<TrajectoryTracker>().AddPoint(transform.position, ballValues.state == BallState.Rolling ? Color.green : Color.red);
        }
    }

    private (Vector3, Quaternion, Vector3, Vector3) IntegrateMotion(float deltaTime, Vector3 initialPosition, Quaternion initialRotation, Vector3 initialVelocity, Vector3 initialAngularVelocity, BallState state)
    {
        switch (integrationMethod)
        {
            case IntegrationMethod.VelocityVerlet:
                return VelocityVerlet(deltaTime, initialPosition, initialRotation, initialVelocity,initialAngularVelocity, state);
            case IntegrationMethod.ExplicitEuler:
                return ExplicitEuler(deltaTime, initialPosition, initialRotation, initialVelocity,initialAngularVelocity, state);
            case IntegrationMethod.RK2:
                return Runge_Kutta_2(deltaTime, initialPosition, initialRotation, initialVelocity,initialAngularVelocity, state);
            case IntegrationMethod.RK4:
                return Runge_Kutta_4(deltaTime, initialPosition, initialRotation, initialVelocity,initialAngularVelocity, state);
        }
        
        return (Vector3.zero, Quaternion.identity, Vector3.zero, Vector3.zero);
    }
    
     private (Vector3, Quaternion, Vector3, Vector3) VelocityVerlet(float deltaTime,Vector3 initialPosition, Quaternion initialRotation, Vector3 initialVelocity, Vector3 initialAngularVelocity, BallState state) 
     {
         //F_k
         Vector3 acceleration = CalculateTotalForces(initialVelocity, initialAngularVelocity, state) / ballValues.mass;
         Vector3 angularAcceleration = CalculateTotalTorque(initialVelocity, initialAngularVelocity, state) / ballValues.inertialMomentum;

         //v_k+1/2
         Vector3 newVelocity = initialVelocity + deltaTime/2 * acceleration;
         Vector3 newAngularVelocity = initialAngularVelocity + deltaTime/2 * angularAcceleration;
         
         if(state == BallState.Rolling)
         {
             float y = newAngularVelocity.y;
             newAngularVelocity = -Vector3.Cross(newVelocity, Vector3.up) / ballValues.radius;
             newAngularVelocity.y = y;
         }

         //p_k+1
         Vector3 newPosition = initialPosition + deltaTime * newVelocity;
         Quaternion newRotation = Quaternion.Euler(Mathf.Rad2Deg * deltaTime * newAngularVelocity) * initialRotation;

         //F_k+1
         acceleration = CalculateTotalForces(newVelocity, newAngularVelocity, state) / ballValues.mass;
         angularAcceleration = CalculateTotalTorque(newVelocity, newAngularVelocity, state) / ballValues.inertialMomentum;

         //v_k+1
         newVelocity += deltaTime/2 * acceleration;
         newAngularVelocity += deltaTime/2 * angularAcceleration;

         if (state != BallState.Bouncing && state != BallState.Stopped)
         {
             newPosition.y = ballValues.radius;

             if (state == BallState.Rolling)
             {
                 float y = newAngularVelocity.y;
                 newAngularVelocity = -Vector3.Cross(newVelocity, Vector3.up) / ballValues.radius;
                 newAngularVelocity.y = y;
             }
         }

         return (newPosition, newRotation, newVelocity, newAngularVelocity);
     }
    
     private (Vector3, Quaternion, Vector3, Vector3) ExplicitEuler(float deltaTime,Vector3 initialPosition, Quaternion initialRotation, Vector3 initialVelocity, Vector3 initialAngularVelocity, BallState state)
    {
        //F_k
        Vector3 acceleration = CalculateTotalForces(initialVelocity, initialAngularVelocity, state) / ballValues.mass;
        Vector3 angularAcceleration = CalculateTotalTorque(initialVelocity, initialAngularVelocity, state) / ballValues.inertialMomentum;
        
        //v_k+1
        Vector3 newVelocity = initialVelocity + deltaTime * acceleration;
        Vector3 newAngularVelocity = initialAngularVelocity + deltaTime * angularAcceleration;
        
        //p_k+1
        Vector3 newPosition = initialPosition + deltaTime * newVelocity;
        Quaternion newRotation = Quaternion.Euler(Mathf.Rad2Deg * deltaTime * newAngularVelocity) * initialRotation;

        if (state != BallState.Bouncing && state != BallState.Stopped)
        {
            newPosition.y = ballValues.radius;

            if (state == BallState.Rolling)
            {
                float y = newAngularVelocity.y;
                newAngularVelocity = -Vector3.Cross(newVelocity, Vector3.up) / ballValues.radius;
                newAngularVelocity.y = y;
            }
        }

        return (newPosition, newRotation, newVelocity, newAngularVelocity);
    }
    
    private (Vector3, Quaternion, Vector3, Vector3) Runge_Kutta_2(float deltaTime,Vector3 initialPosition, Quaternion initialRotation, Vector3 initialVelocity, Vector3 initialAngularVelocity, BallState state)
    {
        //k1
        Vector3 acceleration = CalculateTotalForces(initialVelocity, initialAngularVelocity, state) / ballValues.mass;
        Vector3 k1V = deltaTime * acceleration;
        Vector3 k1P = deltaTime * initialVelocity;
        
        Vector3 angularAcceleration =  CalculateTotalTorque(initialVelocity, initialAngularVelocity, state) / ballValues.inertialMomentum;
        Vector3 k1W = deltaTime * angularAcceleration;
        Vector3 k1T = deltaTime * initialAngularVelocity;
        
        //k2
        acceleration = CalculateTotalForces(initialVelocity + k1V, initialAngularVelocity + k1W, state) / ballValues.mass;
        Vector3 k2V = deltaTime * acceleration;
        Vector3 k2P = deltaTime * (initialVelocity + k1V);
        
        angularAcceleration = CalculateTotalTorque(initialVelocity + k1V, initialAngularVelocity + k1W, state) / ballValues.inertialMomentum;
        Vector3 k2W = deltaTime * angularAcceleration;
        Vector3 k2T = deltaTime * (initialAngularVelocity + k1W);
        
        Vector3 newPosition = initialPosition + (k1P + k2P) / 2;
        Vector3 newVelocity = initialVelocity + (k1V + k2V) / 2;
        
        Vector3 newAngularVelocity = initialAngularVelocity + (k1W + k2W) / 2;
        Quaternion newRotation = Quaternion.Euler(Mathf.Rad2Deg * (k1T + k2T) / 2) * initialRotation;

        if (state != BallState.Bouncing && state != BallState.Stopped)
        {
            newPosition.y = ballValues.radius;

            if (state == BallState.Rolling)
            {
                float y = newAngularVelocity.y;
                newAngularVelocity = -Vector3.Cross(newVelocity, Vector3.up) / ballValues.radius;
                newAngularVelocity.y = y;
                newRotation = Quaternion.Euler(Mathf.Rad2Deg * deltaTime * newAngularVelocity) * initialRotation;
            }
        }

        return (newPosition, newRotation, newVelocity, newAngularVelocity);
    }

    private (Vector3, Quaternion, Vector3, Vector3) Runge_Kutta_4(float deltaTime,Vector3 initialPosition, Quaternion initialRotation, Vector3 initialVelocity, Vector3 initialAngularVelocity, BallState state)
    {
        //k1
        Vector3 acceleration = CalculateTotalForces(initialVelocity, initialAngularVelocity, state) / ballValues.mass;
        Vector3 k1V = deltaTime * acceleration;
        Vector3 k1P = deltaTime * initialVelocity;
        
        Vector3 angularAcceleration = CalculateTotalTorque(initialVelocity, initialAngularVelocity, state) / ballValues.inertialMomentum;
        Vector3 k1W = deltaTime * angularAcceleration;
        Vector3 k1T = deltaTime * initialAngularVelocity;
        
        //k2
        
        acceleration = CalculateTotalForces(initialVelocity + k1V / 2, initialAngularVelocity + k1W / 2, state) / ballValues.mass;
        Vector3 k2V = deltaTime * acceleration;
        Vector3 k2P = deltaTime * (initialVelocity + k1V/2);
        
        angularAcceleration = CalculateTotalTorque(initialVelocity + k1V/2, initialAngularVelocity + k1W/2, state) / ballValues.inertialMomentum;
        Vector3 k2W = deltaTime * angularAcceleration;
        Vector3 k2T = deltaTime * (initialAngularVelocity + k1W/2);
        
        //k3
        acceleration = CalculateTotalForces(initialVelocity + k2V / 2, initialAngularVelocity + k2W / 2, state) / ballValues.mass;
        Vector3 k3V = deltaTime * acceleration;
        Vector3 k3P = deltaTime * (initialVelocity + k2V/2);
        
        angularAcceleration = CalculateTotalTorque(initialVelocity + k2V/2, initialAngularVelocity + k2W/2, state) / ballValues.inertialMomentum;
        Vector3 k3W = deltaTime * angularAcceleration;
        Vector3 k3T = deltaTime * (initialAngularVelocity + k2W/2);
        
        //k4
        acceleration = CalculateTotalForces(initialVelocity + k3V, initialAngularVelocity + k3W, state) / ballValues.mass;
        Vector3 k4V = deltaTime * acceleration;
        Vector3 k4P = deltaTime * (initialVelocity + k3V);
        
        angularAcceleration = CalculateTotalTorque(initialVelocity + k3V, initialAngularVelocity + k3W, state) / ballValues.inertialMomentum;
        Vector3 k4W = deltaTime * angularAcceleration;
        Vector3 k4T = deltaTime * (initialAngularVelocity + k3W);
        
        Vector3 newPosition = initialPosition + (k1P + 2*k2P + 2*k3P + k4P) / 6;
        Vector3 newVelocity = initialVelocity + (k1V + 2*k2V + 2*k3V + k4V) / 6;
        
        Vector3 newAngularVelocity = initialAngularVelocity + (k1W + 2*k2W + 2*k3W + k4W) / 6;
        Quaternion newRotation = Quaternion.Euler(Mathf.Rad2Deg * (k1T + 2*k2T + 2*k3T + k4T) / 6) * initialRotation;
        
        if(state != BallState.Bouncing && state != BallState.Stopped)
        {
            newPosition.y = ballValues.radius;
            
            if (state == BallState.Rolling)
            {
                float y = newAngularVelocity.y;
                newAngularVelocity = -Vector3.Cross(newVelocity, Vector3.up) / ballValues.radius;
                newAngularVelocity.y = y;
                newRotation = Quaternion.Euler(Mathf.Rad2Deg * deltaTime * newAngularVelocity) * initialRotation;
            }
        }
        
        return (newPosition, newRotation, newVelocity, newAngularVelocity);
    }
    
    private (Vector3, Quaternion, Vector3, Vector3, BallState) CheckCollision(float deltaTime, Vector3 initialPosition, Quaternion initialRotation, Vector3 initialVelocity, Vector3 initialAngularVelocity, BallState state)
    {

        //TODO spiegare che cosa fa
        (Vector3 positionAfterCollision, Quaternion rotationAfterCollision, Vector3 velocityAfterCollision, Vector3 angularVelocityAfterCollision) = ContinuousCollisionDetection(deltaTime / 2, deltaTime / 2,initialPosition, initialRotation, initialVelocity, initialAngularVelocity, state);
        
        if (velocityAfterCollision.y < 1e-1)
        {
            velocityAfterCollision.y = 0f;
            positionAfterCollision.y = ballValues.radius;
            state = BallState.Sliding;
        }

        return (positionAfterCollision, rotationAfterCollision, velocityAfterCollision, angularVelocityAfterCollision, state);
    }

    private BallState CheckPureRolling(BallState state, Vector3 friction, Vector3 previousFriction)
    {
        //TODO spiegare perchè utilizzo questa euristica
        if(state == BallState.Sliding && Vector3.Dot(friction, previousFriction) < 0)
        {
            state = BallState.Rolling;
        }

        return state;
    }
    
    private void CheckIsMoving()
    {
        if (ballValues.CalculateTotalEnergy() < 1e-4)
        {
            ballValues.ResetValues();
        }
    }
    
    private (Vector3, Quaternion, Vector3, Vector3) ContinuousCollisionDetection(float deltaTime, float step, Vector3 initialPosition, Quaternion initialRotation, Vector3 initialVelocity, Vector3 initialAngularVelocity, BallState state)
    {
        (Vector3 position, _, _, _) = IntegrateMotion(deltaTime, initialPosition, initialRotation, initialVelocity, initialAngularVelocity, state);
        
        if (position.y - ballValues.radius <= 0f)
        {
            //collision
            if (step < CollisionDetectionError)
            {
                float collisionDeltaTime = deltaTime - step / 2;

                return CalculateBounce(collisionDeltaTime, initialPosition, initialRotation, initialVelocity, initialAngularVelocity, state);
            }
            
            return ContinuousCollisionDetection(deltaTime - step/2, step/2, initialPosition, initialRotation, initialVelocity, initialAngularVelocity, state);
        }
        else
        {
            //no collision
            if (step < CollisionDetectionError)
            {
                float collisionDeltaTime = deltaTime + step / 2;

                return CalculateBounce(collisionDeltaTime, initialPosition, initialRotation, initialVelocity, initialAngularVelocity, state);
            }

            return ContinuousCollisionDetection(deltaTime + step/2, step/2, initialPosition, initialRotation, initialVelocity, initialAngularVelocity, state);
        }
    }

    private (Vector3, Quaternion, Vector3, Vector3) CalculateBounce(float bounceDeltaTime, Vector3 initialPosition, Quaternion initialRotation, Vector3 initialVelocity, Vector3 initialAngularVelocity, BallState state)
    {
        //this first part calculates the new velocity and angular velocity immediately after the bounce
        (Vector3 collisionPoint, Quaternion collisionRotation, Vector3 velocityBeforeCollision, Vector3 angularVelocityBeforeCollision) = IntegrateMotion(bounceDeltaTime, initialPosition, initialRotation, initialVelocity, initialAngularVelocity, state);
        
        //TODO spiegare funzionamento del rimbalzo

        Vector3 velocityAfterCollision = new Vector3();
        Vector3 angularVelocityAfterCollision = new Vector3();
        
        float alpha = 2f / 3f;
        velocityAfterCollision.x = velocityBeforeCollision.x * (1 - alpha * ballValues.coefficientOfHorizontalRestitution)  / (alpha + 1) -
                                   alpha * (1 + ballValues.coefficientOfHorizontalRestitution) / (alpha + 1) * ballValues.radius * angularVelocityBeforeCollision.z;

        velocityAfterCollision.y = -ballValues.coefficientOfVerticalRestitution * velocityBeforeCollision.y;
        velocityAfterCollision.z = velocityBeforeCollision.z * (1 - alpha * ballValues.coefficientOfHorizontalRestitution) / (alpha + 1) +
                 alpha * (1 + ballValues.coefficientOfHorizontalRestitution) / (alpha + 1) * ballValues.radius * angularVelocityBeforeCollision.x;

        angularVelocityAfterCollision.x = (alpha-ballValues.coefficientOfHorizontalRestitution)/(alpha+1) * angularVelocityBeforeCollision.x + (ballValues.coefficientOfHorizontalRestitution+1)/(alpha+1) * velocityBeforeCollision.z/ballValues.radius;
        angularVelocityAfterCollision.y = angularVelocityBeforeCollision.y * ballValues.coefficientRotationRestitutionYaxis;
        angularVelocityAfterCollision.z = (alpha-ballValues.coefficientOfHorizontalRestitution)/(alpha+1) * angularVelocityBeforeCollision.z - (ballValues.coefficientOfHorizontalRestitution+1)/(alpha+1) * velocityBeforeCollision.x/ballValues.radius;
        
        if(!precompute && !targetKickTesting)
            GetComponent<TrajectoryTracker>().AddPoint(collisionPoint, Color.blue);
        
        //this part calculates the trajectory of the ball starting from the moment when it touched the ground to the end of the frame
        float remainingTime = Time.fixedDeltaTime - bounceDeltaTime;
        
        return IntegrateMotion(remainingTime, collisionPoint, collisionRotation, velocityAfterCollision, angularVelocityAfterCollision, state);
    }
    
     private (Vector3, Vector3) FindInitialVelocity(Vector3 initialPosition, float speed, Vector3 spin, float minVerticalAngle, float maxVerticalAngle)
     { 
        float startTime = Time.realtimeSinceStartup;
        
        Vector3 direction = new Vector3(target.transform.position.x - initialPosition.x, 0, target.transform.position.z - initialPosition.z).normalized;
        //TODO espendadere con variabili piu capibili
        direction = new Vector3(direction.x * Mathf.Cos((maxVerticalAngle - minVerticalAngle) / 2 * Mathf.Deg2Rad), Mathf.Sin((maxVerticalAngle - minVerticalAngle) / 2 * Mathf.Deg2Rad), direction.z * Mathf.Cos((maxVerticalAngle - minVerticalAngle) / 2 * Mathf.Deg2Rad));
        Vector3 velocity = direction * speed;
        Vector3 rotatedSpin = RotateSpinTowardVelocityDirection(velocity, spin);

        //TODO count iterazioni
        int count = 0;
        float error = float.MaxValue;
        Quaternion rotation = Quaternion.identity;

        //TODO spiegare cosa sono
        float angle0 = 180f;
        float angle1 = 90f;
        
        float distance = Vector3.Distance(target.transform.position, initialPosition);
        
        //TODO SPIEGARE PERCHE 25 
        //max error depends on the distance from the target
        float maxError = distance < 25 ? precalculationMaxError : precalculationMaxError + Mathf.Pow(distance - 25,2) * 1e-4f;
        
        //TODO utilizzare un random con SEED
        System.Random random = new System.Random();
        
        while (error > maxError && count < maxIterations)
        {
            count++;
            
            _currentTrajectory = new List<Vector3>();
            
            Vector3 rotatedVelocity = rotation * velocity;
                
            //rotate the spin to be in the direction of the current iteration of the velocity
            rotatedSpin = RotateSpinTowardVelocityDirection(rotatedVelocity, spin);
            
            (Vector3 position,_) = PrecomputeTrajectory(initialPosition, Quaternion.identity, rotatedVelocity, rotatedSpin);
            placeholder.transform.position = position;
            
            //check angle over ground plane is less than maxVerticalAngle degrees, otherwise stops calculation
            float dot = Vector3.Dot(rotatedVelocity.normalized, Vector3.up);
            float totalAngle = 90f - Mathf.Acos(dot) * Mathf.Rad2Deg;
            
            if (totalAngle >= maxVerticalAngle || totalAngle <= minVerticalAngle)
            {
                _currentTrajectory = new List<Vector3>();
                
                float velocityMagnitude = rotatedVelocity.magnitude;
                
                Vector2 xzProjection = new Vector2(rotatedVelocity.x, rotatedVelocity.z);
                float xzMagnitude = xzProjection.magnitude;

                float newY;
                if(totalAngle >= maxVerticalAngle)
                    newY = Mathf.Tan(maxVerticalAngle * Mathf.Deg2Rad) * xzMagnitude;
                else
                    newY = Mathf.Tan(minVerticalAngle) * xzMagnitude;
                
                velocity = new Vector3(rotatedVelocity.x, newY, rotatedVelocity.z).normalized * velocityMagnitude;
                
                rotatedSpin = RotateSpinTowardVelocityDirection(rotatedVelocity, spin);

                (position,_) = PrecomputeTrajectory(initialPosition, Quaternion.identity, velocity, rotatedSpin);
                
                error = Vector3.Distance(position, target.transform.position);

                if(!barrierShot)
                    _precalculationTrajectories.Add(_currentTrajectory);
                
                //TODO spiegare il break
                placeholder.transform.position = position;
                break;
            }
            else
            {
                velocity = rotatedVelocity;

                error = Vector3.Distance(position, target.transform.position);

                Vector3 vTarget = (target.transform.position - initialPosition).normalized;
                Vector3 vIteration = (position - initialPosition).normalized;
                float cosTheta = Vector3.Dot(vIteration, vTarget);

                Vector3 rotationAxis = Vector3.Cross(vIteration, vTarget).normalized;

                float angle = Mathf.Acos(Mathf.Clamp(cosTheta, -1f, 1f)) * Mathf.Rad2Deg;

                //adjust slightly the speed if one of the 2 cases happen, otherwise the predictor would keep calculating without improving the shot until maximum iterations are reached
                if ((angle < 1e-3 || Mathf.Abs(angle0 - angle) < 1e-3) && error > maxError && count < maxIterations)
                {
                    velocity *= random.Next(2) == 0 ? 1.0005f : 0.9995f;
                }

                angle0 = angle1;
                angle1 = angle;

                rotation = Quaternion.AngleAxis(angle, rotationAxis);

                if (!barrierShot)
                    _precalculationTrajectories.Add(_currentTrajectory);
            }
        }
        
        float endTime = Time.realtimeSinceStartup;
        
        if(!barrierShot)
        {
            Debug.Log("Prediction duration: " + ((endTime - startTime) * 1e3).ToString("F3") + " ms with " + count +
                      " iterations");
            Debug.Log("Error: " + (error * 1e3).ToString("F3") + " mm");

            if (targetKickTesting)
            {
                _totalComputationTime += (endTime - startTime) * 1e3f;
                errors.Add(error * 1e3f);
                startPositions.Add(_testCurrentPosition);
                distances.Add(Vector3.Distance(_testCurrentPosition, target.transform.position));
                iterations.Add(count);
                computationTime.Add((endTime - startTime) * 1e3f);
                averageComputationTime = _totalComputationTime / (errors.Count);

                if (error * 1e3f > 10f)
                {
                    outliersStartPositions.Add(_testCurrentPosition);
                    outliersError.Add(error * 1e3f);
                }
            }
        }

        return (velocity, rotatedSpin);
     }

     public GameObject noVerticalSpinBallPlaceholder;
     public GameObject noHorizontalSpinBallPlaceholder;
     
private (Vector3, Vector3)  FindInitialVelocityWithConstraint(Vector3 initialPosition, float speed)
    { 
        float startTime = Time.realtimeSinceStartup;
        int count = 1;
        float minVerticalAngle = 0f;
        float maxVerticalAngle = 30f;

        //finds initial velocity to reach the target with no spin and without considering that the ball has to pass above the barrier
        Vector3 direction = new Vector3(target.transform.position.x - initialPosition.x, 0, target.transform.position.z - initialPosition.z).normalized;
        direction = new Vector3(direction.x * Mathf.Cos((maxVerticalAngle - minVerticalAngle) / 2 * Mathf.Deg2Rad), Mathf.Sin((maxVerticalAngle - minVerticalAngle) / 2 * Mathf.Deg2Rad), direction.z * Mathf.Cos((maxVerticalAngle - minVerticalAngle) / 2 * Mathf.Deg2Rad));
        Vector3 velocity = direction * speed;
        _currentTrajectory = new List<Vector3>();
        (_, Vector3 predictedConstraintPosition) = PrecomputeTrajectory(initialPosition, Quaternion.identity, velocity, Vector3.zero);
        
        //rotate velocity towards constraint and then calculate the new trajectory with the new direction
        velocity = RotateVelocityTowardConstraint(initialPosition, predictedConstraintPosition, constraint.transform.position, velocity, maxVerticalAngle, speed);
        (Vector3 noSpinPredictedTargetPosition,_) = PrecomputeTrajectory(initialPosition, Quaternion.identity, velocity, Vector3.zero);
        
        //finds displacement of the previous shot from the target
        Vector3 targetDisplacement = (noSpinPredictedTargetPosition - target.transform.position ).normalized;
        
        //choose the direction of the verticalspin based on the displacement of the previous shot and rotates it towards the direction of the velocity
        Vector3 spin = new Vector3(0, Math.Sign(targetDisplacement.z), -Math.Sign(targetDisplacement.y)) * maximumSpinValue;
        Vector3 rotatedSpin = RotateSpinTowardVelocityDirection(velocity, spin);
        
        _currentTrajectory = new List<Vector3>();
        (Vector3 adjustedSpinPredictedTargetPosition,Vector3 adjustSpinPredictedConstraintPosition) = PrecomputeTrajectory(initialPosition, Quaternion.identity, velocity, rotatedSpin);
        _precalculationTrajectories.Add(_currentTrajectory);
        
        //finds distance from target and from constraint of the trajectory
        float targetError = (adjustedSpinPredictedTargetPosition - target.transform.position).magnitude;
        float constrainError = (adjustSpinPredictedConstraintPosition - constraint.transform.position).magnitude;
        
        float horizontalSpinMagnitude = maximumSpinValue;
        float verticalSpinMagnitude = maximumSpinValue;

        float previousTargetError = float.MaxValue;
        bool stopped = false;
        
        //using this i can have ball passing even over the target(it is ok to pass above the barrier) but aims to the target
        //if(spinMagnitude < maximumSpinValue && adjustSpinPredictedConstraintPosition.y - constraint.transform.position.y < 0)
        { 
            while (count < maxIterations && (targetError > 1e-2 || constrainError > 1e-1))
            {
                _currentTrajectory = new List<Vector3>();
                
                //computes trajectory having spin without horizontal component
                Vector3 noHorizontalSpin = rotatedSpin;
                noHorizontalSpin.y = 0f;
                (Vector3 noHorizontalSpinPredictedTargetPosition,_) = PrecomputeTrajectory(initialPosition, Quaternion.identity, velocity, noHorizontalSpin);
                targetDisplacement = (noHorizontalSpinPredictedTargetPosition - target.transform.position ).normalized;
                
                //finds the new horizontal spin proportionally to the distances of the shot with the previously found spin and the desire shot from the shot with no horizontal spin
                float d1 = Mathf.Abs(adjustedSpinPredictedTargetPosition.z - noHorizontalSpinPredictedTargetPosition.z);
                float d2 = Mathf.Abs(target.transform.position.z - noHorizontalSpinPredictedTargetPosition.z); 
                horizontalSpinMagnitude = Mathf.Min(horizontalSpinMagnitude * d2 / d1, maximumSpinValue);
                spin.y = Math.Sign(targetDisplacement.z) * horizontalSpinMagnitude;
                rotatedSpin.y = spin.y;
                
                //computes trajectory with the new found horizontal spin and rotates velocity towards the constraint
                (_, predictedConstraintPosition) = PrecomputeTrajectory(initialPosition, Quaternion.identity, velocity, rotatedSpin);
                velocity = RotateVelocityTowardConstraint(initialPosition, predictedConstraintPosition,constraint.transform.position, velocity, maxVerticalAngle, speed);
                rotatedSpin = RotateSpinTowardVelocityDirection(velocity, spin);

                //computes new predicted position to find new predicted position of the target
                (adjustedSpinPredictedTargetPosition,_) = PrecomputeTrajectory(initialPosition, Quaternion.identity, velocity, rotatedSpin);
                
                //computes trajectory having spin without vertical component
                Vector3 noVerticalSpin = spin;
                noVerticalSpin.z = 0f;
                (Vector3 noVerticalSpinPredictedTargetPosition,_) = PrecomputeTrajectory(initialPosition, Quaternion.identity, velocity, noVerticalSpin);
                targetDisplacement = (noVerticalSpinPredictedTargetPosition - target.transform.position ).normalized;

                //finds the new vertical spin proportionally to the distances of the shot with the previously found spin and the desire shot from the shot with no vertical spin
                d1 = adjustedSpinPredictedTargetPosition.y - noVerticalSpinPredictedTargetPosition.y;
                d2 = target.transform.position.y - noVerticalSpinPredictedTargetPosition.y;
                verticalSpinMagnitude = Mathf.Min(verticalSpinMagnitude * Math.Abs(d2 / d1), maximumSpinValue);
                spin.z = -Math.Sign(targetDisplacement.y) * verticalSpinMagnitude;
                rotatedSpin = RotateSpinTowardVelocityDirection(velocity, spin);
                
                //computes trajectory with the new found vertical spin and rotates velocity towards the constraint
                (_, predictedConstraintPosition) = PrecomputeTrajectory(initialPosition, Quaternion.identity, velocity, rotatedSpin);
                velocity = RotateVelocityTowardConstraint(initialPosition, predictedConstraintPosition,constraint.transform.position, velocity, maxVerticalAngle, speed);
                rotatedSpin = RotateSpinTowardVelocityDirection(velocity, spin);
                
                //computes new predicted position to find new predicted position of the target
                _currentTrajectory = new List<Vector3>();
                (adjustedSpinPredictedTargetPosition,adjustSpinPredictedConstraintPosition) = PrecomputeTrajectory(initialPosition, Quaternion.identity, velocity, rotatedSpin);
                _precalculationTrajectories.Add(_currentTrajectory);
                
                //finds distance from target and from constraint of the trajectory
                targetError = (adjustedSpinPredictedTargetPosition - target.transform.position).magnitude;
                constrainError = (adjustSpinPredictedConstraintPosition - constraint.transform.position).magnitude;
                
                if (targetError > 3 || Mathf.Abs(targetError - previousTargetError) < 1e-4 && (Mathf.Abs(spin.y) - maximumSpinValue < 1e-6 || Mathf.Abs(spin.z) - maximumSpinValue < 1e-6))
                {
                    stopped = true;
                    break;
                }
                
                previousTargetError = targetError;
                
                count++;
                
                //DEBUG
                //noVerticalSpinBallPlaceholder.transform.position = noVerticalSpinPredictedTargetPosition;
                //noHorizontalSpinBallPlaceholder.transform.position = noHorizontalSpinPredictedTargetPosition;
            }
        }
        
        _currentTrajectory = new List<Vector3>();
        (adjustedSpinPredictedTargetPosition,adjustSpinPredictedConstraintPosition) = PrecomputeTrajectory(initialPosition, Quaternion.identity, velocity, rotatedSpin);
        placeholder.transform.position = adjustedSpinPredictedTargetPosition;
        constraintPlaceholder.transform.position = adjustSpinPredictedConstraintPosition;
        _precalculationTrajectories.Add(_currentTrajectory);
        
        //finds distance from target and from constraint of the trajectory
        targetError = (adjustedSpinPredictedTargetPosition - target.transform.position).magnitude;
        constrainError = (adjustSpinPredictedConstraintPosition - constraint.transform.position).magnitude;

        if (stopped)
        {
            (velocity, rotatedSpin) = FindInitialVelocity(initialPosition, speed, spin, minVerticalAngle, maxVerticalAngle);
            
            _currentTrajectory = new List<Vector3>();
            (adjustedSpinPredictedTargetPosition,adjustSpinPredictedConstraintPosition) = PrecomputeTrajectory(initialPosition, Quaternion.identity, velocity, rotatedSpin);
            placeholder.transform.position = adjustedSpinPredictedTargetPosition;
            constraintPlaceholder.transform.position = adjustSpinPredictedConstraintPosition;
            _precalculationTrajectories.Add(_currentTrajectory);
            
            //finds distance from target and from constraint of the trajectory
            targetError = (adjustedSpinPredictedTargetPosition - target.transform.position).magnitude;
            constrainError = (adjustSpinPredictedConstraintPosition - constraint.transform.position).magnitude;

            count++;
        }
        
        float endTime = Time.realtimeSinceStartup;
        
        Debug.Log("Constrained kick prediction duration: " + ((endTime - startTime)*1e3).ToString("F3") + " ms with " + count + " iterations");
        Debug.Log("Constraint error: " + (constrainError*1e3f).ToString("F3") + " mm");
        Debug.Log("Target error: " + (targetError*1e3f).ToString("F3") + " mm");
        
        if (targetKickTesting)
        {
            _totalComputationTime += (endTime - startTime) * 1e3f;
            errors.Add(targetError*1e3f);
            startPositions.Add(_testCurrentPosition);
            distances.Add(Vector3.Distance(_testCurrentPosition, target.transform.position));
            iterations.Add(count);
            initialSpeeds.Add(speed);
            constraintErrors.Add(Mathf.Abs(constrainError*1e3f));
            zSpin.Add(spin.z);
            ySpin.Add(spin.y);
            computationTime.Add((endTime - startTime) * 1e3f);
            averageComputationTime = _totalComputationTime / (errors.Count);

            if(targetError*1e3f > 10f)
            {
                outliersStartPositions.Add(_testCurrentPosition);
                outliersError.Add(targetError*1e3f);
            }
        }
        
        return (velocity,rotatedSpin);
    }

    private Vector3 RotateVelocityTowardConstraint(Vector3 initialPosition, Vector3 predictedConstraintPosition, Vector3 constraintPosition, Vector3 velocity, float maxVerticalAngle, float speed)
    {
        Vector3 vIteration = (predictedConstraintPosition - initialPosition).normalized;
        Vector3 vTarget = (constraintPosition - initialPosition).normalized;
        Vector3 direction = velocity.normalized;        
        Quaternion rotation = Quaternion.FromToRotation(vIteration, vTarget);
        direction = rotation * direction;
        float dot = Vector3.Dot(direction.normalized, Vector3.up);
        float verticalAngle = 90f - Mathf.Acos(dot) * Mathf.Rad2Deg;
        if(verticalAngle > maxVerticalAngle)
            direction = new Vector3(direction.x * Mathf.Cos(maxVerticalAngle * Mathf.Deg2Rad), Mathf.Sin(maxVerticalAngle * Mathf.Deg2Rad), direction.z * Mathf.Cos(maxVerticalAngle * Mathf.Deg2Rad));
       
        return direction * speed;
    }

    private Vector3 RotateSpinTowardVelocityDirection(Vector3 velocity, Vector3 spin)
    {
        Vector3 targetXZ = new Vector3(velocity.x, 0f, velocity.z).normalized;
        Quaternion targetRotation = Quaternion.Euler(0,-90,0) * Quaternion.LookRotation(targetXZ, Vector3.up);
        
        return targetRotation * spin;
    }
     
    private (Vector3,Vector3) PrecomputeTrajectory(Vector3 initialPosition, Quaternion initialRotation, Vector3 initialVelocity, Vector3 initialAngularVelocity)
    {
        bool stop = false;
        float deltaTime = 1f / precomputationFrameRate;
        
        Vector3 position = initialPosition, newPosition, oldPosition = initialPosition;
        Quaternion rotation = initialRotation, newRotation, oldRotation = initialRotation;
        Vector3 velocity = initialVelocity, newVelocity, oldVelocity = initialVelocity;
        Vector3 angularVelocity = initialAngularVelocity, newAngularVelocity, oldAngularVelocity = initialAngularVelocity;
        
        Vector3 closestPoint = position;
        Vector3 constraintClosestPoint = new Vector3();
        
        float oldTargetError = Vector3.Distance(position, target.transform.position);
        float oldConstraintError = Vector3.Distance(position, constraintClosestPoint);
        
        while(!stop)
        {
            (newPosition, newRotation, newVelocity, newAngularVelocity) = IntegrateMotion(deltaTime, position,  rotation, velocity, angularVelocity, BallState.Bouncing);
            _currentTrajectory.Add(newPosition);
            
            float targetError = Vector3.Distance(position, target.transform.position);
            float newTargetError = Vector3.Distance(newPosition, target.transform.position);

            if (targetError <= oldTargetError && targetError <= newTargetError)
            {
                stop = true;

                closestPoint = oldTargetError <= newTargetError ? FindClosestPoint(deltaTime / 2, deltaTime / 4, oldPosition, oldRotation, oldVelocity, oldAngularVelocity, BallState.Bouncing, target.transform.position) : FindClosestPoint(deltaTime / 2, deltaTime / 4, position, rotation, velocity, angularVelocity, BallState.Bouncing, target.transform.position);
            }
            oldTargetError = targetError;

            if(barrierShot)
            {
                float constraintError = Vector3.Distance(position, constraint.transform.position);
                float newConstraintError = Vector3.Distance(newPosition, constraint.transform.position);

                if (constraintError <= oldConstraintError && constraintError <= newConstraintError)
                {
                    constraintClosestPoint = oldConstraintError <= newConstraintError
                        ? FindClosestPoint(deltaTime / 2, deltaTime / 4, oldPosition, oldRotation, oldVelocity,
                            oldAngularVelocity, BallState.Bouncing, constraint.transform.position)
                        : FindClosestPoint(deltaTime / 2, deltaTime / 4, position, rotation, velocity, angularVelocity, BallState.Bouncing,
                            constraint.transform.position);
                }

                oldConstraintError = constraintError;
            }
            
            oldPosition = position;
            oldVelocity = velocity;
            oldAngularVelocity = angularVelocity;
            oldRotation = rotation;
            
            velocity = newVelocity;
            position = newPosition;
            angularVelocity = newAngularVelocity;
            rotation = newRotation;
        }

        return (closestPoint, constraintClosestPoint);
    }

    public List<Vector3> ComputeKickTrajectory(Vector3 direction, float speed, Vector3 spin)
    {
        List<Vector3> kickTrajectory = new List<Vector3>();
        
        float deltaTime = 1f/precomputationFrameRate;
        int count = 0;
        
        Vector3 targetXY = direction;
        targetXY.y = 0f;

        Quaternion targetRotation = Quaternion.LookRotation(targetXY, Vector3.up);

        Vector3 position = ballValues.position;
        Quaternion rotation = ballValues.rotation;
        Vector3 velocity = direction * speed;
        Vector3 angularVelocity = targetRotation * new Vector3(spin.y, 0, 0) * maximumSpinValue;
        angularVelocity.y = -spin.x * maximumSpinValue;
        
        while(count < 15)
        {
            (Vector3 newPosition, Quaternion newRotation, Vector3 newVelocity, Vector3 newAngularVelocity) = IntegrateMotion(deltaTime, position, rotation, velocity, angularVelocity, ballValues.state);
            
            if (newPosition.y - ballValues.radius <= 0f && ballValues.state == BallState.Bouncing && newVelocity.y < 0)
            {
                (newPosition, newRotation, newVelocity, newAngularVelocity, ballValues.state) = CheckCollision(deltaTime, position, rotation, velocity, angularVelocity, ballValues.state);
            }

            position = newPosition;
            rotation = newRotation;
            velocity = newVelocity;
            angularVelocity = newAngularVelocity;
            
            count++;
            kickTrajectory.Add(position);
        }
        
        
        return kickTrajectory;
    }
    
    private Vector3 FindClosestPoint(float deltaTime, float step , Vector3 position, Quaternion rotation, Vector3 velocity, Vector3 angularVelocity, BallState state, Vector3 targetPosition)
    {
        (Vector3 pMinus, _, _, _) = IntegrateMotion(deltaTime - step, position, rotation, velocity, angularVelocity, state);
        float distance1 = Vector3.Distance(pMinus, targetPosition);
        
        (Vector3 pPlus, _, _, _) = IntegrateMotion(deltaTime + step, position, rotation, velocity, angularVelocity, state);
        float distance2 = Vector3.Distance(pPlus, targetPosition);
        
        Vector3 closestPoint;
        
        if (distance1 < distance2) 
        {
            
            if (step < CollisionDetectionError)
            {
                float correctDeltaTime = (deltaTime - step);

                (closestPoint, _, _, _) = IntegrateMotion(correctDeltaTime, position, rotation, velocity, angularVelocity, state);
                
                return closestPoint;
            }
            
            closestPoint = FindClosestPoint(deltaTime - step, step/2, position, rotation, velocity, angularVelocity, state, targetPosition);
        }
        else
        {
            if (step < CollisionDetectionError)
            {
                float correctDeltaTime = (deltaTime + step);

                (closestPoint, _, _, _) = IntegrateMotion(correctDeltaTime, position, rotation, velocity, angularVelocity, state);

                return closestPoint;
            }

            closestPoint = FindClosestPoint(deltaTime + step, step/2, position, rotation, velocity, angularVelocity, state, targetPosition);
        }

        return closestPoint;
    }
    
    void OnDrawGizmos()
    {
        if(!precompute)
        {
            Color[] colorArray = new Color[_precalculationTrajectories.Count];
            for (int i = 0; i < _precalculationTrajectories.Count; i++)
            {
                float t = (float)i / (_precalculationTrajectories.Count - 1);
                colorArray[i] = Color.Lerp(Color.green, Color.blue, t);
            }

            for (int i = 0; i < _precalculationTrajectories.Count; i++)
            {
                if (_precalculationTrajectories[i].Count < 2) continue;

                Handles.color = colorArray[i]; 

                Vector3[] linePoints = _precalculationTrajectories[i].ToArray();

                Handles.DrawAAPolyLine(7.5f, linePoints);
            }
        }
    }
    
    public Vector3 Position()
    {
        if (frameByFrame)
        {
            return frameArray[frameCount].position;
        }
        return ballValues.position;
    }
    
    public Vector3 Velocity()
    {
        if(frameByFrame)
        {
            return frameArray[frameCount].v_n;
        }
        return ballValues.velocity;
    }
    
    public Vector3 AngularVelocity()
    {
        if(frameByFrame)
        {
            return frameArray[frameCount].w_n;
        }
        return ballValues.angularVelocity;
    }

    public Vector3 Acceleration()
    {
        if(frameByFrame)
        {
            return frameArray[frameCount].a_n;
        }
        
        return CalculateTotalForces(ballValues.velocity, ballValues.angularVelocity, ballValues.state)/ballValues.mass;
    }
    
    public float PotentialEnergy()
    {
        if(frameByFrame)
        {
            return frameArray[frameCount].potentialEnergy;
        }
        return ballValues.CalculatePotentialEnergy();
    }

    public float KineticEnergy()
    {
        if(frameByFrame)
        {
            return frameArray[frameCount].kineticEnergy;
        }
        return ballValues.CalculateKineticEnergy();
    }

    public float RotationalEnergy()
    {
        if(frameByFrame)
        {
            return frameArray[frameCount].rotationalEnergy;
        }
        return ballValues.CalculateRotationalEnergy();
    }

    public float TotalEnergy()
    {
        if(frameByFrame)
        {
            return frameArray[frameCount].totalEnergy;
        }
        return ballValues.CalculateTotalEnergy();
    }
    
    private Vector3 CalculateFrictionTorque(Vector3 velocity, Vector3 angularVelocity, BallState state)
    {
        if(state == BallState.Bouncing)
            return Vector3.zero;
        
        //TODO spiegare
        Vector3 frictionTorque = new Vector3(0, - ballValues.mass * Gravity * ballValues.coefficientOfVerticalAxisSpinningFriction * angularVelocity.y, 0);
        
        if (state == BallState.Rolling)
            return frictionTorque;
        
        Vector3 direction = (angularVelocity + Vector3.Cross(velocity, Vector3.up) / ballValues.radius).normalized;
        Vector3 alpha = -3f/2f * ballValues.coefficientOfSlidingFriction * Gravity / ballValues.radius * direction;

        frictionTorque += alpha * ballValues.inertialMomentum;
        
        return frictionTorque;
    }

    private Vector3 CalculateTotalTorque(Vector3 velocity, Vector3 angularVelocity, BallState state)
    {
        return CalculateFrictionTorque(velocity, angularVelocity, state);
    }

    public Vector3 TotalForces()
    {
        if(frameByFrame)
        {
            return frameArray[frameCount].totalForces;
        }
        return CalculateTotalForces(ballValues.velocity, ballValues.angularVelocity, ballValues.state);
    }

    public Vector3 DragForce()
    {
        if(frameByFrame)
        {
            return frameArray[frameCount].dragForce;
        }
        return CalculateDragForce(ballValues.velocity, ballValues.angularVelocity);
    }

    public Vector3 MagnusForce()
    {
        if(frameByFrame)
        {
            return frameArray[frameCount].magnusForce;
        }
        return CalculateMagnusForce(ballValues.velocity, ballValues.angularVelocity);
    }

    public Vector3 NormalForce()
    {
        if(frameByFrame)
        {
            return frameArray[frameCount].normalForce;
        }
        return CalculateNormalForce(ballValues.velocity, ballValues.angularVelocity, ballValues.state);
    }

    public Vector3 FrictionForce()
    {
        if(frameByFrame)
        {
            return frameArray[frameCount].frictionForce;
        }
        return CalculateFrictionForce(ballValues.velocity, ballValues.angularVelocity, ballValues.state);
    }

    public Vector3 FrictionTorque()
    {
        if(frameByFrame)
        {
            return frameArray[frameCount].frictionTorque;
        }
        return CalculateFrictionTorque(ballValues.velocity, ballValues.angularVelocity, ballValues.state);
    }
    
    public Vector3 TotalTorque()
    {
        if(frameByFrame)
        {
            return frameArray[frameCount].totalTorque;
        }
        return CalculateTotalTorque(ballValues.velocity, ballValues.angularVelocity, ballValues.state);
    }

    public float TimeToCompute()
    {
        if(frameByFrame)
        {
            return frameArray[frameCount].timeToCompute;
        }
        return _timeToCompute;
    }
    
    public float AverageTimeToCompute()
    {
        if (frameArray == null || frameArray.Length == 0)
            return 0;
        
        return _totalTimeToCompute / _frames.Count;
    }
    
            private Vector3 CalculateDragForce(Vector3 velocity, Vector3 angularVelocity)
    {
        //TODO spiegare questi calcoli (paper)
        if(!ballValues.useCustomDragCoefficient)
        {
            float s = angularVelocity.magnitude * ballValues.radius / velocity.magnitude;
            float Re = velocity.magnitude * 2*ballValues.radius / kinematicViscosityOfAir;
            float ReC = ballValues.vc * 2*ballValues.radius / kinematicViscosityOfAir;
            float ReS = ballValues.vs * 2*ballValues.radius / kinematicViscosityOfAir;

            if (velocity.magnitude > ballValues.vc && s > 0.05f)
            {
                ballValues.dragCoefficient = 0.4127f * Mathf.Pow(s, 0.3056f);
            }
            else
                ballValues.dragCoefficient = 0.155f + 0.346f / (1 + Mathf.Exp((Re - ReC) / ReS));
        }
        else
        {
            ballValues.dragCoefficient = ballValues.customDragCoefficient;
        }
        
        return - 0.5f * airDensity * ballValues.dragCoefficient * Mathf.PI * Mathf.Pow(ballValues.radius, 2) * velocity.magnitude * velocity;
    }
    
    private Vector3 CalculateMagnusForce(Vector3 velocity, Vector3 angularVelocity)
    {
        return airDensity * ballValues.liftCoefficient * Mathf.PI * Mathf.Pow(ballValues.radius, 3) *  Vector3.Cross(angularVelocity, velocity);
    }

    private Vector3 CalculateNormalForce(Vector3 velocity, Vector3 angularVelocity, BallState state)
    {
        if(state == BallState.Bouncing)
            return Vector3.zero;
        
        return (Gravity * ballValues.mass - CalculateMagnusForce(velocity, angularVelocity).y) * Vector3.up;
    }

    private Vector3 CalculateFrictionForce(Vector3 velocity, Vector3 angularVelocity, BallState state)
    {
        if(state == BallState.Bouncing)
            return Vector3.zero;

        if(state == BallState.Sliding)
        {
            //sliding friction
            Vector3 direction = (velocity - Vector3.Cross(angularVelocity * ballValues.radius, Vector3.up)).normalized;
            _previousSignOfFriction = _frictionForce;
            _frictionForce = -ballValues.mass * Gravity * ballValues.coefficientOfSlidingFriction * direction;
        }
        else
        {
            //rolling friction, pure rolling
            _frictionForce = -ballValues.coefficientOfRollingFriction * ballValues.mass * Gravity * velocity.normalized;
            _frictionForce.y = 0;
        }
        
        return _frictionForce;
    }
    
    private Vector3 CalculateTotalForces(Vector3 velocity, Vector3 angularVelocity, BallState state)
    {
        Vector3 dragForce = CalculateDragForce(velocity, angularVelocity);
        Vector3 magnusForce = CalculateMagnusForce(velocity, angularVelocity);
        Vector3 normalForce = CalculateNormalForce(ballValues.velocity, ballValues.angularVelocity, state);
        Vector3 frictionForce = CalculateFrictionForce(velocity, angularVelocity, state);
        Vector3 totalForces = Gravity * ballValues.mass * Vector3.down + dragForce + magnusForce + normalForce + frictionForce;

        return totalForces;
    }
}



public enum IntegrationMethod
{
    VelocityVerlet,
    ExplicitEuler,
    RK2,
    RK4
}


