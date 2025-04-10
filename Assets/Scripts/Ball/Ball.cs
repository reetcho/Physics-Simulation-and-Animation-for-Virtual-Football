using System;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;
using UnityEngine.UI;
using Quaternion = UnityEngine.Quaternion;
using Vector2 = UnityEngine.Vector2;
using Vector3 = UnityEngine.Vector3;


public class Ball : MonoBehaviour
{
    private Vector3 p_n; //position_n
    private Vector3 p_n1;//position_n+1
    private Vector3 v_n; //velocity_n
    private Vector3 v_n1;//velocity_n+1
    private Vector3 a_n; //acceleration_n
    private Vector3 w_n; //angularVelocity_n
    private Vector3 w_n1;//angularVelocity_n+1
    private Vector3 alpha_n; //acceleration_n
    private Vector3 alpha_n1; //acceleration_n+1
    private Quaternion theta_n; //angularAcceleration_n
    private Quaternion theta_n1; //angularAcceleration_n1
    
    private bool _isBouncing;
    private bool _isSliding;
    
    public BallState state;
    
    public bool IsMoving;
    
    [Header("Ball properties")]
    [SerializeField] private float radius;
    [SerializeField] private float mass;
    [SerializeField] [Range(0, 1)] private float coefficientOfVerticalRestitution;
    [SerializeField] [Range(-1, 1)] private float coefficientOfHorizontalRestitution;
    [SerializeField] [Range(-1, 1)] private float coefficientRotationRestitutionYaxis;
    private float _inertialMomentum; //assumption of thin sphere -> 2/3 * mass * radius^2

    private float _potentialEnergy;
    private float _kineticEnergy;
    private float _rotationalEnergy;
    private float _totalEnergy;

    private Vector3 _dragForce;
    private Vector3 _magnusForce;
    private Vector3 _normalForce;
    private Vector3 _frictionForce;
    private Vector3 _totalForces;

    private Vector3 _frictionTorque;
    private Vector3 _previousSignOfFriction;
    private Vector3 _totalTorque;
    
    private const float Gravity = 9.81f; 
    private const float CollisionDetectionError = 1e-5f;

    [Header("Friction")]
    [SerializeField] [Range(0, 1)] private float coefficientOfSlidingFriction;
    [SerializeField] [Range(0, 1)] private float coefficientOfRollingFriction; 
    [SerializeField] [Range(0,1)] private float coefficientOfVerticalAxisSpinningFriction;
    
    [Header("Drag and lift force")]
    [SerializeField] private float airDensity;
    [SerializeField] private float kinematicViscosityOfAir = 1.48f * 1e-5f;
    [SerializeField] private float dragCoefficient;
    [SerializeField] private bool customDragCoefficient;
    [SerializeField] private float vc;
    [SerializeField] private float vs;
    [SerializeField] private float angularDragCoefficient;
    [SerializeField] private float liftCoefficient;

    
    [Header("Initial values")]
    [SerializeField] private Vector3 v0;
    [SerializeField] private Vector3 w0;
    [SerializeField] private float initialSpeed;
    public Vector3 p0;
    public float maximumSpinValue = 30f;
    
    [Header("Simulation")]
    [SerializeField] private bool velocityVerlet;
    [SerializeField] private bool explicitEuler;
    [SerializeField] private bool RK2;
    [SerializeField] private bool RK4;
    [SerializeField] private float frameRate;

    [Header("Tracker")]
    public bool frameByFrame;
    public bool interpolateFrames;
    [Range(0,1000)] public int frameCount;
    public float continuousFrameCount;
    private Queue<Frame> _frames;
    public Frame[] frameArray;
    private float _timeToCompute;
    private float _totalTimeToCompute = 0f;

    [Header("Targeted shot")]
    public bool precompute;
    [SerializeField] private int precomputationFrameRate;
    [SerializeField] private float precalculationMaxError;
    [SerializeField] private int maxIterations;
    public bool barrierShot;
    [SerializeField] private GameObject barrier;
    [SerializeField] private GameObject constraint;

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
    private bool distanceToTargetChecked;
    
    [Header("Interaction")]
    [SerializeField] private Slider strengthSlider;

    [SerializeField] private InteractionController spinSlider;
    [SerializeField] private GameObject mainCamera;
    
    
    public void Reset()
    {
        _precalculationTrajectories.Clear();
        distanceToTargetChecked = true;
        
        ResetPhysicsValues();
        p0.y = radius;
        p_n = p0;
        theta_n = Quaternion.identity;

        transform.position = p_n;
        transform.rotation = theta_n;
        
        _frames.Clear();
    }
    
    private void ResetPhysicsValues()
    {
        v_n = Vector3.zero;
        a_n = Vector3.zero;
        w_n = Vector3.zero;
        alpha_n = Vector3.zero;
        
        _potentialEnergy = 0f;
        _kineticEnergy = 0f;
        _rotationalEnergy = 0f;
        _totalEnergy = 0f;

        _dragForce = Vector3.zero;
        _magnusForce = Vector3.zero;
        _normalForce = Vector3.zero;
        _frictionForce = Vector3.zero;
        _totalForces = Vector3.zero;

        _frictionTorque = Vector3.zero;
        _previousSignOfFriction = Vector3.zero;
        _totalTorque = Vector3.zero;
        
        _isBouncing = true;
        _isSliding = true;
        IsMoving = false;
        state = BallState.Stopped;
    }

    public void Kick(Vector3 direction, float speed, Vector3 spin)
    {
        Vector3 targetXZ = direction;
        targetXZ.y = 0f;
        targetXZ.Normalize();
        
        Vector3 realSpin = new Vector3(spin.y, -spin.x, 0) * maximumSpinValue;
        
        Quaternion targetRotation = Quaternion.LookRotation(targetXZ, Vector3.up);
        
        _precalculationTrajectories.Clear();
        _isBouncing = true;
        _isSliding = true;
        IsMoving = true;
        state = BallState.Bouncing;
        v_n = direction * speed;
        w_n = targetRotation * realSpin;
        CalculateTotalForces(v_n,w_n);
        CalculateTotalEnergy();
        
        AddTrajectoryFrame();
    }

    public void TargetedKick(float speed, Vector3 spin)
    {
        _precalculationTrajectories.Clear();
        _isBouncing = true;
        _isSliding = true;
        state = BallState.Bouncing;
        p0 = transform.position;
        
        Vector3 realSpin = new Vector3(0, -spin.x, -spin.y) * maximumSpinValue;
        FindInitialVelocity(speed, realSpin, 0, 45);
        
        
        p_n = transform.position;
        a_n = Vector3.zero;
        
        theta_n = Quaternion.identity;

        _isBouncing = true;
        _isSliding = true;
        distanceToTargetChecked = false;
        IsMoving = true;
        state = BallState.Bouncing;
        
        CalculateTotalForces(v_n,w_n);
        CalculateTotalEnergy();
        
        AddTrajectoryFrame();
    }
    
    public void DoubleTargetedKick(float speed)
    {
        _precalculationTrajectories.Clear();
        _isBouncing = true;
        _isSliding = true;
        state = BallState.Bouncing;
        p0 = transform.position;

        barrierShot = true;

        if (speed < 15f)
            speed = 15f;
        FindInitialVelocityWithConstraint(speed);
        barrierShot = false;
        
        p_n = transform.position;
        a_n = Vector3.zero;
        
        theta_n = Quaternion.identity;

        _isBouncing = true;
        _isSliding = true;
        distanceToTargetChecked = false;
        IsMoving = true;
        state = BallState.Bouncing;
        
        CalculateTotalForces(v_n,w_n);
        CalculateTotalEnergy();
        
        AddTrajectoryFrame();
    }
   
    void Awake()
    {
        Application.targetFrameRate = -1;
        
        Time.fixedDeltaTime = 1/frameRate;
        
        _frames = new Queue<Frame>();
        _inertialMomentum = 2f/3f * mass * radius * radius;
        p0 = transform.position;
        transform.localScale = new Vector3(radius, radius, radius)*2;
        
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
            distanceToTargetChecked = true;

            if (barrierShot)
            {
                initialSpeed = minInitialSpeed;
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
                Vector3 stepDirection;
                
                if (barrierShot)
                {
                    initialSpeed += (maxInitialSpeed - minInitialSpeed) / testSteps;
                    FindInitialVelocityWithConstraint(initialSpeed);
                }
                else
                {
                    stepDirection = (testEndPosition - testStartPosition) / testSteps;
                    _testCurrentPosition = testStartPosition + stepDirection * _currentStep;
                    transform.position = _testCurrentPosition;
                    p0 = transform.position;
                    FindInitialVelocity(initialSpeed, w0,0, 45f);
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
                
                p_n = transform.position;
                a_n = Vector3.zero;
                theta_n = Quaternion.identity;
                _isBouncing = !(p_n.y <= radius);
                _isSliding = true;
                state = BallState.Bouncing;
                distanceToTargetChecked = false;
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
                    transform.position = frameArray[frameCount].position;
                    transform.rotation = frameArray[frameCount].rotation;
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

                transform.position = interpolatedPos;
                transform.rotation = interpolatedRot;
            }

            return;
        }
        
        //if (targetKickTesting || !IsMoving)
        if (targetKickTesting || state == BallState.Stopped)
            return;
        
        CheckIsMoving();
         
        ComputeFrame(Time.fixedDeltaTime);
    }

    private void ComputeFrame(float deltaTime)
    {
        var startTime = Time.realtimeSinceStartup;
        
        IntegrateMotion(deltaTime);
        
        CheckCollision(deltaTime);
        
        CheckPureRolling();
        
        if (!distanceToTargetChecked)
            CheckDistanceFromTarget(deltaTime);
        
        v_n = v_n1;
        w_n = w_n1;
        p_n = p_n1;
        theta_n = theta_n1;
        
        CalculateTotalEnergy();

        transform.position = p_n;
        transform.rotation = theta_n;
        
        var endTime = Time.realtimeSinceStartup;
        _timeToCompute = endTime - startTime;
        _totalTimeToCompute += _timeToCompute;

        AddTrajectoryFrame();
    }

    private void AddTrajectoryFrame()
    {
        //if (IsMoving && !targetKickTesting)
        if (state != BallState.Stopped && !targetKickTesting)
        {
            var frame = new Frame(transform.position, transform.rotation, p_n, v_n, a_n, w_n, _potentialEnergy,
                _kineticEnergy, _rotationalEnergy, _totalEnergy, _dragForce, _magnusForce, _normalForce,
                _frictionForce, _totalForces, _frictionTorque, _totalTorque, _timeToCompute);
            
            if (_frames.Count >= 1000)
            {
                _frames.Dequeue(); 
            }

            _frames.Enqueue(frame);
            frameArray = _frames.ToArray();
            
            //GetComponent<TrajectoryTracker>().AddPoint(transform.position, _isSliding ? Color.red : Color.green);
            GetComponent<TrajectoryTracker>().AddPoint(transform.position, state == BallState.Rolling ? Color.green : Color.red);
        }
    }

    private void IntegrateMotion(float deltaTime)
    {
        if (velocityVerlet)
            VelocityVerlet(deltaTime);
        else if (explicitEuler)
            ExplicitEuler(deltaTime);
        else if (RK2)
            Runge_Kutta_2(deltaTime);
        else if (RK4)
            Runge_Kutta_4(deltaTime);
    }
    
     private void VelocityVerlet(float deltaTime) 
     {
         //F_k
         CalculateTotalForces(v_n, w_n);
         CalculateTotalTorque(v_n, w_n);

         a_n = _totalForces / mass;
         alpha_n = _frictionTorque / _inertialMomentum;

         //v_k+1/2
         v_n1 = v_n + deltaTime/2 * a_n;
         w_n1 = w_n + deltaTime/2 * alpha_n;
         
         if(state == BallState.Rolling)
         //if(!_isSliding && !_isBouncing)
         {
             var y = w_n1.y;
             w_n1 = -Vector3.Cross(v_n1, Vector3.up) / radius;
             w_n1.y = y;
             theta_n1 = Quaternion.Euler(Mathf.Rad2Deg * deltaTime * w_n1) * theta_n;
         }

         //p_k+1
         p_n1 = p_n + deltaTime * v_n1;
         theta_n1 = Quaternion.Euler(Mathf.Rad2Deg * deltaTime * w_n1) * theta_n;

         //F_k+1
         CalculateTotalForces(v_n1, w_n1);
         CalculateTotalTorque(v_n1, w_n1);

         a_n = _totalForces / mass;
         alpha_n = _frictionTorque / _inertialMomentum;

         //v_k+1
         v_n1 += deltaTime/2 * a_n;
         w_n1 += deltaTime/2 * alpha_n;
         
         if(state == BallState.Rolling)
         //if(!_isSliding && !_isBouncing)
         {
             var y = w_n1.y;
             w_n1 = -Vector3.Cross(v_n1, Vector3.up) / radius;
             w_n1.y = y;
             theta_n1 = Quaternion.Euler(Mathf.Rad2Deg * deltaTime * w_n1) * theta_n;
         }
     }
    
     private void ExplicitEuler(float deltaTime)
    {
        //F_k
        CalculateTotalForces(v_n, w_n);
        CalculateTotalTorque(v_n, w_n);
        
        a_n = _totalForces / mass;
        alpha_n = _frictionTorque / _inertialMomentum;
        
        //v_k+1
        v_n1 = v_n + deltaTime * a_n;
        w_n1 = w_n + deltaTime * alpha_n;
        
        //p_k+1
        p_n1 = p_n + deltaTime * v_n1;
        theta_n1 = Quaternion.Euler(Mathf.Rad2Deg * deltaTime * w_n1) * theta_n;
        
        if(state == BallState.Rolling)
        //if(!_isSliding && !_isBouncing)
        {
            var y = w_n1.y;
            w_n1 = -Vector3.Cross(v_n1, Vector3.up) / radius;
            w_n1.y = y;
            theta_n1 = Quaternion.Euler(Mathf.Rad2Deg * deltaTime * w_n1) * theta_n;
        }
    }
    
    private void Runge_Kutta_2(float deltaTime)
    {
        //k1
        CalculateTotalForces(v_n, w_n);
        a_n = _totalForces / mass;
        var k1V = deltaTime * a_n;
        var k1P = deltaTime * v_n;
        
        CalculateTotalTorque(v_n, w_n);
        alpha_n = _frictionTorque / _inertialMomentum;
        var k1W = deltaTime * alpha_n;
        var k1T = deltaTime * w_n;
        
        //k2
        CalculateTotalForces(v_n + k1V, w_n + k1W);
        a_n = _totalForces / mass;
        var k2V = deltaTime * a_n;
        var k2P = deltaTime * (v_n + k1V);
        
        CalculateTotalTorque(v_n + k1V, w_n + k1W);
        alpha_n = _frictionTorque / _inertialMomentum;
        var k2W = deltaTime * alpha_n;
        var k2T = deltaTime * (w_n + k1W);
        
        p_n1 = p_n + (k1P + k2P) / 2;
        v_n1 = v_n + (k1V + k2V) / 2;
        
        w_n1 = w_n + (k1W + k2W) / 2;
        theta_n1 = Quaternion.Euler(Mathf.Rad2Deg * (k1T + k2T) / 2) * theta_n;
        
        if(state == BallState.Rolling)
        //if(!_isSliding && !_isBouncing)
        {
            var y = w_n1.y;
            w_n1 = -Vector3.Cross(v_n1, Vector3.up) / radius;
            w_n1.y = y;
            theta_n1 = Quaternion.Euler(Mathf.Rad2Deg * deltaTime * w_n1) * theta_n;
        }
    }

    private void Runge_Kutta_4(float deltaTime)
    {
        //k1
        CalculateTotalForces(v_n, w_n);
        a_n = _totalForces / mass;
        var k1V = deltaTime * a_n;
        var k1P = deltaTime * v_n;
        
        CalculateTotalTorque(v_n, w_n);
        alpha_n = _frictionTorque / _inertialMomentum;
        var k1W = deltaTime * alpha_n;
        var k1T = deltaTime * w_n;
        
        //k2
        CalculateTotalForces(v_n + k1V/2, w_n + k1W/2);
        a_n = _totalForces / mass;
        var k2V = deltaTime * a_n;
        var k2P = deltaTime * (v_n + k1V/2);
        
        CalculateTotalTorque(v_n + k1V/2, w_n + k1W/2);
        alpha_n = _frictionTorque / _inertialMomentum;
        var k2W = deltaTime * alpha_n;
        var k2T = deltaTime * (w_n + k1W/2);
        
        //k3
        CalculateTotalForces(v_n + k2V/2, w_n + k2W/2);
        a_n = _totalForces / mass;
        var k3V = deltaTime * a_n;
        var k3P = deltaTime * (v_n + k2V/2);
        
        CalculateTotalTorque(v_n + k2V/2, w_n + k2W/2);
        alpha_n = _frictionTorque / _inertialMomentum;
        var k3W = deltaTime * alpha_n;
        var k3T = deltaTime * (w_n + k2W/2);
        
        //k4
        CalculateTotalForces(v_n + k3V, w_n + k3W);
        a_n = _totalForces / mass;
        var k4V = deltaTime * a_n;
        var k4P = deltaTime * (v_n + k3V);
        
        CalculateTotalTorque(v_n + k3V, w_n + k3W);
        alpha_n = _frictionTorque / _inertialMomentum;
        var k4W = deltaTime * alpha_n;
        var k4T = deltaTime * (w_n + k3W);
        
        p_n1 = p_n + (k1P + 2*k2P + 2*k3P + k4P) / 6;
        v_n1 = v_n + (k1V + 2*k2V + 2*k3V + k4V) / 6;
        
        w_n1 = w_n + (k1W + 2*k2W + 2*k3W + k4W) / 6;
        theta_n1 = Quaternion.Euler(Mathf.Rad2Deg * (k1T + 2*k2T + 2*k3T + k4T) / 6) * theta_n;
        
        if(state == BallState.Rolling)
        //if(!_isSliding && !_isBouncing)
        {
            var y = w_n1.y;
            w_n1 = -Vector3.Cross(v_n1, Vector3.up) / radius;
            w_n1.y = y;
            theta_n1 = Quaternion.Euler(Mathf.Rad2Deg * deltaTime * w_n1) * theta_n;
        }
    }
    
    private void CheckCollision(float deltaTime)
    {
        //if (p_n1.y - radius <= 0f && _isBouncing && v_n1.y < 0)
        if (p_n1.y - radius <= 0f && state == BallState.Bouncing && v_n1.y < 0)
        {
            ContinuousCollisionDetection(deltaTime / 2, deltaTime / 2);

            if (v_n1.y < 0)
            {
                v_n1.y = 0;
                p_n1.y = radius;
                _isBouncing = false;
                state = BallState.Sliding;
            }
        }
    }

    private void CheckPureRolling()
    {
        //if(!_isBouncing && _isSliding && Vector3.Dot(_frictionForce, _previousSignOfFriction) < 0)
        if(state == BallState.Sliding && Vector3.Dot(_frictionForce, _previousSignOfFriction) < 0)
        {
            _isSliding = false;
            state = BallState.Rolling;
        }
    }
    
    private void CheckIsMoving()
    {
        if (_totalEnergy < 1e-4)
        {
            ResetPhysicsValues();
        }
    }
    
    public Vector3 Position()
    {
        if (frameByFrame)
        {
            return frameArray[frameCount].p_n;
        }
        return p_n;
    }
    
    public Vector3 Velocity()
    {
        if(frameByFrame)
        {
            return frameArray[frameCount].v_n;
        }
        return v_n;
    }
    
    public Vector3 AngularVelocity()
    {
        if(frameByFrame)
        {
            return frameArray[frameCount].w_n;
        }
        return w_n;
    }

    public Vector3 Acceleration()
    {
        if(frameByFrame)
        {
            return frameArray[frameCount].a_n;
        }
        return a_n;
    }
    
    public float PotentialEnergy()
    {
        if(frameByFrame)
        {
            return frameArray[frameCount].potentialEnergy;
        }
        return _potentialEnergy;
    }

    public float KineticEnergy()
    {
        if(frameByFrame)
        {
            return frameArray[frameCount].kineticEnergy;
        }
        return _kineticEnergy;
    }

    public float RotationalEnergy()
    {
        if(frameByFrame)
        {
            return frameArray[frameCount].rotationalEnergy;
        }
        return _rotationalEnergy;
    }

    public float TotalEnergy()
    {
        if(frameByFrame)
        {
            return frameArray[frameCount].totalEnergy;
        }
        return _totalEnergy;
    }

    private float CalculatePotentialEnergy()
    {
        //if(!_isBouncing)
        if(state != BallState.Bouncing)
            return 0f;
        
        return mass * Gravity * (p_n.y - radius);
    }

    private float CalculateKineticEnergy()
    {
        return 0.5f * mass * v_n.magnitude * v_n.magnitude;
    }

    private float CalculateRotationalEnergy()
    {
        return 0.5f * _inertialMomentum * w_n.magnitude * w_n.magnitude;
    }

    private void CalculateTotalEnergy()
    {
        _kineticEnergy = CalculateKineticEnergy();
        _potentialEnergy = CalculatePotentialEnergy();
        _rotationalEnergy = CalculateRotationalEnergy();
        _totalEnergy = _kineticEnergy + _potentialEnergy + _rotationalEnergy;
    }
    

    private Vector3 CalculateDragForce(Vector3 velocity, Vector3 angularVelocity)
    {
        if(!customDragCoefficient)
        {
            float s = angularVelocity.magnitude * radius / velocity.magnitude;
            float Re = velocity.magnitude * 2*radius / kinematicViscosityOfAir;
            float ReC = vc * 2*radius / kinematicViscosityOfAir;
            float ReS = vs * 2*radius / kinematicViscosityOfAir;

            if (velocity.magnitude > vc && s > 0.05f)
            {
                dragCoefficient = 0.4127f * Mathf.Pow(s, 0.3056f);
            }
            else
                dragCoefficient = 0.155f + 0.346f / (1 + Mathf.Exp((Re - ReC) / ReS));
        }
        
        _dragForce = - 0.5f * airDensity * dragCoefficient * Mathf.PI * Mathf.Pow(radius, 2) * velocity.magnitude * velocity;
        return _dragForce;
    }
    
    private Vector3 CalculateMagnusForce(Vector3 velocity, Vector3 angularVelocity)
    {
        _magnusForce = airDensity * liftCoefficient * Mathf.PI * Mathf.Pow(radius, 3) *  Vector3.Cross(angularVelocity, velocity);
        return _magnusForce;
    }

    private Vector3 CalculateNormalForce()
    {
        //if(_isBouncing)
        if(state == BallState.Bouncing)
            return Vector3.zero;
        
        _normalForce = (Gravity * mass - _magnusForce.y) * Vector3.up;
        return _normalForce;
    }

    private Vector3 CalculateFrictionForce(Vector3 velocity, Vector3 angularVelocity)
    {
        //if(_isBouncing)
        if(state == BallState.Bouncing)
            return Vector3.zero;

        //if(_isSliding)
        if(state == BallState.Sliding)
        {
            //sliding friction
            var direction = (velocity - Vector3.Cross(angularVelocity * radius, Vector3.up)).normalized;
            _previousSignOfFriction = _frictionForce;
            _frictionForce = -mass * Gravity * coefficientOfSlidingFriction * direction;
        }
        else
        {
            //rolling friction, pure rolling
            _frictionForce = -coefficientOfRollingFriction * mass * Gravity * velocity.normalized;
            _frictionForce.y = 0;
        }
        
        return _frictionForce;
    }
    
    private void CalculateTotalForces(Vector3 velocity, Vector3 angularVelocity)
    {
        _dragForce = CalculateDragForce(velocity, angularVelocity);
        _magnusForce = CalculateMagnusForce(velocity, angularVelocity);
        _normalForce = CalculateNormalForce();
        _frictionForce = CalculateFrictionForce(velocity, angularVelocity);
        _totalForces = Gravity * mass * Vector3.down + _dragForce + _magnusForce + _normalForce + _frictionForce;
    }

    private Vector3 CalculateFrictionTorque(Vector3 velocity, Vector3 angularVelocity)
    {
        //if(_isBouncing)
        if(state == BallState.Bouncing)
            return Vector3.zero;
        
        Vector3 frictionTorque = new Vector3(0, - mass * Gravity * coefficientOfVerticalAxisSpinningFriction * angularVelocity.y, 0);
        
        //if (!_isSliding)
        if (state == BallState.Rolling)
            return frictionTorque;
        
        var direction = (angularVelocity + Vector3.Cross(velocity, Vector3.up) / radius).normalized;
        var alpha = -3f/2f * coefficientOfSlidingFriction * Gravity / radius * direction;

        frictionTorque += alpha * _inertialMomentum;
        
        return frictionTorque;
    }

    private void CalculateTotalTorque(Vector3 velocity, Vector3 angularVelocity)
    {
        _frictionTorque = CalculateFrictionTorque(velocity, angularVelocity);
        
        _totalTorque = _frictionTorque;
    }

    public Vector3 TotalForces()
    {
        if(frameByFrame)
        {
            return frameArray[frameCount].totalForces;
        }
        return _totalForces;
    }

    public Vector3 DragForce()
    {
        if(frameByFrame)
        {
            return frameArray[frameCount].dragForce;
        }
        return _dragForce;
    }

    public Vector3 MagnusForce()
    {
        if(frameByFrame)
        {
            return frameArray[frameCount].magnusForce;
        }
        return _magnusForce;
    }

    public Vector3 NormalForce()
    {
        if(frameByFrame)
        {
            return frameArray[frameCount].normalForce;
        }
        return _normalForce;
    }

    public Vector3 FrictionForce()
    {
        if(frameByFrame)
        {
            return frameArray[frameCount].frictionForce;
        }
        return _frictionForce;
    }

    public Vector3 FrictionTorque()
    {
        if(frameByFrame)
        {
            return frameArray[frameCount].frictionTorque;
        }
        return _frictionTorque;
    }
    
    public Vector3 TotalTorque()
    {
        if(frameByFrame)
        {
            return frameArray[frameCount].totalTorque;
        }
        return _totalTorque;
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

    private void ContinuousCollisionDetection(float deltaTime, float step)
    {
        IntegrateMotion(deltaTime);
        
        if (p_n1.y - radius <= 0f)
        {
            //collision
            if (step < CollisionDetectionError)
            {
                var collisionDeltaTime = (deltaTime - step / 2);

                CalculateBounce(collisionDeltaTime);
                
                return;
            }
            
            ContinuousCollisionDetection(deltaTime - step/2, step/2);
        }
        else
        {
            //no collision
            if (step < CollisionDetectionError)
            {
                var collisionDeltaTime = (deltaTime + step / 2);

                CalculateBounce(collisionDeltaTime);

                return;
            }

            ContinuousCollisionDetection(deltaTime + step/2, step/2);
        }
    }

    private void CalculateBounce(float bounceDeltaTime)
    {
        //this first part calculates the new velocity and angular velocity immediately after the bounce
        IntegrateMotion(bounceDeltaTime);
        
        v_n = v_n1;
        w_n = w_n1;
        p_n = p_n1;
        theta_n = theta_n1;
        
        float alpha = 2f / 3f;
        v_n1.x = v_n.x * (1 - alpha * coefficientOfHorizontalRestitution)  / (alpha + 1) -
                 alpha * (1 + coefficientOfHorizontalRestitution) / (alpha + 1) * radius * w_n.z;

        v_n1.y = -coefficientOfVerticalRestitution * v_n.y;
        v_n1.z = v_n.z * (1 - alpha * coefficientOfHorizontalRestitution) / (alpha + 1) +
                 alpha * (1 + coefficientOfHorizontalRestitution) / (alpha + 1) * radius * w_n.x;

        w_n1.x = (alpha-coefficientOfHorizontalRestitution)/(alpha+1) * w_n.x + (coefficientOfHorizontalRestitution+1)/(alpha+1) * v_n.z/radius;
        w_n1.y = w_n.y * coefficientRotationRestitutionYaxis;
        w_n1.z = (alpha-coefficientOfHorizontalRestitution)/(alpha+1) * w_n.z - (coefficientOfHorizontalRestitution+1)/(alpha+1) * v_n.x/radius;

        v_n = v_n1;
        w_n = w_n1;
        
        if(!precompute && !targetKickTesting)
            GetComponent<TrajectoryTracker>().AddPoint(p_n, Color.blue);
        
        if (v_n.y < 1e-6)
        {
            v_n.y = 0;
            p_n.y = radius;
            _isBouncing = false;
            state = BallState.Sliding;
        }
        
        //this part calculates the trajectory of the ball starting from the moment when it touched the ground to the end of the frame

        var remainingTime = Time.fixedDeltaTime - bounceDeltaTime;
        
        IntegrateMotion(remainingTime);
    }
    
     private void FindInitialVelocity(float speed, Vector3 spin, float minVerticalAngle, float maxVerticalAngle)
     { 
        var startTime = Time.realtimeSinceStartup;
        
        Vector3 direction = new Vector3(target.transform.position.x - p0.x, 0, target.transform.position.z - p0.z).normalized;
        direction = new Vector3(direction.x * Mathf.Cos((maxVerticalAngle - minVerticalAngle) / 2 * Mathf.Deg2Rad), Mathf.Sin((maxVerticalAngle - minVerticalAngle) / 2 * Mathf.Deg2Rad), direction.z * Mathf.Cos((maxVerticalAngle - minVerticalAngle) / 2 * Mathf.Deg2Rad));
        Vector3 velocity = direction * speed;
        
        Vector3 rotatedSpin;
        Vector3 rotatedVelocity;
        
        var count = 0;
        var error = float.MaxValue;
        float angle;
        float cosTheta;
        Quaternion rotation = Quaternion.identity;

        float angle0 = 180f;
        float angle1 = 90f;
        
        float distance = Vector3.Distance(target.transform.position, p0);
        
        //max error depends on the distance from the target
        var maxError = distance < 25 ? precalculationMaxError : precalculationMaxError + Mathf.Pow(distance - 25,2) * 1e-4f;
        
        System.Random random = new System.Random();
        
        while (error > maxError && count < maxIterations)
        {
            count++;
            
            _currentTrajectory = new List<Vector3>();
            
            rotatedVelocity = rotation * velocity;
                
            //rotate the spin to be in the direction of the current iteration of the velocity
            rotatedSpin = RotateSpinTowardVelocityDirection(rotatedVelocity, spin);
            
            (Vector3 position,_) = PrecomputeTrajectory(rotatedVelocity, rotatedSpin);
            
            //check angle over ground plane is less than 30 degrees, otherwise stops calculation
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

                (position,_) = PrecomputeTrajectory(velocity, rotatedSpin);

                w_n = rotatedSpin;
                v_n = velocity;
                
                error = Vector3.Distance(position, target.transform.position);

                if(!barrierShot)
                    _precalculationTrajectories.Add(_currentTrajectory);
                break;
            }
            
            velocity = rotatedVelocity;
            w_n = rotatedSpin;
            v_n = velocity;
            
            error = Vector3.Distance(position, target.transform.position);
            
            Vector3 vTarget = (target.transform.position - p0).normalized;
            Vector3 vIteration = (position - p0).normalized;
            cosTheta = Vector3.Dot(vIteration, vTarget); 
            
            Vector3 rotationAxis = Vector3.Cross(vIteration, vTarget).normalized;
            
            angle = Mathf.Acos(Mathf.Clamp(cosTheta, -1f, 1f)) * Mathf.Rad2Deg;
            if (angle < 1e-3)
            {
                //angle = (vTarget - vIteration).magnitude;
                /*
                rotation = Quaternion.AngleAxis(angle, rotationAxis);
                Debug.Log(angle);
                Debug.Log(rotation);
                Debug.Log(rotation.Equals(Quaternion.identity));
                */
            }

            //adjust slightly the speed if one of the 2 cases happen, otherwise the predictor would keep calculating without improving the shot until maximum iterations are reached
            if ((angle < 1e-3 || Mathf.Abs(angle0 - angle) < 1e-3) && error > maxError && count < maxIterations)
            { 
                //angle = (vTarget - vIteration).magnitude;
                velocity *= random.Next(2) == 0 ? 1.0005f : 0.9995f;
            }
            
            angle0 = angle1;
            angle1 = angle;
            
            rotation = Quaternion.AngleAxis(angle, rotationAxis);
            
            if(!barrierShot)
                _precalculationTrajectories.Add(_currentTrajectory);
        }
        
        var endTime = Time.realtimeSinceStartup;
        
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
     }

     public GameObject noVerticalSpinBallPlaceholder;
     public GameObject noHorizontalSpinBallPlaceholder;
     
private void FindInitialVelocityWithConstraint(float speed)
    { 
        var startTime = Time.realtimeSinceStartup;
        int count = 1;
        float minVerticalAngle = 0f;
        float maxVerticalAngle = 30f;

        //finds initial velocity to reach the target with no spin and without considering that the ball has to pass above the barrier
        Vector3 direction = new Vector3(target.transform.position.x - p0.x, 0, target.transform.position.z - p0.z).normalized;
        direction = new Vector3(direction.x * Mathf.Cos((maxVerticalAngle - minVerticalAngle) / 2 * Mathf.Deg2Rad), Mathf.Sin((maxVerticalAngle - minVerticalAngle) / 2 * Mathf.Deg2Rad), direction.z * Mathf.Cos((maxVerticalAngle - minVerticalAngle) / 2 * Mathf.Deg2Rad));
        Vector3 velocity = direction * speed;
        _currentTrajectory = new List<Vector3>();
        (_, Vector3 predictedConstraintPosition) = PrecomputeTrajectory(velocity, Vector3.zero);
        
        //rotate velocity towards constraint and then calculate the new trajectory with the new direction
        velocity = RotateVelocityTowardConstraint(predictedConstraintPosition, constraint.transform.position, velocity, maxVerticalAngle, speed);
        (Vector3 noSpinPredictedTargetPosition,_) = PrecomputeTrajectory(velocity, Vector3.zero);
        
        //finds displacement of the previous shot from the target
        Vector3 targetDisplacement = (noSpinPredictedTargetPosition - target.transform.position ).normalized;
        
        //choose the direction of the verticalspin based on the displacement of the previous shot and rotates it towards the direction of the velocity
        Vector3 spin = new Vector3(0, Math.Sign(targetDisplacement.z), -Math.Sign(targetDisplacement.y)) * maximumSpinValue;
        Vector3 rotatedSpin = RotateSpinTowardVelocityDirection(velocity, spin);
        
        _currentTrajectory = new List<Vector3>();
        (Vector3 adjustedSpinPredictedTargetPosition,Vector3 adjustSpinPredictedConstraintPosition) = PrecomputeTrajectory(velocity, rotatedSpin);
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
                (Vector3 noHorizontalSpinPredictedTargetPosition,_) = PrecomputeTrajectory(velocity, noHorizontalSpin);
                targetDisplacement = (noHorizontalSpinPredictedTargetPosition - target.transform.position ).normalized;
                
                //finds the new horizontal spin proportionally to the distances of the shot with the previously found spin and the desire shot from the shot with no horizontal spin
                var d1 = Mathf.Abs(adjustedSpinPredictedTargetPosition.z - noHorizontalSpinPredictedTargetPosition.z);
                var d2 = Mathf.Abs(target.transform.position.z - noHorizontalSpinPredictedTargetPosition.z); 
                horizontalSpinMagnitude = Mathf.Min(horizontalSpinMagnitude * d2 / d1, maximumSpinValue);
                spin.y = Math.Sign(targetDisplacement.z) * horizontalSpinMagnitude;
                rotatedSpin.y = spin.y;
                
                //computes trajectory with the new found horizontal spin and rotates velocity towards the constraint
                (_, predictedConstraintPosition) = PrecomputeTrajectory(velocity, rotatedSpin);
                velocity = RotateVelocityTowardConstraint(predictedConstraintPosition,constraint.transform.position, velocity, maxVerticalAngle, speed);
                rotatedSpin = RotateSpinTowardVelocityDirection(velocity, spin);

                //computes new predicted position to find new predicted position of the target
                (adjustedSpinPredictedTargetPosition,_) = PrecomputeTrajectory(velocity, rotatedSpin);
                
                //computes trajectory having spin without vertical component
                Vector3 noVerticalSpin = spin;
                noVerticalSpin.z = 0f;
                (Vector3 noVerticalSpinPredictedTargetPosition,_) = PrecomputeTrajectory(velocity, noVerticalSpin);
                targetDisplacement = (noVerticalSpinPredictedTargetPosition - target.transform.position ).normalized;

                //finds the new vertical spin proportionally to the distances of the shot with the previously found spin and the desire shot from the shot with no vertical spin
                d1 = adjustedSpinPredictedTargetPosition.y - noVerticalSpinPredictedTargetPosition.y;
                d2 = target.transform.position.y - noVerticalSpinPredictedTargetPosition.y;
                verticalSpinMagnitude = Mathf.Min(verticalSpinMagnitude * Math.Abs(d2 / d1), maximumSpinValue);
                spin.z = -Math.Sign(targetDisplacement.y) * verticalSpinMagnitude;
                rotatedSpin = RotateSpinTowardVelocityDirection(velocity, spin);
                
                //computes trajectory with the new found vertical spin and rotates velocity towards the constraint
                (_, predictedConstraintPosition) = PrecomputeTrajectory(velocity, rotatedSpin);
                velocity = RotateVelocityTowardConstraint(predictedConstraintPosition,constraint.transform.position, velocity, maxVerticalAngle, speed);
                rotatedSpin = RotateSpinTowardVelocityDirection(velocity, spin);
                
                //computes new predicted position to find new predicted position of the target
                _currentTrajectory = new List<Vector3>();
                (adjustedSpinPredictedTargetPosition,adjustSpinPredictedConstraintPosition) = PrecomputeTrajectory(velocity, rotatedSpin);
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
        (adjustedSpinPredictedTargetPosition,adjustSpinPredictedConstraintPosition) = PrecomputeTrajectory(velocity, rotatedSpin);
        placeholder.transform.position = adjustedSpinPredictedTargetPosition;
        constraintPlaceholder.transform.position = adjustSpinPredictedConstraintPosition;
        _precalculationTrajectories.Add(_currentTrajectory);
        
        //finds distance from target and from constraint of the trajectory
        targetError = (adjustedSpinPredictedTargetPosition - target.transform.position).magnitude;
        constrainError = (adjustSpinPredictedConstraintPosition - constraint.transform.position).magnitude;

        if (stopped)
        {
            FindInitialVelocity(speed, spin, minVerticalAngle, maxVerticalAngle);
            velocity = v_n;
            rotatedSpin = w_n;
            
            _currentTrajectory = new List<Vector3>();
            (adjustedSpinPredictedTargetPosition,adjustSpinPredictedConstraintPosition) = PrecomputeTrajectory(velocity, rotatedSpin);
            placeholder.transform.position = adjustedSpinPredictedTargetPosition;
            constraintPlaceholder.transform.position = adjustSpinPredictedConstraintPosition;
            _precalculationTrajectories.Add(_currentTrajectory);
        
            //finds distance from target and from constraint of the trajectory
            targetError = (adjustedSpinPredictedTargetPosition - target.transform.position).magnitude;
            constrainError = (adjustSpinPredictedConstraintPosition - constraint.transform.position).magnitude;

            count++;
        }
        
        v_n = velocity;
        w_n = rotatedSpin;
        
        var endTime = Time.realtimeSinceStartup;
        
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
        
    }

    private Vector3 RotateVelocityTowardConstraint(Vector3 predrictedConstraintPosition, Vector3 constraintPosition, Vector3 velocity, float maxVerticalAngle, float speed)
    {
        Vector3 vIteration = (predrictedConstraintPosition - p0).normalized;
        Vector3 vTarget = (constraintPosition - p0).normalized;
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
     
    private (Vector3,Vector3) PrecomputeTrajectory(Vector3 velocity, Vector3 spin)
    {
        var stop = false;
        var deltaTime = 1f/precomputationFrameRate;
        
        v_n = velocity;
        p_n = transform.position;
        a_n = Vector3.zero;
        w_n = spin;
        theta_n = Quaternion.identity;
        
        _isBouncing = true;
        _isSliding = true;
        state = BallState.Bouncing;
        
        Vector3 closestPoint = p_n;
        Vector3 constraintClosestPoint = new Vector3();
        
        var savedP0 = p_n;
        var savedV0 = v_n;
        var savedW0 = w_n;
        var savedTheta0 = theta_n;
        var error0 = Vector3.Distance(p_n, target.transform.position);

        bool hasPassedBarrier = false;
        
        while(!stop)
        {
            if(barrierShot && !hasPassedBarrier)
            {
                IntegrateMotion(deltaTime);

                var planeNormal = (p0 - barrier.transform.position).normalized;
                planeNormal.y = 0;

                if (Vector3.Dot(planeNormal, (p_n1 - barrier.transform.position).normalized) < 0)
                {

                    Vector3 positionOnBarrierPlane = FindPointOnPlane(deltaTime, deltaTime, barrier.transform.position, planeNormal);
                    constraintClosestPoint = positionOnBarrierPlane;
                    hasPassedBarrier = true;
                }
            }
            
            IntegrateMotion(deltaTime);
            _currentTrajectory.Add(p_n1);
            
            var errorN = Vector3.Distance(p_n, target.transform.position);
            var errorN1 = Vector3.Distance(p_n1, target.transform.position);

            if (errorN <= error0 && errorN <= errorN1)
            {
                stop = true;
                
                if (error0 <= errorN1)
                {
                    closestPoint = FindClosestPoint(deltaTime / 2, deltaTime / 4, savedP0, savedTheta0, savedV0, savedW0, target.transform.position);
                }
                else
                {
                    closestPoint = FindClosestPoint(deltaTime / 2, deltaTime / 4, p_n, theta_n, v_n, w_n, target.transform.position);
                }
            }
            
            savedP0 = p_n;
            savedV0 = v_n;
            savedW0 = w_n;
            savedTheta0 = theta_n;
            error0 = errorN;

            v_n = v_n1;
            p_n = p_n1;
            w_n = w_n1;
            theta_n = theta_n1;
        }

        return (closestPoint, constraintClosestPoint);
    }

    public List<Vector3> ComputeKickTrajectory(Vector3 direction, float speed, Vector3 spin)
    {
        ResetPhysicsValues();
        List<Vector3> kickTrajectory = new List<Vector3>();
        
        var deltaTime = 1f/precomputationFrameRate;
        var count = 0;
        
        Vector3 targetXY = direction;
        targetXY.y = 0f;

        Quaternion targetRotation = Quaternion.LookRotation(targetXY, Vector3.up);

        w_n = targetRotation * new Vector3(spin.y, 0, 0) * maximumSpinValue;
        w_n.y = -spin.x * maximumSpinValue;
        v_n = direction * speed;
        
        while(count < 15)
        {
            IntegrateMotion(deltaTime);
            
            CheckCollision(deltaTime);
            
            v_n = v_n1;
            p_n = p_n1;
            w_n = w_n1;
            theta_n = theta_n1;
            
            count++;
            kickTrajectory.Add(p_n);
        }

        p_n = transform.position;
        theta_n = transform.rotation;
        ResetPhysicsValues();
        
        return kickTrajectory;
    }
    
    private Vector3 FindPointOnPlane(float deltaTime, float step, Vector3 pPlane, Vector3 planeNormal)
    {
        Vector3 pointOnPlane;
        
        IntegrateMotion(deltaTime);
        
        float currentDistance = Vector3.Dot(planeNormal, p_n1 - pPlane);
        
        if (currentDistance <= 0)
        {
            //collision
            if (step < CollisionDetectionError)
            {
                return p_n1;
            }
            
            pointOnPlane = FindPointOnPlane(deltaTime - step/2, step/2, pPlane, planeNormal);
        }
        else
        {
            //no collision
            if (step < CollisionDetectionError)
            {
                return p_n1;
            }

            pointOnPlane = FindPointOnPlane(deltaTime + step/2, step/2, pPlane, planeNormal);
        }

        return pointOnPlane;
    }
    
    private Vector3 FindClosestPoint(float deltaTime, float step , Vector3 position, Quaternion rotation, Vector3 velocity, Vector3 angularVelocity, Vector3 targetPosition)
    {
        p_n = position;
        v_n = velocity;
        w_n = angularVelocity;
        theta_n = rotation;
        IntegrateMotion(deltaTime - step);
        var p1 = p_n1;
        var distance1 = Vector3.Distance(p1, targetPosition);
        
        IntegrateMotion(deltaTime + step);
        var p2 = p_n1;
        var distance2 = Vector3.Distance(p2, targetPosition);
        
        Vector3 closestPoint;
        
        if (distance1 < distance2) 
        {
            
            if (step < CollisionDetectionError)
            {
                var correctDeltaTime = (deltaTime - step);

                IntegrateMotion(correctDeltaTime);
                
                return p_n1;
            }
            
            closestPoint = FindClosestPoint(deltaTime - step, step/2, position, rotation, velocity, angularVelocity, targetPosition);
        }
        else
        {
            if (step < CollisionDetectionError)
            {
                var correctDeltaTime = (deltaTime + step);

                IntegrateMotion(correctDeltaTime);

                return p_n1;
            }

            closestPoint = FindClosestPoint(deltaTime + step, step/2, position, rotation, velocity, angularVelocity, targetPosition);
        }

        return closestPoint;
    }
    
    private void CheckDistanceFromTarget(float deltaTime)
    {
        Vector3 pSaved = p_n1;
        Vector3 vSaved = v_n1;
        Vector3 wSaved = w_n1;
        Quaternion thetaSaved = theta_n1;
        
        Vector3 planePoint = target.transform.position; // Plane passes through the target
        Vector3 planeNormal = (p0 - planePoint).normalized; // Normal points towards startPosition
        
        float distanceBefore = Vector3.Dot(p_n - planePoint, planeNormal);
        float distanceAfter = Vector3.Dot(p_n1 - planePoint, planeNormal);
        
        if (distanceBefore * distanceAfter < 0) // Sign change -> crossing the plane
        
        {

            Vector3 pointOnLine = FindClosestPoint(deltaTime /2 , deltaTime / 4, p_n, theta_n, v_n, w_n, target.transform.position);
            
            Debug.Log("Runtime error: " + (Vector3.Distance(target.transform.position, pointOnLine)*1e3).ToString("F3") + " mm");
            
            placeholder.transform.position = pointOnLine;
            
            distanceToTargetChecked = true;
        }
        
        p_n1 = pSaved;
        v_n1 = vSaved;
        w_n1 = wSaved;
        theta_n1 = thetaSaved;
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
}


