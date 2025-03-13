using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Xml.Serialization;
using Unity.Mathematics;
using Unity.VisualScripting;
using UnityEditor;
using UnityEngine;
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
    private Vector3 alpha_n;
    private Vector3 alpha_n1;
    private Quaternion theta_n; //angularAcceleration_n
    private Quaternion theta_n1; //angularAcceleration_n1
    
    
    private bool _isBouncing;
    private bool _isSliding;
    private bool _stoppedSliding;
    private bool _collisionAtFrameBefore;
    
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
    private Vector3 _dragTorque;
    private Vector3 _totalTorque;
    
    private const float Gravity = 9.81f; 
    private const float CollisionDetectionError = 1e-5f;

    [Header("Friction")]
    [SerializeField] [Range(0, 1)] private float coefficientOfSlidingFriction;
    [SerializeField] [Range(0, 1)] private float coefficientOfRollingFriction; 
    
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
    private Vector3 p0;
    
    [Header("Simulation")]
    [SerializeField] private bool velocityVerlet;
    [SerializeField] private bool explicitEuler;
    [SerializeField] private bool RK2;
    [SerializeField] private bool RK4;
    [SerializeField] private float frameRate;

    [Header("Tracker")]
    public bool frameByFrame;
    [Range(0,500)] public int frameCount;
    private List<Frame> _frames;
    private float _timeToCompute;
    private float _totalTimeToCompute = 0f;

    [Header("Targeted shot")]
    public bool precompute;
    [SerializeField] private int precomputationFrameRate;
    [SerializeField] private float precalculationMaxError;
    [SerializeField] private int maxIterations;

    [Header("Testing")]
    [SerializeField] private bool targetKickTesting;
    private SaveSystem saveSystem;
    [SerializeField] private String description;
    [SerializeField] private String fileName;
    [SerializeField] private Vector3 testStartPosition;
    [SerializeField] private Vector3 testEndPosition;
    [SerializeField] private int testSteps;
    [SerializeField] private List<Vector3> startPositions;
    [SerializeField] private List<float> distances;
    [SerializeField] private List<float> errors;
    [SerializeField] private List<int> iterations;
    [SerializeField] private List<Vector3> outliersStartPositions;
    [SerializeField] private List<float> outliersError;
    [SerializeField] private float averageComputationTime = 0f;
    private int _currentStep = 0;
    private Vector3 _testCurrentPosition;
    private float _totalComputationTime;
    
    [Header("Other")]
    [SerializeField] private GameObject comparisonSphere;
    [SerializeField] private GameObject target;
    [SerializeField] private GameObject placeholder;
    private List<List<Vector3>> _precalculationTrajectories =  new List<List<Vector3>>();
    private List<Vector3> _currentTrajectory;
    private bool distanceToTargetChecked;


    private List<Vector3> closestPoints = new List<Vector3>();
    private List<Vector3> closestPoints2 = new List<Vector3>();
    
    
    void Awake()
    {
        Application.targetFrameRate = -1;
        Time.fixedDeltaTime = 1/frameRate;
        _frames = new List<Frame>();

        p0 = transform.position;
        p_n = transform.position;
        v_n = v0;
        a_n = Vector3.zero;
        w_n1 = w0;
        w_n = w_n1;
        alpha_n = Vector3.zero;
        theta_n = Quaternion.identity;
        
        transform.localScale = new Vector3(radius, radius, radius)*2;

        _isBouncing = !(p_n.y <= radius);
        _collisionAtFrameBefore = false;
        _isSliding = false;
        _stoppedSliding = false;
        
        _inertialMomentum = 2f/3f * mass * radius * radius;
        
        if(comparisonSphere != null)
            comparisonSphere.GetComponent<Rigidbody>().AddForce(v_n,ForceMode.VelocityChange);

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
        }
        
        if(precompute)
        {
            p0 = transform.position;
            v_n = FindInitialVelocity();
            p_n = transform.position;
            a_n = Vector3.zero;
            w_n1 = w0;
            w_n = w_n1;
            
            Vector3 targetXZ = new Vector3(v_n.x, 0f, v_n.z).normalized;
            
            Quaternion newRotation = Quaternion.LookRotation(Vector3.Cross(Vector3.up, targetXZ), Vector3.up);
        
            w_n = newRotation * w_n;
            w_n.y = w0.y; 
            
            theta_n = Quaternion.identity;

            _isBouncing = !(p_n.y <= radius);

            _isSliding = false;
            _stoppedSliding = false;
            distanceToTargetChecked = false;
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
                Vector3 stepDirection = (testEndPosition - testStartPosition) / testSteps;
                _testCurrentPosition = testStartPosition + stepDirection * _currentStep;
                transform.position = _testCurrentPosition ;
                p0 = transform.position;
                v_n = FindInitialVelocity();
                p_n = transform.position;
                a_n = Vector3.zero;
                w_n1 = w0;
                w_n = w_n1;
                theta_n = Quaternion.identity;

                _isBouncing = !(p_n.y <= radius);

                _isSliding = false;
                _stoppedSliding = false;
                distanceToTargetChecked = false;
                _currentStep++;
                
                if (_currentStep > testSteps)
                {
                    targetKickTesting = false;
                    saveSystem.SaveToJson(fileName,description, startPositions, distances, errors, outliersStartPositions, outliersError, averageComputationTime, iterations);
                    
                    Vector3 targetXZ = new Vector3(v_n.x, 0f, v_n.z).normalized;
            
                    Quaternion newRotation = Quaternion.LookRotation(Vector3.Cross(Vector3.up, targetXZ), Vector3.up);
        
                    w_n = newRotation * w_n;
                    w_n.y = w0.y; 
                }
            }
        }
        
    }


    void FixedUpdate()
    {
        if (targetKickTesting)
            return;
        
        //handles frame by frame mode
        if (frameByFrame)
        {
            if(frameCount <= _frames.Count && _frames.Count > 0)
            {
                transform.position = _frames[frameCount].position;
                transform.rotation = _frames[frameCount].rotation;
            }
            return;
        }

        //clamps to 0 very small values for v_n and w_n
        CheckAndReset(ref v_n);
        CheckAndReset(ref w_n);
         
        if (v_n == Vector3.zero)
        {
            return;
        }
         
        ComputeFrame(Time.fixedDeltaTime);
    }

    private void ComputeFrame(float deltaTime)
    {
        var startTime = Time.realtimeSinceStartup;
        
        CheckPureRolling();

        IntegrateMotion(deltaTime);
        
        CheckCollision(deltaTime);
        
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
        
        if (v_n.magnitude > 1e-3 && !targetKickTesting)
        {
            var frame = new Frame(transform.position, transform.rotation, p_n, v_n, a_n, w_n, _potentialEnergy,
                _kineticEnergy, _rotationalEnergy, _totalEnergy, _dragForce, _magnusForce, _normalForce,
                _frictionForce, _totalForces, _frictionTorque, _dragTorque, _totalTorque, _timeToCompute);
            _frames.Add(frame);
            
            GetComponent<TrajectoryTracker>().trajectoryPoints.Add(transform.position);
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
         //2ND ORDER VELOCITY VERLET

         //F_k
         CalculateTotalForces(v_n, w_n);
         CalculateTotalTorque(v_n, w_n);

         a_n = _totalForces / mass;

         //v_k+1/2
         v_n1 = v_n + deltaTime/2 * a_n;
         if(!_isBouncing)
         {
             if (_isSliding)
                 w_n1 = w_n + deltaTime / 2 * _frictionTorque / _inertialMomentum;
             else
                 w_n1 = -Vector3.Cross(v_n1, Vector3.up) / radius;
         }
         else
         {
             w_n1 = w_n + deltaTime / 2 * _dragTorque / _inertialMomentum;
         }

         //p_k+1
         p_n1 = p_n + deltaTime * v_n1;
         theta_n1 = Quaternion.Euler(Mathf.Rad2Deg * deltaTime * w_n1) * theta_n;

         //F_k+1
         CalculateTotalForces(v_n1, w_n1);
         CalculateTotalTorque(v_n1, w_n1);

         a_n = _totalForces / mass;

         //v_k+1
         v_n1 +=  deltaTime/2 * a_n;
         if (!_isBouncing)
         {
             if (_isSliding)
                 w_n1 += deltaTime / 2 * _frictionTorque / _inertialMomentum;
             else
                 w_n1 = -Vector3.Cross(v_n1, Vector3.up) / radius;
         }
         else
         {
             w_n1 += deltaTime / 2 * _dragTorque / _inertialMomentum;
         }
     }
    
     private void ExplicitEuler(float deltaTime)
    {
        //F_k
        CalculateTotalForces(v_n, w_n);
        CalculateTotalTorque(v_n, w_n);
        
        a_n = _totalForces / mass;
        
        //v_k+1
        v_n1 = v_n + deltaTime * a_n;
        if(!_isBouncing)
        {
            if (_isSliding)
                w_n1 = w_n + deltaTime * _frictionTorque / _inertialMomentum;
            else
                w_n1 = -Vector3.Cross(v_n1, Vector3.up) / radius;
        }
        else
        {
            w_n1 = w_n + deltaTime * _dragTorque / _inertialMomentum;
        }
        
        //p_k+1
        p_n1 = p_n + deltaTime * v_n1;
        theta_n1 = Quaternion.Euler(Mathf.Rad2Deg * deltaTime * w_n1) * theta_n;
    }
    
    private void Runge_Kutta_2(float deltaTime)
    {
        //k1
        CalculateTotalForces(v_n, w_n);
        a_n = _totalForces / mass;
        var k1V = deltaTime * a_n;
        var k1P = deltaTime * v_n;

        //CalculateTotalTorque(v_n, w_n);
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
        
        if(!_isSliding && !_isBouncing)
        {
            w_n1 = -Vector3.Cross(v_n1, Vector3.up) / radius;
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
        
        if(!_isSliding && !_isBouncing)
        {
            w_n1 = -Vector3.Cross(v_n1, Vector3.up) / radius;
            theta_n1 = Quaternion.Euler(Mathf.Rad2Deg * deltaTime * w_n1) * theta_n;;
        }
    }
    
    private void CheckCollision(float deltaTime)
    {
        if (p_n1.y - radius <= 0f && _isBouncing && v_n1.y < 0)
        {
            if (!_collisionAtFrameBefore)
            {
                ContinuousCollisionDetection(deltaTime / 2, deltaTime / 2);
                _collisionAtFrameBefore = true;
            }
            else
            {
                CalculateTotalForces(v_n1, w_n1);
                a_n = _totalForces / mass;
                v_n1.y = 0;
                p_n1.y = radius;
                _isBouncing = false;
            }
        }
        else
        {
            _collisionAtFrameBefore = false;
        }
    }

    private void CheckPureRolling()
    {
        if(!_isBouncing && !_stoppedSliding){
            if ((v_n - Vector3.Cross(w_n * radius, Vector3.up)).magnitude <= 1e-2)
            {
                _isSliding = false;
                _stoppedSliding = true;
                
                if(!precompute && !targetKickTesting)
                    GetComponent<TrajectoryTracker>().stopSlidingPoint.Add(p_n1);
            }
            else
            {
                _isSliding = true;
            }
        }
    }
    
private void CheckAndReset(ref Vector3 vector)
{
    var threshold = 1e-2;
    if (vector.sqrMagnitude < threshold * threshold)
    {
        vector = Vector3.zero;
    }
}

    public Vector3 Position()
    {
        if (frameByFrame)
        {
            return _frames[frameCount].p_n;
        }
        return p_n;
    }
    
    public Vector3 Velocity()
    {
        if(frameByFrame)
        {
            return _frames[frameCount].v_n;
        }
        return v_n;
    }
    
    public Vector3 AngularVelocity()
    {
        if(frameByFrame)
        {
            return _frames[frameCount].w_n;
        }
        return w_n;
    }

    public Vector3 Acceleration()
    {
        if(frameByFrame)
        {
            return _frames[frameCount].a_n;
        }
        return a_n;
    }
    
    public float PotentialEnergy()
    {
        if(frameByFrame)
        {
            return _frames[frameCount].potentialEnergy;
        }
        return _potentialEnergy;
    }

    public float KineticEnergy()
    {
        if(frameByFrame)
        {
            return _frames[frameCount].kineticEnergy;
        }
        return _kineticEnergy;
    }

    public float RotationalEnergy()
    {
        if(frameByFrame)
        {
            return _frames[frameCount].rotationalEnergy;
        }
        return _rotationalEnergy;
    }

    public float TotalEnergy()
    {
        if(frameByFrame)
        {
            return _frames[frameCount].totalEnergy;
        }
        return _totalEnergy;
    }

    private float CalculatePotentialEnergy()
    {
        if(!_isBouncing)
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
        if(_isBouncing)
            return Vector3.zero;
        
        _normalForce = (Gravity * mass - _magnusForce.y) * Vector3.up;
        return _normalForce;
    }

    private Vector3 CalculateFrictionForce(Vector3 velocity, Vector3 angularVelocity)
    {
        if(_isBouncing)
            return Vector3.zero;

        if(_isSliding)
        {
            //sliding friction
            if ((velocity - Vector3.Cross(angularVelocity * radius, Vector3.up)).magnitude > 1e-2)
            {
                var direction = (velocity - Vector3.Cross(angularVelocity * radius, Vector3.up)).normalized;
                _previousSignOfFriction = _frictionForce;
                _frictionForce = -mass * Gravity * coefficientOfSlidingFriction * direction;
                if (Vector3.Dot(_frictionForce, _previousSignOfFriction) < 0)
                {
                    _frictionForce = Vector3.zero;
                    _isSliding = false;
                }
                
            }
            else
                _frictionForce = Vector3.zero;
            
            return _frictionForce;
        }
        
        //rolling friction, pure rolling
        if(velocity.magnitude > 1e-2)
        {
            _frictionForce = -coefficientOfRollingFriction * mass * Gravity * velocity.normalized;
            _frictionForce.y = 0;
        }
        else
            _frictionForce = Vector3.zero;

        
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
        if (!_isSliding)
            return Vector3.zero;
        
        var direction = (angularVelocity + Vector3.Cross(velocity, Vector3.up)/radius).normalized;
        var alpha = -3f/2f * coefficientOfSlidingFriction * Gravity / radius * direction;
        
        return alpha * _inertialMomentum;
    }
    
    private Vector3 CalculateDragTorque(Vector3 velocity, Vector3 angularVelocity)
    {
        if(angularDragCoefficient == 0f || angularVelocity.magnitude < 1e-2)
            return Vector3.zero;

        var vTPlus = velocity.magnitude + radius * Vector3.Dot(angularVelocity, velocity.normalized);
        var vTMinus = velocity.magnitude - radius * Vector3.Dot(angularVelocity, velocity.normalized);
        
        var angularDragCoefficientPlus = 0.0074f * Mathf.Pow(vTPlus * airDensity * radius*2, -1f/5f);
        var angularDragCoefficientMinus = 0.0074f * Mathf.Pow(vTMinus * airDensity * radius*2, -1f/5f);
            
        return -0.5f * angularDragCoefficient * airDensity * Mathf.Pow(radius, 5) * angularVelocity.magnitude * angularVelocity;
    }

    private void CalculateTotalTorque(Vector3 velocity, Vector3 angularVelocity)
    {
        _dragTorque = CalculateDragTorque(velocity, angularVelocity);
        _frictionTorque = CalculateFrictionTorque(velocity, angularVelocity);
        
        _totalTorque = _dragTorque + _frictionTorque;
    }

    public Vector3 TotalForces()
    {
        if(frameByFrame)
        {
            return _frames[frameCount].totalForces;
        }
        return _totalForces;
    }

    public Vector3 DragForce()
    {
        if(frameByFrame)
        {
            return _frames[frameCount].dragForce;
        }
        return _dragForce;
    }

    public Vector3 MagnusForce()
    {
        if(frameByFrame)
        {
            return _frames[frameCount].magnusForce;
        }
        return _magnusForce;
    }

    public Vector3 NormalForce()
    {
        if(frameByFrame)
        {
            return _frames[frameCount].normalForce;
        }
        return _normalForce;
    }

    public Vector3 FrictionForce()
    {
        if(frameByFrame)
        {
            return _frames[frameCount].frictionForce;
        }
        return _frictionForce;
    }

    public Vector3 FrictionTorque()
    {
        if(frameByFrame)
        {
            return _frames[frameCount].frictionTorque;
        }
        return _frictionTorque;
    }
    
    public Vector3 DragTorque()
    {
        if(frameByFrame)
        {
            return _frames[frameCount].dragTorque;
        }
        return _dragTorque;
    }
    
    public Vector3 TotalTorque()
    {
        if(frameByFrame)
        {
            return _frames[frameCount].totalTorque;
        }
        return _totalTorque;
    }

    public float TimeToCompute()
    {
        if(frameByFrame)
        {
            return _frames[frameCount].timeToCompute;
        }
        return _timeToCompute;
    }
    
    public float AverageTimeToCompute()
    {
        if (_frames == null || _frames.Count == 0)
            return 0;
        
        return _totalTimeToCompute / _frames.Count;
    }

    private void ContinuousCollisionDetection(float deltaTime, float step)
    {
        _collisionAtFrameBefore = false;
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
            GetComponent<TrajectoryTracker>().bouncedPoints.Add(p_n);
        
        if (v_n.y < 1e-2)
        {
            v_n.y = 0;
            p_n.y = radius;
            _isBouncing = false;
        }
        
        //this part calculates the trajectory of the ball starting from the moment when it touched the ground to the end of the frame

        var remainingTime = Time.fixedDeltaTime - bounceDeltaTime;
        
        IntegrateMotion(remainingTime);

        v_n = v_n1;
        w_n = w_n1;
        p_n = p_n1;
        
        if(!precompute && !targetKickTesting)
            GetComponent<TrajectoryTracker>().trajectoryPoints.Add(p_n);
    }
    
     private Vector3 FindInitialVelocity()
     { 
        var startTime = Time.realtimeSinceStartup;
        
        Vector3 direction = new Vector3(target.transform.position.x - p0.x, 0, target.transform.position.z - p0.z).normalized;
        direction = new Vector3(direction.x * Mathf.Cos(Mathf.PI / 12), Mathf.Sin(Mathf.PI / 12), direction.z * Mathf.Cos(Mathf.PI / 12));
        Vector3 velocity = direction * initialSpeed;
        
        var count = 0;
        var error = float.MaxValue;
        float angle = 0;
        float cosTheta = 0f;
        Quaternion rotation = Quaternion.identity;
        Vector3 rotationAxis = Vector3.up;

        float angle0 = 180f;
        float angle1 = 90f;
        
        float distance = Vector3.Distance(target.transform.position, p0);
        var maxError = distance < 25 ? precalculationMaxError : precalculationMaxError + Mathf.Pow(distance - 25,2) * 1e-4f;

        while (error > maxError && count < maxIterations)
        {
            _currentTrajectory = new List<Vector3>();
            
            var position = PrecomputeTrajectory(rotation * velocity);
            
            //check angle over ground plane is less than 30 degrees, otherwise stops calculation
            float dot = Vector3.Dot((rotation * velocity).normalized, Vector3.up);
            float totalAngle = 90f - Mathf.Acos(dot) * Mathf.Rad2Deg;
            if (totalAngle > 30f)
            {
                var rotatedVelocity = rotation * velocity;
                
                float velocityMagnitude = rotatedVelocity.magnitude;
                
                Vector2 xzProjection = new Vector2(rotatedVelocity.x, rotatedVelocity.z);
                float xzMagnitude = xzProjection.magnitude;

                float newY = Mathf.Tan(30 * Mathf.Deg2Rad) * xzMagnitude;
                
                velocity = new Vector3(rotatedVelocity.x, newY, rotatedVelocity.z).normalized * velocityMagnitude;

                _precalculationTrajectories.Add(_currentTrajectory);
                break;
            }
            
            velocity = rotation * velocity;
            
            error = Vector3.Distance(position, target.transform.position);
            
            Vector3 vTarget = (target.transform.position - p0).normalized;
            Vector3 vIteration = (position - p0).normalized;
            cosTheta = Vector3.Dot(vIteration, vTarget); 
            
            rotationAxis = Vector3.Cross(vIteration, vTarget).normalized;
            
            angle = Mathf.Acos(Mathf.Clamp(cosTheta, -1f, 1f)) * Mathf.Rad2Deg;
            
            if ((angle < 1e-3 || Mathf.Abs(angle0 - angle) < 1e-3) && error > maxError)
            {
                var random = new System.Random();
                velocity *= random.Next(2) == 0 ? 1.0005f : 0.9995f;
            }
            
            angle0 = angle1;
            angle1 = angle;
            
            rotation = Quaternion.AngleAxis(angle, rotationAxis);
            
            count++;
            
            _precalculationTrajectories.Add(_currentTrajectory);
        }
        
        
        var endTime = Time.realtimeSinceStartup;
        Debug.Log("Prediction duration: " + ((endTime - startTime)*1e3).ToString("F3") + " ms with " + count + " iterations");
        Debug.Log("Error: "+ (error*1e3).ToString("F3") + " mm");
        
        precompute = false;

        if (targetKickTesting)
        {
            _totalComputationTime += (endTime - startTime) * 1e3f;
            errors.Add(error*1e3f);
            startPositions.Add(_testCurrentPosition);
            distances.Add(Vector3.Distance(_testCurrentPosition, target.transform.position));
            iterations.Add(count);
            averageComputationTime = _totalComputationTime / (errors.Count);

            if(error*1e3f > 10f)
            {
                outliersStartPositions.Add(_testCurrentPosition);
                outliersError.Add(error*1e3f);
            }
        }
        
        return velocity;
     }
     
     private Vector3 FindInitialVelocityAndSpin()
     { 
        var startTime = Time.realtimeSinceStartup;
        
        Vector3 direction = new Vector3(target.transform.position.x - p0.x, 0, target.transform.position.z - p0.z).normalized;
        direction = new Vector3(direction.x * Mathf.Cos(Mathf.PI / 12), Mathf.Sin(Mathf.PI / 12), direction.z * Mathf.Cos(Mathf.PI / 12));
        Vector3 velocity = direction * initialSpeed;
        
        var count = 0;
        var error = float.MaxValue;
        float angle = 0;
        float cosTheta = 0f;
        Quaternion rotation = Quaternion.identity;
        Vector3 rotationAxis = Vector3.up;

        float angle0 = 180f;
        float angle1 = 90f;
        
        float distance = Vector3.Distance(target.transform.position, p0);
        var maxError = distance < 25 ? precalculationMaxError : precalculationMaxError + Mathf.Pow(distance - 25,2) * 1e-4f;

        while (error > maxError && count < maxIterations)
        {
            _currentTrajectory = new List<Vector3>();
            
            var position = PrecomputeTrajectory(rotation * velocity);
            
            //check angle over ground plane is less than 30 degrees, otherwise stops calculation
            float dot = Vector3.Dot((rotation * velocity).normalized, Vector3.up);
            float totalAngle = 90f - Mathf.Acos(dot) * Mathf.Rad2Deg;
            if (totalAngle > 30f)
            {
                var rotatedVelocity = rotation * velocity;
                
                float velocityMagnitude = rotatedVelocity.magnitude;
                
                Vector2 xzProjection = new Vector2(rotatedVelocity.x, rotatedVelocity.z);
                float xzMagnitude = xzProjection.magnitude;

                float newY = Mathf.Tan(30 * Mathf.Deg2Rad) * xzMagnitude;
                
                velocity = new Vector3(rotatedVelocity.x, newY, rotatedVelocity.z).normalized * velocityMagnitude;

                _precalculationTrajectories.Add(_currentTrajectory);
                break;
            }
            
            velocity = rotation * velocity;
            
            error = Vector3.Distance(position, target.transform.position);
            
            Vector3 vTarget = (target.transform.position - p0).normalized;
            Vector3 vIteration = (position - p0).normalized;
            cosTheta = Vector3.Dot(vIteration, vTarget); 
            
            rotationAxis = Vector3.Cross(vIteration, vTarget).normalized;
            
            angle = Mathf.Acos(Mathf.Clamp(cosTheta, -1f, 1f)) * Mathf.Rad2Deg;
            
            if ((angle < 1e-3 || Mathf.Abs(angle0 - angle) < 1e-3) && error > maxError)
            {
                var random = new System.Random();
                velocity *= random.Next(2) == 0 ? 1.0005f : 0.9995f;
            }
            
            angle0 = angle1;
            angle1 = angle;
            
            rotation = Quaternion.AngleAxis(angle, rotationAxis);
            
            count++;
            
            _precalculationTrajectories.Add(_currentTrajectory);
        }
        
        
        var endTime = Time.realtimeSinceStartup;
        Debug.Log("Prediction duration: " + ((endTime - startTime)*1e3).ToString("F3") + " ms with " + count + " iterations");
        Debug.Log("Error: "+ (error*1e3).ToString("F3") + " mm");
        
        precompute = false;

        if (targetKickTesting)
        {
            _totalComputationTime += (endTime - startTime) * 1e3f;
            errors.Add(error*1e3f);
            startPositions.Add(_testCurrentPosition);
            distances.Add(Vector3.Distance(_testCurrentPosition, target.transform.position));
            iterations.Add(count);
            averageComputationTime = _totalComputationTime / (errors.Count);

            if(error*1e3f > 10f)
            {
                outliersStartPositions.Add(_testCurrentPosition);
                outliersError.Add(error*1e3f);
            }
        }
        
        return velocity;
     }
     
    private Vector3 PrecomputeTrajectory(Vector3 velocity)
    {
        var stop = false;
        var deltaTime = 1f/precomputationFrameRate;
        
        v_n = velocity;
        p_n = transform.position;
        a_n = Vector3.zero;
        w_n1 = w0;
        w_n = w_n1;
        theta_n = Quaternion.identity;
        
        Vector3 targetXZ = new Vector3(velocity.x, 0f, velocity.z).normalized;
        Quaternion newRotation = Quaternion.LookRotation(Vector3.Cross(Vector3.up, targetXZ), Vector3.up);
        w_n = newRotation * w_n;
        w_n.y = w0.y; 

        _isBouncing = !(p_n.y <= radius);

        _isSliding = false;
        _stoppedSliding = false;
        Vector3 closestPoint = p_n;
        
        var savedP0 = p_n;
        var savedV0 = v_n;
        var savedW0 = w_n;
        var error0 = Vector3.Distance(p_n, target.transform.position);
        
        while(!stop)
        {
            IntegrateMotion(deltaTime);
            
            _currentTrajectory.Add(p_n1);
            
            var errorN = Vector3.Distance(p_n, target.transform.position);
            var errorN1 = Vector3.Distance(p_n1, target.transform.position);

            if (errorN <= error0 && errorN <= errorN1)
            {
                stop = true;
                
                if (error0 <= errorN1)
                {
                    closestPoint = FindClosestPoint(deltaTime / 2, deltaTime / 4, savedP0, savedV0, savedW0, float.MaxValue);
                }
                else
                {
                    closestPoint = FindClosestPoint(deltaTime / 2, deltaTime / 4, p_n, v_n, w_n, float.MaxValue);
                }
            }
            
            savedP0 = p_n;
            savedV0 = v_n;
            savedW0 = w_n;
            error0 = errorN;

            v_n = v_n1;
            p_n = p_n1;
            w_n = w_n1;
        }

        GetComponent<TrajectoryTracker>().stopSlidingPoint.Add(closestPoint);
        return closestPoint;
    }
    
    private Vector3 FindClosestPoint(float deltaTime, float step , Vector3 p, Vector3 v, Vector3 w, float error)
    {
        p_n = p;
        v_n = v;
        w_n = w;
        IntegrateMotion(deltaTime - step);
        var p1 = p_n1;
        var distance1 = Vector3.Distance(p1, target.transform.position);
        
        IntegrateMotion(deltaTime + step);
        var p2 = p_n1;
        var distance2 = Vector3.Distance(p2, target.transform.position);
        
        Vector3 closestPoint;
        
        if (distance1 < distance2) 
        {
            
            if (step < CollisionDetectionError)
            {
                var correctDeltaTime = (deltaTime - step);

                IntegrateMotion(correctDeltaTime);
                
                return p_n1;
            }
            
            closestPoint = FindClosestPoint(deltaTime - step, step/2, p, v, w, error);
        }
        else
        {
            if (step < CollisionDetectionError)
            {
                var correctDeltaTime = (deltaTime + step);

                IntegrateMotion(correctDeltaTime);

                return p_n1;
            }

            closestPoint = FindClosestPoint(deltaTime + step, step/2, p, v, w, error);
        }

        return closestPoint;
    }
    
    private void CheckDistanceFromTarget(float deltaTime)
    {
        Vector3 pSaved = p_n1;
        Vector3 vSaved = v_n1;
        Vector3 wSaved = w_n1;
        
        Vector3 planePoint = target.transform.position; // Plane passes through the target
        Vector3 planeNormal = (p0 - planePoint).normalized; // Normal points towards startPosition
        
        float distanceBefore = Vector3.Dot(p_n - planePoint, planeNormal);
        float distanceAfter = Vector3.Dot(p_n1 - planePoint, planeNormal);
        
        if (distanceBefore * distanceAfter < 0) // Sign change -> crossing the plane
        
        {

            Vector3 pointOnLine = FindClosestPoint(deltaTime /2 , deltaTime / 4, p_n, v_n, w_n, float.MaxValue);
            
            Debug.Log("Runtime error: " + (Vector3.Distance(target.transform.position, pointOnLine)*1e3).ToString("F3") + " mm");
            
            Instantiate(placeholder, pointOnLine, Quaternion.identity);
            
            distanceToTargetChecked = true;
        }

        p_n1 = pSaved;
        v_n1 = vSaved;
        w_n1 = wSaved;
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

public class Frame
{
    public Vector3 position;
    public Quaternion rotation;

    public Vector3 p_n;
    public Vector3 v_n;
    public Vector3 a_n;
    public Vector3 w_n;

    public float potentialEnergy;
    public float kineticEnergy;
    public float rotationalEnergy;
    public float totalEnergy;

    public Vector3 dragForce;
    public Vector3 magnusForce;
    public Vector3 normalForce;
    public Vector3 frictionForce;
    public Vector3 totalForces;

    public Vector3 frictionTorque;
    public Vector3 dragTorque;
    public Vector3 totalTorque;

    public float timeToCompute;


    public Frame(Vector3 position, Quaternion rotation, Vector3 p_n, Vector3 v_n, Vector3 a_n, Vector3 w_n, float potentialEnergy, float kineticEnergy, float rotationalEnergy, float totalEnergy, Vector3 dragForce, Vector3 magnusForce, Vector3 normalForce, Vector3 frictionForce, Vector3 totalForces, Vector3 frictionTorque, Vector3 dragTorque, Vector3 totalTorque, float timeToCompute)
    {
        this.position = position;
        this.rotation = rotation;
        this.p_n = p_n;
        this.v_n = v_n;
        this.a_n = a_n;
        this.w_n = w_n;
        this.potentialEnergy = potentialEnergy;
        this.kineticEnergy = kineticEnergy;
        this.rotationalEnergy = rotationalEnergy;
        this.totalEnergy = totalEnergy;
        this.dragForce = dragForce;
        this.magnusForce = magnusForce;
        this.normalForce = normalForce;
        this.frictionForce = frictionForce;
        this.totalForces = totalForces;
        this.frictionTorque = frictionTorque;
        this.dragTorque = dragTorque;
        this.totalTorque = totalTorque;
        this.timeToCompute = timeToCompute;
    }
    
}


