using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Unity.VisualScripting;
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
    private Vector3 alpha_n; //angularAcceleration_n
    
    private bool _isBouncing;
    private bool _isSliding;
    private bool _stoppedSliding;
    private bool _collisionAtFrameBefore;
    
    [SerializeField] private float radius;
    [SerializeField] private float mass;
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

    [SerializeField] [Range(0, 1)] private float coefficientOfVerticalRestitution;
    [SerializeField] [Range(-1, 1)] private float coefficientOfHorizontalRestitution;
    [SerializeField] [Range(-1, 1)] private float coefficientRotationRestitutionYaxis;
    [SerializeField] [Range(0, 1)] private float coefficientOfSlidingFriction;
    [SerializeField] [Range(0, 1)] private float coefficientOfRollingFriction; 
    [SerializeField] private float airDensity;
    [SerializeField] private float dragCoefficient;
    [SerializeField] private bool customDragCoefficient;
    [SerializeField] private float vc;
    [SerializeField] private float vs;
    [SerializeField] private float kinematicViscosityOfAir = 1.48f * 1e-5f;
    [SerializeField] private float angularDragCoefficient;
    [SerializeField] private float liftCoefficient;

    private Vector3 _p0;
    [SerializeField] private Vector3 v0;
    [SerializeField] private Vector3 w0;

    public bool frameByFrame;
    [Range(0,500)] public int frameCount;
    private List<Frame> _frames;
    private float _timeToCompute;
    private float _totalTimeToCompute = 0f;
    private int _frameCounter = 0;

    public bool precompute;
    [SerializeField] private float initialSpeed;

    [SerializeField] private GameObject comparisonSphere;
    [SerializeField] private GameObject target;
    
    [SerializeField] private bool velocityVerlet;
    [SerializeField] private bool explicitEuler;
    [SerializeField] private bool RK2;
    [SerializeField] private bool RK4;
    [SerializeField] private float frameRate;
    
    void Awake()
    {
        Application.targetFrameRate = -1;
        Time.fixedDeltaTime = 1/frameRate;
        _frames = new List<Frame>();
        
        _p0 = transform.position;
        p_n = transform.position;
        v_n = v0;
        a_n = Vector3.zero;
        w_n1 = w0;
        w_n = w_n1;
        alpha_n = Vector3.zero;
        
        transform.localScale = new Vector3(radius, radius, radius)*2;

        _isBouncing = !(p_n.y <= radius);
        _collisionAtFrameBefore = false;
        _isSliding = false;
        _stoppedSliding = false;
        
        _inertialMomentum = 2f/3f * mass * radius * radius;
        
        if(comparisonSphere != null)
            comparisonSphere.GetComponent<Rigidbody>().AddForce(v_n,ForceMode.VelocityChange);
        
        if(precompute)
        {
            v_n = FindInitialVelocity(transform.position);
            p_n = transform.position;
            a_n = Vector3.zero;
            w_n1 = w0;
            w_n = w_n1;
            alpha_n = Vector3.zero;

            _isBouncing = !(p_n.y <= radius);

            _isSliding = false;
            _stoppedSliding = false;
        }
    }


    void FixedUpdate()
    {
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
        _frameCounter++;
        
        var startTime = Time.realtimeSinceStartup;
        
        CheckPureRolling();

        IntegrateMotion(deltaTime);
        
        CheckCollision(deltaTime);
        
        if (precompute)
            CheckDistanceFromTarget(deltaTime);
        
        v_n = v_n1;
        w_n = w_n1;
        p_n = p_n1;
        
        CalculateTotalEnergy();

        transform.position = p_n;
        
        var endTime = Time.realtimeSinceStartup;
        _timeToCompute = endTime - startTime;
        _totalTimeToCompute += _timeToCompute;  
        
        if (v_n.magnitude > 1e-3)
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
         transform.Rotate(Mathf.Rad2Deg * deltaTime * w_n1, Space.World);

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
        transform.Rotate(Mathf.Rad2Deg * deltaTime * w_n1, Space.World);
    }
    
    private void Runge_Kutta_2(float deltaTime)
        {
            //k1
            CalculateTotalForces(v_n, w_n);
            a_n = _totalForces / mass;
            var k1V = deltaTime * a_n;
            var k1P = deltaTime * v_n;
            
            //k2
            CalculateTotalForces(v_n + k1V, w_n);
            a_n = _totalForces / mass;
            var k2V = deltaTime * a_n;
            var k2P = deltaTime * (v_n + k1V);
            
            p_n1 = p_n + (k1P + k2P) / 2;
            v_n1 = v_n + (k1V + k2V) / 2;
        }
    
        private void Runge_Kutta_4(float deltaTime)
        {
            //k1
            CalculateTotalForces(v_n, w_n);
            a_n = _totalForces / mass;
            var k1V = deltaTime * a_n;
            var k1P = deltaTime * v_n;
            
            //k2
            CalculateTotalForces(v_n + k1V/2, w_n);
            a_n = _totalForces / mass;
            var k2V = deltaTime * a_n;
            var k2P = deltaTime * (v_n + k1V/2);
            
            //k3
            CalculateTotalForces(v_n + k2V/2, w_n);
            a_n = _totalForces / mass;
            var k3V = deltaTime * a_n;
            var k3P = deltaTime * (v_n + k2V/2);
            
            //k4
            CalculateTotalForces(v_n + k3V, w_n);
            a_n = _totalForces / mass;
            var k4V = deltaTime * a_n;
            var k4P = deltaTime * (v_n + k3V);
            
            p_n1 = p_n + (k1P + 2*k2P + 2*k3P + k4P) / 6;
            v_n1 = v_n + (k1V + 2*k2V + 2*k3V + k4V) / 6;
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
            if ((v_n - Vector3.Cross(w_n * radius, Vector3.up)).magnitude <= 1e-3)
            {
                _isSliding = false;
                _stoppedSliding = true;
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
    var threshold = 1e-3;
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
        if(velocity.magnitude > 1e-3)
            _frictionForce = - coefficientOfRollingFriction * mass * Gravity * velocity.normalized;
        else
            _frictionForce = Vector3.zero;

        _frictionForce.y = 0;
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
        alpha_n = -3f/2f * coefficientOfSlidingFriction * Gravity / radius * direction;

        return alpha_n * _inertialMomentum;
    }
    
    private Vector3 CalculateDragTorque(Vector3 angularVelocity)
    {
        if(angularDragCoefficient == 0f || angularVelocity.magnitude < 1e-2)
            return Vector3.zero;

        return -0.5f * angularDragCoefficient * airDensity * Mathf.Pow(radius, 5) * angularVelocity.magnitude * angularVelocity;
    }

    private void CalculateTotalTorque(Vector3 velocity, Vector3 angularVelocity)
    {
        _dragTorque = CalculateDragTorque(angularVelocity);
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
        /*
        CalculateTotalForces(v_n, w_n);
        
        a_n = _totalForces / mass;
        v_n1 = v_n + bounceDeltaTime/2 * a_n;
        p_n1 = p_n + bounceDeltaTime * v_n1;
        
        CalculateTotalForces(v_n1, w_n1);
        
        a_n = _totalForces / mass;
        v_n1 +=  bounceDeltaTime/2 * a_n;
        */
        
        IntegrateMotion(bounceDeltaTime);

        v_n = v_n1;
        w_n = w_n1;
        //p_n1.y = radius;
        p_n = p_n1;

        var alpha = 2f / 3f;
        v_n1.x = v_n.x * (1 - alpha * coefficientOfHorizontalRestitution)  / (alpha + 1) -
                 alpha * (1 + coefficientOfHorizontalRestitution) / (alpha + 1) * radius * w_n.z;

        v_n1.y = -coefficientOfVerticalRestitution * v_n.y;
        v_n1.z = v_n.z * (1 - alpha* coefficientOfHorizontalRestitution) / (alpha + 1) +
                 alpha * (1 + coefficientOfHorizontalRestitution) / (alpha + 1) * radius * w_n.x;

        w_n1.x = (alpha-coefficientOfHorizontalRestitution)/(alpha+1) * w_n.x + (coefficientOfHorizontalRestitution+1)/(alpha+1) * v_n.z/radius;
        w_n1.y = w_n.y * coefficientRotationRestitutionYaxis;
        w_n1.z = (alpha-coefficientOfHorizontalRestitution)/(alpha+1) * w_n.z - (coefficientOfHorizontalRestitution+1)/(alpha+1) * v_n.x/radius;

        GetComponent<TrajectoryTracker>().bouncedPoints.Add(p_n1);
        
        v_n = v_n1;
        w_n = w_n1;
        p_n = p_n1;
        
        if (v_n.y < 1e-2)
        {
            v_n.y = 0;
            p_n.y = radius;
            _isBouncing = false;
        }
        
        //this part calculates the trajectory of the ball starting from the moment when it touched the ground to the end of the frame

        var remainingTime = Time.fixedDeltaTime - bounceDeltaTime;

        /*
        //2ND ORDER VELOCITY VERLET

        //F_k
        CalculateTotalForces(v_n, w_n);
        CalculateTotalTorque(v_n, w_n);
        
        a_n = _totalForces / mass;

        //v_k+1/2
        v_n1 = v_n + remainingTime/2 * a_n;
        if(!_isBouncing)
        {
            if (_isSliding)
                w_n1 = w_n + remainingTime / 2 * _frictionTorque / _inertialMomentum;
            else
                w_n1 = -Vector3.Cross(v_n1, Vector3.up) / radius;
        }
        else
        {
            w_n1 = w_n + remainingTime / 2 * _dragTorque / _inertialMomentum;
        }

        //p_k+1
        p_n1 = p_n + remainingTime * v_n1;
        transform.Rotate(Mathf.Rad2Deg * remainingTime * w_n1, Space.World);

        //F_k+1
        CalculateTotalForces(v_n1, w_n1);
        CalculateTotalTorque(v_n1, w_n1);
        a_n =_totalForces  / mass;

        //v_k+1
        v_n1 +=  remainingTime/2 * a_n;
        if (!_isBouncing)
        {
            if (_isSliding)
                w_n1 += remainingTime / 2 * _frictionTorque / _inertialMomentum;
            else
                w_n1 = -Vector3.Cross(v_n1, Vector3.up) / radius;
        }
        else
        {
            w_n1 += remainingTime/ 2 * _dragTorque / _inertialMomentum;
        }
        */
        
        IntegrateMotion(remainingTime);

        v_n = v_n1;
        w_n = w_n1;
        p_n = p_n1;
    }
    
     private Vector3 FindInitialVelocity(Vector3 startPosition)
     { 
        var startTime = Time.realtimeSinceStartup;
        
        Vector2 direction = new Vector2(target.transform.position.x - startPosition.x, target.transform.position.z - startPosition.z).normalized;
        Vector3 velocity = direction * initialSpeed;
        
        var count = 0;
        var error = float.MaxValue;

        while (error > 1e-2 && count < 10)
        {
            var position = PrecomputeTrajectory(velocity);
            
            var vTarget = startPosition - target.transform.position;
            var vIteration = startPosition - position;
            error = Vector3.Distance(position, target.transform.position);
            
            var dotProduct = Vector3.Dot(vIteration, vTarget);
            float cosTheta = dotProduct / (vTarget.magnitude * vIteration.magnitude);
            
            var angle = Mathf.Acos(Mathf.Clamp(cosTheta, -1f, 1f)) * Mathf.Rad2Deg;
            Vector3 rotationAxis = Vector3.Cross(vIteration, vTarget).normalized;

            Quaternion rotation = Quaternion.AngleAxis(angle, rotationAxis);
            velocity = rotation * velocity;
            

            float dot = Vector3.Dot(velocity, Vector3.up);
            float totalAngle = 90f - Mathf.Acos(dot) * Mathf.Rad2Deg;

            if (totalAngle > 45)
            {
                break;
            }
            
            count++;
        }
        
        var endTime = Time.realtimeSinceStartup;
        Debug.Log("Prediction duration: " + ((endTime - startTime)*1e3).ToString("F3") + " ms with " + count + " iterations");
        Debug.Log("Error: "+ (error*1e3).ToString("F3") + " mm");
        
        return velocity;
     }
     
     private Vector3 PrecomputeTrajectory(Vector3 velocity)
    {
        var stop = false;
        var deltaTime = 1f/25f;

        v_n = velocity;
        p_n = transform.position;
        a_n = Vector3.zero;
        w_n1 = w0;
        w_n = w_n1;
        alpha_n = Vector3.zero;

        _isBouncing = !(p_n.y <= radius);

        _isSliding = false;
        _stoppedSliding = false;
        Vector3 pointOnLine = Vector3.zero;

        while(!stop && _isBouncing)
        {
            IntegrateMotion(deltaTime);

            if (p_n1.x > target.transform.position.x)
            {
                stop = true;
                var y = p_n.y + (target.transform.position.x - p_n.x) / (p_n1.x - p_n.x) * (p_n1.y - p_n.y);
                var z = p_n.z + (target.transform.position.x - p_n.x) / (p_n1.x - p_n.x) * (p_n1.z - p_n.z);
                
                pointOnLine = new Vector3(target.transform.position.x, y, z);
            }
            
            v_n = v_n1;
            w_n = w_n1;
            p_n = p_n1;
        }
        
        return pointOnLine;
    }

    private List<Vector3> trajectoryPoints  = new List<Vector3>();
    private float timeStep = 0.0005f;
    private int frame = 20000;
    
    private void CalculateTrajectory()
    {
        Vector3 position = transform.position;
        Vector3 velocity = v0;
        
        for (int i = 0; i < frame; i++)
        {
            float t = i * timeStep;
            
            float x = position.x + velocity.x * t;
            float z = position.z + velocity.z * t;
            float y = position.y + velocity.y * t - 0.5f * Gravity * t * t;

            // Update position
            trajectoryPoints.Add(new Vector3(x, y, z));

            // Stop if the projectile hits the ground (y ≤ 0)
            if (y <= radius) break;
        }
    }
    
    private void CheckDistanceFromTarget(float deltaTime)
    {
        if (p_n1.x >= target.transform.position.x)
        {
            float y = p_n.y + (target.transform.position.x - p_n.x) / (p_n1.x - p_n.x) * (p_n1.y - p_n.y);
            float z = p_n.z + (target.transform.position.x - p_n.x) / (p_n1.x - p_n.x) * (p_n1.z - p_n.z);
                
            Vector3 pointOnLine = new Vector3(target.transform.position.x, y, z);
            
            Debug.Log("Runtime error: " + (Vector3.Distance(target.transform.position, pointOnLine)*1e3).ToString("F3") + " mm");

            precompute = false;
        }
    }
    
    void OnValidate()
    {
       
        trajectoryPoints.Clear();
        CalculateTrajectory();
    }
    void OnDrawGizmos()
    {
        Gizmos.color = Color.green;
        for (int i = 0; i < trajectoryPoints.Count - 1; i++)
        {
            //Gizmos.DrawLine(trajectoryPoints[i], trajectoryPoints[i + 1]);
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


