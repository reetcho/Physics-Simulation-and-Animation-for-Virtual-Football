using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TMPro;

public class Ball : MonoBehaviour
{
    private Vector3 p_n; //position_n
    private Vector3 p_n1;//position_n+1
    private Vector3 v_n; //velocity_n
    private Vector3 v_n1;//velocity_n+1
    private Vector3 a_n; //acceleration_n
    private Vector3 w_n; //angularVelocity_n
    private Vector3 w_n1;//angularVelocity_n+1
    private bool _isBouncing;
    
    [SerializeField] private float radius;
    [SerializeField] private float mass;
    private float _inertialMomentum; //assumption of thin sphere -> mass * radius^2

    private float _potentialEnergy;
    private float _kineticEnergy;
    private float _rotationalEnergy;
    private float _totalEnergy;

    private Vector3 _dragForce;
    private Vector3 _magnusForce;
    private Vector3 _normalForce;
    private Vector3 _totalForces;
    
    private const float Gravity = 9.81f; 
    private const float CoefficientOfSlidingFriction = 0.5f;
    private const float CollisionDetectionError = 1e-7f;

    [SerializeField] [Range(0, 1)] private float coefficientOfRestitution;
    [SerializeField] [Range(0, 1)] private float coefficientOfSlidingFriction;
    [SerializeField] private float airDensity;
    [SerializeField] private float dragCoefficient;
    [SerializeField] private float liftCoefficient;
    [SerializeField] private float stiffness;
    private float _crossSectArea;
    private float _springOscillationSpeed;
    private float _contactDuration;
    private float _contactTime;
    private bool _contact;
    private float _maxDeformation;
    
    public Vector3 p0;
    public Vector3 v0;
    public Vector3 w0;
    
    float _deltaTime;
    
    void Awake()
    {
        Application.targetFrameRate = -1;
        
        p_n = transform.position;
        p0 =  p_n;
        v_n = v0;
        a_n = Vector3.down * Gravity;
        w_n = w0;
        transform.localScale = new Vector3(radius*2, radius*2, radius*2);
        
        _isBouncing = true;

        _inertialMomentum = 2f/3f * mass * radius * radius;
        _crossSectArea = Mathf.PI * radius * radius;
        _contactDuration = Mathf.PI * Mathf.Sqrt(mass / stiffness);
        _springOscillationSpeed = Mathf.Sqrt(stiffness / mass);
        _contact = false;
        _contactTime = 0f;
        
        _springOscillationSpeed = Mathf.Sqrt(stiffness / mass);
        
        CalculateTotalEnergy();
    }

    
    void Update()
    {
       
        _deltaTime = Time.deltaTime;

       _contactTime += Time.deltaTime;
       
       
        if (_contactTime >= _contactDuration)
        {
            _contact = false;
        }
        

        //implicit euler -> loses energy over time
        //a_n = Gravity * Vector3.down +(CalculateDragForce(v_n) + CalculateMagnusForce(v_n, w_n) + CalculateNormalForce()) / mass;
        //v_n1 = v_n + a_n * Time.deltaTime;
        ///p_n1 = p_n + v_n1 * Time.deltaTime;
        
        //velocity verlet
        a_n = _contact ? Gravity * Vector3.down + (CalculateDragForce(v_n) + CalculateMagnusForce(v_n, w_n) + CalculateNormalForce()) / mass : Gravity * Vector3.down + (CalculateDragForce(v_n) + CalculateMagnusForce(v_n, w_n)) / mass;
        v_n1 = v_n + Time.deltaTime/2 * a_n;
        p_n1 = p_n + Time.deltaTime * v_n1;
        a_n = _contact ? Gravity * Vector3.down + (CalculateDragForce(v_n1) + CalculateMagnusForce(v_n1, w_n) + CalculateNormalForce()) / mass : Gravity * Vector3.down + (CalculateDragForce(v_n1) + CalculateMagnusForce(v_n1, w_n)) / mass;

        if(_contact)
            Debug.Log("contact time: " + _contactTime + " contact duration: " + _contactDuration +  "nomral force: " + _normalForce);
        
        v_n1 +=  Time.deltaTime/2 * a_n;
        w_n1 = w_n;
        
        if (p_n1.y - radius <= 0f && _isBouncing && !_contact && v_n1.y < 0)
        {
            ContinuousCollisionDetection(Time.deltaTime / 2, Time.deltaTime / 2, 1);
        }
        
        v_n = v_n1;
        w_n = w_n1;
        p_n = p_n1;

        CalculateTotalEnergy();
        
        Quaternion deltaRotation = Quaternion.Euler(w_n * Time.deltaTime * Mathf.Rad2Deg);
        transform.localRotation *= deltaRotation;
        transform.position = p_n1;
    }

    public Vector3 Position()
    {
        return p_n;
    }
    
    public Vector3 Velocity()
    {
        return v_n;
    }
    
    public Vector3 AngularVelocity()
    {
        return w_n;
    }

    public Vector3 Acceleration()
    {
        return a_n;
    }

    public float Radius()
    {
        return radius;
    }

    public float PotentialEnergy()
    {
        return _potentialEnergy;
    }

    public float KineticEnergy()
    {
        return _kineticEnergy;
    }

    public float RotationalEnergy()
    {
        return _rotationalEnergy;
    }

    public float TotalEnergy()
    {
        return _totalEnergy;
    }

    private float CalculatePotentialEnergy()
    {
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
    

    private Vector3 CalculateDragForce(Vector3 velocity)
    {
        _dragForce = - 0.5f * airDensity * dragCoefficient * _crossSectArea * velocity.magnitude * velocity;
        return _dragForce;
    }
    
    private Vector3 CalculateMagnusForce(Vector3 velocity, Vector3 angularVelocity)
    {
        _magnusForce = 16f/3f * Mathf.Pow(Mathf.PI, 2) * Mathf.Pow(radius, 3) * airDensity * liftCoefficient * Vector3.Cross(angularVelocity, velocity);
        return _magnusForce;
    }

    private Vector3 CalculateNormalForce()
    {
        if (!_contact)
        {
            _normalForce = Vector3.zero;
            return _normalForce;
        }
        
        var deformation = Math.Abs(_maxDeformation * Mathf.Sin(_springOscillationSpeed * _contactTime));
        
        _normalForce = stiffness * deformation * Vector3.up;
        return _normalForce;
    }

    public Vector3 TotalForces()
    {
        _totalForces = Gravity * Vector3.down * mass + _dragForce + _magnusForce + _normalForce;
        return _totalForces;
    }

    public Vector3 DragForce()
    {
        return _dragForce;
    }

    public Vector3 MagnusForce()
    {
        return _magnusForce;
    }


    public Vector3 NormalForce()
    {
        return _normalForce;
    }

    private void ContinuousCollisionDetection(float deltaTime, float step,  int iteration)
    {
        //implicit euler
        //a_n = Gravity * Vector3.down + (CalculateDragForce(v_n) + CalculateMagnusForce(v_n, w_n)) / mass;
        //var v_int = v_n + a_n * deltaTime;
        //var p_int = p_n + v_int * deltaTime;
        
        //velocity verlet
        a_n = Gravity * Vector3.down + (CalculateDragForce(v_n) + CalculateMagnusForce(v_n, w_n)) / mass;
        var v_int = v_n + deltaTime/2 * a_n;
        var p_int = p_n + deltaTime * v_int;
        
        //Debug.Log("CheckCollisionWithGround  step: " + step + " iteration: " + iteration + " error: " + (p_int.y - radius));
        
        if (p_int.y - radius <= 0f)
        {
            //collision
            if (step < CollisionDetectionError)
            {

                var collisionDeltaTime = (deltaTime - step / 2);
                //implicit euler
                //a_n = Gravity * Vector3.down + (CalculateDragForce(v_n) + CalculateMagnusForce(v_n, w_n)) / mass;
                //v_n1 = v_n + a_n * collisionDeltaTime;
                //p_n1 = p_n + v_n1 * collisionDeltaTime;
                
                //velocity verlet until collision time
                a_n = Gravity * Vector3.down + (CalculateDragForce(v_n) + CalculateMagnusForce(v_n, w_n)) / mass;
                v_n1 = v_n + collisionDeltaTime/2 *  a_n;
                p_n1 = p_n + collisionDeltaTime * v_n1;
                a_n = Gravity * Vector3.down + (CalculateDragForce(v_n1) + CalculateMagnusForce(v_n1, w_n)) / mass;
                v_n1 +=  collisionDeltaTime/2 *  a_n;

                //v_n1.y *= -coefficientOfRestitution;
                
                _maxDeformation = v_n1.y * Mathf.Sqrt(mass / stiffness);
                _contactTime = deltaTime - collisionDeltaTime;
                _contact = true;
                
                if (Math.Abs(v_n1.y) <= 1)
                {
                    v_n1.y = 0f;
                    a_n.y = 0;
                    _isBouncing = false;
                }
                
                //Debug.Log("Collision detected with error: " + (p_n1.y - radius));
                GetComponent<TrajectoryTracker>().bouncedPoints.Add(p_n1);
                return;
            }
            
            ContinuousCollisionDetection(deltaTime - step/2, step/2, iteration+1);
        }
        else
        {
            //no collision
            if (step < CollisionDetectionError)
            {
                var collisionDeltaTime = (deltaTime + step / 2);
                //implicit euler
                //a_n = Gravity * Vector3.down + (CalculateDragForce(v_n) + CalculateMagnusForce(v_n, w_n)) / mass;
                //v_n1 = v_n + a_n * collisionDeltaTime;
                //p_n1 = p_n + v_n1 * collisionDeltaTime;
                
                //velocity verlet until collision time
                a_n = Gravity * Vector3.down + (CalculateDragForce(v_n) + CalculateMagnusForce(v_n, w_n)) / mass;
                v_n1 = v_n + collisionDeltaTime/2 * a_n;
                p_n1 = p_n + collisionDeltaTime * v_n1;
                a_n = Gravity * Vector3.down + (CalculateDragForce(v_n1) + CalculateMagnusForce(v_n1, w_n)) / mass;
                v_n1 +=  collisionDeltaTime/2 * a_n;
                
                //v_n1.y *= -coefficientOfRestitution;

                _maxDeformation = v_n1.y * Mathf.Sqrt(mass / stiffness);
                _contactTime = 0f;
                _contact = true;

                if (Math.Abs(v_n1.y) <= 1e-2)
                {
                    v_n1.y = 0f;
                    a_n.y = 0;
                    _isBouncing = false; 
                }

                Debug.Log("Collision detected with error: " + Math.Abs(p_n1.y - radius));
                GetComponent<TrajectoryTracker>().bouncedPoints.Add(p_n1);
                return;
            }

            ContinuousCollisionDetection(deltaTime + step/2, step/2, iteration+1);
        }
    }
}
