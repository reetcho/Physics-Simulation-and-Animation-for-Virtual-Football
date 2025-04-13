using UnityEngine;

public class Frame
{
    public Vector3 position;
    public Quaternion rotation;

    public Vector3 p_n;
    public Vector3 v_n;
    public Vector3 a_n;
    public Vector3 w_n;
    
    public Ball ball;

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
    public Vector3 totalTorque;

    public float timeToCompute;


    public Frame(Vector3 position, Quaternion rotation, Vector3 p_n, Vector3 v_n, Vector3 a_n, Vector3 w_n, float potentialEnergy, float kineticEnergy, float rotationalEnergy, float totalEnergy, Vector3 dragForce, Vector3 magnusForce, Vector3 normalForce, Vector3 frictionForce, Vector3 totalForces, Vector3 frictionTorque, Vector3 totalTorque, float timeToCompute)
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
        this.totalTorque = totalTorque;
        this.timeToCompute = timeToCompute;
    }
    
}