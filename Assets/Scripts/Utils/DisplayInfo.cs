using UnityEngine;
using UnityEditor;
using System.Collections.Generic;
using Unity.VisualScripting;

[InitializeOnLoad]
public class DisplayInfo
{
    private static BallPhysics _ballPhysics;

    static DisplayInfo()
    {
        SceneView.duringSceneGui += OnSceneGUI;
    }

    private static void OnSceneGUI(SceneView sceneView)
    {
        if(!Application.isPlaying)
            return;
        
        if (_ballPhysics == null)
        {
            _ballPhysics = Object.FindObjectOfType<BallPhysics>(); 
            if (_ballPhysics == null) return;
        }

        Handles.BeginGUI();

        GUIStyle textStyle = new GUIStyle();
        textStyle.normal.textColor = Color.red;
        textStyle.fontSize = 12;
        textStyle.fontStyle = FontStyle.Bold;

        Rect positionRect = new Rect(10, 10, 200, 30);
        Rect velocityRect = new Rect(10, 30, 200, 30);
        Rect angularVelocityRect = new Rect(10, 50, 200, 30);
        Rect accelerationRect = new Rect(10, 70, 200, 30);
        
        Rect potentialRect = new Rect(10, 100, 200, 30);
        Rect kineticRect = new Rect(10, 120, 200, 30);
        Rect rotationRect = new Rect(10, 140, 200, 30);
        Rect totalEnergyRect = new Rect(10, 160, 200, 30);
        
        Rect dragForceRect = new Rect(10, 190, 200, 30);
        Rect magnusForceRect = new Rect(10, 210, 200, 30);
        Rect normalForceRect = new Rect(10, 230, 200, 30);
        Rect frictionForceRect = new Rect(10, 250, 200, 30);
        Rect totalForceRect = new Rect(10, 270, 200, 30);
        
        Rect frictionTorque = new Rect(10, 300, 200, 30);
        Rect totalTorques = new Rect(10, 320, 200, 30);
        
        Rect timeToCompute = new Rect(10, 350, 200, 30);

        GUI.Label(positionRect, "Position: " + _ballPhysics.Position(), textStyle);
        GUI.Label(velocityRect, "Velocity: " + _ballPhysics.Velocity(), textStyle);
        GUI.Label(angularVelocityRect, "Angular velocity: " + _ballPhysics.AngularVelocity(), textStyle);
        GUI.Label(accelerationRect, "Acceleration: " + _ballPhysics.Acceleration(), textStyle);
        
        textStyle.normal.textColor = Color.magenta;
        GUI.Label(potentialRect, "Potential energy: " + _ballPhysics.PotentialEnergy(), textStyle);
        GUI.Label(kineticRect, "Kinetic energy: " + _ballPhysics.KineticEnergy(), textStyle);
        GUI.Label(rotationRect, "Rotational energy: " + _ballPhysics.RotationalEnergy(), textStyle);
        GUI.Label(totalEnergyRect, "Total energy: " + _ballPhysics.TotalEnergy(), textStyle);
        
        textStyle.normal.textColor = Color.green;
        GUI.Label(dragForceRect, "Drag force: " + _ballPhysics.DragForce(), textStyle);
        GUI.Label(magnusForceRect, "Magnus force: " + _ballPhysics.MagnusForce(), textStyle);
        GUI.Label(normalForceRect, "Normal force: " + _ballPhysics.NormalForce(), textStyle);
        GUI.Label(frictionForceRect, "Friction force: " + _ballPhysics.FrictionForce(), textStyle);
        GUI.Label(totalForceRect, "Total forces: " + _ballPhysics.TotalForces(), textStyle);
        
        textStyle.normal.textColor = Color.blue;
        GUI.Label(frictionTorque, "Friction torque: " + _ballPhysics.FrictionTorque(), textStyle);
        GUI.Label(totalTorques, "Total torques: " + _ballPhysics.TotalTorque(), textStyle);
        
        textStyle.normal.textColor = Color.black;
        GUI.Label(timeToCompute, "Time to compute: " +  _ballPhysics.TimeToCompute(), textStyle);
        
        Handles.EndGUI();
    }
}

[InitializeOnLoad]
public static class SphereProjectionEditor
{
    private static BallPhysics _ballPhysics; 
    private static Transform groundTransform; 

    static SphereProjectionEditor()
    {
        SceneView.duringSceneGui += OnSceneGUI;
    }

    private static void OnSceneGUI(SceneView sceneView)
    {
        if (_ballPhysics == null || groundTransform == null)
        {
            _ballPhysics = GameObject.Find("Ball").GetComponent<BallPhysics>(); 
            GameObject ground = GameObject.Find("Ground"); 
            if (ground) groundTransform = ground.transform;

            if (_ballPhysics == null || groundTransform == null) return;
        }
        
        Handles.color = Color.magenta;
    }
    
}
