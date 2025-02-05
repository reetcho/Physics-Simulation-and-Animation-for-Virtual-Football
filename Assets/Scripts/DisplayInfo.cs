using UnityEngine;
using UnityEditor;
using System.Collections.Generic;

[InitializeOnLoad]
public class DisplayInfo
{
    private static Ball ball;

    static DisplayInfo()
    {
        SceneView.duringSceneGui += OnSceneGUI;
    }

    private static void OnSceneGUI(SceneView sceneView)
    {
        if (ball == null)
        {
            ball = Object.FindObjectOfType<Ball>(); 
            if (ball == null) return;
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
        Rect totalForceRect = new Rect(10, 250, 200, 30);

        GUI.Label(positionRect, "Position: " + ball.Position(), textStyle);
        GUI.Label(velocityRect, "Velocity: " + ball.Velocity(), textStyle);
        GUI.Label(angularVelocityRect, "Angular velocity: " + ball.AngularVelocity(), textStyle);
        GUI.Label(accelerationRect, "Acceleration: " + ball.Acceleration(), textStyle);
        
        textStyle.normal.textColor = Color.magenta;
        GUI.Label(potentialRect, "Potential energy: " + ball.PotentialEnergy(), textStyle);
        GUI.Label(kineticRect, "Kinetic energy: " + ball.KineticEnergy(), textStyle);
        GUI.Label(rotationRect, "Rotational energy: " + ball.RotationalEnergy(), textStyle);
        GUI.Label(totalEnergyRect, "Total energy: " + ball.TotalEnergy(), textStyle);
        
        textStyle.normal.textColor = Color.green;
        GUI.Label(dragForceRect, "Drag force: " + ball.DragForce(), textStyle);
        GUI.Label(magnusForceRect, "Magnus force: " + ball.MagnusForce(), textStyle);
        GUI.Label(normalForceRect, "Normal force: " + ball.NormalForce(), textStyle);
        GUI.Label(totalForceRect, "Total forces: " + ball.TotalForces(), textStyle);
        
        Handles.EndGUI();
    }
}

[InitializeOnLoad]
public static class SphereProjectionEditor
{
    private static Ball ball; 
    private static Transform groundTransform; 

    static SphereProjectionEditor()
    {
        SceneView.duringSceneGui += OnSceneGUI;
    }

    private static void OnSceneGUI(SceneView sceneView)
    {
        if (ball == null || groundTransform == null)
        {
            ball = GameObject.Find("Ball").GetComponent<Ball>(); 
            GameObject ground = GameObject.Find("Ground"); 
            if (ground) groundTransform = ground.transform;

            if (ball == null || groundTransform == null) return;
        }

        Vector3 planeNormal = groundTransform.up; 
        Vector3 planePoint = groundTransform.position; 
        Vector3 sphereCenter = ball.Position();
        float sphereRadius = ball.Radius(); 

        Vector3 projectionCenter = sphereCenter - planeNormal * Vector3.Dot(sphereCenter - planePoint, planeNormal);

        Handles.color = Color.magenta;
        Handles.DrawSolidDisc(projectionCenter, planeNormal, sphereRadius);
    }
    
}

[RequireComponent(typeof(LineRenderer))]
public class PersistentTrajectory : MonoBehaviour
{
    private LineRenderer lineRenderer;
    private List<Vector3> trajectoryPoints = new List<Vector3>();
    private static Ball ball; 

    void Start()
    {
        lineRenderer = GetComponent<LineRenderer>();
        lineRenderer.startWidth = 0.05f;
        lineRenderer.endWidth = 0.05f;
        lineRenderer.positionCount = 0;
        ball = GameObject.Find("Ball").GetComponent<Ball>();
    }

    void Update()
    {
        trajectoryPoints.Add(ball.Position());
        lineRenderer.positionCount = trajectoryPoints.Count;
        lineRenderer.SetPositions(trajectoryPoints.ToArray());
    }
}

[InitializeOnLoad]
public static class TrajectoryEditor
{
    private static Ball ball; // Reference to your Ball object
    private static float timeStep = 0.05f; // Time step per iteration

    static TrajectoryEditor()
    {
        SceneView.duringSceneGui += OnSceneGUI;
    }

    private static void OnSceneGUI(SceneView sceneView)
    {
        if (ball == null)
        {
            ball = Object.FindObjectOfType<Ball>(); // Find the Ball in the scene
            if (ball == null) return; // Exit if no Ball found
        }

        Handles.color = Color.green; // Set trajectory color
        List<Vector3> trajectoryPoints = CalculateTrajectory(ball.Position(), ball.Velocity(), Physics.gravity);

        Handles.DrawAAPolyLine(3, trajectoryPoints.ToArray());
    }

    private static List<Vector3> CalculateTrajectory(Vector3 startPos, Vector3 startVel, Vector3 gravity)
    {
        List<Vector3> points = new List<Vector3>();
        Vector3 position = startPos;
        Vector3 velocity = startVel;

        while (position.y >= 0)
        {
            points.Add(position);
            velocity += gravity * timeStep;
            position += velocity * timeStep;
        }

        return points;
    }
}