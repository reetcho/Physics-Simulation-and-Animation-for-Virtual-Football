using System;
using UnityEditor;
using UnityEngine;
    
public class Test : MonoBehaviour
{
    [SerializeField] private BallPhysics ballPhysics;
    
    [SerializeField] private String description;
    [SerializeField] private String fileName;
    [SerializeField] private int numberOfSteps;
    
    [Header("Test parameters")]
    [SerializeField] private Vector2 startPosition;
    [SerializeField] private Vector2 endPosition;
    [SerializeField] private float speed;
    [SerializeField] private Vector3 spin;
    
    [Header("Constraint test parameters")]
    [SerializeField] private float minInitialSpeed;
    [SerializeField] private float maxInitialSpeed;

    public void StartTest()
    {
        if(Application.isPlaying)
            ballPhysics.StartTestForTargetedKick(description, fileName, numberOfSteps, new Vector3(startPosition.x, ballPhysics.ball.radius, startPosition.y), new Vector3(endPosition.x, ballPhysics.ball.radius, endPosition.y), speed, spin);
    }
    
    public void StartConstraintTest()
    {
        if(Application.isPlaying)
            ballPhysics.StartTestForTargetedKickWithConstraint(description, fileName, numberOfSteps, minInitialSpeed, maxInitialSpeed);
    }
    
    public void StopTest()
    {
        if(Application.isPlaying)
            ballPhysics.StopTest();
    }
}

[CustomEditor(typeof(Test))]
public class MyComponentEditor : Editor
{
    public override void OnInspectorGUI()
    {
        base.OnInspectorGUI();

        if (GUILayout.Button("Start test"))
        {
            Test test = (Test)target;
            test.StartTest();
        }
        
        if (GUILayout.Button("Start constraint test"))
        {
            Test test = (Test)target;
            test.StartConstraintTest();
        }
        
        if (GUILayout.Button("Stop test"))
        {
            Test test = (Test)target;
            test.StopTest();
        }
    }
}