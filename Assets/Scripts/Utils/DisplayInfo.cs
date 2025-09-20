
#if UNITY_EDITOR
using UnityEngine;
using UnityEditor;

[InitializeOnLoad]
public class DisplayInfo
{
    private static Simulation _simulation;

    static DisplayInfo()
    {
        SceneView.duringSceneGui += OnSceneGUI;
    }

    private static void OnSceneGUI(SceneView sceneView)
    {
        if(!Application.isPlaying)
            return;
        
        if (_simulation == null)
        {
            _simulation = Object.FindObjectOfType<Simulation>(); 
            if (_simulation == null) return;
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
        Rect frictionForceRect = new Rect(10, 230, 200, 30);
        Rect totalForceRect = new Rect(10, 250, 200, 30);
        
        Rect frictionTorque = new Rect(10, 280, 200, 30);
        Rect dragTorque = new Rect(10, 300, 200, 30);
        Rect totalTorques = new Rect(10, 320, 200, 30);
        
        Rect timeToCompute = new Rect(10, 350, 200, 30);

        GUI.Label(positionRect, "Position: " + _simulation.Position() + " m", textStyle);
        GUI.Label(velocityRect, "Velocity: " + _simulation.Velocity() + " m/s", textStyle);
        GUI.Label(angularVelocityRect, "Angular velocity: " + _simulation.AngularVelocity() + " rad/s", textStyle);
        GUI.Label(accelerationRect, "Acceleration: " + _simulation.Acceleration() + " m/s^2", textStyle);
        
        textStyle.normal.textColor = Color.magenta;
        GUI.Label(potentialRect, "Potential energy: " + _simulation.PotentialEnergy() + " J", textStyle);
        GUI.Label(kineticRect, "Kinetic energy: " + _simulation.KineticEnergy() + " J", textStyle);
        GUI.Label(rotationRect, "Rotational energy: " + _simulation.RotationalEnergy() + " J", textStyle);
        GUI.Label(totalEnergyRect, "Total energy: " + _simulation.TotalEnergy() + " J", textStyle);
        
        textStyle.normal.textColor = Color.green;
        GUI.Label(dragForceRect, "Drag force: " + _simulation.DragForce() + " N", textStyle);
        GUI.Label(magnusForceRect, "Magnus force: " + _simulation.MagnusForce() + " N", textStyle);
        GUI.Label(frictionForceRect, "Friction force: " + _simulation.FrictionForce() + " N", textStyle);
        GUI.Label(totalForceRect, "Total forces: " + _simulation.TotalForces() + " N", textStyle);
        
        textStyle.normal.textColor = Color.blue;
        GUI.Label(frictionTorque, "Friction torque: " + _simulation.FrictionTorque() + " mN * m", textStyle);
        GUI.Label(dragTorque, "Drag torque: " + _simulation.DragTorque()*1e3f + " mN * m", textStyle);
        GUI.Label(totalTorques, "Total torques: " + _simulation.TotalTorque() + " mN * m", textStyle);
        
        textStyle.normal.textColor = Color.black;
        GUI.Label(timeToCompute, "Time to compute: " +  _simulation.TimeToCompute() * 1e3f + " μs", textStyle);
        
        Handles.EndGUI();
    }
}
#endif

