using UnityEditor;
using UnityEngine;


[ExecuteInEditMode]
public class GridMeterLabels : MonoBehaviour
{
    // Set how many meters you want to show along the X and Y axes
    public float gridSpacing = 10.0f;
    public float maxX = 200;


    private void OnDrawGizmos()
    {
        DrawGridLabels();
    }

    private void DrawGridLabels()
    {
        // Loop through the grid and draw labels every meter
        for (float x = -maxX; x <= maxX; x += gridSpacing)
        {
                Vector3 position = new Vector3(x, -0.5f, 0);
                Handles.Label(position, $"{x} m");
        }
        
        for (float y = -maxX; y <= maxX; y += gridSpacing)
        {
            Vector3 position = new Vector3(-0.5f, 0, y);
            Handles.Label(position, $"{y} m");
        }
    }
}