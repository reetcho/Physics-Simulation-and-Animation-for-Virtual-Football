using System.Collections.Generic;
using UnityEngine;

public class TrajectoryTracker : MonoBehaviour
{
    private float sphereSize;
    public Queue<(Vector3 position, Color color)> trajectoryPoints = new Queue<(Vector3, Color)>();
    [SerializeField] [Range(0f, 10f)] private float factor;
    private const int MaxPoints = 1000; // Limite massimo di elementi

    void Start()
    {
        sphereSize = transform.lossyScale.x / 2;
    }

    void OnDrawGizmos()
    {
        foreach (var point in trajectoryPoints)
        {
            Gizmos.color = point.color; // Imposta il colore
            Gizmos.DrawSphere(point.position, sphereSize * factor); // Disegna la sfera
        }
    }

    // Metodo per aggiungere un punto alla coda
    public void AddPoint(Vector3 position, Color color)
    {
        // Se la coda ha raggiunto il limite massimo, rimuovi l'elemento più vecchio
        if (trajectoryPoints.Count >= MaxPoints)
        {
            trajectoryPoints.Dequeue(); // Rimuovi l'elemento più vecchio
        }

        // Aggiungi il nuovo punto
        trajectoryPoints.Enqueue((position, color));
    }
}