using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.InputSystem;

public class OrbitingCamera : MonoBehaviour
{
    public Transform target;
    public float distance = 10.0f;
    public float height = 5.0f;
    public float rotationSpeed = 20.0f;
    public float verticalSpeed = 20.0f;
    
    private float currentRotationAngle = -90.0f;
    private float currentHeight = -0.5f;
    private Vector3 offset = Vector3.left;
    
    void Start()
    {
        transform.position = target.position + offset;
    }

    void LateUpdate()
    {
        if (target == null)
        {
            Debug.LogWarning("No target assigned for the orbiting camera.");
            return;
        }

        // Ottieni il movimento dalla levetta destra e dal mouse
        Vector2 lookInput = Vector2.zero;
        if (Gamepad.current != null)
        {
            lookInput = Gamepad.current.rightStick.ReadValue();
        }
        if (Mouse.current != null && Input.GetMouseButton(2) && !IsPointerOverUIElement())
        {
            lookInput += Mouse.current.delta.ReadValue();
        }

        // Aggiorna la rotazione e altezza della telecamera
        currentRotationAngle += lookInput.x * rotationSpeed * Time.deltaTime;
        currentHeight -= lookInput.y * verticalSpeed * Time.deltaTime;
        
        currentHeight = Mathf.Clamp(currentHeight, 0.1f, height);

        // Calcola la nuova posizione in orbita
        offset = new Vector3(
            Mathf.Sin(currentRotationAngle * Mathf.Deg2Rad) * distance,
            currentHeight,
            Mathf.Cos(currentRotationAngle * Mathf.Deg2Rad) * distance
        );
        // Aggiorna la posizione e l'orientamento della camera
        transform.position = target.position + offset;
        transform.LookAt(target);
    }

    private bool IsPointerOverUIElement()
    {
        EventSystem eventSystem = EventSystem.current;
        if (eventSystem == null) return false;
        return eventSystem.IsPointerOverGameObject();
    }
}