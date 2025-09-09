using UnityEngine;

public class CameraLookAtTarget : MonoBehaviour
{
    [SerializeField] private Transform target;
    [SerializeField] private Camera cam;
    [SerializeField] private float minFOV = 30f;
    [SerializeField] private float maxFOV = 90f;
    [SerializeField] private float minSize = 5f;
    [SerializeField] private float maxSize = 20f;
    [SerializeField] private float minDistance = 2f;
    [SerializeField] private float maxDistance = 50f;

    void Start()
    {
        if (target != null)
        {
            transform.LookAt(target);
        }
    }

    void LateUpdate()
    {
        if (target == null || cam == null) return;

        // Look at the target
        transform.LookAt(target);

        // Calculate distance
        float distance = Vector3.Distance(transform.position, target.position);
        float t = Mathf.InverseLerp(minDistance, maxDistance, distance);

        // Adjust zoom based on camera type
        if (cam.orthographic)
        {
            cam.orthographicSize = Mathf.Lerp(minSize, maxSize, t);
        }
        else
        {
            cam.fieldOfView = Mathf.Lerp(minFOV, maxFOV, t);
        }
    }
}