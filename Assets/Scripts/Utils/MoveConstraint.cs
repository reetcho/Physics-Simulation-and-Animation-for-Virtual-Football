using UnityEngine;
using UnityEngine.InputSystem;

public class MoveConstraint : MonoBehaviour
{
    [SerializeField] GameObject target;
    public float yMin, yMax, zMin, zMax;
    public float speed = 10;

    void Update()
    {
        if (Gamepad.current != null)
        {
            Vector2 stickInput = Gamepad.current.leftStick.ReadValue();
            if (stickInput.magnitude > 0.1f)
            {
                Vector2 movement = speed * Time.deltaTime * stickInput;
                Vector3 newPosition = target.transform.localPosition + new Vector3(0, movement.y, -movement.x);

                newPosition.y = Mathf.Clamp(newPosition.y, yMin, yMax);
                newPosition.z = Mathf.Clamp(newPosition.z, zMin, zMax);

                target.transform.localPosition = newPosition;
            }
        }
    }
}
