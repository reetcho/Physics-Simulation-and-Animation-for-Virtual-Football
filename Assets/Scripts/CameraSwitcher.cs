using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.InputSystem;

public class CameraSwitcher : MonoBehaviour
{
    public Camera[] cameras; // Lista di camere da assegnare nell'Inspector
    private int currentCameraIndex = 0;
    public Camera currentCamera;
    public MoveConstraint targetMoveSystem;
    public MoveConstraint constraintMoveSystem;
    [SerializeField] private InteractionController ballInteractionSystem;
    [SerializeField] private GameObject kickUI;

    void Start()
    {
        // Disattiva tutte le camere tranne la prima
        for (int i = 0; i < cameras.Length; i++)
        {
            cameras[i].gameObject.SetActive(i == currentCameraIndex);
        }
        currentCamera = cameras[currentCameraIndex];
    }

    void Update()
    {
        if(Gamepad.current != null)
        {
            if (Gamepad.current.leftShoulder.wasReleasedThisFrame)
            {
                SwitchCamera(-1);
            }
            else if (Gamepad.current.rightShoulder.wasReleasedThisFrame)
            {
                SwitchCamera(1);
            }
        }
    }

    void SwitchCamera(int direction)
    {
        // Disattiva la camera attuale
        cameras[currentCameraIndex].gameObject.SetActive(false);

        // Calcola il nuovo indice ciclico
        currentCameraIndex = (currentCameraIndex + direction + cameras.Length) % cameras.Length;

        // Attiva la nuova camera
        cameras[currentCameraIndex].gameObject.SetActive(true);
        currentCamera = cameras[currentCameraIndex];
        
        if(cameras[currentCameraIndex].gameObject.CompareTag("GoalCamera"))
        {
            targetMoveSystem.enabled = true;
            ballInteractionSystem.enabled = false;
            kickUI.SetActive(false);
        }
        else if(cameras[currentCameraIndex].gameObject.CompareTag("BarrierCamera"))
        {
            constraintMoveSystem.enabled = true;
            ballInteractionSystem.enabled = false;
            kickUI.SetActive(false);
        }
        else
        {
            targetMoveSystem.enabled = false;
            constraintMoveSystem.enabled = false;
            ballInteractionSystem.enabled = true;
            if(!ballInteractionSystem.replay)
                kickUI.SetActive(true);
        }
    }
}