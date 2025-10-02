using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.InputSystem;
using UnityEngine.UI;
using Utils;

public class InteractionSystem : MonoBehaviour, IDragHandler
{
    public RectTransform background;
    public RectTransform handle;
    public Slider chargingSlider;
    public LineRenderer directionLine;
    public BallSimulation simulation;
    public BallProperties ball;
    public TextMeshProUGUI UItextModeInfo;
    public TextMeshProUGUI UItextShotInfo;
    public CameraSwitcher cameraSwitcher; 
    [SerializeField] private GameObject aimLine;
    [SerializeField] private GameObject spinSlider;
    [SerializeField] private GameObject kickUI;

    private Vector2 normalizedValue;
    public float moveSpeed = 2.0f;
    private bool isCharging = false;
    private float chargeTime = 0f;
    private float maxChargeTime = 1f;
    private Vector3 direction = Vector3.right;
    public float rotationSpeed = 50f;
    public float maxVerticalAngle = 90f;
    private float horizontalAngle = 0f;
    private float verticalAngle = 0f;

    public float replaySpeed;
    private float _continuousFrameCount;

    private List<Vector3> _trajectoryPreview;

    private Vector3 savedPos;
    private Quaternion savedRot;

    private int mode = 0;
    public bool replay = false;
    
    public BallProperties simulationBall;
    public DisplayShotComputation displayShotComputation;

    void Start()
    {
        normalizedValue = new Vector2(0f, 0f);
        UpdateHandlePosition();
        if (directionLine != null)
        {
            directionLine.positionCount = 2;
        }
        AdjustDirection(90, 15);
        _trajectoryPreview = ComputeKickTrajectory(direction, 30, GetSpinValue());
        UpdateDirectionLine();
    }

    void Update()
    {
        if (simulation.ball.state == BallStates.Stopped && !directionLine.enabled && !simulation.useFrameByFrameMode)
        {
            _trajectoryPreview = ComputeKickTrajectory(direction, 30, GetSpinValue());
            directionLine.enabled = true;
            kickUI.SetActive(true);
            UpdateDirectionLine();
        }

        if (Gamepad.current != null && simulation.ball.state == BallStates.Stopped && !simulation.useFrameByFrameMode)
        {
            if (Gamepad.current.squareButton.isPressed)
            {
                isCharging = true;
            }
            else if (Gamepad.current.squareButton.wasReleasedThisFrame)
            {
                Vector3 spin = new Vector3(0, -GetSpinValue().x, -GetSpinValue().y) * 30f;
                switch (mode)
                {
                    case 0: simulation.Kick(direction, chargingSlider.value, spin);
                        break;
                }

                isCharging = false;
                chargeTime = 0f;
                chargingSlider.value = 0f;
                directionLine.enabled = false;
                kickUI.SetActive(!kickUI.activeInHierarchy);
            }

            if (Gamepad.current.triangleButton.wasReleasedThisFrame)
            {
                mode += 1;
                if (mode > 2)
                    mode = 0;

                switch (mode)
                {
                    case 0: UItextModeInfo.SetText("NORMAL KICK MODE");
                        aimLine.SetActive(true);
                        spinSlider.SetActive(true);
                        break;
                    case 1: UItextModeInfo.SetText("TARGETED KICK MODE");
                        aimLine.SetActive(false);
                        spinSlider.SetActive(true);
                        break;
                    case 2: UItextModeInfo.SetText("TWO CONSTRAINTS KICK MODE");
                        aimLine.SetActive(false);
                        spinSlider.SetActive(false);
                        break;
                }

            }

            Vector2 dpadInput = Gamepad.current.dpad.ReadValue();
            AdjustDirection(dpadInput.x * rotationSpeed * Time.deltaTime, dpadInput.y * rotationSpeed * Time.deltaTime);
            UpdateDirectionLine();
        }

        if (isCharging)
        {
            chargeTime += Time.deltaTime;
            chargingSlider.value =
                Mathf.Clamp01(chargeTime / maxChargeTime) * (chargingSlider.maxValue - chargingSlider.minValue) +
                chargingSlider.minValue;
        }

        if (Gamepad.current != null && Gamepad.current.startButton.wasPressedThisFrame && !simulation.useFrameByFrameMode)
        {
            simulation.Reset();
            displayShotComputation.DeleteComputedTrajectories();
            normalizedValue = Vector2.zero;
            UpdateHandlePosition();
            direction = Vector3.right;
            horizontalAngle = 0f;
            verticalAngle = 0f;
            AdjustDirection(90, 15);
            _trajectoryPreview = ComputeKickTrajectory(direction, 30, GetSpinValue());
            UpdateDirectionLine();
            UItextShotInfo.SetText("");
        }

        if (Gamepad.current != null && Gamepad.current.selectButton.wasPressedThisFrame)
        {
            if(ball.Frames[0] != null)
            {
                simulation.useFrameByFrameMode = !simulation.useFrameByFrameMode;
                simulation.FrameCount = ball.NewestFrameIndex - 1;
                _continuousFrameCount = ball.NewestFrameIndex - 1;
                simulation.ContinuousFrameCount = _continuousFrameCount;

                if (simulation.useFrameByFrameMode)
                {
                    replay = true;
                    UItextModeInfo.SetText("REPLAY MODE");
                    savedPos = ball.position;
                    savedRot = ball.orientation;
                    kickUI.SetActive(false);

                }
                else
                {
                    replay = false;
                    ball.position = savedPos;
                    ball.orientation = savedRot;
                    switch (mode)
                    {
                        case 0:
                            UItextModeInfo.SetText("NORMAL KICK MODE");
                            break;
                        case 1:
                            UItextModeInfo.SetText("TARGETED KICK MODE");
                            break;
                        case 2:
                            UItextModeInfo.SetText("TWO CONSTRAINTS KICK MODE");
                            break;
                    }

                    if (simulation.ball.state == BallStates.Stopped)
                        kickUI.SetActive(true);
                }
            }
        }
    }

    private void FixedUpdate()
    {
        if (Gamepad.current != null)
        {
            if (simulation.useFrameByFrameMode)
            {
                float r2Value = Gamepad.current.rightTrigger.ReadValue();
                float l2Value = Gamepad.current.leftTrigger.ReadValue();

                if (r2Value > 0.05f)
                {
                    _continuousFrameCount += r2Value * replaySpeed * Time.fixedDeltaTime;

                    if (_continuousFrameCount > ball.NewestFrameIndex-1)
                    {
                        if (_continuousFrameCount < ball.OldestFrameIndex || ball.OldestFrameIndex == 0)
                        {
                            _continuousFrameCount = ball.NewestFrameIndex-1;
                        }
                    }
                    
                    if (_continuousFrameCount > ball.Frames.Length - 1)
                    {
                        _continuousFrameCount = 0;
                    }
                    
                    simulation.FrameCount = Mathf.FloorToInt(_continuousFrameCount);
                }

                if (l2Value > 0.05f)
                {
                    _continuousFrameCount -= l2Value * replaySpeed * Time.fixedDeltaTime;
                    
                    if (_continuousFrameCount < 0)
                    {
                        if(ball.OldestFrameIndex != 0)
                            _continuousFrameCount = ball.Frames.Length - 1;
                        else
                            _continuousFrameCount = 0;
                    }

                    if (_continuousFrameCount < ball.OldestFrameIndex && _continuousFrameCount > ball.NewestFrameIndex)
                    {
                        _continuousFrameCount = ball.OldestFrameIndex;
                    }
                    
                    simulation.FrameCount = Mathf.FloorToInt(_continuousFrameCount);
                }
                
                _continuousFrameCount = Mathf.Clamp(_continuousFrameCount, 0, ball.Frames.Length - 1);
                simulation.ContinuousFrameCount = _continuousFrameCount;
            }
            
            if(simulation.ball.state == BallStates.Stopped)
            {
                Vector2 stickInput = Gamepad.current.leftStick.ReadValue();

                if (Gamepad.current.circleButton.isPressed && stickInput.magnitude > 0.1f)
                {
                    if(cameraSwitcher.currentCamera.CompareTag("FreeCamera"))
                    {
                        MoveFreeCamera(stickInput);
                    }
                    else
                    {
                        MoveBallWithCamera(stickInput);
                        _trajectoryPreview = ComputeKickTrajectory(direction, 30, GetSpinValue());
                    }
                }
                else if (stickInput.magnitude > 0.1f)
                {
                    normalizedValue += stickInput * moveSpeed * Time.deltaTime;
                    if (normalizedValue.magnitude > 1f)
                    {
                        normalizedValue = normalizedValue.normalized; // Mantiene il vettore nella circonferenza
                    }

                    UpdateHandlePosition();
                    _trajectoryPreview = ComputeKickTrajectory(direction, 25, GetSpinValue());
                }
            }
        }
    }

    public void OnDrag(PointerEventData eventData)
    {
        RectTransformUtility.ScreenPointToLocalPointInRectangle(
            background,
            eventData.position,
            eventData.pressEventCamera,
            out Vector2 localPoint
        );

        // Converti in coordinate normalizzate centrali (-1,1)
        float x = localPoint.x / (background.rect.width / 2);
        float y = localPoint.y / (background.rect.height / 2);

        Vector2 newValue = new Vector2(x, y);

        // Restringi il movimento all'interno del cerchio
        if (newValue.magnitude > 1f)
        {
            newValue = newValue.normalized;
        }

        normalizedValue = newValue;
        UpdateHandlePosition();
    }

    private void UpdateHandlePosition()
    {
        float x = normalizedValue.x * (background.rect.width / 2);
        float y = normalizedValue.y * (background.rect.height / 2);
        handle.anchoredPosition = new Vector2(x, y);
    }

    public Vector2 GetSpinValue()
    {
        return normalizedValue; // Ora è già tra -1 e 1
    }
    private void AdjustDirection(float horizontalDelta, float verticalDelta)
    {
        if (horizontalDelta == 0 && verticalDelta == 0)
            return;

        horizontalAngle += horizontalDelta;
        verticalAngle = Mathf.Clamp(verticalAngle + verticalDelta, 0, maxVerticalAngle);

        direction = Quaternion.Euler(-verticalAngle, horizontalAngle, 0) * Vector3.forward;

        _trajectoryPreview = ComputeKickTrajectory(direction, 25, GetSpinValue());
    }

    private void UpdateDirectionLine()
    {
        if (directionLine != null && simulation.ball.state == BallStates.Stopped)
        {
            directionLine.positionCount = _trajectoryPreview.Count;

            for (int i = 0; i < _trajectoryPreview.Count; i++)
            {
                directionLine.SetPosition(i, _trajectoryPreview[i]);
            }
        }
    }

    private void MoveBallWithCamera(Vector2 stickInput)
    {
        if (cameraSwitcher == null || ball == null) return;

        Camera currentCamera = cameraSwitcher.currentCamera;
        if (currentCamera == null) return;

        // Ottieni la direzione frontale e destra della telecamera attuale
        Vector3 forward = currentCamera.transform.forward;
        Vector3 right = currentCamera.transform.right;

        // Mantieni il movimento solo sul piano orizzontale
        forward.y = 0;
        right.y = 0;
        forward.Normalize();
        right.Normalize();

        // Calcola la direzione del movimento
        Vector3 moveDirection = (right * stickInput.x + forward * stickInput.y).normalized;

        // Sposta la palla
        ball.position += moveDirection * moveSpeed * Time.deltaTime;
        ball.resetPosition = ball.position;
    }
    
    private void MoveFreeCamera(Vector2 stickInput)
    {
        if (cameraSwitcher == null || ball == null) return;

        Camera currentCamera = cameraSwitcher.currentCamera;
        if (currentCamera == null) return;

        // Ottieni la direzione frontale e destra della telecamera attuale
        Vector3 forward = currentCamera.transform.forward;
        Vector3 right = currentCamera.transform.right;

        // Mantieni il movimento solo sul piano orizzontale
        forward.y = 0;
        right.y = 0;
        forward.Normalize();
        right.Normalize();

        // Calcola la direzione del movimento
        Vector3 moveDirection = (right * stickInput.x + forward * stickInput.y).normalized;

        // Sposta la palla
        currentCamera.transform.position += moveDirection * moveSpeed * Time.deltaTime;

    }
    
    public List<Vector3> ComputeKickTrajectory(Vector3 direction, float speed, Vector3 spin)
    {
        List<Vector3> kickTrajectory = new List<Vector3>();
        
        float deltaTime = 1f/100f;
        Vector3 targetXY = direction;
        targetXY.y = 0f;
        Quaternion targetRotation = Quaternion.LookRotation(targetXY.normalized, Vector3.up);

        simulationBall.position = ball.position;
        simulationBall.orientation = ball.orientation;
        simulationBall.velocity = direction * speed;
        simulationBall.angularVelocity = targetRotation * new Vector3(spin.y, 0, 0) * 30;
        simulationBall.angularVelocity.y = -spin.x * 30;
        simulationBall.state = BallStates.Bouncing;
        
        for (int i = 0; i < 15; i++)
        {
            simulation.ComputeNewFrame(deltaTime, simulationBall);
            kickTrajectory.Add(simulationBall.position);
        }
        
        return kickTrajectory;
    }

    public void ShowComputationTime(double computationTime, float error)
    {
        UItextShotInfo.SetText("TIME TO COMPUTE: " + computationTime.ToString("0.00") + " ms" + "\nERROR: " + error.ToString("0.00") + " cm");
    }
}
