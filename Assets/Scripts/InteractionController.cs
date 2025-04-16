using System;
using System.Collections.Generic;
using TMPro;
using UnityEditor.AnimatedValues;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.InputSystem;
using UnityEngine.Serialization;
using UnityEngine.UI;

public class InteractionController : MonoBehaviour, IDragHandler
{
    public RectTransform background;
    public RectTransform handle;
    public Slider chargingSlider;
    public LineRenderer directionLine;
    public BallPhysics ballPhysics;
    public Ball ball;
    public TextMeshProUGUI UItext;
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
    
    public Ball simulationBall;

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
        if (ballPhysics.ballValues.state == BallState.Stopped && !directionLine.enabled && !ballPhysics.frameByFrame)
        {
            _trajectoryPreview = ComputeKickTrajectory(direction, 30, GetSpinValue());
            directionLine.enabled = true;
            kickUI.SetActive(true);
            UpdateDirectionLine();
        }

        if (Gamepad.current != null && ballPhysics.ballValues.state == BallState.Stopped && !ballPhysics.frameByFrame)
        {
            if (Gamepad.current.squareButton.isPressed)
            {
                isCharging = true;
            }
            else if (Gamepad.current.squareButton.wasReleasedThisFrame)
            {
                Vector3 spin = new Vector3(0, -GetSpinValue().x, -GetSpinValue().y) * ballPhysics.maximumSpinValue;
                switch (mode)
                {
                    case 0: ballPhysics.Kick(direction, chargingSlider.value, spin);
                        break;
                    case 1: ballPhysics.TargetedKick(chargingSlider.value, spin);
                        break;
                    case 2: ballPhysics.DoubleTargetedKick(chargingSlider.value);
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
                    case 0: UItext.SetText("NORMAL KICK MODE");
                        aimLine.SetActive(true);
                        spinSlider.SetActive(true);
                        break;
                    case 1: UItext.SetText("TARGETED KICK MODE");
                        aimLine.SetActive(false);
                        spinSlider.SetActive(true);
                        break;
                    case 2: UItext.SetText("DOUBLE TARGETED KICK MODE");
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

        if (Gamepad.current != null && Gamepad.current.startButton.wasPressedThisFrame && !ballPhysics.frameByFrame)
        {
            ballPhysics.Reset();
            normalizedValue = Vector2.zero;
            UpdateHandlePosition();
            direction = Vector3.right;
            horizontalAngle = 0f;
            verticalAngle = 0f;
            AdjustDirection(90, 15);
            _trajectoryPreview = ComputeKickTrajectory(direction, 30, GetSpinValue());
            UpdateDirectionLine();
        }

        if (Gamepad.current != null && Gamepad.current.selectButton.wasPressedThisFrame)
        {
            ballPhysics.frameByFrame = !ballPhysics.frameByFrame;
            ballPhysics.frameCount = ballPhysics.frameArray.Length-1;
            _continuousFrameCount = ballPhysics.frameArray.Length - 1;
            ballPhysics.continuousFrameCount = _continuousFrameCount;

            if (ballPhysics.frameByFrame)
            {
                replay = true;
                UItext.SetText("REPLAY MODE");
                savedPos = ball.position;
                savedRot = ball.rotation;
                kickUI.SetActive(false);

            }
            else
            {
                replay = false;
                ball.position = savedPos;
                ball.rotation = savedRot;
                switch (mode)
                {
                    case 0: UItext.SetText("NORMAL KICK MODE");
                        break;
                    case 1: UItext.SetText("TARGETED KICK MODE");
                        break;
                    case 2: UItext.SetText("DOUBLE TARGETED KICK MODE");
                        break;
                }
                if(ballPhysics.ballValues.state == BallState.Stopped)
                    kickUI.SetActive(true);
            }
        }
    }

    private void FixedUpdate()
    {
        if (Gamepad.current != null)
        {
            if (ballPhysics.frameByFrame)
            {
                float r2Value = Gamepad.current.rightTrigger.ReadValue();
                float l2Value = Gamepad.current.leftTrigger.ReadValue();

                if (r2Value > 0.05f)
                {
                    _continuousFrameCount += r2Value * replaySpeed * Time.fixedDeltaTime;
                    
                    if (_continuousFrameCount > ballPhysics.frameArray.Length - 1)
                    {
                        _continuousFrameCount = ballPhysics.frameArray.Length - 1;
                    }
                    
                    ballPhysics.frameCount = Mathf.Min(Mathf.FloorToInt(_continuousFrameCount), ballPhysics.frameArray.Length - 2);
                }

                if (l2Value > 0.05f)
                {
                    _continuousFrameCount -= l2Value * replaySpeed * Time.fixedDeltaTime;
                    
                    if (_continuousFrameCount < 0)
                    {
                        _continuousFrameCount = 0;
                    }
                    
                    ballPhysics.frameCount = Mathf.Max(Mathf.FloorToInt(_continuousFrameCount), 0);
                }
                
                _continuousFrameCount = Mathf.Clamp(_continuousFrameCount, 0, ballPhysics.frameArray.Length - 1);
                ballPhysics.continuousFrameCount = _continuousFrameCount;
            }
            
            if(ballPhysics.ballValues.state == BallState.Stopped)
            {
                Vector2 stickInput = Gamepad.current.leftStick.ReadValue();

                if (Gamepad.current.circleButton.isPressed && stickInput.magnitude > 0.1f)
                {
                    MoveBallWithCamera(stickInput);
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
        if (directionLine != null && ballPhysics.ballValues.state == BallState.Stopped)
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
    
    public List<Vector3> ComputeKickTrajectory(Vector3 direction, float speed, Vector3 spin)
    {
        List<Vector3> kickTrajectory = new List<Vector3>();
        
        float deltaTime = 1f/100f;
        Vector3 targetXY = direction;
        targetXY.y = 0f;
        Quaternion targetRotation = Quaternion.LookRotation(targetXY.normalized, Vector3.up);

        simulationBall.position = ball.position;
        simulationBall.rotation = ball.rotation;
        simulationBall.velocity = direction * speed;
        simulationBall.angularVelocity = targetRotation * new Vector3(spin.y, 0, 0) * ballPhysics.maximumSpinValue;
        simulationBall.angularVelocity.y = -spin.x * ballPhysics.maximumSpinValue;
        
        for (int i = 0; i < 15; i++)
        {
            ballPhysics.ComputeFrame(deltaTime, simulationBall);
            kickTrajectory.Add(simulationBall.position);
        }
        
        return kickTrajectory;
    }
}
