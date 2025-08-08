using UnityEngine;
using WebSocketSharp;
using System.Text;

public class CylinderProjectionPublisher : MonoBehaviour
{
    public Transform controller;
    public Transform trigger;
    public Transform cylinder;
    public GameObject greenSphere;
    public float radialThreshold = 0.05f;
    public float transparentAlpha = 0.3f;

    public enum Hand { Right, Left }
    public Hand hand = Hand.Right;

    public string rosBridgeUrl = "ws://localhost:9090";

    Renderer sphereRenderer;
    bool isLocked = false;
    Vector3 lockedLocalPos;

    WebSocket ws;
    StringBuilder sb = new StringBuilder(256);
    string topicName;

    void Awake()
    {
        QualitySettings.vSyncCount = 0;
        Application.targetFrameRate = 300;
    }

    void Start()
    {
        sphereRenderer = greenSphere.GetComponent<Renderer>();
        greenSphere.SetActive(false);
        SetAlpha(transparentAlpha);

        ws = new WebSocket(rosBridgeUrl);
        ws.Connect();

        topicName = "/unity/" + (hand == Hand.Right ? "right_sphere" : "left_sphere");
        ws.Send("{\"op\":\"advertise\",\"topic\":\"" + topicName + "\",\"type\":\"geometry_msgs/Twist\"}");
    }

    void Update()
    {
        Vector3 ctrlPos = controller.position;
        Vector3 cylPos = cylinder.position;
        Vector3 axis = cylinder.up.normalized;
        Vector3 toCtrl = ctrlPos - cylPos;
        float projLen = Vector3.Dot(toCtrl, axis);
        Vector3 projPoint = cylPos + axis * projLen;
        float radialDist = Vector3.Distance(ctrlPos, projPoint);

        float angleX = trigger.localEulerAngles.x;
        if (angleX > 180f) angleX -= 360f;
        bool triggerPressed = Mathf.Approximately(angleX, -15f);

        // Locking logic
        if (!isLocked)
        {
            if (radialDist <= radialThreshold && triggerPressed)
            {
                isLocked = true;
                lockedLocalPos = cylinder.InverseTransformPoint(projPoint);
                greenSphere.SetActive(true);
                SetAlpha(1f);
            }
        }
        else if (!triggerPressed)
        {
            isLocked = false;
            SetAlpha(transparentAlpha);
        }

        if (isLocked)
        {
            greenSphere.transform.position = cylinder.TransformPoint(lockedLocalPos);
            if (!greenSphere.activeSelf) greenSphere.SetActive(true);

            Vector3 localPos = greenSphere.transform.localPosition;
            sb.Clear();
            sb.Append("{\"op\":\"publish\",\"topic\":\"").Append(topicName)
              .Append("\",\"msg\":{")
              .Append("\"linear\":{\"x\":").Append(localPos.x)
              .Append(",\"y\":").Append(-localPos.y + 0.77f)
              .Append(",\"z\":").Append(localPos.z).Append("},")
              .Append("\"angular\":{\"x\":0,\"y\":0,\"z\":0}}}");
            ws.Send(sb.ToString());
        }
        else
        {
            // Normal sliding logic
            if (radialDist <= radialThreshold)
            {
                if (!greenSphere.activeSelf) greenSphere.SetActive(true);
                greenSphere.transform.position = projPoint;
            }
            else if (greenSphere.activeSelf)
            {
                greenSphere.SetActive(false);
            }

        }
    }

    void SetAlpha(float a)
    {
        Color c = sphereRenderer.material.color;
        c.a = a;
        sphereRenderer.material.color = c;
    }

    void OnDestroy()
    {
        if (ws != null)
        {
            ws.Close();
            ws = null;
        }
    }
}
