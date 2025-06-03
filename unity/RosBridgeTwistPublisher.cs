using UnityEngine;
using WebSocketSharp;
using System.Text;

public class RosBridgeTwistPublisher : MonoBehaviour
{
    public Transform rightController;
    public Transform thumbStick;
    WebSocket ws;
    StringBuilder sb = new StringBuilder(256);

    void Awake()
    {
        QualitySettings.vSyncCount = 0;
        Application.targetFrameRate = 300;
    }

    void Start()
    {
        ws = new WebSocket("ws://localhost:9090");
        ws.Connect();
        ws.Send("{\"op\":\"advertise\",\"topic\":\"/unity/controller_pose\",\"type\":\"geometry_msgs/Twist\"}");
        ws.Send("{\"op\":\"advertise\",\"topic\":\"/unity/touchpad\",\"type\":\"std_msgs/Float32\"}");
    }

    void Update()
    {
        if (ws.ReadyState != WebSocketState.Open) return;

        Vector3 p = rightController.position;
        Vector3 euler = rightController.rotation.eulerAngles;

        sb.Clear();
        sb.Append("{\"op\":\"publish\",\"topic\":\"/unity/controller_pose\",\"msg\":{")
          .Append("\"linear\":{\"x\":").Append(p.x).Append(",\"y\":").Append(p.y).Append(",\"z\":").Append(p.z).Append("},")
          .Append("\"angular\":{\"x\":").Append(euler.x).Append(",\"y\":").Append(euler.y).Append(",\"z\":").Append(euler.z).Append("}}}");
        ws.Send(sb.ToString());

        float angleX = thumbStick.localEulerAngles.x;
        if (angleX > 180f) angleX -= 360f;
        float value = angleX / 30f;

        sb.Clear();
        sb.Append("{\"op\":\"publish\",\"topic\":\"/unity/touchpad\",\"msg\":{\"data\":").Append(value).Append("}}");
        ws.Send(sb.ToString());
    }

    void OnDestroy()
    {
        ws.Close();
        ws = null;
    }
}
