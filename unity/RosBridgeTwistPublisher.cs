using UnityEngine;
using WebSocketSharp;
using System.Text;

public class RosBridgeTwistPublisher : MonoBehaviour
{
    public Transform rightController;
    WebSocket ws;
    readonly StringBuilder sb = new StringBuilder(128);

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
    }

    void Update()
    {
        if (ws.ReadyState != WebSocketState.Open) return;

        Vector3 p = rightController.position;
        Vector3 r = rightController.rotation.eulerAngles;         

        sb.Clear();
        sb.Append("{\"op\":\"publish\",\"topic\":\"/unity/controller_pose\",\"msg\":{\"linear\":{\"x\":")
          .Append(p.x).Append(",\"y\":").Append(p.y).Append(",\"z\":").Append(p.z)
          .Append("},\"angular\":{\"x\":").Append(r.x).Append(",\"y\":").Append(r.y).Append(",\"z\":").Append(r.z)
          .Append("}}}");
        ws.Send(sb.ToString());
    }

    void OnDestroy()
    {
        ws?.Close();
        ws = null;
    }
}
