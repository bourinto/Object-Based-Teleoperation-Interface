using UnityEngine;
using WebSocketSharp;
using System;

[Serializable]
public class MultiArrayMsg
{
    public float[] data;
}

[Serializable]
public class GripperCommand
{
    public float position;
}

[Serializable]
class Envelope
{
    public string op;
    public string topic;
}

[Serializable]
class JointEnvelope : Envelope
{
    public MultiArrayMsg msg;
}

[Serializable]
class GripperEnvelope : Envelope
{
    public GripperCommand msg;
}

public class RosBridgeJointSubscriber : MonoBehaviour
{
    public Transform J1;
    public Transform J2;
    public Transform J3;
    public Transform J4;
    public Transform J5;
    public Transform J6;

    public Transform PS36;
    public Transform PS38;
    public Transform PS40;
    public Transform PS42;
    public Transform PS49;
    public Transform PS50;

    WebSocket ws;

    float[] latestAngles = new float[6];
    float latestGripper = 0f;

    readonly object lockObj = new object();
    bool gotNewAngles = false;
    bool gotNewGripper = false;

    void Start()
    {
        ws = new WebSocket("ws://localhost:9090");
        ws.OnMessage += HandleMessage;
        ws.Connect();
        ws.Send("{\"op\":\"subscribe\",\"topic\":\"/xarm6/joints_values\",\"type\":\"std_msgs/Float64MultiArray\"}");
        ws.Send("{\"op\":\"subscribe\",\"topic\":\"/xarm6/gripper_cmd\",\"type\":\"control_msgs/GripperCommand\"}");
    }

    void HandleMessage(object sender, MessageEventArgs e)
    {
        Envelope env = JsonUtility.FromJson<Envelope>(e.Data);
        if (env.op != "publish") return;

        if (env.topic == "/xarm6/joints_values")
        {
            JointEnvelope jm = JsonUtility.FromJson<JointEnvelope>(e.Data);
            if (jm.msg != null && jm.msg.data != null && jm.msg.data.Length >= 6)
            {
                lock (lockObj)
                {
                    Array.Copy(jm.msg.data, latestAngles, 6);
                    gotNewAngles = true;
                }
            }
        }
        else if (env.topic == "/xarm6/gripper_cmd")
        {
            GripperEnvelope gm = JsonUtility.FromJson<GripperEnvelope>(e.Data);
            if (gm.msg != null)
            {
                float g = 50f / 850f * Mathf.Max(gm.msg.position, 0f) - 5f;
                lock (lockObj)
                {
                    latestGripper = g;
                    gotNewGripper = true;
                }
            }
        }
    }

    void Update()
    {
        if (gotNewAngles)
        {
            float[] a;
            lock (lockObj)
            {
                a = (float[])latestAngles.Clone();
                gotNewAngles = false;
            }

            Vector3 e;
            e = J1.localEulerAngles; J1.localEulerAngles = new Vector3(e.x, -a[0], e.z);
            e = J2.localEulerAngles; J2.localEulerAngles = new Vector3(e.x, e.y, -a[1]);
            e = J3.localEulerAngles; J3.localEulerAngles = new Vector3(e.x, e.y, -a[2]);
            e = J4.localEulerAngles; J4.localEulerAngles = new Vector3(e.x, a[3], e.z);
            e = J5.localEulerAngles; J5.localEulerAngles = new Vector3(e.x, e.y, -a[4]);
            e = J6.localEulerAngles; J6.localEulerAngles = new Vector3(e.x, a[5], e.z);
        }

        if (gotNewGripper)
        {
            float g;
            lock (lockObj)
            {
                g = latestGripper;
                gotNewGripper = false;
            }

            Vector3 e;
            e = PS36.localEulerAngles; PS36.localEulerAngles = new Vector3(-g, e.y, e.z);
            e = PS38.localEulerAngles; PS38.localEulerAngles = new Vector3(g, e.y, e.z);
            e = PS40.localEulerAngles; PS40.localEulerAngles = new Vector3(-g, e.y, e.z);
            e = PS42.localEulerAngles; PS42.localEulerAngles = new Vector3(g, e.y, e.z);
            e = PS49.localEulerAngles; PS49.localEulerAngles = new Vector3(-g, e.y, e.z);
            e = PS50.localEulerAngles; PS50.localEulerAngles = new Vector3(g, e.y, e.z);
        }
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
