using JetBrains.Annotations;
using SimpleJSON;
using System;
using System.Collections;
using System.IO.Ports;
using System.Linq;
using UnityEditor.Experimental.GraphView;
using UnityEngine;

public class arduinoSerial : MonoBehaviour
{
    public string port;
    public int imuCnt;
    public Vector3 upperarmPos;
    public Vector3 forearmPos;
    public Vector3 wristPos;
    public GameObject objectToControl;
    public Matrix4x4[] transformations;

    private SerialPort stream;
    private bool pose_flag = false;
    private Vector3 forearmLen;
    private Vector3 upperarmLen;
    void Start()
    {
        imuCnt = 1;
        transformations = new Matrix4x4[imuCnt];
        stream = new SerialPort(port, 9600);
        stream.ReadTimeout = 50;
        stream.Open();
        forearmLen = new Vector3((float)0.24, 0, 0);
        upperarmLen = new Vector3((float)0.30, 0, 0);
    }

    // Update is called once per frame
    void Update()
    {
        if (pose_flag == false)
        {
            WriteToArduino("update_pose");
            StartCoroutine(
                AsynchronousReadFromArduino
                ((string s) => UpdatePose(s),
                    () => { pose_flag = false; },
                    5f
                )
            );
        }
    }

    public void WriteToArduino(string message)
    {
        stream.WriteLine(message);
        stream.BaseStream.Flush();
    }

    public IEnumerator AsynchronousReadFromArduino(Action<string> callback, Action fail = null, float timeout = float.PositiveInfinity)
    {
        DateTime initialTime = DateTime.Now;
        DateTime nowTime;
        TimeSpan diff = default(TimeSpan);

        string dataString = null;
        pose_flag = true;

        diff = DateTime.Now - initialTime;
        while (diff.Milliseconds < timeout)
        {
            try
            {
                dataString = stream.ReadLine();
            }
            catch (TimeoutException)
            {
                dataString = null;
            }
            
            if (dataString != null)
            {
                callback(dataString);
                pose_flag = false;
            }
            else
            {
                yield return null;
            }
            nowTime = DateTime.Now;
            diff = nowTime - initialTime;
        }
        pose_flag = false;
    }

    public void forwardKinematics(JSONNode imuData)
    {
        float x, y, z, w;
        Quaternion quat;

        imuCnt = imuData[0]["cnt"];
        upperarmPos = new Vector3(0, 0, 0);
        forearmPos = new Vector3(0, 0, 0);
        for (int i = 0; i < imuCnt; i++)
        {
            x = (imuData[i]["x"]);
            y = (imuData[i]["y"]);
            z = (imuData[i]["z"]);
            w = (imuData[i]["w"]);
            quat = new Quaternion(x, y, z, w);
            quat = quat.normalized;
            transformations[i] = quatToMatrix(quat);
        }
        if (imuCnt > 0)
        {
            upperarmPos = transformations[0] * upperarmLen;
        }
        if (imuCnt > 1)
        {
            forearmPos = transformations[1] * forearmLen;
            wristPos = upperarmPos + forearmPos;
        }
        return;
    }
    public void UpdatePose(string s)
    {
        JSONNode imuData = JSONNode.Parse(s);
        forwardKinematics(imuData);
    }
    public Matrix4x4 quatToMatrix(Quaternion q)
    {
        Matrix4x4 R = Matrix4x4.identity;
        float sqw = q.w * q.w;
        float sqx = q.x * q.x;
        float sqy = q.y * q.y;
        float sqz = q.z * q.z;

        // invs (inverse square length) is only required if quaternion is not already normalised
        float invs = 1;// (sqx + sqy + sqz + sqw);
        R[0,0] = (sqx - sqy - sqz + sqw) * invs; // since sqw + sqx + sqy + sqz =1/invs*invs
        R[1,1] = (-sqx + sqy - sqz + sqw) * invs;
        R[2,2] = (-sqx - sqy + sqz + sqw) * invs;

        float tmp1 = q.x * q.y;
        float tmp2 = q.z * q.w;
        R[1,0] = (float) 2.0 * (tmp1 + tmp2) * invs;
        R[0,1] = (float) 2.0 * (tmp1 - tmp2) * invs;

        tmp1 = q.x * q.z;
        tmp2 = q.y * q.w;
        R[2,0] = (float) 2.0 * (tmp1 - tmp2) * invs;
        R[0,2] = (float) 2.0 * (tmp1 + tmp2) * invs;

        tmp1 = q.y * q.z;
        tmp2 = q.x * q.w;
        R[2,1] = (float) 2.0 * (tmp1 + tmp2) * invs;
        R[1,2] = (float) 2.0 * (tmp1 - tmp2) * invs;
        return R;
    }
}
