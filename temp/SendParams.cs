using System;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Geometry;
using UnityEngine;

public class SendParams : MonoBehaviour
{

    private const int NUM_BETAS = 10;
    private const int NUM_EXPRESSIONS = 10;
    private const int NUM_JOINTS = 24;

    private float[] parameters = new float[NUM_BETAS + (NUM_JOINTS * 3) + NUM_EXPRESSIONS];
    // Variables required for ROS communication
    [SerializeField]
    private string topic_name = "/smpl_params";
    private ROSConnection m_Ros;

    // prefab to track
    private GameObject target;
    void Start()
    {
        m_Ros = ROSConnection.GetOrCreateInstance();
        m_Ros.RegisterPublisher<Float32MultiArrayMsg>(topic_name);

        // retrieve avatar from ZED Body Tracking Manager -> Avatars
        target = GameObject.Find("Humanoid");

    }

    public void Publish()
    {
        Debug.Log(target.transform.position);

        var msg = new Float32MultiArrayMsg();
        for (int i = 0; i < NUM_BETAS; i++)
        {
            parameters[i] = target.GetComponent<SMPLX>().betas[i];
        }
        for(int i = 0; i < NUM_EXPRESSIONS; i++)
        {
            parameters[i + NUM_BETAS] = target.GetComponent<SMPLX>().expressions[i];
        }
        // Debug.Log("Joint Rotations: " + target.GetComponent<SMPLX>()._transformFromName.Length);

        int j = 0;
        foreach(var item in target.GetComponent<SMPLX>()._transformFromName)
        {
            Debug.Log("Processing : " + item.Key);
            Quaternion quat = item.Value.rotation;
            var euler = quat.eulerAngles;

            // Local joint coordinate systems
            //   SMPL-X: X-Right, Y-Up, Z-Back, Right-handed
            //   Unity:  X-Left,  Y-Up, Z-Back, Left-handed
            parameters[j + NUM_BETAS + NUM_EXPRESSIONS] = -euler.x;
            parameters[j + 1 + NUM_BETAS + NUM_EXPRESSIONS] = euler.y;
            parameters[j + 2 + NUM_BETAS + NUM_EXPRESSIONS] = euler.z;
            j += 3;
            if(j >= NUM_JOINTS * 3)
            {
                break;
            }
        }

        Debug.Log("Parameters: " + parameters);
        msg.data = parameters;
        m_Ros.Publish(topic_name, msg); 
    }
    void Update()
    {
        if (target == null)
        {
            target = GameObject.Find("Humanoid");
            return;
        }
        // check if target is enabled
        Publish();
    }
}