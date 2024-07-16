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
    private const int NUM_JOINTS = 55;

    private float[] parameters = new float[NUM_BETAS + NUM_EXPRESSIONS + NUM_JOINTS * 3];
    [SerializeField]
    private ROSConnection m_Ros;
    private string topic_name = "/smpl_params";

    // prefab to track

    public float delay = 0.1f;
    private float lastTime = 0.0f;
    private GameObject target;
    void Start()
    {
        m_Ros = ROSConnection.GetOrCreateInstance();
        m_Ros.RegisterPublisher<Float32MultiArrayMsg>(topic_name);

        // retrieve avatar from ZED Body Tracking Manager -> Avatars
        target = GameObject.Find("Humanoid");

    }

    public static Vector3 RodriguesFromQuat(Quaternion quat)
{
    // Invert the quaternion to align with the original Rodrigues vector
    Quaternion invertedQuat = Quaternion.Inverse(quat);
    
    // Get the axis and angle from the inverted quaternion
    Vector3 axis;
    float angle_deg;
    invertedQuat.ToAngleAxis(out angle_deg, out axis);

    // Convert angle to radians
    float angle_rad = angle_deg * Mathf.Deg2Rad;

    // Adjust the axis to match the SMPL coordinate system
    axis.x = -axis.x; // Invert X due to coordinate system difference
    
    // Scale the axis by the angle to get the Rodrigues vector
    Vector3 rvec = axis * angle_rad;

    return rvec;
}
    public void Publish()
    {
        Debug.Log(target.transform.position);

        var msg = new Float32MultiArrayMsg();
        for (int i = 0; i < NUM_BETAS; i++)
        {
            parameters[i] = target.GetComponent<SMPLX>().betas[i];
        }
        for (int i = 0; i < NUM_EXPRESSIONS; i++)
        {
            parameters[i + NUM_BETAS] = target.GetComponent<SMPLX>().expressions[i];
        }
        // Debug.Log("Joint Rotations: " + target.GetComponent<SMPLX>()._transformFromName.Length);


        string output = "";
        int j = NUM_BETAS + NUM_EXPRESSIONS;
        bool start = false;

        string[] joints = target.GetComponent<SMPLX>()._bodyJointNames;

        foreach (var jointname in joints)
        {
            var trans = target.GetComponent<SMPLX>()._transformFromName[jointname];

            // get local rotation
            Quaternion quat = trans.localRotation;
            
            
            Vector3 rvec = RodriguesFromQuat(quat);
            parameters[j] = rvec.x; 
            parameters[j + 1] = rvec.y;
            parameters[j + 2] = rvec.z;

            // Debug.Log("Processing : " + item.Key + " EULER: " + parameters[j] + " " + parameters[j + 1] + " " + parameters[j + 2]);
            output += jointname + "\t\t " + parameters[j] + " " + parameters[j + 1] + " " + parameters[j + 2] + "\n";
            j += 3;
            if (j >= NUM_JOINTS * 3)
            {
                break;
            }
            msg.data = parameters;
            m_Ros.Publish(topic_name, msg);

        }
        // Debug.Log(output);
    }
    void Update()
    {
        if (target == null)
        {
            target = GameObject.Find("Humanoid");
            return;
        }
        if (Time.time - lastTime > delay)
        {
            lastTime = Time.time;
            Publish();
        }
    }
}