using System;
using RosMessageTypes.Geometry;
using RosMessageTypes.BuilderbotMycobot;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.UrdfImporter;
using UnityEngine;

public class SourceDestinationPublisher : MonoBehaviour
{
    const int k_NumRobotJoints = 6;

    public static readonly string[] LinkNames =
         { "base/link1", "/link2", "/link3", "/link4", "/link5", "/link6" };

    // Variables required for ROS communication
    [SerializeField]
    string m_TopicName = "/mycobot_joints";

    [SerializeField]
    GameObject joint;
    public GameObject Joint { get => joint; set => joint = value; }

    [SerializeField]
    GameObject m_MyCobot;
    [SerializeField]
    GameObject m_Target;
    // [SerializeField]
    // GameObject m_TargetPlacement;
    readonly Quaternion m_PickOrientation = Quaternion.Euler(90, 90, 0);

    // Robot Joints
    UrdfJointRevolute[] m_JointArticulationBodies;

    // ROS Connector
    ROSConnection m_Ros;

    void Start()
    {
        // Get ROS connection static instance
        m_Ros = ROSConnection.GetOrCreateInstance();
        m_Ros.RegisterPublisher<MyCobotMoveitJointsMsg>(m_TopicName);

        m_JointArticulationBodies = new UrdfJointRevolute[k_NumRobotJoints];

        var linkName = string.Empty;
        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            linkName += LinkNames[i];
            m_JointArticulationBodies[i] = m_MyCobot.transform.Find(linkName).GetComponent<UrdfJointRevolute>();
        }
    }

    public void Publish()
    {
        var sourceDestinationMessage = new MyCobotMoveitJointsMsg();

        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            sourceDestinationMessage.joints[i] = m_JointArticulationBodies[i].GetPosition();
        }

        // Pick Pose
        sourceDestinationMessage.goal_pose = new PoseMsg
        {
            position = m_Target.transform.position.To<FLU>(),
            orientation = Quaternion.Euler(90, m_Target.transform.eulerAngles.y, 0).To<FLU>()
        };

        //So a Unity (x,y,z) coordinate is equivalent to the ROS (z,-x,y) coordinate.

        Vector3 position = joint.transform.position;
        Vector3 rospose = new Vector3(position.z, -position.x, position.y);

        Debug.Log("\nEnd effector pose (in Unity): " + position);
        
        Debug.Log("End effector pose (in ROS): " + rospose);

        // // Place Pose
        // sourceDestinationMessage.place_pose = new PoseMsg
        // {
        //     position = m_TargetPlacement.transform.position.To<FLU>(),
        //     orientation = m_PickOrientation.To<FLU>()
        // };

        // Finally send the message to server_endpoint.py running in ROS
        m_Ros.Publish(m_TopicName, sourceDestinationMessage);
    }
}