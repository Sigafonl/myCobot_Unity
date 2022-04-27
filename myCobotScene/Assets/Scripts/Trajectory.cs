using System;
using System.Collections;
using RosMessageTypes.Geometry;
using RosMessageTypes.BuilderbotMycobot;
using RosMessageTypes.Trajectory;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.UrdfImporter;
using UnityEngine;


// モーションプランニングサービスの実行機能 - Motion planning service execution function
public class Trajectory : MonoBehaviour
{

    // 定数 - Constants
    const int numRobotJoints = 6;
    const float k_JointAssignmentWait = 0.1f;
    const float k_PoseAssignmentWait = 0.5f;
    
    private static readonly string rosServiceName = "builderbot_moveit";

    [SerializeField]
    GameObject myCobot;
    public GameObject MyCobot { get => myCobot; set => myCobot = value; }

    [SerializeField]
    GameObject target;
    public GameObject Target { get => target; set => target = value; }

     // Assures that the gripper is always positioned above the m_Target cube before grasping.
    readonly Quaternion pickOrientation = Quaternion.Euler(90, 90, 0);
    readonly Vector3 pickPoseOffset = Vector3.up * 0.15f;

    // 変数 - Variables
    public ArticulationBody[] jointArticulationBodies; // 関節 - joints

    private ROSConnection rc; // ROSコネクション - Connection

    // スタート - Start 
    void Start()
    {
        // ROSコネクションの準備 - Start ROS connection
        this.rc = ROSConnection.GetOrCreateInstance();
        
        // サービスのレスポンスのコールバックの登録 - Registering service response callbacks
        this.rc.RegisterRosService<MoverServiceRequest, MoverServiceResponse>(rosServiceName);

        jointArticulationBodies = new ArticulationBody[numRobotJoints];

        // Populate the articulation bodies
        var linkName = string.Empty;
        for (var i = 0; i < numRobotJoints; i++)
        {
            linkName += SourceDestinationPublisher.LinkNames[i];
            jointArticulationBodies[i] = myCobot.transform.Find(linkName).GetComponent<ArticulationBody>();
        }
    }

    /// <summary>
    ///     Get the current values of the robot's joint angles.
    /// </summary>
    /// <returns>MycobotMoveitJoints</returns>
    MyCobotMoveitJointsMsg CurrentJointConfig()
    {
        var joints = new MyCobotMoveitJointsMsg();

        // Set the joint values for every joint
        for (var i = 0; i < numRobotJoints; i++)
        {
            joints.joints[i] = jointArticulationBodies[i].jointPosition[0];
        }

        return joints;
    }

    // 関節のパブリッシュ - Publish joints
    /// <summary>
    ///     Create a new MoverServiceRequest with the current values of the robot's joint angles,
    ///     the target cube's current position and rotation, and the targetPlacement position and rotation.
    ///     Call the MoverService using the ROSConnection and if a trajectory is successfully planned,
    ///     execute the trajectories in a coroutine.
    /// </summary>
    public void Publish()
    {
        // メッセージの作成 - Compose a message
        var request = new MoverServiceRequest();

        // 関節の角度の指定 - Specifying the angle of the joint
        request.joints_input = CurrentJointConfig();
    
        // ゴール姿勢の指定 - Designation of goal posture
        // Pick Pose
        request.goal_pose = new PoseMsg
        {
            //(x, y + offset, z)
            position = (target.transform.position).To<FLU>(), 
            
            // The hardcoded x/z angles assure that the gripper is always positioned above the target cube before grasping.
            orientation = Quaternion.Euler(90, target.transform.eulerAngles.y, 0).To<FLU>()
            // orientation = Quaternion.Euler(90, 90, 0).To<FLU>()
        };

        Debug.Log("Target Object (xyz): " + round(target.transform.position.x) +  ", " + round(target.transform.position.y) +  ", " + round(target.transform.position.z));
        Debug.Log("Expected Goal (xyz): " + round(target.transform.position.z) +  ", " + (-round(target.transform.position.x)) +  ", " + round(target.transform.position.y));
        Debug.Log("Current Goal (xyz): " + round(request.goal_pose.position.x) +  ", " + round(request.goal_pose.position.y) +  ", " + round(request.goal_pose.position.z));

        Debug.Log("request: " + request);

        // サービスへのリクエスト送信 - Send a request to the service
        this.rc.SendServiceMessage<MoverServiceResponse>(rosServiceName, request, TrajectoryResponse);
    }

    double round(double var)
    {
        double value = (int)(var * 100 + .5);
        return (double)value / 100;
    }

    // サービスのレスポンス受信時に呼ばれる - Called when a service response is received
    void TrajectoryResponse(MoverServiceResponse response)
    {
        Debug.Log(response);
        if (response.trajectory != null && response.trajectory.joint_trajectory.points.Length > 0)
        //if (response.trajectory.Length > 0)
        {
            Debug.Log("Trajectory returned.");
            StartCoroutine(ExecuteTrajectories(response));
        }
        else
        {
            Debug.LogError("No trajectory returned from MoverService.");
        }
    }

    // モーションプランニングの実行 - Performing motion planning
    IEnumerator ExecuteTrajectories(MoverServiceResponse response)
    {
        if (response.trajectory != null)
        {
            Debug.Log(response);
            
            // For every robot pose in trajectory plan
            foreach (var t in response.trajectory.joint_trajectory.points)
            {
                float[] result = new float[6];
                
                for (var i = 0; i < t.positions.Length; i++)
                {
                    result[i] = (float)t.positions[i] * Mathf.Rad2Deg;
                    //float[] result = jointPositions.Select(r=> (float)r * Mathf.Rad2Deg).ToArray();
                }

                // Set the joint values for every joint
                for (var i = 0; i < this.jointArticulationBodies.Length; i++)
                {
                    var joint1XDrive = this.jointArticulationBodies[i].xDrive;
                    joint1XDrive.target = result[i];
                    this.jointArticulationBodies[i].xDrive = joint1XDrive;
                }

                 yield return new WaitForSeconds(k_JointAssignmentWait);
            }

            yield return new WaitForSeconds(k_PoseAssignmentWait);
        }

        // if (response.trajectory != null)
        // {
        //     // For every trajectory plan returned
        //     for (var poseIndex = 0; poseIndex < response.trajectory.Length; poseIndex++)
        //     {
        //         // For every robot pose in trajectory plan
        //         foreach (var t in response.trajectory[poseIndex].joint_trajectory.points)
        //         {
        //             float[] result = new float[t.positions.Length];
        //             var jointPositions = t.positions;
                   
        //            for (var i = 0; i < t.positions.Length; i++)
        //             {
        //                 result[i] = (float)t.positions[i] * Mathf.Rad2Deg;
        //                 //float[] result = jointPositions.Select(r=> (float)r * Mathf.Rad2Deg).ToArray();
        //             }

        //             // Set the joint values for every joint
        //             for (var joint = 0; joint < jointArticulationBodies.Length; joint++)
        //             {
        //                 var joint1XDrive = jointArticulationBodies[joint].xDrive;
        //                 joint1XDrive.target = result[joint];
        //                 jointArticulationBodies[joint].xDrive = joint1XDrive;
        //             }

        //             // Wait for robot to achieve pose for all joint assignments
        //             yield return new WaitForSeconds(k_JointAssignmentWait);
        //         }

        //         // Wait for the robot to achieve the final pose from joint assignment
        //         yield return new WaitForSeconds(k_PoseAssignmentWait);
        //     }
        // }
    }
}
