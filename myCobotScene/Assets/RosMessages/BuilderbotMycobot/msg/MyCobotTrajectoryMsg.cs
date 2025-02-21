//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.BuilderbotMycobot
{
    [Serializable]
    public class MyCobotTrajectoryMsg : Message
    {
        public const string k_RosMessageName = "builderbot_mycobot/MyCobotTrajectory";
        public override string RosMessageName => k_RosMessageName;

        public Moveit.RobotTrajectoryMsg[] trajectory;

        public MyCobotTrajectoryMsg()
        {
            this.trajectory = new Moveit.RobotTrajectoryMsg[0];
        }

        public MyCobotTrajectoryMsg(Moveit.RobotTrajectoryMsg[] trajectory)
        {
            this.trajectory = trajectory;
        }

        public static MyCobotTrajectoryMsg Deserialize(MessageDeserializer deserializer) => new MyCobotTrajectoryMsg(deserializer);

        private MyCobotTrajectoryMsg(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.trajectory, Moveit.RobotTrajectoryMsg.Deserialize, deserializer.ReadLength());
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.WriteLength(this.trajectory);
            serializer.Write(this.trajectory);
        }

        public override string ToString()
        {
            return "MyCobotTrajectoryMsg: " +
            "\ntrajectory: " + System.String.Join(", ", trajectory.ToList());
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize);
        }
    }
}
