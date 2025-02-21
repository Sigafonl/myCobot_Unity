//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.MycobotCommunication
{
    [Serializable]
    public class GetAnglesRequest : Message
    {
        public const string k_RosMessageName = "mycobot_communication/GetAngles";
        public override string RosMessageName => k_RosMessageName;


        public GetAnglesRequest()
        {
        }
        public static GetAnglesRequest Deserialize(MessageDeserializer deserializer) => new GetAnglesRequest(deserializer);

        private GetAnglesRequest(MessageDeserializer deserializer)
        {
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
        }

        public override string ToString()
        {
            return "GetAnglesRequest: ";
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
