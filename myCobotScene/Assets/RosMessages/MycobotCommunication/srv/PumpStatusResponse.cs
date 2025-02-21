//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.MycobotCommunication
{
    [Serializable]
    public class PumpStatusResponse : Message
    {
        public const string k_RosMessageName = "mycobot_communication/PumpStatus";
        public override string RosMessageName => k_RosMessageName;

        public bool Flag;

        public PumpStatusResponse()
        {
            this.Flag = false;
        }

        public PumpStatusResponse(bool Flag)
        {
            this.Flag = Flag;
        }

        public static PumpStatusResponse Deserialize(MessageDeserializer deserializer) => new PumpStatusResponse(deserializer);

        private PumpStatusResponse(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.Flag);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.Flag);
        }

        public override string ToString()
        {
            return "PumpStatusResponse: " +
            "\nFlag: " + Flag.ToString();
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize, MessageSubtopic.Response);
        }
    }
}
