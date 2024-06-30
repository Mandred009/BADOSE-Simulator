// Script to create and send telemetry data of the drone
// This version sends the following data:
// X,Y,Z location ; Rotation about X,Y,Z ; Velocity about X,Y,Z

using System.Collections;
using System.Collections.Generic;
using UnityEngine;


namespace ROS2
{
    public class Drone_TalkerROS : MonoBehaviour
    {
        private ROS2UnityComponent ros2Unity;
        private ROS2Node ros2Node;
        private IPublisher<std_msgs.msg.String> pos_pub;
        private Rigidbody rb;
        public string drone_no;
        // Start is called before the first frame update
        void Start()
        {
            ros2Unity = GetComponent<ROS2UnityComponent>();
            rb = this.GetComponent<Rigidbody>();
            drone_no=gameObject.name;
        }

        // Update is called once per frame
        void Update()
        {
            if (ros2Unity.Ok())
            {
                if (ros2Node == null)
                {
                    ros2Node = ros2Unity.CreateNode("DroneTalkerNode"+drone_no);
                    pos_pub = ros2Node.CreatePublisher<std_msgs.msg.String>("drone_pos_"+drone_no);
                }
                std_msgs.msg.String msg = new std_msgs.msg.String();
                Vector3 postion = transform.position;
                Quaternion rot = transform.rotation;
                Vector3 rotation = rot.eulerAngles;
                // Encoding into string message
                msg.Data = "X: " + postion.x + " :Y: " + postion.y + " :Z: " + postion.z + " :RX: " + rotation.x + " :RY: " + rotation.y + " :RZ: " + rotation.z + " :VX: " + rb.velocity.x+ " :VY: " + rb.velocity.y + " :VZ: " + rb.velocity.z;
                pos_pub.Publish(msg);
            }
        }
    }

}
