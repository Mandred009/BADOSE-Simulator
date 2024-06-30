// Script to create and send telemetry data of the boat/ship
// This version sends the following data:
// X,Y,Z location ; Rotation about X,Y,Z ; Velocity about X,Y,Z ; Angular Velocity about Y
// Front X,Y,Z coordinates

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace ROS2
{

    public class ShipTalker_ROS : MonoBehaviour
    {
        private ROS2UnityComponent ros2Unity;
        private ROS2Node ros2Node;
        private IPublisher<std_msgs.msg.String> pos_pub;
        public string ship_no;
        public Rigidbody rb;
        public GameObject front;

        void Start()
        {
            ros2Unity = GetComponent<ROS2UnityComponent>();
            ship_no=gameObject.name;
            rb = this.GetComponent<Rigidbody>();
        }

        // Update is called once per frame
        void Update()
        {
            if (ros2Unity.Ok())
            {
                if (ros2Node == null)
                {
                    ros2Node = ros2Unity.CreateNode("ShipTalkerNode"+ship_no);
                    pos_pub = ros2Node.CreatePublisher<std_msgs.msg.String>("ship_pos_"+ship_no);
                }
                std_msgs.msg.String msg = new std_msgs.msg.String();
                Vector3 postion = transform.position;
                Quaternion rot = transform.rotation;
                Vector3 rotation = rot.eulerAngles;
                msg.Data = "X: " + postion.x + " :Y: " + postion.y + " :Z: " + postion.z + " :RX: " + rotation.x + " :RY: " + rotation.y + " :RZ: " + rotation.z + " :AV: " + rb.angularVelocity.y+ 
                " :FX: " + front.transform.position.x+" :FY: " + front.transform.position.y+" :FZ: " + front.transform.position.z + " :VX: " + rb.velocity.x+ " :VY: " + rb.velocity.y+ " :VZ: " + rb.velocity.z;
                pos_pub.Publish(msg);
            }
        }
    }

}