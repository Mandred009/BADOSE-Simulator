// Script to handle ship/boat control and ros2 commands
// 2 rotor control is used to move the boat

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Text.RegularExpressions;
using TMPro;

namespace ROS2
{
    public class ShipController : MonoBehaviour
    {
        public float speedL = 0.0f;
        public float speedR = 0.0f;
        public Rigidbody rb;
        public Vector3 movementL;
        public Vector3 movementR;
        public GameObject rotorL;
        public GameObject rotorR;

        private ROS2UnityComponent ros2Unity;
        private ROS2Node ros2Node;
        private ISubscription<std_msgs.msg.String> control_sub;
        private string control_msg = "o"; // Initialize to avoid null checks
        public string ship_no;
        public TextMeshPro ship_no_txt;


        void Start()
        {
            rb = this.GetComponent<Rigidbody>();
            rb.mass=StaticData.ship_mass;
            ros2Unity = GetComponent<ROS2UnityComponent>();
            ship_no = gameObject.name;
            ship_no_txt.text=ship_no;
        }

        void FixedUpdate()
        {
            if (ros2Node == null && ros2Unity.Ok())
            {
                ros2Node = ros2Unity.CreateNode("ShipRos2" + ship_no);
                control_sub = ros2Node.CreateSubscription<std_msgs.msg.String>(
                "control_" + ship_no, msg => {
                    Debug.Log(msg.Data);
                    // Update the class-level variable with the message data
                    control_msg = msg.Data;
                });
            }
            
            var tL = rotorL.transform.forward;
            var tR = rotorR.transform.forward;
            if(control_msg[0]=='T')
            {
                float[] extractedCoords = TeleportExtract(control_msg);
                transform.position=new Vector3(extractedCoords[0],transform.position.y,extractedCoords[1]);
                transform.rotation=Quaternion.Euler(new Vector3(0,extractedCoords[2],0));
            }
            else
            {
                cmd_inference(control_msg);
            }
            movementL = tL * 1;
            movementR = tR * 1;
            rb.AddForceAtPosition(movementL * speedL,rotorL.transform.position);
            rb.AddForceAtPosition(movementR * speedR,rotorR.transform.position);
        }

        void cmd_inference(string inp)
        {
            if (!string.IsNullOrEmpty(inp))
            {
                if(inp.IndexOf('*')>0)
                {
                    //send in the form Rspeed*Lspeed
                    var sep=inp.IndexOf('*');
                    var str1=inp.Substring(sep+1);
                    var str2=inp.Substring(0,sep);
                    var key1=str1[0]; //always contain left speed
                    var key2=str2[0]; //always contain right speed
                    var value1=0.0f;
                    float.TryParse(str1.Substring(1),out value1);
                    var value2=0.0f;
                    float.TryParse(str2.Substring(1),out value2);
                    speedR=value2;
                    speedL=value1;
                    control_msg="";
                }
            }
        }

        float[] TeleportExtract(string input)
        {
            // Regular expression to match numbers within parentheses
            Regex regex = new Regex(@"-?\d+");
            MatchCollection matches = regex.Matches(input);

            float[] numbers = new float[matches.Count];
            for (int i = 0; i < matches.Count; i++)
            {
                numbers[i] = float.Parse(matches[i].Value);
            }

            return numbers;
        }
    }
}
