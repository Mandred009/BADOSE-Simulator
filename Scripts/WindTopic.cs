// Script to create a wind characteristic node and takes data in this format
// W(wind_force,rotation_x,rotation_y,rotation_z)

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Text.RegularExpressions;
using TMPro;  
using System;

namespace ROS2
{
    public class WindTopic : MonoBehaviour
    {
        private ROS2UnityComponent ros2Unity;
        private ROS2Node ros2Node;
        private ISubscription<std_msgs.msg.String> control_sub;
        private string control_msg = "W(0,0,0,0)"; //(force,rotationx,rotationy,rotationz)
        public TextMeshProUGUI wind_txt;

        void Start()
        {
            ros2Unity = GetComponent<ROS2UnityComponent>();
        }


        void FixedUpdate()
        {
            if (ros2Node == null && ros2Unity.Ok())
            {
                ros2Node = ros2Unity.CreateNode("WindNode");
                control_sub = ros2Node.CreateSubscription<std_msgs.msg.String>(
                "wind_control", msg => {
                    Debug.Log(msg.Data);
                    control_msg = msg.Data;
                });
            }
            if (control_msg[0]=='W')
            {
                float[] extractedvalues = WindValuesExtract(control_msg);

                transform.rotation=Quaternion.Euler(new Vector3(extractedvalues[1],extractedvalues[2],extractedvalues[3]));
                StaticData.wind_force=extractedvalues[0];
                var windval=transform.forward*extractedvalues[0];
                wind_txt.text="WIND: "+Math.Round(windval[0], 2).ToString()+" "+Math.Round(windval[1], 2).ToString()+" "+Math.Round(windval[2], 2).ToString()+" N ";

            }
            
        }
        float[] WindValuesExtract(string input)
        {
            // Regular expression to match numbers within parentheses
            Regex regex = new Regex(@"-?\d+(\.\d+)?");
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

