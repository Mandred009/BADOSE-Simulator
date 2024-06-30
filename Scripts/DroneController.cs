// Script to handle drone physics and control

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Text.RegularExpressions;

namespace ROS2
{
    public class DroneController : MonoBehaviour
    {
        public GameObject motor1;
        public GameObject motor2;
        public GameObject motor3;
        public GameObject motor4;
        public Transform windPoint;
        private Rigidbody rb;
        private Transform t_m1;
        private Transform t_m2;
        private Transform t_m3;
        private Transform t_m4;
        private ROS2UnityComponent ros2Unity;
        private ROS2Node ros2Node;
        private ISubscription<std_msgs.msg.String> control_sub;
        private string control_msg = "0*0*0*0"; // Initialize with default value
        public string drone_no;
        public bool drone_cam_val;
        public GameObject droneCam;

        float[] thrust_vals;

        private float mass_each;

        void Start()
        {
            rb = this.GetComponent<Rigidbody>();
            rb.mass=StaticData.drone_mass;
            mass_each = rb.mass / 4.0f; // Calculating the mass for each motor unit

            t_m1 = motor1.GetComponent<Transform>();
            t_m2 = motor2.GetComponent<Transform>();
            t_m3 = motor3.GetComponent<Transform>();
            t_m4 = motor4.GetComponent<Transform>();

            drone_no = gameObject.name;
            drone_cam_val = StaticData.drone_cam;

            ros2Unity = GetComponent<ROS2UnityComponent>();
            thrust_vals = ExtractThrustValues("0*0*0*0");

            droneCam.SetActive(drone_cam_val);

        }

        void FixedUpdate()
        {
            if (ros2Node == null && ros2Unity.Ok()) // Checking for successful ros2 connection
            {
                ros2Node = ros2Unity.CreateNode("DroneROS2" + drone_no);
                control_sub = ros2Node.CreateSubscription<std_msgs.msg.String>(
                "control_" + drone_no, msg => {
                    Debug.Log(msg.Data);
                    control_msg = msg.Data;
                });
            }
            

            if(control_msg[0]=='T') // Checking for teleport values and command
            {
                float[] extractedCoords = TeleportExtract(control_msg);
                transform.position=new Vector3(extractedCoords[0],transform.position.y,extractedCoords[1]);
                transform.rotation=Quaternion.Euler(new Vector3(0,extractedCoords[2],0));
            }
            else
            {
                // Extract thrust values and ensure it is not null
                float[] new_thrust_vals = ExtractThrustValues(control_msg);
                if (new_thrust_vals != null)
                {
                    thrust_vals = new_thrust_vals;
                }

            }

            // Using thrust values to apply force at the rotors
            rb.AddForceAtPosition(thrust_vals[0] * transform.up, t_m1.position, ForceMode.Force);
            rb.AddForceAtPosition(thrust_vals[1] * transform.up, t_m2.position, ForceMode.Force);
            rb.AddForceAtPosition(thrust_vals[2] * transform.up, t_m3.position, ForceMode.Force);
            rb.AddForceAtPosition(thrust_vals[3] * transform.up, t_m4.position, ForceMode.Force);
            
            // Torque application due to rotor movement.
            var torque_net=(thrust_vals[0]+thrust_vals[3])-(thrust_vals[1]+thrust_vals[2]);
            rb.AddTorque(torque_net * transform.up);

            // Wind Force

            rb.AddForce(StaticData.wind_force*windPoint.forward,ForceMode.Force);
        }

        float[] ExtractThrustValues(string input)
        {
            // Split the input string by the '*' character
            string[] parts = input.Split('*');

            // Create a list to hold the valid numbers
            List<float> nums = new List<float>();

            // Convert each substring to an integer and store it in the list
            foreach (string part in parts)
            {
                if (float.TryParse(part, out float number))
                {
                    nums.Add(number);
                }
                else
                {
                    Debug.LogError($"Failed to parse '{part}' as an integer.");
                    // Return a default array if parsing fails
                    return new float[] { 0, 0, 0, 0 };
                }
            }

            // Ensure the list has exactly four elements
            if (nums.Count != 4)
            {
                Debug.LogError("Input string does not contain exactly four thrust values.");
                return new float[] { 0, 0, 0, 0 };
            }

            // Convert the list to an array
            return nums.ToArray();
        }
        float[] TeleportExtract(string input)
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
