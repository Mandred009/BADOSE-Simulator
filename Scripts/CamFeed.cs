// Script for streaming camera data of drone through ros2 node

using System;
using UnityEngine;
using ROS2;

namespace CameraFeed
{
    public class CamFeed : MonoBehaviour
    {
        private ROS2UnityComponent ros2Unity;
        private ROS2Node ros2Node;
        private IPublisher<std_msgs.msg.String> pos_pub;
        public Camera sensorCamera;
        private Texture2D camText; // To store the camera video on rendering texture

        void Start()
        {
            // Initialize ROS2 Unity Component
            ros2Unity = GetComponent<ROS2UnityComponent>();

            if (ros2Unity == null)
            {
                Debug.LogError("ROS2UnityComponent not found on GameObject.");
                return;
            }

            if (sensorCamera == null)
            {
                Debug.LogError("Sensor Camera is not assigned.");
                return;
            }

            // Check if the camera has a target texture assigned
            if (sensorCamera.targetTexture == null)
            {
                Debug.LogError("Sensor Camera does not have a target texture assigned.");
                return;
            }

            // Create a Texture2D to store the camera feed
            camText = new Texture2D(sensorCamera.targetTexture.width, sensorCamera.targetTexture.height, TextureFormat.RGB24, false);

            // Initialize ROS2 node and publisher
            InitializeROS2();
        }

        void Update()
        {
            if (ros2Unity != null && ros2Unity.Ok())
            {
                try
                {
                    // Capture camera feed
                    CaptureCameraFeed();

                    // Convert the captured Texture2D to a base64 string
                    string base64Image = Texture2DToString(camText);

                    // Publish the base64 image string
                    PublishImage(base64Image);
                }
                catch (Exception ex)
                {
                    Debug.LogError("Error capturing or publishing camera feed: " + ex.Message);
                }
            }
        }

        private void InitializeROS2()
        {
            if (ros2Unity != null && ros2Unity.Ok())
            {
                ros2Node = ros2Unity.CreateNode("CamNode" + transform.parent.gameObject.name);
                pos_pub = ros2Node.CreatePublisher<std_msgs.msg.String>("cam_feed_data" + transform.parent.gameObject.name);
                Debug.Log("ROS2 Node and Publisher initialized.");
            }
            else
            {
                Debug.LogError("Failed to initialize ROS2 Node and Publisher.");
            }
        }

        void CaptureCameraFeed()
        {
            // Set the active RenderTexture to the camera's target texture
            var oldRT = RenderTexture.active;
            RenderTexture.active = sensorCamera.targetTexture;

            // Render the camera view and read the pixels into the Texture2D
            sensorCamera.Render();
            camText.ReadPixels(new Rect(0, 0, sensorCamera.targetTexture.width, sensorCamera.targetTexture.height), 0, 0);
            camText.Apply();

            // Restore the original RenderTexture
            RenderTexture.active = oldRT;
        }

        string Texture2DToString(Texture2D texture)
        {
            // Encode the Texture2D to PNG and convert to base64 string
            byte[] textureBytes = texture.EncodeToPNG(); // Use EncodeToJPG() for JPG format
            return Convert.ToBase64String(textureBytes);
        }

        void PublishImage(string base64Image)
        {
            // Create a ROS2 message and publish the base64 image string
            std_msgs.msg.String msg = new std_msgs.msg.String();
            msg.Data = base64Image;
            pos_pub.Publish(msg);
        }
    }
}
