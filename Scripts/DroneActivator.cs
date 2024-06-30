// Script to spawn desired number of drones in the simulation

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DroneActivator : MonoBehaviour
{
    [SerializeField]
    public List<GameObject> drone_objects = new List<GameObject>(10);

    public int no_of_drones;

    // Start is called before the first frame update
    void Start()
    {
        no_of_drones = StaticData.drone_no;

        // Ensure the list has the correct number of GameObjects
        if (drone_objects.Count < no_of_drones)
        {
            Debug.LogError("Not enough GameObjects assigned in the inspector.");
            return;
        }

        // Call the SpawnDrones method to activate the drones
        SpawnDrones();
    }

    void SpawnDrones() // Drones are already pooled and are just activated on call
    {
        for (int i = 0; i < no_of_drones; i++)
        {
            if (drone_objects[i] != null)
            {
                drone_objects[i].SetActive(true);
            }
            else
            {
                Debug.LogWarning("Drone object at index " + i + " is null.");
            }
        }
    }
}
