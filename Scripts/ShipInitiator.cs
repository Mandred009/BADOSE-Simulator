// Script to spawn the required number of boats
// Unlike drones, boats are not pooled and are Instantiated on func call


using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ShipInitiator : MonoBehaviour
{
    public int no_of_ships;
    public GameObject ship_prefab;
    public GameObject parentObject;
    public Vector3 spawnPosition = new Vector3(0, 0, 0);
    public float xplus=0;

    void Start()
    {
        no_of_ships=StaticData.ships_no;
        spawnPosition=parentObject.transform.position;
        xplus=parentObject.transform.position.x;
        SpawnShips();
    }

    void SpawnShips()
    {
        for (int i = 0; i < no_of_ships; i++)
        {
            GameObject newObject = Instantiate(ship_prefab, spawnPosition, Quaternion.Euler(0,0,0));
            // newObject.transform.position=parentObject.transform.position;
            newObject.name = "ship" + (i + 1).ToString();

            Color randomColor = new Color(Random.value, Random.value, Random.value);

            // Change the color of the object
            Renderer renderer = newObject.GetComponent<Renderer>();
            if (renderer != null)
            {
                renderer.material.color = randomColor;
            }

            xplus += 500;
            spawnPosition.x = xplus;

        }

    }
}
