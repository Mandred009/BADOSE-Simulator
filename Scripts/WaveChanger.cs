// Script to handle wave turbulence changes depending on slider value

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering.HighDefinition;
using UnityEngine.UI;

public class WaveChanger : MonoBehaviour
{
    public WaterSurface water;
    public Slider windSpeed;

    void Start()
    {
        water = FindObjectOfType<WaterSurface>();
    }


    void Update()
    {
        water.largeWindSpeed=windSpeed.value;
    }
}
