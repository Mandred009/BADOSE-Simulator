// Script to handle values and interactions in the main menu

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.SceneManagement;
using TMPro;

public class MainMenu : MonoBehaviour
{
    public Slider drone_no;
    public Slider boat_no;
    public Slider drone_mass;
    public Slider boat_mass;
    public Toggle drone_cam;
    public TextMeshProUGUI drone_txt;
    public TextMeshProUGUI boat_txt;
    public TextMeshProUGUI drone_mass_txt;
    public TextMeshProUGUI boat_mass_txt;

    void Update()
    {
        drone_txt.text=drone_no.value.ToString();
        boat_txt.text=boat_no.value.ToString();
        drone_mass_txt.text=drone_mass.value.ToString();
        boat_mass_txt.text=boat_mass.value.ToString();
    }
    
    public void ChangeScene() // Saves values and starts simulation on start sim button press
    {
        StaticData.ships_no=(int)boat_no.value;
        StaticData.drone_no=(int)drone_no.value;
        StaticData.drone_mass=(int)drone_mass.value;
        StaticData.ship_mass=(int)boat_mass.value;
        StaticData.drone_cam=drone_cam.isOn;

        SceneManager.LoadScene("OutdoorsScene");
    }
    
}
