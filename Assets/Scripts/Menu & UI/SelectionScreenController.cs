using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;

public class SelectionScreenController : MonoBehaviour
{
    public void selectCessna172SP()
    {
        PlayerPrefs.SetString("chosen_aircraft", "cessna_172sp");
        loadGame();
    }

    public void selectPiperPA18SuperCub()
    {
        PlayerPrefs.SetString("chosen_aircraft", "piper_pa18_super_cub");
        loadGame();
    }

    public void selectDeHavillandCanadaDash8Q400()
    {
        PlayerPrefs.SetString("chosen_aircraft", "dehavilland_canada_dash8_q400");
        loadGame();
    }

    public void selectBoeing737_800()
    {
        PlayerPrefs.SetString("chosen_aircraft", "boeing_737_800");
        loadGame();
    }

    public void selectBoeing747_400F()
    {
        PlayerPrefs.SetString("chosen_aircraft", "boeing_747_400f");
        loadGame();
    }

    void loadGame()
    {
        SceneManager.LoadScene("SCN_LoadingScreen");
    }
}
