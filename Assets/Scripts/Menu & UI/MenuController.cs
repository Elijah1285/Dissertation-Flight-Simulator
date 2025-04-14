using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;

public class MenuController : MonoBehaviour
{
    public void startGame()
    {
        SceneManager.LoadScene("SCN_LoadingScreen");
    }

    public void quitGame()
    {
        Application.Quit();
    }
}
