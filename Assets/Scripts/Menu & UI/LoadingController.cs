using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.UI;

public class LoadingController : MonoBehaviour
{
    [SerializeField] Slider loading_bar;

    void Start()
    {
        StartCoroutine(loadLevelAsync());
    }

    IEnumerator loadLevelAsync()
    {
        AsyncOperation load_operation = SceneManager.LoadSceneAsync("SCN_FlightEnvironment");

        while (!load_operation.isDone)
        {
            float progress_value = Mathf.Clamp01(load_operation.progress / 0.9f);
            loading_bar.value = progress_value;
            yield return null;
        }
    }
}
