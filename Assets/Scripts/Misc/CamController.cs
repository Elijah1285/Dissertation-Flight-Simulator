using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CamController : MonoBehaviour
{
    [SerializeField] GameObject first_person_camera;
    [SerializeField] GameObject third_person_camera;

    public void togglePerspective()
    {
        if (first_person_camera.activeSelf)
        {
            first_person_camera.SetActive(false);
            third_person_camera.SetActive(true);
        }
        else
        {
            first_person_camera.SetActive(true);
            third_person_camera.SetActive(false);
        }
    }
}
