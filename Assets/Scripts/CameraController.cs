using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraController : MonoBehaviour
{
    [SerializeField] float x_sensitivity;
    [SerializeField] float y_sensitivity;

    float x_rotation;
    float y_rotation;

    void Update()
    {
        //mouse input
        float mouse_x = Input.GetAxisRaw("Mouse X") * x_sensitivity * Time.deltaTime;
        float mouse_y = Input.GetAxisRaw("Mouse Y") * y_sensitivity * Time.deltaTime;

        x_rotation -= mouse_y; //rotation around the x (pitch) axis using y (vertical) mouse movement
        y_rotation += mouse_x; //rotation around the y (yaw) axis using x (horizontal) mouse movement

        //rotate camera
        transform.localRotation = Quaternion.Euler(x_rotation, y_rotation, 0.0f);
    }
}
