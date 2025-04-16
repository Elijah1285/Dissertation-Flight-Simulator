using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PropellerOrFan : MonoBehaviour
{
    bool spinning = false;
    float rotation_speed;

    void Update()
    {
        if (spinning)
        {
            transform.Rotate(0.0f, 0.0f, rotation_speed);
        }
    }

    public void setSpinning(bool new_spinning)
    {
        spinning = new_spinning;
    }

    public void setRotationSpeed(float new_rotation_speed)
    {
        rotation_speed = new_rotation_speed;
    }
}
