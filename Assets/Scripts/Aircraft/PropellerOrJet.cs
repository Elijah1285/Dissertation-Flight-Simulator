using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PropellerOrJet : MonoBehaviour
{
    float current_rotation_speed = 0.0f;
    float target_rotation_speed = 0.0f;

    [SerializeField] float idle_rotation_speed;
    [SerializeField] float rotation_speed_throttle_increase_multiplier; //how much does the propeller/jet speed up with throttle
    [SerializeField] float angular_acceleration;

    void Update()
    {
        if (current_rotation_speed < target_rotation_speed)
        {
            current_rotation_speed += angular_acceleration * Time.deltaTime;

            if (current_rotation_speed > target_rotation_speed)
            {
                current_rotation_speed = target_rotation_speed;
            }
        }
        else if (current_rotation_speed > target_rotation_speed)
        {
            current_rotation_speed -= angular_acceleration * Time.deltaTime;

            if (current_rotation_speed < target_rotation_speed)
            {
                current_rotation_speed = target_rotation_speed;
            }
        }

        transform.Rotate(0.0f, 0.0f, current_rotation_speed * Time.deltaTime);
    }

    public void setTargetRotationSpeed(float new_target_rotation_speed)
    {
        target_rotation_speed = new_target_rotation_speed;
    }

    public float getIdleRotationSpeed()
    {
        return idle_rotation_speed;
    }

    public float getCurrentRotationSpeed()
    {
        return current_rotation_speed;
    }

    public float getRotationSpeedThrottleIncreaseMultiplier()
    {
        return rotation_speed_throttle_increase_multiplier;
    }
}
