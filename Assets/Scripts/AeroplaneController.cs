using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AeroplaneController : MonoBehaviour
{
    float throttle;
    float max_thrust;

    float angle_of_attack;
    float angle_of_attack_yaw;

    Vector3 velocity;
    Vector3 local_velocity;
    Vector3 last_velocity;

    Vector3 local_angular_velocity;

    Vector3 local_g_force;

    Rigidbody rb;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
    }

    void FixedUpdate()
    {
        float dt = Time.fixedDeltaTime;

        getInput();

        calculateState(dt);
        calculateAngleOfAttack();
        calculateGForce(dt);
    }

    void getInput()
    {
        //throttle = 
    }

    void calculateState(float dt)
    {
        var inverted_rotation = Quaternion.Inverse(rb.rotation);
        velocity = rb.velocity;
        local_velocity = inverted_rotation * velocity; //transform world velocity into local space
        local_angular_velocity = inverted_rotation * rb.angularVelocity; // transform into local space
       
    }

    void calculateAngleOfAttack()
    {
        if (local_velocity.sqrMagnitude < 0.1f)
        {
            angle_of_attack = 0.0f;
            angle_of_attack_yaw = 0.0f;
            return;
        }

        angle_of_attack = Mathf.Atan2(-local_velocity.y, local_velocity.z);
        angle_of_attack_yaw = Mathf.Atan2(local_velocity.x, local_velocity.z);
    }

    void calculateGForce(float dt)
    {
        var inverted_rotation = Quaternion.Inverse(rb.rotation);
        var acceleration = (velocity - last_velocity) / dt;
        local_g_force = inverted_rotation * acceleration;   
        last_velocity = velocity;
    }

    void updateThrust()
    {
        rb.AddRelativeForce(throttle * max_thrust * Vector3.forward);
    }
}
