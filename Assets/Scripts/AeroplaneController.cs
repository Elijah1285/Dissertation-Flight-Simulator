using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AeroplaneController : MonoBehaviour
{
    float throttle = 0.0f;
    [SerializeField] float max_thrust;

    bool flaps_deployed;
    [SerializeField] float flaps_drag;

    bool airbrake_deployed;
    [SerializeField] float airbrake_drag;

    float angle_of_attack;
    float angle_of_attack_yaw;

    Vector3 velocity;
    Vector3 local_velocity;
    Vector3 last_velocity;

    Vector3 local_angular_velocity;

    Vector3 local_g_force;

    Rigidbody rb;

    [Header("Drag Settings")]
    [SerializeField] AnimationCurve drag_forward;
    [SerializeField] AnimationCurve drag_back;
    [SerializeField] AnimationCurve drag_left;
    [SerializeField] AnimationCurve drag_right;
    [SerializeField] AnimationCurve drag_top;
    [SerializeField] AnimationCurve drag_bottom;

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
        updateThrust();
        updateDrag();
    }

    void getInput()
    {
        if (Input.GetKey("up"))
        {
            if (throttle < 1.0f)
            {
                throttle += Time.deltaTime;
            }
        }
        if (Input.GetKey("down"))
        {
            if (throttle > 0.0f)
            {
                throttle -= Time.deltaTime;
            }
        }
        Debug.Log(throttle);
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

    void updateDrag()
    {
        var _local_velocity = local_velocity;
        var local_velocity_squared = _local_velocity.sqrMagnitude;

        //account for drag from airbrakes/flaps
        float airbrake_drag = airbrake_deployed ? this.airbrake_drag : 0;
        float flaps_drag = flaps_deployed ? this.flaps_drag : 0;

        //calculate coefficient of drag depending on direction of velocity
        var coefficient = scale6
            (
            _local_velocity.normalized,
            drag_right.Evaluate(Mathf.Abs(_local_velocity.x)), drag_left.Evaluate(Mathf.Abs(_local_velocity.x)),
            drag_top.Evaluate(Mathf.Abs(_local_velocity.y)), drag_bottom.Evaluate(Mathf.Abs(_local_velocity.y)),
            drag_forward.Evaluate(Mathf.Abs(_local_velocity.z)) + airbrake_drag + flaps_drag, //include drag from airbrake/flaps for forward coefficient
            drag_back.Evaluate(Mathf.Abs(_local_velocity.z))
            );

        var drag = coefficient.magnitude * local_velocity_squared * -_local_velocity.normalized; //drag is opposite to direction of velocity
        Debug.Log(drag);

        rb.AddRelativeForce(drag); 
    }


    public static Vector3 scale6
        (
        Vector3 value,
        float positive_x, float negative_x,
        float positive_y, float negative_y,
        float positive_z, float negative_z
        )
    {
        Vector3 result = value;
        
        //x
        if (result.x > 0)
        {
            result.x *= positive_x;
        }
        else if (result.x < 0)
        {
            result.x *= negative_x;
        }

        //y
        if (result.y > 0)
        {
            result.y *= positive_y;
        }
        else if (result.y < 0)
        {
            result.y *= negative_y;
        }

        //z
        if (result.z > 0)
        {
            result.z *= positive_z;
        }
        else if (result.z < 0)
        {
            result.z *= negative_z;
        }

        return result;
    }
}
