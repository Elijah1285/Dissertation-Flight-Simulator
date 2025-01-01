using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AeroplaneController : MonoBehaviour
{  
    [SerializeField] float max_thrust;

    bool flaps_deployed;
    [SerializeField] float flaps_drag;

    bool airbrake_deployed;
    [SerializeField] float airbrake_drag;

    float angle_of_attack;
    float angle_of_attack_yaw;

    [SerializeField] float lift_power;
    [SerializeField] float induced_drag;
    [SerializeField] float flaps_lift_power;
    [SerializeField] float flaps_angle_of_attack_bias;

    [SerializeField] float rudder_power;

    Vector3 velocity;
    Vector3 local_velocity;
    Vector3 last_velocity;

    Vector3 local_angular_velocity;

    Vector3 local_g_force;

    Rigidbody rb;

    [SerializeField] AnimationCurve lift_angle_of_attack_curve;
    [SerializeField] AnimationCurve rudder_angle_of_attack_curve;

    [Header("Drag Settings")]
    [SerializeField] AnimationCurve drag_forward;
    [SerializeField] AnimationCurve drag_back;
    [SerializeField] AnimationCurve drag_left;
    [SerializeField] AnimationCurve drag_right;
    [SerializeField] AnimationCurve drag_top;
    [SerializeField] AnimationCurve drag_bottom;

    [Header("Steering Settings")]
    [SerializeField] Vector3 turn_speed;
    [SerializeField] Vector3 turn_acceleration;
    [SerializeField] AnimationCurve steering_curve;

    //input
    float throttle = 0.0f;
    Vector3 control_surface_input = Vector3.zero;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
    }

    void Update()
    {
        getInput();
    }

    void FixedUpdate()
    {
        float dt = Time.fixedDeltaTime;

        calculateState(dt);
        calculateAngleOfAttack();
        calculateGForce(dt);
        updateThrust();
        updateDrag();
        updateLift();
        updateSteering(dt);

        Debug.Log("Pitch: " + Input.GetAxis("Pitch"));
        Debug.Log("Yaw: " + Input.GetAxis("Yaw"));
        Debug.Log("Roll: " + Input.GetAxis("Roll"));
        Debug.Log("Throttle: " + Input.GetAxis("Throttle"));
    }

    void getInput()
    {
        //keyboard / mouse
        //throttle
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

        //HOTAS (flight controllers)
        //throttle
        throttle = Input.GetAxis("Throttle");

        //control surfaces
        control_surface_input.x = -Input.GetAxis("Pitch");
        control_surface_input.y = Input.GetAxis("Yaw");
        control_surface_input.z = -Input.GetAxis("Roll");
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

    Vector3 calculateLift(float angle_of_attack, Vector3 right_axis, float lift_power, AnimationCurve angle_of_attack_curve)
    {
        var lift_velocity = Vector3.ProjectOnPlane(local_velocity, right_axis);
        var lift_velocity_squared = lift_velocity.sqrMagnitude;

        //coefficient varies with angle of attack
        var lift_coefficient = angle_of_attack_curve.Evaluate(angle_of_attack * Mathf.Rad2Deg);
        var lift_force = lift_velocity_squared * lift_coefficient * lift_power;

        //lift is perpendicular to velocity
        var lift_direction = Vector3.Cross(lift_velocity.normalized, right_axis);
        var lift = lift_direction * lift_force;

        //induced drag varies with square of lift coefficient
        var drag_force = lift_coefficient * lift_coefficient * this.induced_drag;
        var drag_direction = -lift_velocity.normalized;
        var induced_drag = drag_direction * lift_velocity_squared * drag_force;

        return lift + induced_drag;
    }

    float calculateSteering(float dt, float angular_velocity, float target_angular_velocity, float angular_acceleration)
    {
        var error = target_angular_velocity - angular_velocity;
        var _acceleration = angular_acceleration * dt;

        return Mathf.Clamp(error, -_acceleration, _acceleration);
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

        rb.AddRelativeForce(drag); 
    }

    void updateLift()
    {
        if (local_velocity.sqrMagnitude < 1.0f)
        {
            return;
        }

        float _flaps_lift_power = flaps_deployed ? this.flaps_lift_power : 0;
        float _flaps_angle_of_attack_bias = flaps_deployed ? this.flaps_angle_of_attack_bias : 0;

        var lift_force = calculateLift
            (
            angle_of_attack + (_flaps_angle_of_attack_bias * Mathf.Deg2Rad), Vector3.right,
            lift_power + _flaps_lift_power, lift_angle_of_attack_curve
            );

        var yaw_force = calculateLift(angle_of_attack_yaw, Vector3.up, rudder_power, rudder_angle_of_attack_curve);

        rb.AddRelativeForce(lift_force);
        rb.AddRelativeForce(yaw_force); 
    }

    void updateSteering(float dt)
    {
        var speed = Mathf.Max(0, local_velocity.z);
        var steering_power = steering_curve.Evaluate(speed);

        var target_angular_velocity = Vector3.Scale(control_surface_input, turn_speed * steering_power);
        var angular_velocity = local_angular_velocity * Mathf.Rad2Deg;

        var correction = new Vector3
            (
            calculateSteering(dt, angular_velocity.x, target_angular_velocity.x, turn_acceleration.x * steering_power),
            calculateSteering(dt, angular_velocity.y, target_angular_velocity.y, turn_acceleration.y * steering_power),
            calculateSteering(dt, angular_velocity.z, target_angular_velocity.z, turn_acceleration.z * steering_power)
            );

        rb.AddRelativeTorque(correction * Mathf.Deg2Rad, ForceMode.VelocityChange);
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
