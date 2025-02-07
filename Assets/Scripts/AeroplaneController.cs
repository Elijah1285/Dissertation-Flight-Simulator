using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.UI;
using TMPro;

public class AeroplaneController : MonoBehaviour
{  
    bool flaps_deployed;
    bool airbrake_deployed;

    float angle_of_attack;
    float sideslip;

    float pedals_indicator_x_center_position;

    Vector2 stick_indicator_center_position;

    Vector3 velocity;
    Vector3 local_velocity;
    Vector3 last_velocity;

    Vector3 local_angular_velocity;
    Vector3 local_g_force;

    Rigidbody rb;

    [Header("Engine/Thrust Settings")]
    [SerializeField] float max_thrust;

    [Header("Lift/Angle of Attack Settings")]
    [SerializeField] float lift_power;
    [SerializeField] float flaps_lift_power;
    [SerializeField] float flaps_angle_of_attack_bias;

    [SerializeField] AnimationCurve lift_angle_of_attack_curve;
    [SerializeField] AnimationCurve rudder_angle_of_attack_curve;

    [Header("Drag/Friction Settings")]
    [SerializeField] float induced_drag;
    [SerializeField] float airbrake_drag;
    [SerializeField] float flaps_drag;

    [SerializeField] AnimationCurve drag_forward;
    [SerializeField] AnimationCurve drag_back;
    [SerializeField] AnimationCurve drag_left;
    [SerializeField] AnimationCurve drag_right;
    [SerializeField] AnimationCurve drag_top;
    [SerializeField] AnimationCurve drag_bottom;

    [SerializeField] AnimationCurve induced_drag_curve;
    [SerializeField] AnimationCurve rudder_induced_drag_curve;

    [SerializeField] Collider left_main_gear_collider;
    [SerializeField] Collider right_main_gear_collider;

    [SerializeField] PhysicMaterial normal_gear_physics_material;
    [SerializeField] PhysicMaterial braking_gear_physics_material;

    [Header("Control Settings")]
    [SerializeField] Vector3 turn_speed;
    [SerializeField] Vector3 turn_acceleration;
    [SerializeField] AnimationCurve steering_curve;

    [Header("Misc")]
    [SerializeField] float rudder_power;
    [SerializeField] float prop_idle_rotation_speed;
    [SerializeField] Propeller propeller;
    [SerializeField] Image stick_indicator;
    [SerializeField] Image pedals_indicator;

    [Header("UI objects")]
    [SerializeField] TextMeshProUGUI engine_text;
    [SerializeField] TextMeshProUGUI throttle_text;
    [SerializeField] TextMeshProUGUI brakes_text;
    [SerializeField] TextMeshProUGUI airspeed_text;
    [SerializeField] TextMeshProUGUI altitude_text;
    [SerializeField] TextMeshProUGUI rate_of_climb_text;

    //input
    bool engine_running = false;
    bool brakes_active = true;
    float throttle_input = 0.0f;
    Vector3 control_surface_input = Vector3.zero;

    Vector2 stick_input;
    float pedals_input;

    AircraftControls aircraft_controls;

    void Awake()
    {
        //initialise controls object
        aircraft_controls = new AircraftControls();

        //buttons
        aircraft_controls.Flight.Engine.performed += context => toggleEngine();

        aircraft_controls.Flight.ThrottleUp.performed += context => increaseThrottleInput();
        aircraft_controls.Flight.ThrottleDown.performed += context => decreaseThrottleInput();
        
        aircraft_controls.Flight.ToggleBrakes.performed += context => toggleBrakes();

        //axis
        aircraft_controls.Flight.Stick.performed += context => stick_input = context.ReadValue<Vector2>();
        aircraft_controls.Flight.Pedals.performed += context => pedals_input = context.ReadValue<float>();
    }

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        aircraft_controls.Flight.Enable();
        propeller.setRotationSpeed(prop_idle_rotation_speed);

        stick_indicator_center_position = stick_indicator.GetComponent<RectTransform>().position;
        pedals_indicator_x_center_position = pedals_indicator.GetComponent<RectTransform>().position.x;
    }

    void Update()
    {
        updateInput();
        updateUI();
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
    }

    void updateInput()
    {
        //joystick input
        control_surface_input.x = stick_input.y; //pitch input
        control_surface_input.y = -pedals_input; //yaw input
        control_surface_input.z = -stick_input.x; //roll input

        //update controls indicators
        stick_indicator.GetComponent<RectTransform>().position = stick_indicator_center_position + (stick_input * 100.0f);
        pedals_indicator.GetComponent<RectTransform>().position = new Vector2(pedals_indicator_x_center_position + (pedals_input * -120.0f), pedals_indicator.GetComponent<RectTransform>().position.y);
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
        //set 0 angle of attack if very slow or stationary
        if (local_velocity.sqrMagnitude < 0.1f)
        {
            angle_of_attack = 0.0f;
            sideslip = 0.0f;
            return;
        }

        //calcculate vertical angle of attack
        angle_of_attack = Mathf.Atan2(-local_velocity.y, local_velocity.z);

        //calculate lateral angle of attack A.K.A sideslip
        sideslip = Mathf.Atan2(local_velocity.x, local_velocity.z);
    }

    void calculateGForce(float dt)
    {
        var inverted_rotation = Quaternion.Inverse(rb.rotation);
        var acceleration = (velocity - last_velocity) / dt;
        local_g_force = inverted_rotation * acceleration;   
        last_velocity = velocity;
    }

    Vector3 calculateLift(float _angle_of_attack, Vector3 right_axis, float _lift_power, AnimationCurve _angle_of_attack_curve, AnimationCurve _induced_drag_curve)
    {
        var lift_velocity = Vector3.ProjectOnPlane(local_velocity, right_axis);
        var lift_velocity_square_magnitude = lift_velocity.sqrMagnitude;

        //coefficient varies with angle of attack
        var lift_coefficient = _angle_of_attack_curve.Evaluate(_angle_of_attack * Mathf.Rad2Deg);
        var lift_force = lift_velocity_square_magnitude * lift_coefficient * _lift_power;

        //lift is perpendicular to velocity
        var lift_direction = Vector3.Cross(lift_velocity.normalized, right_axis);
        var lift = lift_direction * lift_force;

        //induced drag varies with square of lift coefficient
        var drag_force = lift_coefficient * lift_coefficient;
        var drag_direction = -lift_velocity.normalized;
        var induced_drag = drag_direction * lift_velocity_square_magnitude * drag_force * this.induced_drag * _induced_drag_curve.Evaluate(Mathf.Max(0, local_velocity.z));

        return lift + induced_drag;
    }

    float calculateSteering(float dt, float angular_velocity, float target_angular_velocity, float angular_acceleration)
    {
        var error = target_angular_velocity - angular_velocity;
        var _acceleration = angular_acceleration * dt;

        return Mathf.Clamp(error, -_acceleration, _acceleration);
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

    void updateUI()
    {
        airspeed_text.text = "Airspeed: " + Mathf.FloorToInt(transform.InverseTransformDirection(rb.velocity).z * 1.944f).ToString() + " kt";
        altitude_text.text = "Altitude: " + Mathf.FloorToInt(transform.position.y * 3.281f).ToString() + " ft";
        rate_of_climb_text.text = "Rate of Climb: " + Mathf.FloorToInt(rb.velocity.y * 196.9f).ToString() + " fpm";
    }

    void updateThrust()
    {
        if (engine_running)
        {
            rb.AddRelativeForce(throttle_input * max_thrust * Vector3.forward);
        }
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
        if (local_velocity.sqrMagnitude < 1f)
        {
            return;
        }

        float _flaps_lift_power = flaps_deployed ? this.flaps_lift_power : 0;
        float _flaps_angle_of_attack_bias = flaps_deployed ? this.flaps_angle_of_attack_bias : 0;

        var lift_force = calculateLift
            (
            angle_of_attack + (_flaps_angle_of_attack_bias * Mathf.Deg2Rad), Vector3.right,
            lift_power + _flaps_lift_power, lift_angle_of_attack_curve, induced_drag_curve
            );

        var yaw_force = calculateLift(sideslip, Vector3.up, rudder_power, rudder_angle_of_attack_curve, rudder_induced_drag_curve);

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

    void updatePropSpeed()
    {
        propeller.setRotationSpeed(prop_idle_rotation_speed + (throttle_input * 5.0f));
    }

    void updateEngineText()
    {
        if (engine_running)
        {
            engine_text.text = "Engine: RUNNING";
        }
        else
        {
            engine_text.text = "Engine: STOPPED";
        }
    }

    void updateThrottleText()
    {
        throttle_text.text = "Throttle: " + Mathf.FloorToInt(throttle_input * 100.0f).ToString() + "%";
    }

    void updateBrakesText()
    {
        if (brakes_active)
        {
            brakes_text.text = "Brakes: Engaged";
        }
        else
        {
            brakes_text.text = "Brakes: Disengaged";
        }
    }

    void increaseThrottleInput()
    {
        if (throttle_input < 1.0f)
        {
            throttle_input += 0.1f;
        }

        updatePropSpeed();
        updateThrottleText();
    }

    void decreaseThrottleInput()
    {
        if (throttle_input > 0.0f)
        {
            throttle_input -= 0.1f;
        }

        updatePropSpeed();
        updateThrottleText();
    }

    void toggleEngine()
    {
        engine_running = !engine_running;

        if (engine_running)
        {
            propeller.setSpinning(true);
        }
        else
        {
            propeller.setSpinning(false);
        }

        updateEngineText();
    }

    void toggleBrakes()
    {
        brakes_active = !brakes_active;

        if (brakes_active)
        {
            left_main_gear_collider.material = braking_gear_physics_material;
            right_main_gear_collider.material = braking_gear_physics_material;
        }
        else
        {
            left_main_gear_collider.material = normal_gear_physics_material;
            right_main_gear_collider.material = normal_gear_physics_material;
        }

        updateBrakesText();
    }
}
