using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.UI;
using TMPro;

public class AeroplaneController : MonoBehaviour
{
    bool airbrakes_deployed = false;
    
    bool moving_flaps = false;
    float flaps_state = 0.0f;
    float flaps_current_angle = 0.0f;
    float flaps_target_angle = 0.0f;

    float angle_of_attack = 0.0f;
    float sideslip = 0.0f;

    float pedals_indicator_x_center_position = 0.0f;
    float throttle_indicator_y_idle_position = 0.0f;

    Vector2 stick_indicator_center_position;

    Vector3 velocity;
    Vector3 local_velocity;
    Vector3 last_velocity;

    Vector3 local_angular_velocity;
    Vector3 local_g_force;

    Rigidbody rb;

    Coroutine gear_coroutine;

    [Header("Engine/Thrust Settings")]
    [SerializeField] float max_thrust;

    [Header("Lift/Angle of Attack Settings")]
    [SerializeField] float lift_power;
    [SerializeField] float flaps_lift_power;
    [SerializeField] float flaps_angle_of_attack_increase;

    [SerializeField] AnimationCurve lift_angle_of_attack_curve;
    [SerializeField] AnimationCurve vertical_stabiliser_angle_of_attack_curve;

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
    [SerializeField] AnimationCurve vertical_stabiliser_induced_drag_curve;

    [SerializeField] Collider[] left_main_gear_colliders;
    [SerializeField] Collider[] right_main_gear_colliders;

    [SerializeField] PhysicMaterial normal_gear_physics_material;
    [SerializeField] PhysicMaterial braking_gear_physics_material;

    [Header("Steering Settings")]
    [SerializeField] Vector3 turn_speed;
    [SerializeField] Vector3 base_max_turn_acceleration;
    [SerializeField] AnimationCurve steering_curve;

    [Header("Gear Settings")]
    [SerializeField] bool retractable_gear;
    [SerializeField] RetractableGear[] landing_gear;

    [Header("Misc")]   
    [SerializeField] bool affected_by_prop_torque;
    [SerializeField] float vertical_stabiliser_power;
    [SerializeField] PropellerOrJet[] propellers_or_jets;

    [Header("Control Surface Settings")]
    [SerializeField] float aileron_deflection;
    [SerializeField] float elevator_deflection;
    [SerializeField] float rudder_deflection;
    [SerializeField] float flaps_deflection;

    [SerializeField] float flaps_deploy_rate;

    [SerializeField] Transform left_aileron_pivot;
    [SerializeField] Transform right_aileron_pivot;
    [SerializeField] Transform rudder_pivot;

    [SerializeField] Transform[] elevator_pivots;
    [SerializeField] Transform[] flaps_pivots;   

    [Header("UI objects")]
    [SerializeField] TextMeshProUGUI engine_text;
    [SerializeField] TextMeshProUGUI throttle_text;
    [SerializeField] TextMeshProUGUI flaps_text;
    [SerializeField] TextMeshProUGUI brakes_text;
    [SerializeField] TextMeshProUGUI airspeed_text;
    [SerializeField] TextMeshProUGUI altitude_text;
    [SerializeField] TextMeshProUGUI rate_of_climb_text;
    [SerializeField] Image stick_indicator;
    [SerializeField] Image pedals_indicator;
    [SerializeField] Image throttle_indicator;

    //input variables
    bool engine_running = false;
    bool brakes_active = true;
    float throttle_input = 0.0f;
    Vector3 control_surface_input = Vector3.zero;

    Vector2 stick_input;
    float pedals_input;

    IA_AircraftControls aircraft_controls;

    //initialisation
    void Awake()
    {
        //initialise controls object
        aircraft_controls = new IA_AircraftControls();

        //buttons
        aircraft_controls.Flight.Engine.performed += context => toggleEngine();

        aircraft_controls.Flight.ThrottleUp.performed += context => increaseThrottleInput();
        aircraft_controls.Flight.ThrottleDown.performed += context => decreaseThrottleInput();
        
        aircraft_controls.Flight.ToggleGear.performed += context => toggleGear();
        aircraft_controls.Flight.ToggleBrakes.performed += context => toggleBrakes();
        
        aircraft_controls.Flight.DeployFlaps.performed += context => deployFlaps();
        aircraft_controls.Flight.RetractFlaps.performed += context => retractFlaps();
        
        aircraft_controls.Flight.ToggleAirbrakes.performed += context => toggleAirbrakes();

        //axis
        aircraft_controls.Flight.Stick.performed += context => stick_input = context.ReadValue<Vector2>();
        aircraft_controls.Flight.Pedals.performed += context => pedals_input = context.ReadValue<float>();
    }

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        aircraft_controls.Flight.Enable();

        stick_indicator_center_position = stick_indicator.GetComponent<RectTransform>().position;
        pedals_indicator_x_center_position = pedals_indicator.GetComponent<RectTransform>().position.x;
        throttle_indicator_y_idle_position = throttle_indicator.GetComponent<RectTransform>().position.y;
    }

    //input

    //axis inputs
    void getInput()
    {
        //joystick input
        control_surface_input.x = stick_input.y; //pitch input
        control_surface_input.y = -pedals_input; //yaw input
        control_surface_input.z = -stick_input.x; //roll input

        //update controls indicators
        stick_indicator.GetComponent<RectTransform>().position = stick_indicator_center_position + (stick_input * 100.0f);
        pedals_indicator.GetComponent<RectTransform>().position = new Vector2(pedals_indicator_x_center_position + (pedals_input * -120.0f), pedals_indicator.GetComponent<RectTransform>().position.y);
        throttle_indicator.GetComponent<RectTransform>().position = new Vector2(throttle_indicator.GetComponent<RectTransform>().position.x, throttle_indicator_y_idle_position + (throttle_input * 240.0f));

        //update control surfaces
        left_aileron_pivot.localRotation = Quaternion.Euler(-stick_input.x * aileron_deflection, left_aileron_pivot.localEulerAngles.y, left_aileron_pivot.localEulerAngles.z);
        right_aileron_pivot.localRotation = Quaternion.Euler(stick_input.x * aileron_deflection, right_aileron_pivot.localEulerAngles.y, right_aileron_pivot.localEulerAngles.z);
        rudder_pivot.localRotation = Quaternion.Euler(rudder_pivot.localEulerAngles.x, pedals_input * rudder_deflection, rudder_pivot.localEulerAngles.z);

        for (int i = 0; i < elevator_pivots.Length; i++)
        {
            elevator_pivots[i].localRotation = Quaternion.Euler(-stick_input.y * elevator_deflection, elevator_pivots[i].localEulerAngles.y, elevator_pivots[i].localEulerAngles.z);
        }        
    }

    void increaseThrottleInput()
    {
        if (throttle_input < 1.0f)
        {
            throttle_input += 0.1f;
        }

        updateThrottleText();

        if (engine_running)
        {
            updatePropOrFanTargetSpeed();
        }
    }

    void decreaseThrottleInput()
    {
        if (throttle_input > 0.0f)
        {
            throttle_input -= 0.1f;
        }

        updateThrottleText();

        if (engine_running)
        {
            updatePropOrFanTargetSpeed();
        }
    }

    void deployFlaps()
    {
        if (flaps_state < 1.0f)
        {
            flaps_state += 0.25f;
        }

        updateFlapsTargetAngleAndUI();
    }

    void retractFlaps()
    {
        if (flaps_state > 0.0f)
        {
            flaps_state -= 0.25f;
        }

        updateFlapsTargetAngleAndUI();
    }

    //boolean inputs
    void toggleEngine()
    {
        engine_running = !engine_running;

        //if (engine_running)
        //{
        //    for (int i = 0; i < propellers_or_jets.Length; i++)
        //    {
        //        propellers_or_jets[i].setSpinning(true);
        //    }            
        //}
        //else
        //{
        //    for (int i = 0; i < propellers_or_jets.Length; i++)
        //    {
        //        propellers_or_jets[i].setSpinning(false);
        //    }
        //}

        updateEngineText();
    }

    void toggleGear()
    {
        if (retractable_gear)
        {
            for (int i = 0; i < landing_gear.Length; i++)
            {
                landing_gear[i].toggleGear();
            }
        }
    }

    void toggleBrakes()
    {
        brakes_active = !brakes_active;

        if (brakes_active)
        {
            for (int i = 0; i < left_main_gear_colliders.Length; i++)
            {
                left_main_gear_colliders[i].material = braking_gear_physics_material;
            }

            for (int i = 0; i < right_main_gear_colliders.Length; i++)
            {
                right_main_gear_colliders[i].material = braking_gear_physics_material;
            }
        }
        else
        {
            for (int i = 0; i < left_main_gear_colliders.Length; i++)
            {
                left_main_gear_colliders[i].material = normal_gear_physics_material;
            }

            for (int i = 0; i < right_main_gear_colliders.Length; i++)
            {
                right_main_gear_colliders[i].material = normal_gear_physics_material;
            }
        }

        updateBrakesText();
    }

    void toggleAirbrakes()
    {
        airbrakes_deployed = !airbrakes_deployed;
    }

    //calculations
    void calculateLocalVelocities(float dt)
    {
        Quaternion inverted_rotation = Quaternion.Inverse(rb.rotation);
        velocity = rb.velocity;
        local_velocity = inverted_rotation * velocity; //transform world velocity into local space 
        local_angular_velocity = inverted_rotation * rb.angularVelocity; // transform angular velocity into local space
    }

    void calculateAngleOfAttackAndSideslip()
    {
        //set 0 angle of attack and sideslip if very slow or stationary
        if (local_velocity.sqrMagnitude < 0.1f)
        {
            angle_of_attack = 0.0f;
            sideslip = 0.0f;
            return;
        }

        //calculate angle of attack
        angle_of_attack = Mathf.Atan2(-local_velocity.y, local_velocity.z);

        //calculate sideslip
        sideslip = Mathf.Atan2(local_velocity.x, local_velocity.z);
    }

    void calculateGForce(float dt)
    {
        var inverted_rotation = Quaternion.Inverse(rb.rotation);
        var acceleration = (velocity - last_velocity) / dt;
        local_g_force = inverted_rotation * acceleration;   
        last_velocity = velocity;
    }

    Vector3 calculateLiftAndInducedDrag(float _angle_of_attack, Vector3 right_axis, float local_lift_power, AnimationCurve _angle_of_attack_curve, AnimationCurve _induced_drag_curve)
    {
        //filter out lateral velocity that doesn't contribute to lift
        Vector3 lift_velocity = Vector3.ProjectOnPlane(local_velocity, right_axis);
        
        float lift_velocity_square_magnitude = lift_velocity.sqrMagnitude;

        //lift coefficient varies with angle of attack
        float lift_coefficient = _angle_of_attack_curve.Evaluate(_angle_of_attack * Mathf.Rad2Deg);
        float lift_force = lift_velocity_square_magnitude * lift_coefficient * local_lift_power;

        //lift is perpendicular to velocity and right axis
        Vector3 lift_direction = Vector3.Cross(lift_velocity.normalized, right_axis);
        Vector3 lift = lift_direction * lift_force;

        //induced drag varies with square of lift coefficient
        float drag_force = lift_coefficient * lift_coefficient * this.induced_drag;
        Vector3 drag_direction = -lift_velocity.normalized;
        Vector3 induced_drag = drag_direction * lift_velocity_square_magnitude * drag_force * _induced_drag_curve.Evaluate(Mathf.Max(0, local_velocity.z));

        return lift + induced_drag;
    }

    float calculateSteering(float dt, float current_angular_velocity, float target_angular_velocity, float max_angular_acceleration)
    {
        float angular_velocity_error = target_angular_velocity - current_angular_velocity;
        float dt_max_angular_acceleration = max_angular_acceleration * dt;

        return Mathf.Clamp(angular_velocity_error, -dt_max_angular_acceleration, dt_max_angular_acceleration);
    }

    void calculatePropTorque()
    {
        
    }

    float calculateAirDensity(float metric_altitude)
    {
        //standard temperature at sea level in Kelvin
        const float sea_level_standard_temperature = 288.15f;

        //standard air pressure at sea level is Pascals
        const float sea_level_standard_pressure = 101325f;

        //Temperature lapse rate in Kelvin per metre
        const float temperature_lapse_rate = 0.0065f;

        //Specific gas constant (how sensitive air density is to temperature change) for dry air in Joules per Kilogram per Kelvin
        const float air_specific_gas_constant = 287.05f;

        //gravity of earth
        const float gravity = 9.807f;

        //calculate air density from values
        float temperature = sea_level_standard_temperature - (temperature_lapse_rate * metric_altitude);
        float air_pressure = sea_level_standard_pressure * Mathf.Pow(1 - ((temperature_lapse_rate * metric_altitude) / sea_level_standard_temperature), (gravity / (air_specific_gas_constant * temperature_lapse_rate)));
        float density = air_pressure / (air_specific_gas_constant * temperature);

        return density; 
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

    //updates
    
    //Non-physics based updates
    void Update()
    {
        getInput();
        updateFlightVariablesUI();
        updateFlaps();
    }

    //UI
    void updateFlightVariablesUI()
    {
        airspeed_text.text = "IAS: " + Mathf.Max(Mathf.FloorToInt(transform.InverseTransformDirection(rb.velocity).z * 1.944f), 0.0f).ToString() + " kt";
        altitude_text.text = "ALT: " + Mathf.Max(Mathf.FloorToInt(transform.position.y * 3.281f), 0.0f).ToString() + " ft";
        rate_of_climb_text.text = "ROC: " + Mathf.RoundToInt(rb.velocity.y * 196.9f).ToString() + " fpm";
    }

    void updateEngineText()
    {
        if (engine_running)
        {
            engine_text.text = "ENG: ON";
        }
        else
        {
            engine_text.text = "ENG: OFF";
        }
    }

    void updateBrakesText()
    {
        if (brakes_active)
        {
            brakes_text.text = "BRK: ON";
        }
        else
        {
            brakes_text.text = "BRK: OFF";
        }
    }

    void updateThrottleText()
    {
        throttle_text.text = "THR: " + Mathf.FloorToInt(throttle_input * 100.0f).ToString() + "%";
    }

    void updateFlapsTargetAngleAndUI()
    {
        flaps_target_angle = flaps_state * -flaps_deflection;
        moving_flaps = true;

        flaps_text.text = "FLP: " + Mathf.FloorToInt(flaps_state * 100.0f).ToString() + "%";
    }

    //Non-UI
    void updatePropOrFanTargetSpeed()
    {
        for (int i = 0; i < propellers_or_jets.Length; i++)
        {
            propellers_or_jets[i].setTargetRotationSpeed(propellers_or_jets[i].getIdleRotationSpeed() + (throttle_input * propellers_or_jets[i].getRotationSpeedThrottleIncreaseMultiplier()));
        }
    }

    void updateFlaps()
    {
        if (moving_flaps)
        {
            //update current flaps angle
            if (flaps_current_angle < flaps_target_angle)
            {
                flaps_current_angle += flaps_deploy_rate * Time.deltaTime;

                //check if should stop moving flaps
                if (flaps_current_angle > flaps_target_angle)
                {
                    flaps_current_angle = flaps_target_angle;
                    moving_flaps = false;
                }
            }
            else
            {
                flaps_current_angle -= flaps_deploy_rate * Time.deltaTime;

                //check if should stop moving flaps
                if (flaps_current_angle < flaps_target_angle)
                {
                    flaps_current_angle = flaps_target_angle;
                    moving_flaps = false;
                }
            }

            //apply flaps angle to flaps pivots
            for (int i = 0; i < flaps_pivots.Length; i++)
            {
                flaps_pivots[i].transform.localRotation = Quaternion.Euler(flaps_current_angle, flaps_pivots[i].localEulerAngles.y, flaps_pivots[i].localEulerAngles.z);
            }
        }
    }

    //physics based updates
    void FixedUpdate()
    {
        float dt = Time.fixedDeltaTime;

        calculateLocalVelocities(dt);
        calculateAngleOfAttackAndSideslip();
        calculateGForce(dt);
        updateThrust();
        updateDrag();
        updateLift();
        updateSteering(dt);
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
        //account for drag from airbrakes/flaps
        float local_airbrake_drag;

        if (airbrakes_deployed)
        {
            local_airbrake_drag = airbrake_drag;
        }
        else
        {
            local_airbrake_drag = 0.0f;
        }

        float local_flaps_drag = flaps_drag * flaps_state;

        //calculate coefficient of drag depending on direction of velocity
        Vector3 drag_coefficient = scale6
            (
            local_velocity.normalized,
            drag_right.Evaluate(Mathf.Abs(local_velocity.x)), drag_left.Evaluate(Mathf.Abs(local_velocity.x)),
            drag_top.Evaluate(Mathf.Abs(local_velocity.y)), drag_bottom.Evaluate(Mathf.Abs(local_velocity.y)),
            drag_forward.Evaluate(Mathf.Abs(local_velocity.z)) + local_airbrake_drag + local_flaps_drag, //include drag from airbrake/flaps for forward coefficient
            drag_back.Evaluate(Mathf.Abs(local_velocity.z))
            );

        //use the drag equation to calculate drag: Drag = 0.5 * Air density * Velocity^2 * Coefficient of drag * Reference area
        Vector3 drag = 0.5f * calculateAirDensity(transform.position.y) * local_velocity.sqrMagnitude * drag_coefficient.magnitude * -local_velocity.normalized; //drag is opposite to direction of velocity

        rb.AddRelativeForce(drag);
    }

    void updateLift()
    {
        if (local_velocity.sqrMagnitude < 1f)
        {
            return;
        }

        //extra lift provided by flaps
        float local_flaps_lift_power = flaps_lift_power * flaps_state;

        //with flaps deployed, wings behave as if they are at a higher angle of attack
        float local_flaps_angle_of_attack_increase = flaps_angle_of_attack_increase * flaps_state;

        //vertical lift from wings
        Vector3 lift_force = calculateLiftAndInducedDrag
            (
            angle_of_attack + (local_flaps_angle_of_attack_increase * Mathf.Deg2Rad), Vector3.right,
            lift_power + local_flaps_lift_power, lift_angle_of_attack_curve, induced_drag_curve
            );

        //lateral lift from vertical stabiliser
        Vector3 vertical_stabiliser_force = calculateLiftAndInducedDrag
            (
            sideslip, Vector3.up, vertical_stabiliser_power, vertical_stabiliser_angle_of_attack_curve,
            vertical_stabiliser_induced_drag_curve
            );

        rb.AddRelativeForce(lift_force);
        rb.AddRelativeForce(vertical_stabiliser_force); 
    }

    void updateSteering(float dt)
    {
        float airspeed = Mathf.Max(0.0f, local_velocity.z);
        float steering_power = steering_curve.Evaluate(airspeed);

        Vector3 current_angular_velocity = local_angular_velocity * Mathf.Rad2Deg;
        Vector3 target_angular_velocity = Vector3.Scale(control_surface_input, turn_speed * steering_power);        

        Vector3 angular_velocity_correction = new Vector3
            (
            calculateSteering(dt, current_angular_velocity.x, target_angular_velocity.x, base_max_turn_acceleration.x * steering_power),
            calculateSteering(dt, current_angular_velocity.y, target_angular_velocity.y, base_max_turn_acceleration.y * steering_power),
            calculateSteering(dt, current_angular_velocity.z, target_angular_velocity.z, base_max_turn_acceleration.z * steering_power)
            );

        rb.AddRelativeTorque(angular_velocity_correction * Mathf.Deg2Rad, ForceMode.VelocityChange);
    }
}
