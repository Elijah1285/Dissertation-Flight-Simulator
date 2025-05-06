using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.UI;
using TMPro;

public class AeroplaneController : MonoBehaviour
{
    //engine variables
    enum EngineState
    {
        NOT_RUNNING = 1,
        RUNNING = 2,
        TRANSITION = 3
    }

    EngineState engine_state = EngineState.NOT_RUNNING;

    //flight variables
    //airbrakes
    bool airbrakes_deployed = false;  
    bool moving_airbrakes = false;
    float airbrakes_current_angle = 0.0f;
    float airbrakes_target_angle = 0.0f;

    //flaps
    bool moving_flaps = false;
    float flaps_state = 0.0f;
    float flaps_current_angle = 0.0f;
    float flaps_target_angle = 0.0f;

    float angle_of_attack = 0.0f;
    float sideslip = 0.0f;

    Vector3 velocity;
    Vector3 local_velocity;
    Vector3 last_velocity;

    Vector3 local_angular_velocity;
    Vector3 g_force;

    //UI variables
    float pedals_indicator_x_center_position = 0.0f;
    float throttle_indicator_y_idle_position = 0.0f;
    Vector2 stick_indicator_center_position;

    //misc
    Rigidbody rb;

    //input variables
    bool brakes_active = true;
    float throttle_input = 0.0f;

    Vector2 stick_input;
    float pedals_input;

    IA_AircraftControls aircraft_controls;

    [Header("Engine, Thrust & Propeller Settings")]
    [SerializeField] float thrust_multiplier;
    [SerializeField] float engine_throttle_pitch_increase_multiplier;
    [SerializeField] AudioClip engine_startup_sound;
    [SerializeField] AudioClip engine_shutdown_sound;

    [SerializeField] bool affected_by_propeller_torque;
    [SerializeField] float propeller_diameter;
    [SerializeField] AnimationCurve propeller_torque_coefficient_curve;
    [SerializeField] PropellerOrJet[] propellers_or_jets;

    [Header("Lift/Angle of Attack Settings")]
    [SerializeField] float wing_surface_area;
    [SerializeField] float wing_aspect_ratio;
    [SerializeField] float wing_oswald_efficiency;
    [SerializeField] float wing_lift_multiplier;
    [SerializeField] float wing_induced_drag_multiplier;
    [SerializeField] AnimationCurve wing_angle_of_attack_lift_curve;
    [SerializeField] AnimationCurve wing_angle_of_attack_induced_drag_curve;

    [SerializeField] float vertical_stabiliser_surface_area;
    [SerializeField] float vertical_stabiliser_aspect_ratio;
    [SerializeField] float vertical_stabiliser_oswald_efficiency;
    [SerializeField] float vertical_stabiliser_lift_multiplier;
    [SerializeField] float vertical_stabiliser_induced_drag_multiplier;
    [SerializeField] AnimationCurve vertical_stabiliser_sideslip_correction_curve;
    [SerializeField] AnimationCurve vertical_stabiliser_sideslip_induced_drag_curve;

    [SerializeField] float flaps_lift_power;
    [SerializeField] float flaps_angle_of_attack_increase;

    [Header("Drag/Friction Settings")]
    [SerializeField] float drag_multiplier;
    [SerializeField] float airbrake_drag;
    [SerializeField] float flaps_drag;

    [SerializeField] AnimationCurve drag_forward;
    [SerializeField] AnimationCurve drag_back;
    [SerializeField] AnimationCurve drag_left;
    [SerializeField] AnimationCurve drag_right;
    [SerializeField] AnimationCurve drag_top;
    [SerializeField] AnimationCurve drag_bottom;

    [SerializeField] Collider[] left_main_gear_colliders;
    [SerializeField] Collider[] right_main_gear_colliders;

    [SerializeField] PhysicMaterial normal_gear_physics_material;
    [SerializeField] PhysicMaterial braking_gear_physics_material;

    [Header("Steering Settings")]
    [SerializeField] float ground_steering_multiplier;
    [SerializeField] float max_taxi_steering_speed; //so turn rate while on the ground is limited, this is in m/s
    [SerializeField] float ground_angular_drag;
    [SerializeField] Vector3 turn_multiplier;
    [SerializeField] Vector3 max_turn_acceleration;
    [SerializeField] AnimationCurve steering_curve;
    [SerializeField] AnimationCurve angle_of_attack_steering_curve;
    [SerializeField] LayerMask ground_layer;

    [Header("Gear Settings")]
    [SerializeField] bool retractable_gear;  
    [SerializeField] float ground_check_distance;
    [SerializeField] Transform nose_or_tail_wheel;
    [SerializeField] RetractableGear[] landing_gear;

    [Header("Control Surface Settings")]
    [SerializeField] bool has_airbrakes;

    [SerializeField] float aileron_deflection;
    [SerializeField] float elevator_deflection;
    [SerializeField] float rudder_deflection;
    [SerializeField] float flaps_deflection;
    [SerializeField] float airbrakes_deflection;

    [SerializeField] float airbrakes_deploy_rate;
    [SerializeField] float flaps_deploy_rate;
    [SerializeField] AudioClip flaps_transition_sound;
    [SerializeField] AudioClip wind_sound;

    [SerializeField] Transform left_aileron_pivot;
    [SerializeField] Transform right_aileron_pivot;
    [SerializeField] Transform rudder_pivot;

    [SerializeField] Transform[] elevator_pivots;
    [SerializeField] Transform[] flaps_pivots;   
    [SerializeField] Transform[] airbrakes_pivots;

    [Header("Audio")]
    [SerializeField] float speed_for_wind_sound;
    [SerializeField] AudioSource engine_audio_source;
    [SerializeField] AudioSource flaps_audio_source;
    [SerializeField] AudioSource wind_audio_source;

    [Header("UI objects")]
    //main ui
    TextMeshProUGUI engine_text;
    TextMeshProUGUI brakes_text;
    TextMeshProUGUI landing_gear_text;
    TextMeshProUGUI throttle_text;
    TextMeshProUGUI flaps_text;
    TextMeshProUGUI spoilers_text;   
    TextMeshProUGUI airspeed_text;
    TextMeshProUGUI altitude_text;
    TextMeshProUGUI rate_of_climb_text;
    TextMeshProUGUI g_force_text;
    Image stick_indicator;
    Image pedals_indicator;
    Image throttle_indicator;

    //extra ui
    TextMeshProUGUI thrust_text;
    TextMeshProUGUI drag_text;
    TextMeshProUGUI wing_lift_text;
    TextMeshProUGUI vertical_stabiliser_lift_text;
    TextMeshProUGUI weight_text;
    TextMeshProUGUI angle_of_attack_text;
    TextMeshProUGUI sideslip_text;
    TextMeshProUGUI propeller_torque_text;

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
        
        //get references to ui objects
        //main ui
        engine_text = GameObject.Find("engine_text").GetComponent<TextMeshProUGUI>();
        brakes_text = GameObject.Find("brakes_text").GetComponent<TextMeshProUGUI>();
        landing_gear_text = GameObject.Find("landing_gear_text").GetComponent<TextMeshProUGUI>();
        throttle_text = GameObject.Find("throttle_text").GetComponent<TextMeshProUGUI>();
        flaps_text = GameObject.Find("flaps_text").GetComponent<TextMeshProUGUI>();
        spoilers_text = GameObject.Find("spoilers_text").GetComponent<TextMeshProUGUI>();
        airspeed_text = GameObject.Find("airspeed_text").GetComponent<TextMeshProUGUI>();
        altitude_text = GameObject.Find("altitude_text").GetComponent<TextMeshProUGUI>();
        rate_of_climb_text = GameObject.Find("rate_of_climb_text").GetComponent<TextMeshProUGUI>();
        g_force_text = GameObject.Find("g_force_text").GetComponent<TextMeshProUGUI>();
        stick_indicator = GameObject.Find("stick_indicator").GetComponent<Image>();
        pedals_indicator = GameObject.Find("pedals_indicator").GetComponent<Image>();
        throttle_indicator = GameObject.Find("throttle_indicator").GetComponent<Image>();

        //extra
        thrust_text = GameObject.Find("thrust_text").GetComponent<TextMeshProUGUI>();
        drag_text = GameObject.Find("drag_text").GetComponent<TextMeshProUGUI>();
        wing_lift_text = GameObject.Find("wing_lift_text").GetComponent<TextMeshProUGUI>();
        vertical_stabiliser_lift_text = GameObject.Find("vertical_stabiliser_lift_text").GetComponent<TextMeshProUGUI>();
        weight_text = GameObject.Find("weight_text").GetComponent<TextMeshProUGUI>();
        angle_of_attack_text = GameObject.Find("angle_of_attack_text").GetComponent<TextMeshProUGUI>();
        sideslip_text = GameObject.Find("sideslip_text").GetComponent<TextMeshProUGUI>();
        propeller_torque_text = GameObject.Find("propeller_torque_text").GetComponent<TextMeshProUGUI>();

        //save default positions of controls indicators
        stick_indicator_center_position = stick_indicator.GetComponent<RectTransform>().position;
        pedals_indicator_x_center_position = pedals_indicator.GetComponent<RectTransform>().position.x;
        throttle_indicator_y_idle_position = throttle_indicator.GetComponent<RectTransform>().position.y;
    }

    //input

    //axis inputs
    void getInput()
    {
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
        throttle_input += 0.1f;

        if (throttle_input > 1.0f)
        {
            throttle_input = 1.0f;
        }

        updateThrottleText();

        if (engine_state == EngineState.RUNNING)
        {
            updatePropOrTurbineTargetSpeedAndAudio();
        }
    }

    void decreaseThrottleInput()
    {
        throttle_input -= 0.1f;

        if (throttle_input < 0.0f)
        {
            throttle_input = 0.0f;
        }

        updateThrottleText();

        if (engine_state == EngineState.RUNNING)
        {
            updatePropOrTurbineTargetSpeedAndAudio();
        }
    }

    void deployFlaps()
    {
        if (flaps_state < 1.0f)
        {
            flaps_state += 0.25f;
        }

        onFlapsTransition();
    }

    void retractFlaps()
    {
        if (flaps_state > 0.0f)
        {
            flaps_state -= 0.25f;
        }

        onFlapsTransition();
    }

    //boolean inputs
    void toggleEngine()
    {
        if (engine_state == EngineState.NOT_RUNNING || engine_state == EngineState.RUNNING)
        {
            if (engine_state == EngineState.NOT_RUNNING)
            {
                //start the startup procedure if not running
                engine_state = EngineState.TRANSITION;
                engine_text.text = "ENG: STRT";

                //spin up the propellers/jets
                for (int i = 0; i < propellers_or_jets.Length; i++)
                {
                    PropellerOrJet propeller_or_jet = propellers_or_jets[i];
                    propeller_or_jet.setTargetRotationSpeed(propeller_or_jet.getIdleRotationSpeed());
                }

                //play startup sound
                engine_audio_source.PlayOneShot(engine_startup_sound);

                //wait for startup to finish before setting engine to running
                StartCoroutine(waitForClipToEnd(engine_startup_sound, () =>
                {
                    engine_state = EngineState.RUNNING;
                    engine_text.text = "ENG: ON";
                    engine_audio_source.Play();

                    updatePropOrTurbineTargetSpeedAndAudio();
                }
                ));
            }
            else if (engine_state == EngineState.RUNNING)
            {
                //start the shutdown procedure if running
                engine_state = EngineState.TRANSITION;
                engine_text.text = "ENG: SHTDWN";
                engine_audio_source.pitch = 1.0f;

                //spin down the propellers/jets
                for (int i = 0; i < propellers_or_jets.Length; i++)
                {
                    PropellerOrJet propeller_or_jet = propellers_or_jets[i];
                    propeller_or_jet.onEngineShutdown();
                    propeller_or_jet.setTargetRotationSpeed(0.0f);
                }

                //stop engine run audio and play shutdown sound
                engine_audio_source.Stop();
                engine_audio_source.PlayOneShot(engine_shutdown_sound);

                //wait for shutdown to finish before allowing engine toggle
                StartCoroutine(waitForClipToEnd(engine_shutdown_sound, () =>
                {
                    engine_state = EngineState.NOT_RUNNING;
                    engine_text.text = "ENG: OFF";
                }
                ));
            }
        }
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
        if (has_airbrakes)
        {
            moving_airbrakes = true;
            airbrakes_deployed = !airbrakes_deployed;

            if (airbrakes_deployed)
            {
                airbrakes_target_angle = airbrakes_deflection;
                wind_audio_source.Play();
            }
            else
            {
                airbrakes_target_angle = 0.0f;
                wind_audio_source.Stop();
            }
        }
    }

    //calculations (some calculations are also done directly in the updates)
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
        angle_of_attack_text.text = "AOA: " + (angle_of_attack * Mathf.Rad2Deg).ToString("F1") + "°";

        //calculate sideslip
        sideslip = Mathf.Atan2(local_velocity.x, local_velocity.z);
        sideslip_text.text = "SDSLP: " + (sideslip * Mathf.Rad2Deg).ToString("F1") + "°";
    }

    void calculateGForce(float dt)
    {
        var inverted_rotation = Quaternion.Inverse(rb.rotation);
        var acceleration = (velocity - last_velocity) / dt;
        g_force = inverted_rotation * acceleration;   
        last_velocity = velocity;
    }

    Vector3 calculateLiftAndInducedDrag(float local_angle_of_attack, Vector3 right_axis, float local_lift_multiplier,
        AnimationCurve local_angle_of_attack_lift_curve, AnimationCurve local_angle_of_attack_induced_drag_curve, float air_density,
        float local_oswald_efficiency, float local_surface_aspect_ratio, float local_surface_area, float local_induced_drag_multiplier)
    {
        //convert aoa from radians to degrees
        float angle_of_attack_degrees = local_angle_of_attack * Mathf.Rad2Deg;

        //filter out component of velocity that doesn't contribute to lift
        Vector3 lift_velocity = Vector3.ProjectOnPlane(local_velocity, right_axis);
        
        float lift_velocity_square_magnitude = lift_velocity.sqrMagnitude;

        //lift coefficient varies with angle of attack
        float lift_coefficient = local_angle_of_attack_lift_curve.Evaluate(angle_of_attack_degrees);

        //calculate lift force using the lift equation
        float lift_force = 0.5f * air_density * local_surface_area * lift_velocity_square_magnitude * lift_coefficient * local_lift_multiplier;

        //lift is perpendicular to velocity and right axis
        Vector3 lift_direction = Vector3.Cross(lift_velocity.normalized, right_axis);
        Vector3 lift = lift_direction * lift_force;

        //calculate induced drag coefficient
        float induced_drag_coefficient = Mathf.Pow(lift_coefficient, 2) / (Mathf.PI * local_oswald_efficiency * local_surface_aspect_ratio);

        //calculate induced drag force       
        float induced_drag_force = 0.5f * air_density * lift_velocity_square_magnitude * local_surface_area * induced_drag_coefficient * local_angle_of_attack_induced_drag_curve.Evaluate(angle_of_attack_degrees) * local_induced_drag_multiplier;
        Vector3 drag_direction = -lift_velocity.normalized;
        Vector3 induced_drag = drag_direction * induced_drag_force;

        return lift + induced_drag;
    }

    float calculateControlSurfaceRotation(float dt, float current_angular_velocity, float target_angular_velocity, float max_angular_acceleration)
    {
        float angular_velocity_error = target_angular_velocity - current_angular_velocity;
        float dt_max_angular_acceleration = max_angular_acceleration * dt;

        return Mathf.Clamp(angular_velocity_error, -dt_max_angular_acceleration, dt_max_angular_acceleration);
    }

    float calculatePropellerTorque(PropellerOrJet propeller, float air_density)
    {
        //calculate revolutions per second (used for torque equation)
        float propeller_revolutions_per_second = propeller.getCurrentRotationSpeed() / 360.0f;

        //revolutions per minute for finding torque coefficient
        float propeller_revolutions_per_minute = propeller_revolutions_per_second * 60.0f;

        //evaluate the torque coefficient curve to find out the torque coefficient
        float propeller_torque_coefficient = propeller_torque_coefficient_curve.Evaluate(propeller_revolutions_per_minute);

        //use the equation to calculate propeller torque
        float propeller_torque = propeller_torque_coefficient * air_density * Mathf.Pow(propeller_revolutions_per_second, 2) * Mathf.Pow(propeller_diameter, 5);

        return propeller_torque;
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
        float air_temperature = sea_level_standard_temperature - (temperature_lapse_rate * metric_altitude);
        float air_pressure = sea_level_standard_pressure * Mathf.Pow(1 - ((temperature_lapse_rate * metric_altitude) / sea_level_standard_temperature), (gravity / (air_specific_gas_constant * temperature_lapse_rate)));
        float air_density = air_pressure / (air_specific_gas_constant * air_temperature);

        return air_density; 
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
        updateAirbrakes();
    }

    //UI
    void updateFlightVariablesUI()
    {
        airspeed_text.text = "IAS: " + Mathf.Max(Mathf.FloorToInt(transform.InverseTransformDirection(rb.velocity).z * 1.944f), 0.0f).ToString() + " kt";
        altitude_text.text = "ALT: " + Mathf.Max(Mathf.FloorToInt(transform.position.y * 3.281f), 0.0f).ToString() + " ft";
        rate_of_climb_text.text = "ROC: " + Mathf.RoundToInt(rb.velocity.y * 196.9f).ToString() + " fpm";
        g_force_text.text = "G:   " + g_force.ToString("F1");
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

    //Non-UI
    void updatePropOrTurbineTargetSpeedAndAudio()
    {
        //update target speed of propellers/turbines
        for (int i = 0; i < propellers_or_jets.Length; i++)
        {
            PropellerOrJet propeller_or_jet = propellers_or_jets[i]; 
            propeller_or_jet.setTargetRotationSpeed(propeller_or_jet.getIdleRotationSpeed() + (throttle_input * propeller_or_jet.getRotationSpeedThrottleIncreaseMultiplier()));
        }

        //update engine audio
        engine_audio_source.pitch = 1.0f + (throttle_input * engine_throttle_pitch_increase_multiplier);
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
                    flaps_audio_source.Stop();
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
                    flaps_audio_source.Stop();
                }
            }

            //apply flaps angle to flaps pivots
            for (int i = 0; i < flaps_pivots.Length; i++)
            {
                flaps_pivots[i].transform.localRotation = Quaternion.Euler(flaps_current_angle, flaps_pivots[i].localEulerAngles.y, flaps_pivots[i].localEulerAngles.z);
            }
        }
    }

    void updateAirbrakes()
    {
        if (has_airbrakes)
        {
            if (moving_airbrakes)
            {
                //update current airbrakes angle
                if (airbrakes_current_angle < airbrakes_target_angle)
                {
                    airbrakes_current_angle += airbrakes_deploy_rate * Time.deltaTime;

                    //check if should stop moving airbrakes
                    if (airbrakes_current_angle > airbrakes_target_angle)
                    {
                        airbrakes_current_angle = airbrakes_target_angle;
                        moving_airbrakes = false;
                    }
                }
                else
                {
                    airbrakes_current_angle -= airbrakes_deploy_rate * Time.deltaTime;

                    //check if should stop moving airbrakes
                    if (airbrakes_current_angle < airbrakes_target_angle)
                    {
                        airbrakes_current_angle = airbrakes_target_angle;
                        moving_airbrakes = false;
                    }
                }

                //apply airbrakes angle to airbrakes pivots
                for (int i = 0; i < airbrakes_pivots.Length; i++)
                {
                    airbrakes_pivots[i].transform.localRotation = Quaternion.Euler(airbrakes_current_angle, airbrakes_pivots[i].localEulerAngles.y, airbrakes_pivots[i].localEulerAngles.z);
                }
            }

            //wind sound is only heard when aircraft is fast enough
            if (airbrakes_deployed && (transform.InverseTransformDirection(rb.velocity).z * 1.944f) >= speed_for_wind_sound)
            {
                wind_audio_source.Play();
            }
            else if (wind_audio_source.isPlaying)
            {
                wind_audio_source.Stop();
            }
        }
    }

    //physics based updates
    void FixedUpdate()
    {
        float dt = Time.fixedDeltaTime;
        float air_density = calculateAirDensity(transform.position.y);

        calculateLocalVelocities(dt);
        calculateAngleOfAttackAndSideslip();
        calculateGForce(dt);
        updateThrust(air_density);
        updatePropellerTorque(air_density);
        updateDrag(air_density);
        updateLift(air_density);
        updateControlSurfaceRotation(dt);
    }

    void updateThrust(float air_density)
    {
        if (engine_state == EngineState.RUNNING)
        {
            float thrust_force = throttle_input * thrust_multiplier * air_density;

            rb.AddRelativeForce(thrust_force * Vector3.forward);
            thrust_text.text = "THRST: " + (int) thrust_force + "N";
        }
    }

    void updatePropellerTorque(float air_density)
    {
        if (affected_by_propeller_torque)
        {
            for (int i = 0; i < propellers_or_jets.Length; i++)
            {
                float propeller_torque = calculatePropellerTorque(propellers_or_jets[i], air_density);

                propeller_torque_text.text = "TRQ: " + (int) propeller_torque + "Nm";
                rb.AddRelativeTorque(Vector3.forward * propeller_torque);
            }
        }
    }

    void updateDrag(float air_density)
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
        Vector3 drag = 0.5f * air_density * local_velocity.sqrMagnitude * drag_coefficient.magnitude * drag_multiplier * -local_velocity.normalized; //drag is opposite to direction of velocity

        drag_text.text = "DRAG: " + (int) drag.magnitude + "N";

        rb.AddRelativeForce(drag);
    }

    void updateLift(float air_density)
    {
        if (local_velocity.sqrMagnitude < 1f)
        {
            return;
        }

        //extra lift provided by flaps
        float local_flaps_lift_multiplier = flaps_lift_power * flaps_state;

        //with flaps deployed, wings behave as if they are at a higher angle of attack
        float local_flaps_angle_of_attack_increase = flaps_angle_of_attack_increase * flaps_state;

        //vertical lift from wings
        Vector3 lift_force = calculateLiftAndInducedDrag
            (
            angle_of_attack + (local_flaps_angle_of_attack_increase * Mathf.Deg2Rad), Vector3.right,
            wing_lift_multiplier * local_flaps_lift_multiplier, wing_angle_of_attack_lift_curve, wing_angle_of_attack_induced_drag_curve, air_density,
            wing_oswald_efficiency, wing_aspect_ratio, wing_surface_area, wing_induced_drag_multiplier);

        //lateral lift from vertical stabiliser
        Vector3 vertical_stabiliser_force = calculateLiftAndInducedDrag
            (
            sideslip, Vector3.up, vertical_stabiliser_lift_multiplier, vertical_stabiliser_sideslip_correction_curve,
            vertical_stabiliser_sideslip_induced_drag_curve, air_density, vertical_stabiliser_oswald_efficiency, vertical_stabiliser_aspect_ratio,
            vertical_stabiliser_surface_area, vertical_stabiliser_induced_drag_multiplier);

        wing_lift_text.text = "WING: " + (int) lift_force.magnitude + "N";
        vertical_stabiliser_lift_text.text = "VSTAB: " + (int) vertical_stabiliser_force.magnitude + "N";
        weight_text.text = "WGHT: " + (int) (-Physics.gravity.y * rb.mass) + "N";

        rb.AddRelativeForce(lift_force);
        rb.AddRelativeForce(vertical_stabiliser_force); 
    }

    void updateControlSurfaceRotation(float dt)
    {
        //aerodynamic steering from control surfaces
        float airspeed = Mathf.Max(0.0f, local_velocity.z);
        float control_surface_effectiveness = steering_curve.Evaluate(airspeed) * angle_of_attack_steering_curve.Evaluate(angle_of_attack);

        Vector3 control_surface_input = new Vector3(stick_input.y, -pedals_input, -stick_input.x);

        Vector3 current_angular_velocity = local_angular_velocity * Mathf.Rad2Deg;
        Vector3 target_angular_velocity = Vector3.Scale(control_surface_input, turn_multiplier * control_surface_effectiveness);        

        Vector3 angular_velocity_correction = new Vector3
            (
            calculateControlSurfaceRotation(dt, current_angular_velocity.x, target_angular_velocity.x, max_turn_acceleration.x * control_surface_effectiveness),
            calculateControlSurfaceRotation(dt, current_angular_velocity.y, target_angular_velocity.y, max_turn_acceleration.y * control_surface_effectiveness),
            calculateControlSurfaceRotation(dt, current_angular_velocity.z, target_angular_velocity.z, max_turn_acceleration.z * control_surface_effectiveness)
            );

        Debug.Log(angular_velocity_correction);

        rb.AddRelativeTorque(angular_velocity_correction * Mathf.Deg2Rad, ForceMode.VelocityChange);

        //steering from nose/tail wheel when on the ground
        //raycast to check if wheel is on the ground
        bool nose_or_tail_wheel_on_ground = Physics.Raycast(nose_or_tail_wheel.position, Vector3.down, ground_check_distance, ground_layer);
        
        //apply ground steering when wheel is on ground
        if (nose_or_tail_wheel_on_ground)
        {
            rb.angularDrag = ground_angular_drag;
            rb.AddRelativeTorque(-pedals_input * Mathf.Min(local_velocity.z, max_taxi_steering_speed) * ground_steering_multiplier * Vector3.up, ForceMode.Acceleration);
        }
        else
        {
            rb.angularDrag = 0.2f;
        }
    }

    void onFlapsTransition() //function to be called when flaps are moved
    {
        flaps_target_angle = flaps_state * -flaps_deflection;
        moving_flaps = true;
        flaps_text.text = "FLP: " + Mathf.FloorToInt(flaps_state * 100.0f).ToString() + "%";
        flaps_audio_source.Play();
    }

    IEnumerator waitForClipToEnd(AudioClip clip, System.Action on_complete)
    {
        if (clip == null)
        {
            on_complete?.Invoke();
            yield break;
        }

        //wait for clip to end and invoke action
        yield return new WaitForSeconds(clip.length);
        on_complete?.Invoke();
    }
}
