using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RetractableGear : MonoBehaviour
{
    bool is_retracted = false;

    const float retracted_angle = 100.0f;

    Quaternion deployed_rotation;
    Quaternion retracted_rotation;

    Coroutine gear_coroutine;

    [SerializeField] float rotation_duration;
    [SerializeField] Vector3 rotation_axis = Vector3.right; //what axis will the gear assembly rotate around
    [SerializeField] MeshRenderer[] mesh_renderers;

    void Start()
    {
        deployed_rotation = transform.localRotation;
        retracted_rotation = deployed_rotation * Quaternion.AngleAxis(retracted_angle, rotation_axis);
    }

    public void toggleGear()
    {
        Quaternion target_rotation;

        if (is_retracted)
        {
            toggleMesh(true);

            target_rotation = deployed_rotation;
        }
        else
        {
            target_rotation = retracted_rotation;
        }

        if (gear_coroutine != null)
        {
            StopCoroutine(gear_coroutine);
        }

        gear_coroutine = StartCoroutine(gearCoroutine(transform.localRotation, target_rotation));
        is_retracted = !is_retracted;
    }

    void toggleMesh(bool show_mesh)
    {
        for (int i = 0; i < mesh_renderers.Length; i++)
        {
            mesh_renderers[i].enabled = show_mesh;
        }
    }

    IEnumerator gearCoroutine(Quaternion start_rotation, Quaternion target_rotation)
    {
        float elapsed_time = 0.0f;

        while (elapsed_time < rotation_duration)
        {
            float progress = elapsed_time / rotation_duration;
            transform.localRotation = Quaternion.Slerp(start_rotation, target_rotation, progress);

            elapsed_time += Time.deltaTime;

            yield return null;
        }

        transform.localRotation = target_rotation;

        if (target_rotation == retracted_rotation)
        {
            toggleMesh(false);
        }
        
        gear_coroutine = null;
    }
}
