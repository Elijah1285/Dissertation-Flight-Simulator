using UnityEngine;

[ExecuteInEditMode]
public class DynamicTextureTiling : MonoBehaviour
{
    [SerializeField] Vector2 material_tiling_multiplier;

    // Reference to the original material with the texture
     Material original_material;

    void Start()
    {
        // Ensure we have a material
        original_material = GetComponent<Renderer>().sharedMaterial;

        // Create a new material instance for this object
        Material material_instance = new Material(original_material);

        // Apply the new material to the object
        GetComponent<Renderer>().sharedMaterial = material_instance;

        // Get the initial scale of the object
        Vector3 initial_scale = transform.localScale;

        // Set the texture tiling based on the initial scale
        SetTextureTiling(material_instance, initial_scale);
    }

    void Update()
    {
        // Adjust texture tiling based on the current scale
        SetTextureTiling(GetComponent<Renderer>().sharedMaterial, new Vector3(transform.localScale.x * material_tiling_multiplier.x, 1.0f, transform.localScale.z * material_tiling_multiplier.y));
    }

    void SetTextureTiling(Material material, Vector3 scale)
    {
        // Calculate tiling based on the scale
        Vector2 tiling = new Vector2(scale.x, scale.z);

        // Apply tiling to the material
        material.mainTextureScale = tiling;
    }
}
