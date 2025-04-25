using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GameController : MonoBehaviour
{
    //since aircraft are of different heights, they must be initially spawned at different altitudes
    [SerializeField] float cessna_172sp_vertical_spawn_offset;
    [SerializeField] float piper_pa18_super_cub_vertical_spawn_offset;
    [SerializeField] float dehavilland_canada_dash8_q400_vertical_spawn_offset;
    [SerializeField] float boeing_737_800_vertical_spawn_offset;
    [SerializeField] float boeing_747_400f_vertical_spawn_offset;

    //since the piper super cub is a taildragger, it is pitched up when on the ground
    [SerializeField] float piper_pa18_super_cub_pitch_spawn_offset;

    //aircraft prefabs
    [SerializeField] GameObject cessna_172sp_prefab;
    [SerializeField] GameObject piper_pa18_super_cub_prefab;
    [SerializeField] GameObject dehavilland_canada_dash8_q400_prefab;
    [SerializeField] GameObject boeing_737_800_prefab;
    [SerializeField] GameObject boeing_747_400f_prefab;

    void Start()
    {
        switch (PlayerPrefs.GetString("chosen_aircraft"))
        {
            case "cessna_172sp":
                {
                    Instantiate(cessna_172sp_prefab, new Vector3(0.0f, cessna_172sp_vertical_spawn_offset, 0.0f), Quaternion.identity);

                    break;
                }

            case "piper_pa18_super_cub":
                {
                    Instantiate(piper_pa18_super_cub_prefab, new Vector3(0.0f, piper_pa18_super_cub_vertical_spawn_offset, 0.0f), Quaternion.identity);

                    break;
                }

            case "dehavilland_canada_dash8_q400":
                {
                    Instantiate(dehavilland_canada_dash8_q400_prefab, new Vector3(0.0f, dehavilland_canada_dash8_q400_vertical_spawn_offset, 0.0f), Quaternion.identity);

                    break;
                }

            case "boeing_737_800":
                {
                    Instantiate(boeing_737_800_prefab, new Vector3(0.0f, boeing_737_800_vertical_spawn_offset, 0.0f), Quaternion.identity);

                    break;
                }

            case "boeing_747_400f":
                {
                    Instantiate(boeing_747_400f_prefab, new Vector3(0.0f, boeing_747_400f_vertical_spawn_offset, 0.0f), Quaternion.identity);

                    break;
                }
        }
    }
}
