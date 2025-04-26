using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GameController : MonoBehaviour
{
    //aircraft spawn points
    [SerializeField] Transform cessna_172sp_spawn_point;
    [SerializeField] Transform piper_pa18_super_cub_spawn_point;
    [SerializeField] Transform dehavilland_canada_dash8_q400_spawn_point;
    [SerializeField] Transform boeing_737_800_spawn_point;
    [SerializeField] Transform boeing_747_400f_spawn_point;

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
                    Instantiate(cessna_172sp_prefab, cessna_172sp_spawn_point.position, cessna_172sp_spawn_point.rotation);

                    break;
                }

            case "piper_pa18_super_cub":
                {
                    Instantiate(piper_pa18_super_cub_prefab, piper_pa18_super_cub_spawn_point.position, piper_pa18_super_cub_spawn_point.rotation);

                    break;
                }

            case "dehavilland_canada_dash8_q400":
                {
                    Instantiate(dehavilland_canada_dash8_q400_prefab, dehavilland_canada_dash8_q400_spawn_point.position, dehavilland_canada_dash8_q400_spawn_point.rotation);

                    break;
                }

            case "boeing_737_800":
                {
                    Instantiate(boeing_737_800_prefab, boeing_737_800_spawn_point.position, boeing_737_800_spawn_point.rotation);

                    break;
                }

            case "boeing_747_400f":
                {
                    Instantiate(boeing_747_400f_prefab, boeing_747_400f_spawn_point.position, boeing_747_400f_spawn_point.rotation);

                    break;
                }
        }
    }
}
