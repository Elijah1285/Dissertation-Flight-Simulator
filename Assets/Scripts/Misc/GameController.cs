using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GameController : MonoBehaviour
{
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
                    Instantiate(cessna_172sp_prefab);

                    break;
                }
        }
    }
}
