using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;

public class SelectionScreenController : MonoBehaviour
{
    public enum AircraftType
    {
        CESSNA_172SP = 1,
        PIPER_SUPER_CUB = 2,
        DHC_DASH_8_Q400 = 3,
        BOEING_737_800 = 4,
        BOEING_747_400F = 5
    }

    public enum SelectionScreen
    {
        MAIN_SCREEN = 1,
        CESSNA_172_SP_INFO = 2,
        PIPER_SUPER_CUB_INFO = 3,
        DHC_DASH_8_Q400_INFO = 4,
        BOEING_737_800_INFO = 5,
        BOEING_747_400F_INFO = 6
    }

    [SerializeField] GameObject main_screen;
    [SerializeField] GameObject cessna_172sp_info_screen;
    [SerializeField] GameObject piper_pa18_super_cub_info_screen;
    [SerializeField] GameObject dhc_dash_8_q400_info_screen;
    [SerializeField] GameObject boeing_737_800_info_screen;
    [SerializeField] GameObject boeing_747_400f_info_screen;

    public void selectAircraft(int aircraft_index)
    {
        AircraftType aircraft = (AircraftType) aircraft_index;

        switch (aircraft)
        {
            case AircraftType.CESSNA_172SP:
                {
                    PlayerPrefs.SetString("chosen_aircraft", "cessna_172sp");

                    break;
                }
            case AircraftType.PIPER_SUPER_CUB:
                {
                    PlayerPrefs.SetString("chosen_aircraft", "piper_pa18_super_cub");

                    break;
                }
            case AircraftType.DHC_DASH_8_Q400:
                {
                    PlayerPrefs.SetString("chosen_aircraft", "dehavilland_canada_dash8_q400");

                    break;
                }
            case AircraftType.BOEING_737_800:
                {
                    PlayerPrefs.SetString("chosen_aircraft", "boeing_737_800");

                    break;
                }
            case AircraftType.BOEING_747_400F:
                {
                    PlayerPrefs.SetString("chosen_aircraft", "boeing_747_400f");

                    break;
                }
        }

        loadGame();
    }

    public void switchScreen(int screen_index)
    {
        SelectionScreen screen = (SelectionScreen) screen_index;

        switch (screen)
        {
            case SelectionScreen.MAIN_SCREEN:
                {
                    main_screen.SetActive(true);
                    cessna_172sp_info_screen.SetActive(false);
                    piper_pa18_super_cub_info_screen.SetActive(false);
                    dhc_dash_8_q400_info_screen.SetActive(false);
                    boeing_737_800_info_screen.SetActive(false);
                    boeing_747_400f_info_screen.SetActive(false);

                    break;
                }
            case SelectionScreen.CESSNA_172_SP_INFO:
                {
                    main_screen.SetActive(false);
                    cessna_172sp_info_screen.SetActive(true);
                    piper_pa18_super_cub_info_screen.SetActive(false);
                    dhc_dash_8_q400_info_screen.SetActive(false);
                    boeing_737_800_info_screen.SetActive(false);
                    boeing_747_400f_info_screen.SetActive(false);

                    break;
                }
            case SelectionScreen.PIPER_SUPER_CUB_INFO:
                {
                    main_screen.SetActive(false);
                    cessna_172sp_info_screen.SetActive(false);
                    piper_pa18_super_cub_info_screen.SetActive(true);
                    dhc_dash_8_q400_info_screen.SetActive(false);
                    boeing_737_800_info_screen.SetActive(false);
                    boeing_747_400f_info_screen.SetActive(false);

                    break;
                }
            case SelectionScreen.DHC_DASH_8_Q400_INFO:
                {
                    main_screen.SetActive(false);
                    cessna_172sp_info_screen.SetActive(false);
                    piper_pa18_super_cub_info_screen.SetActive(false);
                    dhc_dash_8_q400_info_screen.SetActive(true);
                    boeing_737_800_info_screen.SetActive(false);
                    boeing_747_400f_info_screen.SetActive(false);

                    break;
                }
            case SelectionScreen.BOEING_737_800_INFO:
                {
                    main_screen.SetActive(false);
                    cessna_172sp_info_screen.SetActive(false);
                    piper_pa18_super_cub_info_screen.SetActive(false);
                    dhc_dash_8_q400_info_screen.SetActive(false);
                    boeing_737_800_info_screen.SetActive(true);
                    boeing_747_400f_info_screen.SetActive(false);

                    break;
                }
            case SelectionScreen.BOEING_747_400F_INFO:
                {
                    main_screen.SetActive(false);
                    cessna_172sp_info_screen.SetActive(false);
                    piper_pa18_super_cub_info_screen.SetActive(false);
                    dhc_dash_8_q400_info_screen.SetActive(false);
                    boeing_737_800_info_screen.SetActive(false);
                    boeing_747_400f_info_screen.SetActive(true);

                    break;
                }
        }
    }

    void loadGame()
    {
        SceneManager.LoadScene("SCN_LoadingScreen");
    }
}
