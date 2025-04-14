using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TMPro;

public class ChangeButtonTextColourOnHover : MonoBehaviour
{
    [SerializeField] AudioClip blip_sound;

    public void hoveredOverButton()
    {
        GetComponent<TMP_Text>().color = Color.yellow;
        GetComponent<AudioSource>().PlayOneShot(blip_sound);
    }

    public void unhoveredOverButton()
    {
        GetComponent<TMP_Text>().color = Color.white;
    }
}
