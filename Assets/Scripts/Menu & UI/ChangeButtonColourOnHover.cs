using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using TMPro;

public class ChangeButtonColourOnHover : MonoBehaviour
{
    [SerializeField] Graphic graphic;
    [SerializeField] AudioClip blip_sound;
    [SerializeField] AudioSource audio_source;

    public void hoveredOverButton()
    {
        graphic.color = Color.yellow;
        audio_source.PlayOneShot(blip_sound);
    }

    public void unhoveredOverButton()
    {
        graphic.color = Color.white;
    }

    void OnDisable()
    {
        graphic.color = Color.white;
    }
}
