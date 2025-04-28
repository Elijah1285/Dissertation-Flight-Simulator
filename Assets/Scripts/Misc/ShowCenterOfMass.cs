using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[ExecuteAlways]
[RequireComponent(typeof(Rigidbody))]

public class ShowCenterOfMass : MonoBehaviour
{
    Rigidbody rb;

    void OnDrawGizmos()
    {
        rb = GetComponent<Rigidbody>();

        Gizmos.color = Color.red;

        Gizmos.DrawSphere(rb.worldCenterOfMass, 0.3f);
    }
}
