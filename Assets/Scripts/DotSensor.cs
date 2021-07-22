using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DotSensor : MonoBehaviour
{
    public string TriggerObjectName = "Line";

    private bool _isDetected;
    public bool isDetected
    {
        get
        {
            return _isDetected;
        }
    }

    private void OnTriggerEnter(UnityEngine.Collider other)
    {
        //Debug.Log("OnTriggerEnter with " + other.gameObject.name);
        if (other.gameObject.name.Contains(TriggerObjectName))
        {
            _isDetected = true;
        }
    }

    private void OnTriggerExit(UnityEngine.Collider other)
    {
        //Debug.Log("OnTriggerExit with " + other.gameObject.name);
        if (other.gameObject.name.Contains(TriggerObjectName))
        {
            _isDetected = false;
        }
    }
}
