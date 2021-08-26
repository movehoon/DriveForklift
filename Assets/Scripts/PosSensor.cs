using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PosSensor : MonoBehaviour
{
    public string TriggerObjectName = "TAG";

    private int _detectedID;
    public int DetectedID
    {
        get
        {
            return _detectedID;
        }
    }

    private void OnTriggerEnter(UnityEngine.Collider other)
    {
        //Debug.Log("OnTriggerEnter with " + other.gameObject.name);
        if (other.gameObject.name.Contains(TriggerObjectName))
        {
            string strID = other.gameObject.name.Replace(TriggerObjectName, "");
            _detectedID = int.Parse(strID);
        }
    }

    private void OnTriggerExit(UnityEngine.Collider other)
    {
        //Debug.Log("OnTriggerExit with " + other.gameObject.name);
        if (other.gameObject.name.Contains(TriggerObjectName))
        {
            _detectedID = 0;
        }
    }
}
