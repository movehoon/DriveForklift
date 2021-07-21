using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class LineSensor : MonoBehaviour
{
    public string TriggerObjectName = "Line";
    public DotSensor[] dotSensors;

    private int _value;
    public int Value
    {
        get
        {
            _value = 0;
            if (dotSensors != null)
            {
                for(int i=0; i<dotSensors.Length; i++)
                {
                    if (dotSensors[i].isDetected)
                    {
                        _value += (1 << i);
                    }
                }
            }
            return _value;
        }
    }

    public void Start()
    {
        if (dotSensors != null)
        {
            for (int i = 0; i < dotSensors.Length; i++)
            {
                dotSensors[i].TriggerObjectName = TriggerObjectName;
            }
        }
    }
}
