using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class LineSensor : MonoBehaviour
{
    public string TriggerObjectName = "Line";
    public int sensorWidth = 160;
    public DotSensor[] dotSensors;

    private List<int> detectedLines = new List<int>() { 0, 0, 0 };
    private List<int> detectedPos = new List<int>() { 0, 0, 0 };

    public enum SENSOR_MODE {
        MODE_DEFAULT = 0,
        MODE_LEFT = 1,
        MODE_CENTER = 2,
        MODE_RIGHT = 3,
    };

    private SENSOR_MODE _mode;
    public SENSOR_MODE Mode
    {
        get => _mode;
        set => _mode = (value <= SENSOR_MODE.MODE_RIGHT) ? value : SENSOR_MODE.MODE_DEFAULT;
    }

    private int _rawValue;
    public int RawValue
    {
        get
        {
            _rawValue = 0;
            if (dotSensors != null)
            {
                for(int i=0; i<dotSensors.Length; i++)
                {
                    if (dotSensors[i].isDetected)
                    {
                        _rawValue += (1 << i);
                    }
                }
            }
            return _rawValue;
        }
    }

    private int Point2Position(long point)
    {
        float pos = 0;
        float count = 0;
        for (int i = 0; i < dotSensors.Length; i++)
        {
            if ((point & (1<<i)) != 0)
            {
                pos += i;
                count++;
            }
        }
        if (count > 0)
        {
            pos = (((pos / count) - (dotSensors.Length/2) + 0.5f) / (dotSensors.Length/2)) * (sensorWidth/2);
        }
        return (int)pos;
    }

    private long _position;
    public long Position
    {
        get
        {
            _position = 0;
            for (int i = 0; i < detectedLines.Count; i++)
            {
                detectedLines[i] = 0;
            }

            int lineIndex = 0;
            bool lineDetected = false;

            if (dotSensors != null)
            {
                for (int i = 0; i < dotSensors.Length; i++)
                {
                    if (dotSensors[i].isDetected)
                    {
                        if (!lineDetected)
                        {
                            lineDetected = true;
                        }
                        detectedLines[lineIndex] += (1 << i);
                    }
                    else
                    {
                        if (lineDetected)
                        {
                            lineDetected = !lineDetected;
                            lineIndex++;
                            if (lineIndex >= detectedLines.Count)
                            {
                                break;
                            }
                        }
                    }
                }
            }

            for (int i = 0; i < detectedLines.Count; i++)
            {
                detectedPos[i] = Point2Position(detectedLines[i]);
                //Debug.Log("[" + i.ToString() + "]From: " + detectedLines[i].ToString("X") + ", To: " + detectedPos[i]);
            }

            if (_mode == LineSensor.SENSOR_MODE.MODE_LEFT)
            {
                _position = detectedPos[0];
            }
            else if (_mode == LineSensor.SENSOR_MODE.MODE_RIGHT)
            {
                for (int i=0; i<3; i++)
                {
                    if (detectedLines[i] != 0)
                    {
                        _position = detectedPos[i];
                    }
                }
            }
            else
            {
                // result to middest
                int mid_diff = 10000;
                for (int i=0; i<3; i++)
                {
                    if (detectedLines[i] != 0)
                    {
                        if (mid_diff > Math.Abs(detectedPos[i]))
                        {
                            mid_diff = Math.Abs(detectedPos[i]);
                            _position = detectedPos[i];
                        }
                    }
                }
            }
            return _position;
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
