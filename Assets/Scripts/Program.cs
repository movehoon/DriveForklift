using System;
using System.Collections;
using System.Collections.Generic;
using System.IO.Ports;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.SceneManagement;

//[RequireComponent(typeof(NewCarController))]
public class Program : MonoBehaviour
{
    public LineSensor lineSensor;
    public ModbusManager modbusManager;

    public Dropdown dropDown_PortNames;
    public Button button_Connect;
    public InputField if_ActualSpeed;
    public InputField if_ActualSteer;
    public InputField if_SensorValue;
    public InputField if_ControlPerSecond;

    public NewCarController m_Car; // the car controller we want to use

    int controlCount;
    float duration;

    public void OnButtonReset()
    {
        //Application.LoadLevel(Application.loadedLevel);
        SceneManager.LoadScene("SimFactory");
    }


    public void OnConnect()
    {
        if (!modbusManager.IsConnected())
        {
            string portName = dropDown_PortNames.options[dropDown_PortNames.value].text;
            if (modbusManager.Connect(portName))
            {
                Debug.Log("Modbus Connected with " + portName);
                button_Connect.GetComponentInChildren<Text>().text = "Disconnect";
            }
            else
            {
                Debug.Log("Modbus Connection failed with " + portName);
            }
        }
        else
        {
            modbusManager.Disconnect();
            Debug.Log("Modbus disconnected");
            button_Connect.GetComponentInChildren<Text>().text = "Connect";
        }
    }

    private void Awake()
    {
        //m_Car = GetComponent<NewCarController>();
    }
    // Start is called before the first frame update
    void Start()
    {
        string[] portNames = SerialPort.GetPortNames();
        dropDown_PortNames.ClearOptions();
        foreach (string portName in portNames)
        {
            dropDown_PortNames.options.Add(new Dropdown.OptionData() { text = portName });
        }
    }

    //private void FixedUpdate()
    private void Update()
    {
        //Debug.Log("LineSensor: " + lineSensor.Value.ToString());
        float v = Input.GetAxis("Vertical");
        float pos = 0;
        // pass the input to the car!
        //float h = ((float)(short)modbusManager.GetRegister(5))/1000.0f;
        float h = Input.GetAxis("Horizontal");
        float CurrentVelocity = m_Car.CurrentSpeed;
        if (m_Car.Reverse)
        {
            CurrentVelocity *= -1;
        }
        //if (lineSensor.Value == 384)
        //{
        //    pos = 0;
        //}
        //else
        //{
        //    for (int i=0; i<16; i++)
        //    {
        //        if ((lineSensor.Value & (0x01<<i)) != 0)
        //        {
        //            if (m_Car.CurrentSpeed > 0)
        //            {
        //                pos = (i - 8) * 0.3f;
        //            }
        //            else if (m_Car.CurrentSpeed < 0)
        //            {
        //                pos = (8 - i) * 0.3f;
        //            }
        //            else
        //            {
        //                pos = (i - 8) * 0.3f;
        //            }
        //            //if (i < 8)
        //            //{
        //            //    pos = (8 - i) * 0.1f;
        //            //}
        //            //else if (i > 8)
        //            //{
        //            //    pos = (i - 8) * 0.1f;
        //            //}
        //            break;
        //        }
        //    }
        //}
        ////Debug.Log("Pos: " + pos.ToString());
        //if (Math.Abs(pos) < 0.001f)
        //{
        //    pos = h;
        //}
        //Debug.Log("LineSensor: " + lineSensor.Value.ToString() + ", h: " + h.ToString());
        //Debug.Log("v: " + v.ToString());
        //Debug.Log("CurrentVelocity: " + CurrentVelocity.ToString());
        //Debug.Log("m_Car.Revs: " + m_Car.Revs.ToString());
        //Debug.Log("m_Car.Skidding: " + m_Car.Skidding.ToString());
        //Debug.Log("m_Car.Forward: " + m_Car.Forward.ToString());

        if_ActualSpeed.text = CurrentVelocity.ToString("0.00");
        if_ActualSteer.text = m_Car.CurrentSteerAngle.ToString("0.00");
        if_SensorValue.text = lineSensor.Value.ToString();
        controlCount++;
        duration += Time.deltaTime;
        if (duration > 1)
        {
            duration -= 1;
            if_ControlPerSecond.text = controlCount.ToString() + "cps";
            Debug.Log("cps: " + controlCount.ToString());
            controlCount = 0;
        }

        float TargetSteerAngle = ((float)(short)modbusManager.GetRegister(5))/5000.0f;

        modbusManager.WriteRegister(0, (ushort)(CurrentVelocity * 1000));
        modbusManager.WriteRegister(1, (ushort)(m_Car.CurrentSteerAngle * 1000));
        modbusManager.WriteRegister(2, (ushort)lineSensor.Value);

#if !MOBILE_INPUT
        float handbrake = Input.GetAxis("Jump");
        m_Car.Move(TargetSteerAngle, v, v, handbrake);

#else
            m_Car.Move(h, v, v, 0f);
#endif
    }
}
