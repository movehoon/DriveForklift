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
    public LineSensor stopSensor;
    public PosSensor posSensor;
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
    ushort modbusCount;
    ushort destination;

    public void OnButtonReset()
    {
        //Application.LoadLevel(Application.loadedLevel);
        SceneManager.LoadScene("SimFactory");
    }

    public void SetDestination(int id)
    {
        destination = (ushort)id;
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
        string savedName = PlayerPrefs.GetString("DEV_NAME");
        Debug.Log("Find " + savedName);
        for (int i=0; i<dropDown_PortNames.options.Count; i++)
        {
            if (dropDown_PortNames.options[i].text.Equals(savedName))
            {
                dropDown_PortNames.value = i;
                break;
            }
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

        if_ActualSpeed.text = CurrentVelocity.ToString("0.00");
        if_ActualSteer.text = m_Car.CurrentSteerAngle.ToString("0.00");
        if_SensorValue.text = lineSensor.Value.ToString();
        controlCount++;
        duration += Time.deltaTime;
        if (duration > 1)
        {
            duration -= 1;
            if_ControlPerSecond.text = controlCount.ToString() + "cps";
            //Debug.Log("cps: " + controlCount.ToString());
            controlCount = 0;
        }

        float TargetSteerAngle = ((float)(short)modbusManager.GetHReg(111))/100.0f;
        float TargetDrive = ((float)(short)modbusManager.GetHReg(112)) / 1000.0f;
        //Debug.Log("TargetSteerAngle: " + TargetSteerAngle.ToString());
        modbusCount++;
        modbusManager.SetHReg(101, modbusCount);
        modbusManager.SetHReg(102, (ushort)(CurrentVelocity * 1000));
        modbusManager.SetHReg(103, (ushort)(m_Car.CurrentSteerAngle * 1000));
        modbusManager.SetHReg(104, (ushort)lineSensor.Value);
        modbusManager.SetHReg(105, (ushort)stopSensor.Value);
        modbusManager.SetHReg(106, (ushort)posSensor.DetectedID);
        modbusManager.SetHReg(107, (ushort)destination);
        //Debug.Log("Hreg102: " + modbusManager.GetHReg(102));
        //Debug.Log("Hreg105: " + modbusManager.GetHReg(105));
        //Debug.Log("Hreg106: " + modbusManager.GetHReg(106));
#if !MOBILE_INPUT
        //float handbrake = Input.GetAxis("Jump");
        float handbrake = ((float)(short)modbusManager.GetHReg(113));
        m_Car.Move(TargetSteerAngle, TargetDrive, TargetDrive, handbrake);

#else
            m_Car.Move(h, v, v, 0f);
#endif
        //Debug.Log("posSensor: " + posSensor.DetectedID);
    }
}
