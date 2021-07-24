using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO.Ports;
using Modbus.Device;

public class ModbusManager : MonoBehaviour
{
    SerialPort _port;
    ModbusSerialMaster _master;

    private const int SLAVE_ADDRESS = 1;
    //private const int COIL_ADDRESS = 0;
    //private const int COIL_COUNT = 10;
    //private const int ISTS_ADDRESS = 100;
    //private const int ISTS_COUNT = 10;
    private const int HREG_ADDRESS = 0;
    private const int HREG_COUNT = 10;

    private float UPDATE_PERIOD = 0.1f;
    private float duration;

    //private bool[] mb_coil = new bool[COIL_COUNT];
    //private bool[] mb_ists = new bool[ISTS_COUNT];
    private ushort[] mb_hreg = new ushort[HREG_COUNT];
    private ushort[] mb_hreg_wr = new ushort[5];

    Dictionary<ushort, ushort> writeReg = new Dictionary<ushort, ushort>();

    public bool Connect(string portName)
    {
        if (_port == null && _master == null)
        {
            _port = new SerialPort(portName, 115200, Parity.None, 8, StopBits.One);
            _port.ReadTimeout = 500;
            _port.WriteTimeout = 500;
            _port.Open();

            _master = ModbusSerialMaster.CreateRtu(_port);

            //ReadState();
            return true;
        }
        return false;
    }

    public void Disconnect()
    {
        _master.Dispose();
        _master = null;

        _port.Close();
        _port.Dispose();
        _port = null;
    }

    public bool IsConnected()
    {
        if (_port != null)
            return _port.IsOpen;
        return false;
    }

    //public bool WriteCoil(ushort addr, bool value)
    //{
    //    if (IsConnected())
    //    {
    //        _master.WriteSingleCoil(SLAVE_ADDRESS, addr, value);
    //        return true;
    //    }
    //    return false;
    //}

    //public bool ReadISTS(ushort addr)
    //{
    //    if (IsConnected())
    //    {
    //        return mb_ists[addr];
    //    }
    //    return false;
    //}

    public bool WriteRegister(ushort addr, ushort value)
    {
        if (IsConnected())
        {
            if (writeReg.ContainsKey(addr))
            {
                writeReg[addr] = value;
            }
            else
            {
                writeReg.Add(addr, value);
            }
            //_master.WriteSingleRegister(SLAVE_ADDRESS, addr, value);
            return true;
        }
        return false;
    }

    public ushort GetRegister(ushort addr)
    {
        ushort res = 0;
        if (addr < HREG_COUNT)
        {
            res = mb_hreg[addr];
        }
        return res;
    }

    public void SyncState()
    {
        try
        {
            // Read the current state of the output
            //mb_coil = _master.ReadCoils(SLAVE_ADDRESS, COIL_ADDRESS, 10);
            //Debug.Log(mb_coil[0].ToString());
            //Debug.Log(mb_coil[1].ToString());

            //mb_ists = _master.ReadInputs(SLAVE_ADDRESS, ISTS_ADDRESS, 10);
            //Debug.Log(mb_ists[0].ToString());
            //Debug.Log(mb_ists[1].ToString());

            mb_hreg = _master.ReadHoldingRegisters(SLAVE_ADDRESS, HREG_ADDRESS, 10);
            Debug.Log(mb_hreg[0].ToString());
            Debug.Log(mb_hreg[1].ToString());

            if (writeReg.Count > 0)
            {
                foreach (KeyValuePair<ushort, ushort> item in writeReg)
                {
                    if (item.Key < HREG_COUNT)
                    {
                        mb_hreg_wr[item.Key] = item.Value;
                    }
                }
                writeReg.Clear();
                _master.WriteMultipleRegisters(SLAVE_ADDRESS, HREG_ADDRESS, mb_hreg_wr);
                //_master.WriteSingleRegister(SLAVE_ADDRESS, HREG_ADDRESS+1, mb_hreg[0]);
            }

            //// Update the UI
            //if (state[0])
            //{
            //    StateLabel.Text = "On";
            //}
            //else
            //{
            //    StateLabel.Text = "Off";
            //}
        }
        catch (Exception ex)
        {
            Debug.Log(ex.ToString());
        }
    }

    // Start is called before the first frame update
    void Start()
    {
        StartCoroutine("Sync");
    }

    // Update is called once per frame
    void Update()
    {
        duration += Time.deltaTime;
        if (duration > UPDATE_PERIOD)
        {
            duration -= UPDATE_PERIOD;
            //if (IsConnected())
            //{
            //    SyncState();
            //}
        }
    }

    private IEnumerator Sync()
    {
        while (true)
        {
            yield return new WaitForSeconds(0.01f);
            if (IsConnected())
            {
                SyncState();
            }
        }
    }
}