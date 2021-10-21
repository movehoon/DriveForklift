using System;
using System.Collections;
using System.Collections.Generic;
using System.IO.Ports;
using System.Threading;
using UnityEngine;
using Modbus.Data;
using Modbus.Device;
using Modbus.IO;
using Modbus.Utility;
//using Modbus.Serial;

public enum ModbusInterface
{
    RTU,
    TCP,
}
public class ModbusManager : MonoBehaviour
{
    SerialPort _port;
    ModbusSerialMaster _master;
    ModbusSerialSlave _slave;

    Thread thread;
    bool _reqReconnect;

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

    public void SetHReg(int addr, ushort value)
    {
        if (_slave != null)
        {
            if (_slave.DataStore != null)
            {
                _slave.DataStore.HoldingRegisters[addr] = value;
            }
        }
    }
    public ushort GetHReg(int addr)
    {
        if (_slave != null)
        {
            if (_slave.DataStore != null)
            {
                return _slave.DataStore.HoldingRegisters[addr];
            }
        }
        return 0;
    }

    private void Modbus_Request_Event(object sender, Modbus.Device.ModbusSlaveRequestEventArgs e)
    {
        //disassemble packet from master
        byte fc = e.Message.FunctionCode;
        Debug.Log("Modbus_Request_Event " + fc.ToString());
        byte[] data = e.Message.MessageFrame;
        byte[] byteStartAddress = new byte[] { data[3], data[2] };
        byte[] byteNum = new byte[] { data[5], data[4] };
        Int16 StartAddress = BitConverter.ToInt16(byteStartAddress, 0);
        Int16 NumOfPoint = BitConverter.ToInt16(byteNum, 0);
        Debug.Log(fc.ToString() + "," + StartAddress.ToString() + "," + NumOfPoint.ToString());
    }

    private void Modbus_DataStoreWriteTo(object sender, Modbus.Data.DataStoreEventArgs e)
    {
        //this.Text = "DataType=" + e.ModbusDataType.ToString() + "  StartAdress=" + e.StartAddress;
        int iAddress = e.StartAddress;//e.StartAddress;
        Debug.Log("Modbus_DataStoreWriteTo " + e.ModbusDataType.ToString());
        switch (e.ModbusDataType)
        {
            case ModbusDataType.HoldingRegister:
                for (int i = 0; i < e.Data.B.Count; i++)
                {
                    //Set AO                 
                    _slave.DataStore.HoldingRegisters[e.StartAddress + i + 1] = e.Data.B[i];
                    //e.Data.B[i] already write to slave.DataStore.HoldingRegisters[e.StartAddress + i + 1]
                    //e.StartAddress starts from 0
                    //You can set AO value to hardware here

                    //DoAOUpdate(iAddress, e.Data.B[i].ToString());
                    iAddress++;
                }
                break;

            case ModbusDataType.Coil:
                for (int i = 0; i < e.Data.A.Count; i++)
                {
                    //Set DO
                    _slave.DataStore.CoilDiscretes[e.StartAddress + i + 1] = e.Data.A[i];
                    //e.Data.A[i] already write to slave.DataStore.CoilDiscretes[e.StartAddress + i + 1]
                    //e.StartAddress starts from 0
                    //You can set DO value to hardware here

                    //DoDOUpdate(iAddress, e.Data.A[i]);
                    iAddress++;
                    if (e.Data.A.Count == 1)
                    {
                        break;
                    }
                }
                break;
        }
    } 


    public bool Connect(string portName)
    {
        if (_port == null && _master == null)
        {
            _port = new SerialPort(portName, 115200, Parity.None, 8, StopBits.One);
            _port.ReadTimeout = 50;
            _port.WriteTimeout = 50;
            _port.Open();
#if true
            //_master = ModbusSerialMaster.CreateRtu(_port);
            //var adapter = new SerialPortAdapter(_port);
            _slave = ModbusSerialSlave.CreateRtu(1, _port);
            _slave.ModbusSlaveRequestReceived += new EventHandler<ModbusSlaveRequestEventArgs>(Modbus_Request_Event);
            //_slave.ListenAsync().GetAwaiter().GetResult();
            _slave.DataStore = Modbus.Data.DataStoreFactory.CreateDefaultDataStore();
            _slave.DataStore.DataStoreWrittenTo += new EventHandler<DataStoreEventArgs>(Modbus_DataStoreWriteTo);

            //_slave.Listen();
            //thread = new Thread(_Connect);
            //thread.Start();
            StartCoroutine("_Connect");
#endif
            PlayerPrefs.SetString("DEV_NAME", portName);
            Debug.Log("Modbus Connect Done");

            //ReadState();
            return true;
        }
        return false;
    }

    //private void _Connect()
    //{
    //    while (true)
    //    {
    //        try
    //        {
    //            _slave.Listen();
    //        }
    //        catch (Exception ex)
    //        {
    //            //_reqReconnect = true;
    //            Debug.Log(ex.ToString());
    //        }
    //        Thread.Sleep(10);
    //    }
    //}

    private IEnumerator _Connect()
    {
        bool _run = true;
        while (_run)
        {
            yield return new WaitForSeconds(0.01f);
            try
            {
                if (_slave != null)
                {
                    _slave.Listen();
                }
                else
                {
                    _run = false;
                }
            }
            catch (Exception ex)
            {
                Debug.Log(ex.ToString());
            }
        }
        yield return null;
    }

    public void Disconnect()
    {
        if (thread != null)
        {
            thread.Abort();
            thread = null;
        }

        //_master.Dispose();
        //_master = null;

        if (_slave != null)
        {
            _slave.Dispose();
            _slave = null;
        }

        if (_port != null)
        {
            _port.Close();
            _port.Dispose();
            _port = null;
        }
    }

    void Reconnect()
    {
        Disconnect();
        Connect(PlayerPrefs.GetString("DEV_NAME"));
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
            //Debug.Log(mb_hreg[0].ToString());
            //Debug.Log(mb_hreg[1].ToString());

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
        //StartCoroutine("Sync");
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

        if (_reqReconnect)
        {
            _reqReconnect = false;
            Reconnect();
        }
    }

    private void OnApplicationQuit()
    {
        if (thread != null)
        {
            thread.Abort();
            thread = null;
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