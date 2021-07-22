using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;

//[RequireComponent(typeof(NewCarController))]
public class Program : MonoBehaviour
{
    public LineSensor lineSensor;

    public NewCarController m_Car; // the car controller we want to use

    public void OnButtonReset()
    {
        //Application.LoadLevel(Application.loadedLevel);
        SceneManager.LoadScene("SimFactory");
    }

    private void Awake()
    {
        //m_Car = GetComponent<NewCarController>();
    }
    // Start is called before the first frame update
    void Start()
    {
        
    }

    private void FixedUpdate()
    {
        Debug.Log("LineSensor: " + lineSensor.Value.ToString());

        // pass the input to the car!
        float h = Input.GetAxis("Horizontal");
        //if (lineSensor.Value == 384)
        //{
        //    h = 0;
        //}
        //else
        //{
        //    for (int i=0; i<16; i++)
        //    {
        //        if ((lineSensor.Value & (0x01<<i)) != 0)
        //        {
        //            if (i < 8)
        //            {
        //                h = (8 - i) * 0.1f;
        //            }
        //            else if (i > 8)
        //            {
        //                h = i * 0.1f;
        //            }
        //            break;
        //        }
        //    }
        //}
        //Debug.Log("LineSensor: " + lineSensor.Value.ToString() + ", h: " + h.ToString());
        float v = Input.GetAxis("Vertical");

#if !MOBILE_INPUT
        float handbrake = Input.GetAxis("Jump");
        m_Car.Move(h, v, v, handbrake);

#else
            m_Car.Move(h, v, v, 0f);
#endif
    }
}
