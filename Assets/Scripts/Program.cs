using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;

public class Program : MonoBehaviour
{
    public LineSensor lineSensor;

    public void OnButtonReset()
    {
        //Application.LoadLevel(Application.loadedLevel);
        SceneManager.LoadScene("SimFactory");
    }
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        Debug.Log("LineSensor: " + lineSensor.Value.ToString());
    }
}
