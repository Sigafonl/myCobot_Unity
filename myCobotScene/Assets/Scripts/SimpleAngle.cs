using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SimpleAngle : MonoBehaviour
{
   
    [SerializeField]
    GameObject joint;
    public GameObject Joint { get => joint; set => joint = value; }
   
    // Start is called before the first frame update
    void Start()
    {
         joint.transform.position = transform.position;
    }

    // Update is called once per frame
    void Update()
    {
        joint.transform.position = transform.position;
    }
}
