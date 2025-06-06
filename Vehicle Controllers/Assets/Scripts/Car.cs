using System.Collections.Generic;
using UnityEngine;

public class Car : Vehicle
{
    [SerializeField] private bool V = false;
    [SerializeField] private Rigidbody rb;
    [SerializeField] internal List<Suspension> suspensions = new();
    [SerializeField] internal List<Wheel> Wheels = new();

    internal void Awake()
    {
    }

    void Update()
    {
        CalculateSuspensionRaycast(transform, ref rb, ref suspensions, ref Wheels,  V);
        //CalcualteSuspension(ref suspensions,ref Wheels);
    }


    /*
     * TODO : Figure out how to implement the force calcualtions in a neat solution so that
     * only one funciton needs to be called and whether it is possible to instead have this fucntion in the Vehichle class
    */
    //private void CalcualteSuspension(ref List<Suspension> _suspensions,ref List<Wheel> _wheels)
    //{
    //    for (int i = 0; i < _suspensions.Count; i++)
    //    {
    //        Suspension suspensionVarTempHolder = _suspensions[i];
    //        Wheel wheelVarTempHolder = _wheels[i];
    //        _suspensions[i].LocalToWorld(transform);
    //        CalculateSuspensionRaycast(transform, ref rb, ref suspensionVarTempHolder, ref wheelVarTempHolder, ref V);
    //        _suspensions[i] = suspensionVarTempHolder;
    //        _wheels[i] = wheelVarTempHolder;
    //    }
    //}

    /*
     {
      if (Input.GetKey(KeyCode.W))
        {
            engineForceMultiplier++;
            //rb.AddForce(transform.forward, ForceMode.Acceleration);
        }
        else if (Input.GetKey(KeyCode.S))
        {
            engineForceMultiplier--;
            //rb.AddForce(-transform.forward, ForceMode.Acceleration);
        }
        else
        {
            RecenterThrottle();
        }
        engineForceMultiplier = Mathf.Clamp(engineForceMultiplier, -1.5f, 1.5f);

        ///Steering
        if (Input.GetKey(KeyCode.A))
        {
            steerAngle -= steerForce;
        }
        else if (Input.GetKey(KeyCode.D))
        {
            steerAngle += steerForce;
        }
        else
        {
            RecenterSteering();
        }

        steerAngle = Mathf.Clamp(steerAngle, -maximumSteeringAngle, maximumSteeringAngle);
        suspensionTransforms[0].localEulerAngles = new Vector3(suspensionTransforms[0].gameObject.transform.localEulerAngles.x, steerAngle, suspensionTransforms[0].gameObject.transform.localEulerAngles.z);
        suspensionTransforms[1].localEulerAngles = new Vector3(suspensionTransforms[0].gameObject.transform.localEulerAngles.x, steerAngle, suspensionTransforms[0].gameObject.transform.localEulerAngles.z);
    }

    void RecenterSteering()
    {
        if (steerAngle > 0.05f || steerAngle < -0.05f)
            steerAngle = Mathf.Lerp(steerAngle, 0, Time.deltaTime);
        else steerAngle = 0;
    }

    void RecenterThrottle()
    {
        if (engineForceMultiplier > 0.05f || engineForceMultiplier < -0.05f)
            engineForceMultiplier = Mathf.Lerp(engineForceMultiplier, 0, Time.deltaTime);
        else engineForceMultiplier = 0;
    }
     
     
     
     */

    private void Reset()
    {
        GetComponent<Rigidbody>();
    }
}
