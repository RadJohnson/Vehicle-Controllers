using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RADVehichles;
using Unity.VisualScripting;
using System;
public class SpringTest : MonoBehaviour
{

    [Header("Suspension")]
    [SerializeField] private float suspensionLength;//NEEDS TO BE AN ARRAY SO THAT IT IS UNIQUE TO EACH WHEEL
    private float suspensionLengthOld;//NEEDS TO BE AN ARRAY SO THAT IT IS UNIQUE TO EACH WHEEL
    [SerializeField] private float suspensionTravel = 0.2f;// Rectify this stuff to make it easier to set susspension length (maximum distance the spring can travel from rest length)
    [SerializeField] private float restLength = 0.5f;
    [SerializeField] private float stiffness = 4700f;
    [SerializeField] private float dampingStiffness = 640f;
    [SerializeField] private float suspensionMaxLength;
    [SerializeField] private float suspensionMinLength;
    public float dampingForce;
    public float suspensionForce;

    public GameObject wheel;
    public float wheelRadius;
    public float wheelDistanceFromSuspensionOrigin;

    public float wheelSuspensionAcceleration;
    public float wheelMass;

    void Awake()
    {
        suspensionMinLength = restLength - suspensionTravel;
        suspensionMaxLength = restLength + suspensionTravel;

        wheelRadius = wheel.GetComponent<SphereCollider>().radius;
    }


    //TODO : make so that the suspension travels downwards
    //TODO : Realise that the rigibody version is fine so long as there are no more than around 100 rbs
    void Update()
    {

        wheelDistanceFromSuspensionOrigin = wheel.transform.position.y - transform.position.y;//This would need changing to improve this

        suspensionLengthOld = suspensionLength;

        suspensionLength = wheelDistanceFromSuspensionOrigin - wheelRadius;

        suspensionLength = Mathf.Clamp(suspensionLength, suspensionMinLength, suspensionMaxLength);

        dampingForce = dampingStiffness * VehicleMaths.Acceleration(suspensionLengthOld, suspensionLength, Time.fixedDeltaTime);

        suspensionForce = VehicleMaths.HookesLaw(stiffness, restLength - suspensionLength) + dampingForce;

        //if (!spiderCar)//this amkes the car more stable when turned on (could consider making it switch on and off based off of some condition)
        //    suspensionForce = Mathf.Max(0.0f, suspensionForce);//Mathf.Clamp(suspensionForce, 0, Mathf.Infinity);

        //wheel.GetComponent<Rigidbody>().AddForce(0,suspensionForce,0);


        //TODO : fix issue with distance check errors caused by rotation

        //implemented movement by translating figuring out its acceleration using F=M*A
        wheelSuspensionAcceleration = suspensionForce / wheelMass;
        //wheel.transform.localPosition += new Vector3(0, wheelSuspensionAcceleration * Time.deltaTime, 0);
        
        wheel.GetComponent<Rigidbody>().MovePosition(new Vector3(transform.position.x, wheel.transform.position.y + wheelSuspensionAcceleration * Time.fixedDeltaTime, transform.position.z));
        

        //wheel.transform.localPosition = new Vector3(wheel.transform.localPosition.x, Mathf.Clamp(wheel.transform.localPosition.y, suspensionMinLength, suspensionMaxLength), wheel.transform.localPosition.z);

    }
    private void OnDrawGizmos()
    {
        //Wheel height
        //Gizmos.color = Color.gray;
        //Gizmos.DrawRay(transform.position - (transform.up * suspensionLength), -transform.up * wheelRadius);

        Gizmos.color = Color.green;
        Gizmos.DrawRay(transform.position, suspensionForce * -transform.up);
    }
}
