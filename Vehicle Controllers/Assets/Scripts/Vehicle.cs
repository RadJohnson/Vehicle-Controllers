using RADVehichles;
using System;
using UnityEngine;


public class Vehicle : MonoBehaviour
{
    [Serializable]
    internal class Suspension
    {
        [Header("Suspension")]
        [SerializeField] internal Transform suspensionTransform;
        internal float suspensionLength;
        internal float suspensionLengthOld;
        [SerializeField] internal float suspensionTravel;
        [SerializeField] internal float restLength;
        [SerializeField] internal float stiffness;
        [SerializeField] internal float dampingStiffness;
        internal float suspensionMinLength;
        internal float suspensionMaxLength;
        [SerializeField] internal bool steering;
        [SerializeField] internal bool drive;


        internal Suspension()
        {
            suspensionTravel = 0.2f;
            restLength = 0.5f;
            stiffness = 2000f;
            dampingStiffness = 250f;
            //suspensionTransform = _suspensionTransform;
            //steering = _steering;
            //drive = _drive;
            suspensionMinLength = restLength - suspensionTravel;// Rectify this stuff to make it easier to set suspension length
            suspensionMaxLength = restLength + suspensionTravel;// Rectify this stuff to make it easier to set suspension length
            suspensionLength = suspensionMaxLength;
            suspensionLengthOld = suspensionMaxLength;
        }

        internal Suspension(Transform _suspensionTransform, float _suspensionTravel, float _restLength, float _stiffness, float _dampingStiffness, bool _steering, bool _drive)
        {
            suspensionTransform = _suspensionTransform;
            suspensionTravel = _suspensionTravel;
            restLength = _restLength;
            stiffness = _stiffness;
            dampingStiffness = _dampingStiffness;
            steering = _steering;
            drive = _drive;

            suspensionMinLength = _restLength - _suspensionTravel;// Rectify this stuff to make it easier to set suspension length
            suspensionMaxLength = _restLength + _suspensionTravel;// Rectify this stuff to make it easier to set suspension length
            suspensionLength = suspensionMaxLength;
            suspensionLengthOld = suspensionMaxLength;
        }
    }

    [Serializable]
    internal class Wheel
    {
        [Header("Wheel")]
        [SerializeField] internal GameObject wheelModel;
        [SerializeField] internal float wheelRadius;
        [SerializeField] internal float frictionCoefficient;




        internal Wheel(GameObject _wheelModel, float _wheelRadius, float _frictionCoefficient)
        {
            wheelModel = _wheelModel;
            wheelRadius = _wheelRadius;
            frictionCoefficient = _frictionCoefficient;
        }
    }

    [Serializable]
    internal class Engine
    {
        [Header("Engine")]
        [SerializeField] private int numberOfCylinders;

    }

    [Serializable]
    internal class Transmission
    {
        [Header("Transmission")]
        [SerializeField] private int numberOfGears;
    }

    internal static float CalculateSuspensionForceRaycast(RaycastHit ray, ref Suspension suspension, ref Wheel wheel, ref bool stickySuspension)
    {
        // TODO : convert ref suspension to ref suspension array
        // TODO : add for loop to loop over all suspension


        ///Suspension
        float suspensionForce, dampingForce;
        suspension.suspensionLengthOld = suspension.suspensionLength;

        suspension.suspensionLength = ray.distance - wheel.wheelRadius;

        suspension.suspensionLength = Mathf.Clamp(suspension.suspensionLength, suspension.suspensionMinLength, suspension.suspensionMaxLength);

        dampingForce = suspension.dampingStiffness * VehicleMaths.Acceleration(suspension.suspensionLengthOld, suspension.suspensionLength, Time.fixedDeltaTime);

        suspensionForce = VehicleMaths.HookesLawCompressionSpring(suspension.stiffness, suspension.restLength - suspension.suspensionLength) + dampingForce;

        if (!stickySuspension)//this makes the car more stable when turned on (could consider making it switch on and off based off of some condition)
            suspensionForce = Mathf.Clamp(suspensionForce, 0, Mathf.Infinity);
        // this line above is needed so that the car doesnt stick to ceilings or walls

        //This bit needs to hold the objects that are on the suspension points to be fully robust and handle more than one object without issue
        //GameObject objectOnSusspension = ray.collider.gameObject;
        //if (objectOnSusspension.TryGetComponent(out Rigidbody otherObjectRb))
        //{
        //    otherObjectRb.AddForceAtPosition(new Vector3(0, VehicleMaths.HookesLawCompressionSpring(suspension.stiffness, suspension.restLength - suspension.suspensionLength) +
        //        (suspension.dampingStiffness * Acceleration(objectOnSusspensionLastPos.y, objectOnSusspension.transform.position.y, Time.fixedDeltaTime)), 0), ray.point);
        //}
        //objectOnSusspensionLastPos = ray.collider.gameObject.transform.position;

        return suspensionForce;
    }


    internal static float LateralFriction(Transform _suspensionTransform, Rigidbody rb, Wheel _wheel)
    {
        Vector3 tireWorldVel = rb.GetPointVelocity(_suspensionTransform.position);
        Vector3 tireLocalVel = _suspensionTransform.InverseTransformDirection(tireWorldVel);
        return VehicleMaths.Friction(_wheel.frictionCoefficient, tireLocalVel.x);
    }



    /*
     * tank track implementation 
     * hinge/spring between each track segment pulling together
     * force that pushes outwards againest the tracks
     * way to get a select fdew segments near the drive sprocket
     * then method ot translate force into the selected segments and move the segments round the wheels
    */
}
