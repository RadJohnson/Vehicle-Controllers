using RADVehichles;
using System;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class Vehicle : MonoBehaviour
{
    // TODO : Decide how I want to store wheels and if I want them to be kept within each Suspension
    // TODO : Implement steering angle float between 0 and 180
    [Serializable]
    internal class Suspension
    {
        [Header("Suspension")]
        /*[SerializeField]*/
        internal Vector3 suspensionWorldPosition;//the one to work with
        [SerializeField] internal Vector3 suspensionLocalPosition;//the one to set initially
        //[SerializeField] internal Ray raycast;//TODO: see how to make unserialised data such as a Ray serializeable
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
        [SerializeField] internal float steerAngle;
        //[SerializeField] internal Wheel wheel;


        internal Suspension()
        {
            suspensionTravel = 0.2f;
            //raycast = new Ray(suspensionLocalPosition, new Vector3(0, -1, 0));//TODO : Make Direction of raycast easy to make a specific direction
            restLength = 0.5f;
            stiffness = 2000f;
            dampingStiffness = 250f;
            suspensionWorldPosition = new();
            //steering = _steering;
            //drive = _drive;
            suspensionMinLength = restLength - suspensionTravel;// Rectify this stuff to make it easier to set suspension length
            suspensionMaxLength = restLength + suspensionTravel;// Rectify this stuff to make it easier to set suspension length
            suspensionLength = suspensionMaxLength;
            suspensionLengthOld = suspensionMaxLength;
        }

        internal Suspension(Vector3 _suspensionPosition, float _suspensionTravel, float _restLength, float _stiffness, float _dampingStiffness, bool _steering, bool _drive)
        {
            suspensionWorldPosition = _suspensionPosition;
            //raycast = new Ray(suspensionLocalPosition, new Vector3(0, -1, 0));//TODO : Make Direction of raycast easy to make a specific direction
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

        internal void LocalToWorld(Transform CarTransform)
        {
            Vector3 rotatedPosition = CarTransform.rotation * suspensionLocalPosition;
            suspensionWorldPosition = rotatedPosition + CarTransform.position;
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

    internal static float[] CalculateSuspensionsForceRaycast(/*RaycastHit _ray,*/ ref List<Suspension> _suspensions, ref List<Wheel> _wheels, ref bool _stickySuspension)
    {
        float[] suspensionForces = new float[_suspensions.Count];

        for (int i = 0; i < _suspensions.Count; i++)
        {
            //if (_suspensions[i].raycast == null)
            //{
            //
            //}

            if (Physics.Raycast(_suspensions[i].suspensionWorldPosition, Vector3.down, out RaycastHit intersectPoint, _suspensions[i].suspensionMaxLength + _wheels[i].wheelRadius))
            {
                ///Suspension
                float dampingForce;
                _suspensions[i].suspensionLengthOld = _suspensions[i].suspensionLength;

                _suspensions[i].suspensionLength = intersectPoint.distance - _wheels[i].wheelRadius;

                _suspensions[i].suspensionLength = Mathf.Clamp(_suspensions[i].suspensionLength, _suspensions[i].suspensionMinLength, _suspensions[i].suspensionMaxLength);

                dampingForce = _suspensions[i].dampingStiffness * VehicleMaths.Acceleration(_suspensions[i].suspensionLengthOld, _suspensions[i].suspensionLength, Time.fixedDeltaTime);

                suspensionForces[i] = VehicleMaths.HookesLaw(_suspensions[i].stiffness, _suspensions[i].restLength - _suspensions[i].suspensionLength) + dampingForce;

                if (!_stickySuspension)//this line makes the _car more stable(less likely to roll) when turned true
                    suspensionForces[i] = Mathf.Clamp(suspensionForces[i], 0, Mathf.Infinity);
                // this line ^^above is needed so that the _car doesnt stick to ceilings or walls

                // TODO : Add functionlailty for the suspension to push back against any objects that are on it

                //This bit needs to hold the objects that are on the suspension points to be fully robust and handle more than one object without issue
                //GameObject objectOnSusspension = ray.collider.gameObject;
                //if (objectOnSusspension.TryGetComponent(out Rigidbody otherObjectRb))
                //{
                //    otherObjectRb.AddForceAtPosition(new Vector3(0, VehicleMaths.HookesLaw(suspension.stiffness, suspension.restLength - suspension.suspensionLength) +
                //        (suspension.dampingStiffness * Acceleration(objectOnSusspensionLastPos.y, objectOnSusspension.transform.position.y, Time.fixedDeltaTime)), 0), ray.point);
                //}
                //objectOnSusspensionLastPos = ray.collider.gameObject.transform.position;
            }
            //else
            //{
            //    suspensionForces[i] = 0;
            //    //_suspensions[i]. lateralForce = 0;
            //    //_suspensions[i]. driveForce = 0;
            //    //_suspensions[i]. finalForceWorld = Vector3.zero;
            //    _suspensions[i].suspensionLength = _suspensions[i].suspensionMaxLength;
            //}
        }
        return suspensionForces;
    }

    static float CalculateSuspensionForceRaycast(/*RaycastHit _ray,*/ ref Suspension _suspension, ref Wheel _wheel, ref bool _stickySuspension)
    {
        float suspensionForce = 0;
        if (Physics.Raycast(_suspension.suspensionWorldPosition, Vector3.down, out RaycastHit intersectPoint, _suspension.suspensionMaxLength + _wheel.wheelRadius))
        {
            ///Suspension
            float dampingForce;
            _suspension.suspensionLengthOld = _suspension.suspensionLength;

            _suspension.suspensionLength = intersectPoint.distance - _wheel.wheelRadius;

            _suspension.suspensionLength = Mathf.Clamp(_suspension.suspensionLength, _suspension.suspensionMinLength, _suspension.suspensionMaxLength);

            dampingForce = _suspension.dampingStiffness * VehicleMaths.Acceleration(_suspension.suspensionLengthOld, _suspension.suspensionLength, Time.fixedDeltaTime);

            suspensionForce = VehicleMaths.HookesLaw(_suspension.stiffness, _suspension.restLength - _suspension.suspensionLength) + dampingForce;

            if (!_stickySuspension)//this line makes the _car more stable(less likely to roll) when turned true
                suspensionForce = Mathf.Clamp(suspensionForce, 0, Mathf.Infinity);
            // this line ^^above is needed so that the _car doesnt stick to ceilings or walls

            // TODO : Add functionlailty for the suspension to push back against any objects that are on it

            //This bit needs to hold the objects that are on the suspension points to be fully robust and handle more than one object without issue
            //GameObject objectOnSusspension = ray.collider.gameObject;
            //if (objectOnSusspension.TryGetComponent(out Rigidbody otherObjectRb))
            //{
            //    otherObjectRb.AddForceAtPosition(new Vector3(0, VehicleMaths.HookesLaw(suspension.stiffness, suspension.restLength - suspension.suspensionLength) +
            //        (suspension.dampingStiffness * Acceleration(objectOnSusspensionLastPos.y, objectOnSusspension.transform.position.y, Time.fixedDeltaTime)), 0), ray.point);
            //}
            //objectOnSusspensionLastPos = ray.collider.gameObject.transform.position;
        }
        //else
        //{
        //    suspensionForces[i] = 0;
        //    //_suspensions[i]. lateralForce = 0;
        //    //_suspensions[i]. driveForce = 0;
        //    //_suspensions[i]. finalForceWorld = Vector3.zero;
        //    _suspensions[i].suspensionLength = _suspensions[i].suspensionMaxLength;
        //}

        return suspensionForce;
    }

    //May need the tranform of the _car i think
    static float LateralFriction(Transform car,  Rigidbody _rb,  Vector3 _suspensionTransform, Wheel _wheel)
    {
        Vector3 tireWorldVel = _rb.GetPointVelocity(_suspensionTransform);
        Vector3 tireLocalVel = car.InverseTransformDirection(tireWorldVel);
        return VehicleMaths.Friction(_wheel.frictionCoefficient, tireLocalVel.x);
    }


    //TODO : Put all functions into one easy to call function that takes the suspension points and maybe the controls input

    //There apears to be an issue where the raycasts enter the car when it rotates and causes issues whith the supension working off of itself
    internal void CalculateSuspensionRaycast(Transform _car, ref Rigidbody _rb, ref List<Suspension> _suspensions, ref List<Wheel> _wheels,  bool _stickySuspension)//Rigidbody likely doesnt need to be ref since no values in the rigid body change
    {
        Vector3 forces = new();
        float dampingForce = 0f;
        //float[] suspensionForces = new float[_suspensions.Count];

        for (int i = 0; i < _suspensions.Count; i++)
        {
            _suspensions[i].LocalToWorld(transform);

            if (Physics.Raycast(_suspensions[i].suspensionWorldPosition, -_car.up, out RaycastHit intersectPoint, _suspensions[i].suspensionMaxLength + _wheels[i].wheelRadius))
            {
                //Debug.Log("entered raycast check");
                //Debug.Log(intersectPoint.rigidbody);
                _suspensions[i].suspensionLengthOld = _suspensions[i].suspensionLength;

                _suspensions[i].suspensionLength = intersectPoint.distance - _wheels[i].wheelRadius;

                _suspensions[i].suspensionLength = Mathf.Clamp(_suspensions[i].suspensionLength, _suspensions[i].suspensionMinLength, _suspensions[i].suspensionMaxLength);

                dampingForce = _suspensions[i].dampingStiffness * VehicleMaths.Acceleration(_suspensions[i].suspensionLengthOld, _suspensions[i].suspensionLength, Time.fixedDeltaTime);

                /*suspensionForces[i]*/
                forces.y = VehicleMaths.HookesLaw(_suspensions[i].stiffness, _suspensions[i].restLength - _suspensions[i].suspensionLength) + dampingForce;

                if (!_stickySuspension)//this line makes the _car more stable(less likely to roll) when turned true
                    /*suspensionForces[i]*/
                    forces.y = Mathf.Clamp(/*suspensionForces[i]*/forces.y, 0, Mathf.Infinity);


                //LATERAL FORCE
                forces.x = -LateralFriction(_car,  _rb,  _suspensions[i].suspensionWorldPosition, _wheels[i]);


                //Rolling Resistance
                //Frr = -Crr * velocity.normalized() * velocity.magnitude()


            }
            else
            {
                //Debug.Log("Did not enter raycast check");

                forces = new();
                _suspensions[i].suspensionLength = _suspensions[i].suspensionMaxLength;
            }

            //Debug.Log(forces);

            //need to rectify this and make force alight with world direction

            forces = _car.TransformDirection(forces);

            _rb.AddForceAtPosition(forces, _suspensions[i].suspensionWorldPosition);

            //totalSuspensionForce.y = CalculateSuspensionForceRaycast(ref _suspension, ref _wheel, ref _stickySuspension);
            //totalSuspensionForce.x = LateralFriction(_car, ref _suspension.suspensionWorldPosition, ref _rb, ref _wheel);
            //_rb.AddForceAtPosition(totalSuspensionForce, _suspension.suspensionWorldPosition);
        }

    }

    // TODO : wheel add 3 raycasts or something maybe to get better tyre position


    //TODO : Come up with better name for this function and populate it with correct functions
    //void CalculateSuspension()
    //{
    //
    //}


    /*
     * tank track implementation 
     * hinge/spring between each track segment pulling together
     * force that pushes outwards againest the tracks
     * way to get a select fdew segments near the drive sprocket
     * then method ot translate force into the selected segments and move the segments round the wheels
    */


}

