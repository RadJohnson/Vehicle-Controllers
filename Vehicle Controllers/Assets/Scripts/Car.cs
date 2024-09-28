using System;
using UnityEngine;

public class Car : Vehicle
{
    [SerializeField] Suspension[] suspensions;
    [SerializeField] Wheel[] Wheels;

    // new Suspension()
    // new Wheel(,0.43f, 200)
    //
    // Vehicle car = new Vehicle(, 4, , 4);

    private void Awake()
    {
        suspensions = new Suspension[4];
        
        for (int i = 0; i < suspensions.Length; i++) 
        {
            suspensions[i] = new Suspension();
        }
    }


    private void Update()
    {
        //CalculateSuspensionForceRaycast();
    }
}
