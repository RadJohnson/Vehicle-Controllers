using System.Collections.Generic;
using UnityEditor;
using UnityEngine;
using UnityEngine.UIElements;
using static Vehicle;

//TODO : figure out how to make a more effective editor script that will work for other vehichle types when they are implemented
[CustomEditor(typeof(Car))]
public class CarEditorScript : Editor
{

    GUIStyle guiStyle;


    private Car car;

    private bool toggleCentreOfMassHandle;
    private bool toggleDirectionHandles;
    private bool toggleSuspensionHandles;

    //private string centreOfMassButtonText = "Centre Of Mass visibility toggle";//TODO : remove or implement later
    private Vector3 centerOfMass;
    private Tool lastTool;
    private bool toolStored;

    private GameObject lastActiveGameObject;

    private Rigidbody rigidbody;



    private void OnEnable()
    {
        lastTool = Tools.current;
        //lastActiveGameObject = Selection.activeGameObject;//TODO : 
        car = (Car)target;
        rigidbody = car.GetComponent<Rigidbody>();

        guiStyle = new GUIStyle();//TODO : Create multiple styles that are "static" for each GUI element that is being handled

    }
    private void OnDisable()
    {
        Tools.current = lastTool;
    }


    public override void OnInspectorGUI()
    {
        base.OnInspectorGUI();//Regular inspector gubbins from the actual script


        toggleCentreOfMassHandle = EditorGUILayout.Toggle("Toggle Centre Of Mass Handle", toggleCentreOfMassHandle);//TODO : Add reset button for centre of mass position
        toggleDirectionHandles = EditorGUILayout.Toggle("Toggle Direction Handles", toggleDirectionHandles);
        toggleSuspensionHandles = EditorGUILayout.Toggle("Toggle Suspension Handles", toggleSuspensionHandles);


        //if (toggleCentreOfMassHandle || toggleSuspensionHandles)
        //{
        //    StoreLastTool(ref toolStored);
        //}
        //else
        //{
        //    StoreLastTool(ref toolStored);
        //}


        //TODO : Figure out how to visualy improve the above handle toggles 
        //if (GUILayout.Button(centreOfMassButtonText))
        //{
        //
        //    //centerOfMass = car.GetComponent<Rigidbody>().centerOfMass;
        //
        //}



        EditorGUILayout.LabelField("Suspension");

        if (GUILayout.Button("Generate Suspension"))
        {
            car.suspensions.Add(new Vehicle.Suspension());
        }
        if (GUILayout.Button("Delete Suspension") && car.suspensions.Count != 0)
        {
            car.suspensions.RemoveAt(car.suspensions.Count - 1);
        }

        if (GUI.changed)
        {
            EditorUtility.SetDirty(car);
        }
    }

    private void OnSceneGUI()
    {

        if (car == null)
        {
            return;
        }

        if (toggleCentreOfMassHandle == true)
        {
            //TODO : Add function here similar to the handles toggle to store the previous centre of mass
            rigidbody.centerOfMass = Handles.PositionHandle(rigidbody.centerOfMass, Quaternion.identity);//TODO : Fix handle location so that it is in local space not world space

            //TODO : When everything else is working as intended Make a hazard striped sphere to help visually show centre of mass
            //Handles.color = Color.yellow;
            //Handles.SphereHandleCap(0, rigidbody.centerOfMass, Quaternion.identity, 0.5f, EventType.Repaint);

        }


        if (toggleDirectionHandles == true)
        {
            DirectionHandles();
        }

        if (toggleSuspensionHandles == true)
        {
            if (car.suspensions != null && car.suspensions.Count > 0)
            {
                SuspensionHandles();
            }
        }
    }

    void StoreLastTool(ref bool _toggled)
    {
        if (lastTool != Tool.None && !_toggled)
        {
            Tools.current = lastTool;
            _toggled = true;
        }
        else
        {
            lastTool = Tools.current;
            Tools.current = Tool.None;
            _toggled = false;
        }
    }

    //TODO : Refactor Below Functions to improve readability and reusability
    void DirectionHandles()
    {
        Handles.color = Color.magenta;
        Handles.ArrowHandleCap(0, car.transform.position, Quaternion.LookRotation(car.transform.forward), 1f, EventType.Repaint);
        guiStyle.fontSize = 18;
        guiStyle.normal.textColor = Color.black;
        Handles.Label(car.transform.position + car.transform.forward + new Vector3(0.15f, 0f, 0f), "Forward", guiStyle);

        Handles.color = Color.cyan;
        Handles.ArrowHandleCap(0, car.transform.position, Quaternion.LookRotation(car.transform.up, car.transform.forward), 1f, EventType.Repaint);
        guiStyle.fontSize = 18;
        guiStyle.normal.textColor = Color.black;
        Handles.Label(car.transform.position + car.transform.up + new Vector3(0f, 0.15f, 0f), "Up", guiStyle);
    }

    void SuspensionHandles(/*List<Vehicle.Suspension> suspensions*/)
    {
        foreach (var suspension in car.suspensions)
        {
            //Calculate world position using LocalToWorld function
            Vector3 rotatedPosition = car.transform.rotation * suspension.suspensionLocalPosition;
            suspension.suspensionWorldPosition = rotatedPosition + car.transform.position;
        }

        for (int i = 0; i < car.suspensions.Count; i++)
        {
            //Get the suspension
            Car.Suspension suspension = car.suspensions[i];

            //Display a position handle for each suspension
            EditorGUI.BeginChangeCheck();

            //TODO : change this block so that it reads better
            //Adjust the handle to work in local space relative to the car
            //Vector3 localPos = suspension.suspensionLocalPosition;
            //Vector3 worldPos = car.transform.TransformPoint(localPos); // Convert to world position for the handle

            
            Vector3 newWorldPos = Handles.PositionHandle(LocalToWorld(car.transform, suspension.suspensionLocalPosition), Quaternion.identity);// TODO : Consider making a toggle to allow the handles rotation to match the vehichles

            if (EditorGUI.EndChangeCheck())
            {
                Undo.RecordObject(car, "Move Suspension");

                //Convert the world position back to local for storage
                suspension.suspensionLocalPosition = car.transform.InverseTransformPoint(newWorldPos);
            }

            //Display the label above each suspension point
            guiStyle.normal.textColor = Color.yellow;
            guiStyle.fontSize = 18;
            Handles.Label(newWorldPos + Vector3.up * 0.5f, $"Suspension {i}", guiStyle);
        }
    }

    Vector3 LocalToWorld(Transform _transform, Vector3 _localPosition)
    {
        return _transform.TransformPoint(_localPosition);
    }
}