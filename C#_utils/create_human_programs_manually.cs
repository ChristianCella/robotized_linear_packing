/*
This snippet allows to create a 'wait' operation for the human. This simulates just the presence of the operator
in the simulation, without any action.
*/

using System;
using System.IO;
using Tecnomatix.Engineering;
using EngineeringInternalExtension;
using Tecnomatix.Engineering.ModelObjects;

public class MainScript
{
    // Define the vectors for ERP1
    /*
    static double[] poses_x = new double[] { -1248.0, 3000.0, 1248.0, 1248.0, 3000.0, -1248.0, 3000.0 }; 
    static double[] poses_y = new double[] { -940.0, 3000.0, -940.0, 940.0, 3000.0, 940.0, 3000.0 }; 
    static double[] rot_z = new double[] { Math.PI / 2, Math.PI / 2, Math.PI / 2, -Math.PI / 2, Math.PI / 2, -Math.PI / 2, Math.PI / 2 }; 
    static double[] durations = new double[] { 10.0, 15.0, 10.0, 10.0, 15.0, 10.0, 15.0 }; 
    static string[] op_names = new string[] { "A", "safe1", "B", "C", "safe2", "D", "safe3" };
    */

    // Define the vectors for ERP2
    static double[] poses_x = new double[] { 1248.0, 3000.0, -1248.0, 3000.0, 1248.0, -1248.0, 3000.0 }; 
    static double[] poses_y = new double[] { -940.0, 3000.0, 940.0, 3000.0, 940.0, 940.0, 3000.0 }; 
    static double[] rot_z = new double[] { Math.PI / 2, Math.PI / 2, -Math.PI / 2, Math.PI / 2, -Math.PI / 2, -Math.PI / 2, Math.PI / 2 }; 
    static double[] durations = new double[] { 8.0, 7.0, 10.0, 15.0, 10.0, 10.0, 15.0 }; 
    static string[] op_names = new string[] { "B", "safe1", "D", "safe2", "C", "D", "safe3" }; // No station A

    public static void MainWithOutput(ref StringWriter output)
    {
        // Initialization variables for the pick and place 	
        TxHumanTsbSimulationOperation op = null;
        TxHumanTSBTaskCreationDataEx taskCreationData = new TxHumanTSBTaskCreationDataEx();

        // Get the human		
        TxObjectList humans = TxApplication.ActiveDocument.GetObjectsByName("Jack");
        TxHuman human = humans[0] as TxHuman;

        // Create all the operations
        for(int i = 0; i < poses_x.Length; i++)
        {
            // Apply the correct position to the human
            TxVector translation = new TxVector(poses_x[i], poses_y[i], 142.0);
            TxVector orientation = new TxVector(0, 0, rot_z[i]);
            TransformPose(human, translation, orientation);

            // Important: these informations are FUNDAMENTAL to not make the script crash
            TxObjectList cube_pick = TxApplication.ActiveSelection.GetItems();
            cube_pick = TxApplication.ActiveDocument.GetObjectsByName("Cube_01");
            var component = cube_pick[0] as ITxLocatableObject;
            
            TxObjectList ref_frame_cube_place = TxApplication.ActiveSelection.GetItems();
            ref_frame_cube_place = TxApplication.ActiveDocument.GetObjectsByName("fr_Cube_02");
            var frame_cube_place = ref_frame_cube_place[0] as ITxLocatableObject;
            var target_place = new TxTransformation(frame_cube_place.AbsoluteLocation);	
            var position_place = new TxTransformation(component.AbsoluteLocation);
            position_place.Translation = new TxVector(target_place[0, 3], target_place[1, 3], target_place[2, 3]);
            position_place.RotationRPY_ZYX = target_place.RotationRPY_ZYX;

            // Create the simulation  		
            op = TxHumanTSBSimulationUtilsEx.CreateSimulation(op_names[i]);
            op.SetInitialContext();
            
            // Fill all the fields: without some of these, the script crashes
            taskCreationData.PrimaryObject = component;
            taskCreationData.TargetLocation = position_place;
            taskCreationData.Human = human;				
            taskCreationData.TaskType = TsbTaskType.HUMAN_Wait;	
            taskCreationData.TaskDuration = durations[i]; // seconds
            TxHumanTsbTaskOperation tsbPoseTaskInt = op.CreateTask(taskCreationData);
            op.ApplyTask(tsbPoseTaskInt, 1);
            TxApplication.RefreshDisplay();

            // Make modifications effective
            op.ForceResimulation();
            
        }
        
    }

    private static void TransformPose(ITxObject item, TxVector translation, TxVector orientation)
    {
        // Make sure the item is an ITxLocatableObject
        ITxLocatableObject locatableItem = item as ITxLocatableObject;

        // Move the base of a certain quantity		
        var pose = new TxTransformation(locatableItem.LocationRelativeToWorkingFrame);
        pose.Translation = translation;
        pose.RotationRPY_XYZ = orientation;
        locatableItem.LocationRelativeToWorkingFrame = pose;

        TxApplication.RefreshDisplay();
    }
}
