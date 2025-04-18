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
    public static void MainWithOutput(ref StringWriter output)
    {
        // Initialization variables for the pick and place 	
        TxHumanTsbSimulationOperation op = null;
        TxHumanTSBTaskCreationDataEx taskCreationData = new TxHumanTSBTaskCreationDataEx();

        // Get the human		
        TxObjectList humans = TxApplication.ActiveDocument.GetObjectsByName("Jack");
        TxHuman human = humans[0] as TxHuman;

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
        op = TxHumanTSBSimulationUtilsEx.CreateSimulation("Wait");
        op.SetInitialContext();
        
        // Fill all the fields: without some of these, the script crashes
        taskCreationData.PrimaryObject = component;
        taskCreationData.TargetLocation = position_place;
        taskCreationData.Human = human;				
   		taskCreationData.TaskType = TsbTaskType.HUMAN_Wait;	
        taskCreationData.TaskDuration = 10;
        TxHumanTsbTaskOperation tsbPoseTaskInt = op.CreateTask(taskCreationData);
        op.ApplyTask(tsbPoseTaskInt, 1);
        TxApplication.RefreshDisplay();

        // Make modifications effective
        op.ForceResimulation();
    }
}
