/*
This snippet allows to concatenate operations in a compound operation. The fundamental part is that the operations
must be present in the document (possibly created by some previous scripts). They must be targeted as objects:
thanks to the AddObject method, they will be grouped inside a compound operation.
The problem that this script does NOT tackle is that they start from the same time instsnt: another script
must be created to create the real Gantt chart.
*/

using System;
using System.IO;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Tecnomatix.Engineering;
using Tecnomatix.Engineering.Olp;
using Tecnomatix.Engineering.Plc;
using Tecnomatix.Engineering.Utilities;
using Tecnomatix.Engineering.ModelObjects;
using System.Windows.Forms;

public class MainScript
{
    public static void MainWithOutput()
    {

        // Define static variables
        string comp_op_name = "Robot_program"; // T_r
        string pp_root = "P&P_";
        string move_base_root = "MoveBase";
        string pose_root = "BasePose";

        // Define the vector of names
        string[] item_names = new string[] { "Cube_01", "Cube_00", "Cube_02", "Cube_11", "Cube_12", "Cube_10" };

        // Create the compound operation and save it in a variable
        TxCompoundOperationCreationData dat = new TxCompoundOperationCreationData(comp_op_name);
        TxApplication.ActiveDocument.OperationRoot.CreateCompoundOperation(dat);       
        TxObjectList operations = TxApplication.ActiveDocument.GetObjectsByName(comp_op_name);
    	var comp_op = operations[0] as TxCompoundOperation;

        // Get the line  	
        TxObjectList objects = TxApplication.ActiveDocument.GetObjectsByName("Line");
        var line = objects[0] as TxDevice;

        // Define the vector os durations
        double durations = 0;

        // Simulation player
        TxSimulationPlayer Player = TxApplication.ActiveDocument.SimulationPlayer;

        // Initialize the pose data
        TxPose current_pose = TxApplication.ActiveDocument.GetObjectsByName("MIDDLE")[0] as TxPose;
        
        // Get all the operations
        for (int i = 0; i < item_names.Length; i++)
        {
            // Move the robot to the new start position
            if (i > 0)
            {
                current_pose = TxApplication.ActiveDocument.GetObjectsByName(pose_root + item_names[i-1])[0] as TxPose;
            }
            line.CurrentPose = current_pose.PoseData;

            // Get the move base operation
            TxObjectList move_base = TxApplication.ActiveDocument.GetObjectsByName(move_base_root + item_names[i]);
            var add_move_base_op = move_base[0] as ITxObject;
            var move_base_op = move_base[0] as ITxOperation;
            TxApplication.ActiveDocument.CurrentOperation = move_base_op; 
            Player.PlaySilently();          
            Player.Rewind(); 
            double move_base_duration = move_base_op.Duration;

            // Get the pick and place operation
            TxObjectList pick_place = TxApplication.ActiveDocument.GetObjectsByName(pp_root + item_names[i]);
            var add_pick_place_op = pick_place[0] as ITxObject;
            var pick_place_op = pick_place[0] as ITxOperation;
            TxApplication.ActiveDocument.CurrentOperation = pick_place_op; 
            Player.PlaySilently();          
            Player.Rewind(); 
            double pick_place_duration = pick_place_op.Duration;
                       
            // Add the operations to the compound operation
            comp_op.AddObject(add_move_base_op);
            comp_op.AddObject(add_pick_place_op);
            
            // Sequence the operations
            comp_op.SetChildOperationRelativeStartTime(move_base_op, durations + i * 0.005);
            durations = durations + move_base_duration;
            comp_op.SetChildOperationRelativeStartTime(pick_place_op, durations + i * 0.005);
            durations = durations + pick_place_duration;
        }

    }
}