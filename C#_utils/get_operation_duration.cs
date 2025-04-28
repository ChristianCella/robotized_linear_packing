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
    public static void MainWithOutput(ref StringWriter output)
    {

        // Define static variables
        string pp_root = "P&P_";
        string move_base_root = "MoveBase";

        // Define the vector of names
        string[] item_names = new string[] { "Cube_01", "Cube_00", "Cube_02", "Cube_12", "Cube_11", "Cube_10" }; // ERP1 complete
        // string[] item_names = new string[] { "Cube_01", "Cube_00", "Cube_02", "Cube_12", "Cube_11", "Cube_10" }; // ERP2 complete
        //string[] item_names = new string[] { "Cube_02", "Cube_01", "Cube_00", "Cube_12", "Cube_11", "Cube_10" }; // ERP2 time

        // Get all the operations
        for (int i = 0; i < item_names.Length; i++)
        {

            // Get the move base operation
            TxObjectList move_base = TxApplication.ActiveDocument.GetObjectsByName(move_base_root + item_names[i]);
            var add_move_base_op = move_base[0] as ITxObject;
            var move_base_op = move_base[0] as ITxOperation;
            TxApplication.ActiveDocument.CurrentOperation = move_base_op; 
            double move_base_duration = move_base_op.Duration;

            // Get the pick and place operation
            TxObjectList pick_place = TxApplication.ActiveDocument.GetObjectsByName(pp_root + item_names[i]);
            var add_pick_place_op = pick_place[0] as ITxObject;
            var pick_place_op = pick_place[0] as ITxOperation;
            TxApplication.ActiveDocument.CurrentOperation = pick_place_op; 
            double pick_place_duration = pick_place_op.Duration;
                       
            output.WriteLine("Item: " + item_names[i] + " - Move base duration: " + move_base_duration + " - Pick and place duration: " + pick_place_duration);
        }

    }
}