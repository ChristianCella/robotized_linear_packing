/*
This snippet allows to create a new pose for a device in the Tecnomatix Plant Simulation software. It was used to create the poses
for the robot base on te Camozzi line, but with some small adaptatioons it can be used to create poses also for
the fingers of the grippers.
*/
 
using System.Collections;
using System;
using System.IO;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Tecnomatix.Engineering;
using Tecnomatix.Engineering.Olp;
using Tecnomatix.Engineering.Plc;
 
public class MainScript
{
    public static void Main(ref StringWriter output)
    {
        // Define the vectors
    	double[] base_poses = new double[] { -158.0, -396.0, 250.0, 22.0, 248.0, 676.0 }; // With respect to the line center
    	string[] item_names = new string[] { "Cube_01", "Cube_00", "Cube_02", "Cube_11", "Cube_12", "Cube_10" };
        
        // Define parameters
        string device_name = "Line";
        string pose_name = "BasePose";
        double x_offset = 1500; // Offset to the right of the line center

        // Get the device by name
		TxObjectList selectedObjects = TxApplication.ActiveSelection.GetItems();
        selectedObjects = TxApplication.ActiveDocument.GetObjectsByName(device_name);
        TxDevice line_device = selectedObjects[0] as TxDevice;

        // Create all the poses
        for (int i = 0; i < base_poses.Length; i++)
        {
            // Create the pose name
            TxPoseData openposeData = new TxPoseData();
            ArrayList openarraylist = new ArrayList();
            openarraylist.Add(base_poses[i] + x_offset); // X coordinate
            openposeData.JointValues = openarraylist;
            TxPoseCreationData NewPose = new TxPoseCreationData("BasePose" + item_names[i], openposeData);
            TxPose new_base_pose = line_device.CreatePose(NewPose);
        }

    
    }	
}