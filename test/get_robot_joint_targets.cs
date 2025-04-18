// Copyright 2019 Siemens Industry Software Ltd.
using System;
using System.IO;
using System.Windows.Forms;
using Tecnomatix.Engineering;

public class MainScript
{
  
    public static void MainWithOutput(ref StringWriter output)
    {
    	// Decision varible
    	int dec_var = 7; // operation of which the robot joints are needed
    
    	// Define the vectors
    	double[] base_poses = new double[] { 214.0, 239.0, -252.0, -281.0, 94.0, 107.0, 250.0, 500.0 };
    	string[] item_names = new string[] { "Cube_02", "Cube_03", "Cube_01", "Cube_00", "Cube_11", "Cube_10", "Cube_12", "Cube_13" };
    	
    	// Define the static variables
    	bool save_in_file = true;
    	bool visualize_each_pose = false;
    	string item_name = item_names[dec_var];
    	double base_pos = base_poses[dec_var];
    	string op_name = "RobotProgram_" + item_name;
    	string rob_name = "GoFa12";
    	
    	// Get the robot by name
    	TxRobot robot = TxApplication.ActiveDocument.GetObjectsByName(rob_name)[0] as TxRobot;
        ITxDevice robot_device = TxApplication.ActiveDocument.GetObjectsByName(rob_name)[0] as ITxDevice;
        
        // Put the robot to the correct configuration
        TxVector translation = new TxVector(base_pos, 0, 1); // 1mm of offset to avoid collisions
        TxVector orientation = new TxVector(0, 0, 0);
        TransformPose(robot, translation, orientation);
        
        // Get the waypoints of the desired robotic operation
        TxContinuousRoboticOperation MyOp = TxApplication.ActiveDocument.GetObjectsByName(op_name)[0] as TxContinuousRoboticOperation;
        TxTypeFilter filter = new TxTypeFilter(typeof(TxRoboticViaLocationOperation));
        TxObjectList points = MyOp.GetAllDescendants(filter);
        
        // Loop through all the points of the specific operation
        int k = 0; // counter
        output.WriteLine("Pick&Place operation number: " + dec_var.ToString() + " called: " + op_name + "\n");
        output.WriteLine("The robot base position is: " + base_pos.ToString() + " mm wrt center");
        foreach(var point in points)
        {
        	// Get the point
        	TxRoboticViaLocationOperation point_new = point as TxRoboticViaLocationOperation;
        	TxRobotInverseData inv = new TxRobotInverseData(point_new.AbsoluteLocation);
        	var poses = robot.CalcInverseSolutions(inv);
        	var pose = poses[0] as TxPoseData;
        	if(k == 1 && item_name == "Cube_03") // Minor temporary imperfection
        	{
        		pose = poses[2] as TxPoseData;
        	}
        	robot.CurrentPose = pose;
        	if(visualize_each_pose)
        	{
        		TxMessageBox.Show(point_new.Name.ToString(), "Visualize robot pose", MessageBoxButtons.OK,
				MessageBoxIcon.Information);
        	}
        	
        	// refresh the display
        	TxApplication.RefreshDisplay();
        	output.WriteLine("\n");
        	output.Write("Point: " + point_new.Name.ToString() + "\n");
        	
        	// Define the list of joints
        	TxObjectList joints = robot_device.DrivingJoints;
        	for (int i = 0; i < joints.Count; i++)
        	{
            	TxJoint joint = joints[i] as TxJoint;          	
            	output.Write("Joint number: " + joint.Name.ToString() + "; Value: " + joint.CurrentValue.ToString() + "\n");
            	        	
        	}
        	
        	// Update the counter
        	k++;
        }

        // Save output to file (if needed)
        if(save_in_file)
        {
        
			string fileName = "robot_output_" + op_name + ".txt";
			string path = "C:/Users/chris/OneDrive - Politecnico di Milano/Politecnico di Milano/PhD - dottorato/Works and suggestions by Marco/Final project Camozzi/Joint positions/" + fileName;

        	//string path = "C:/Users/chris/OneDrive - Politecnico di Milano/Politecnico di Milano/PhD - dottorato/Works and suggestions by Marco/Final project Camozzi/Joint positions/robot_output.txt"; // You can customize the path here
        	File.WriteAllText(path, output.ToString());
        	MessageBox.Show("Output saved to: " + Path.GetFullPath(path));
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
