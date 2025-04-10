/*
    Use this script if you want to test a specific Pick&Place operation without using a socket.
*/

using System;
using System.Net;
using System.Net.Sockets;
using System.Runtime.InteropServices;
using System.Text;
using System.IO;
using System.Windows.Forms;
using Tecnomatix.Engineering;
using System.Collections.Generic;
using Tecnomatix.Engineering.Olp;
using System.Linq;
using System.Collections;

public class MainScript
{
    // General variables
    static string robot_name = "GoFa12";
    static string gripper_name = "Suction_cup";
    static string item_name = "Cube_13";
    static string program_name = "RobotProgram_" + item_name;
    static string tcp_flange_name = "TOOLFRAME";
    static string tcp_ee_name = "tgripper_tf";

	// Variables to create the robot program
    static string new_motion_type = "PTP";
    static string new_speed = "100%";
    static string new_accel = "100%";
    static string new_blend = "fine";

    public static void Main()
    {
    	// Get robot and gripper
    	TxRobot Robot = GetRobot(robot_name);
        ITxObject tool = GetGripper(gripper_name);
        
        // Define the place pose (manually, center of the box)
        double[] place_pose = new double[4];
        place_pose[0] = -350;
        place_pose[1] = -425;
        place_pose[2] = -7.14;
        place_pose[3] = 0;
        
    	// Instantiate the item to be picked
        ITxObject considered_item = GetItem(item_name);
           
        // Create the robotc operation
        TxContinuousRoboticOperation MyOp = RobotPickPlace(
            Robot, 
            considered_item, 
            tool, 
            program_name, 
            item_name, 
            tcp_flange_name, 
            tcp_ee_name, 
            200.0, 
            place_pose);       
    }
    
    /*
    User-defined methods
    */

    // Get the manipulator
    private static TxRobot GetRobot(string robotName)
    {
        TxRobot Robot = null;
        int index = 0;
        while (Robot == null)
        {
			
			Robot = TxApplication.ActiveDocument.GetObjectsByName(robotName)[index] as TxRobot;

        }

        return Robot;
    }

    // Get the gripper
    private static ITxObject GetGripper(string gripperName)
    {
        ITxObject Gripper = TxApplication.ActiveDocument.GetObjectsByName(gripperName)[0] as ITxObject;
        return Gripper;
    }

    // Get the object
    private static ITxObject GetItem(string item_name)
    {
        ITxObject Item = TxApplication.ActiveDocument.GetObjectsByName(item_name)[0];
        return Item;
    }
    
    private static TxContinuousRoboticOperation InitializeRoboticOperation(TxRobot robot, string op_name)
    {
        // Create the new operation
        TxContinuousRoboticOperationCreationData data = new TxContinuousRoboticOperationCreationData(op_name);
        TxApplication.ActiveDocument.OperationRoot.CreateContinuousRoboticOperation(data);

        // Get the created operation
        TxContinuousRoboticOperation MyOp = TxApplication.ActiveDocument.GetObjectsByName(op_name)[0] as TxContinuousRoboticOperation;

        // Associate the robot to the operation
        MyOp.Robot = robot;

        return MyOp;
    }

    // Create the robotic program
    public static TxContinuousRoboticOperation RobotPickPlace(
        TxRobot robot, 
        ITxObject considered_item,
        ITxObject tool,
        string op_name, 
        string item_name, 
        string tcp_flange_name,
        string tcp_ee_name,
        double z_offset,
        double[] place_pose_received)
    {
        // Initialize operation
        TxContinuousRoboticOperation MyOp = InitializeRoboticOperation(robot, op_name);

        // Create operation using atomic operations
        Approach("pick", item_name, tcp_ee_name, z_offset, MyOp, place_pose_received);
        Overfly(item_name, MyOp, z_offset, place_pose_received);
        Approach("place", item_name, tcp_ee_name, z_offset, MyOp, place_pose_received);

        // Specify the parameters
        TxTypeFilter filter = new TxTypeFilter(typeof(TxRoboticViaLocationOperation));
        TxObjectList points = MyOp.GetAllDescendants(filter);
        TxOlpControllerUtilities ControllerUtils = new TxOlpControllerUtilities();

        ITxOlpRobotControllerParametersHandler paramHandler = (ITxOlpRobotControllerParametersHandler)
        ControllerUtils.GetInterfaceImplementationFromController(robot.Controller.Name,
        typeof(ITxOlpRobotControllerParametersHandler), typeof(TxRobotSimulationControllerAttribute),
        "ControllerName");

        for (int ii = 0; ii < points.Count; ii++)
        {
            SetWaypointValues(points[ii].Name.ToString(), paramHandler, tcp_ee_name);
        }

        // OLP command for attaching/detaching the obejct
        SuctionOn(points[2].Name.ToString(), tool, considered_item);
        SuctionOff(points[5].Name.ToString(), considered_item);

        return MyOp;
    }

    // First elementary operation
    static void Approach(
        string op_type, 
        string item_name, 
        string tcp_ee_name, // TOOLFRAME
        double z_offset, 
        TxContinuousRoboticOperation MyOp, 
        double[] place_pose_received)
    {
        // Create the 3 necessary points
        TxRoboticViaLocationOperationCreationData Point1 = new TxRoboticViaLocationOperationCreationData();
        Point1.Name = "point1_" + op_type + "_" + item_name; // First point
        TxRoboticViaLocationOperationCreationData Point2 = new TxRoboticViaLocationOperationCreationData();
        Point2.Name = "point2_" + op_type + "_" + item_name; // Second point
        TxRoboticViaLocationOperationCreationData Point3 = new TxRoboticViaLocationOperationCreationData();
        Point3.Name = "point3_" + op_type + "_" + item_name; // Third point

        // Add the points to the operation
        TxRoboticViaLocationOperation FirstPoint = MyOp.CreateRoboticViaLocationOperation(Point1);
        TxRoboticViaLocationOperation SecondPoint = MyOp.CreateRoboticViaLocationOperation(Point2);
        TxRoboticViaLocationOperation ThirdPoint = MyOp.CreateRoboticViaLocationOperation(Point3);

        // Pick the object
        if (op_type == "pick")
        {
            TxObjectList ref_frame_obj_pick = TxApplication.ActiveSelection.GetItems();
            ref_frame_obj_pick = TxApplication.ActiveDocument.GetObjectsByName("fr_" + item_name);
            TxFrame frame_obj_pick = ref_frame_obj_pick[0] as TxFrame;
            var position_pick = new TxTransformation(frame_obj_pick.AbsoluteLocation);

            // First point -> Home
            ConfigureInitialPosition(FirstPoint, tcp_ee_name);

            // Third point: Pick position
            ThirdPoint.AbsoluteLocation = position_pick;
            var pickOffsetPosition = new TxTransformation(ThirdPoint.AbsoluteLocation);
            pickOffsetPosition.Translation = new TxVector(position_pick[0, 3], position_pick[1, 3], position_pick[2, 3]);
            ThirdPoint.AbsoluteLocation = pickOffsetPosition;

            // Second point: Pick approach position (higher approach)
            SecondPoint.AbsoluteLocation = position_pick;
            var pickApproachPosition = new TxTransformation(SecondPoint.AbsoluteLocation);
            pickApproachPosition.Translation = new TxVector(position_pick[0, 3], position_pick[1, 3], position_pick[2, 3] + z_offset);
            SecondPoint.AbsoluteLocation = pickApproachPosition;
        }
        else if (op_type == "place")
        {
            // Get the object to be placed
            var place_point_x = place_pose_received[0]; 
            var place_point_y = place_pose_received[1]; 
            var place_point_z = place_pose_received[2];
            var rotation = place_pose_received[3];
            var place_point = new TxTransformation(new TxVector(place_point_x, place_point_y, place_point_z), TxTransformation.TxTransformationType.Translate);
            // Third point -> Home
            ConfigureInitialPosition(ThirdPoint, tcp_ee_name);

            // First point: Place position
            double rotZ = 0; // Nominal value
            if (rotation == 1)
            {
                rotZ = Math.PI / 2; // Rotate 90 degrees
            }

            // First point: place position
            FirstPoint.AbsoluteLocation = place_point;
            var placeOffsetPosition = new TxTransformation(FirstPoint.AbsoluteLocation);
            placeOffsetPosition.Translation = place_point.Translation;
            placeOffsetPosition.RotationRPY_XYZ = new TxVector(Math.PI, 0, rotZ);
            FirstPoint.AbsoluteLocation = placeOffsetPosition;

            // Second point: Place approach position (higher approach)
            SecondPoint.AbsoluteLocation = place_point;
            var placeApproachPosition = new TxTransformation(SecondPoint.AbsoluteLocation);
            placeApproachPosition.Translation = new TxVector(place_point_x, place_point_y, place_point_z + z_offset);
            placeApproachPosition.RotationRPY_XYZ = new TxVector(Math.PI, 0, rotZ);
            SecondPoint.AbsoluteLocation = placeApproachPosition;
           
        }
    }

    // Second elementary operation
    static void Overfly(
        string item_name, 
        TxContinuousRoboticOperation MyOp,
        double z_offset,
        double[] place_pose_recieved)
    {
        // Create the 2 necessary points   
        TxRoboticViaLocationOperationCreationData Point1 = new TxRoboticViaLocationOperationCreationData();
        Point1.Name = "point1_Overfly_" + item_name;
        TxRoboticViaLocationOperationCreationData Point2 = new TxRoboticViaLocationOperationCreationData();
        Point2.Name = "point2_Overfly_" + item_name;

        // Add the points to the operation
        TxRoboticViaLocationOperation FirstPoint = MyOp.CreateRoboticViaLocationOperation(Point1);
        TxRoboticViaLocationOperation SecondPoint = MyOp.CreateRoboticViaLocationOperation(Point2);

        // Define the pick position
        TxObjectList ref_frame_obj_pick = TxApplication.ActiveSelection.GetItems();
        ref_frame_obj_pick = TxApplication.ActiveDocument.GetObjectsByName("fr_" + item_name);
        TxFrame frame_obj_pick = ref_frame_obj_pick[0] as TxFrame;
        var position_pick = new TxTransformation(frame_obj_pick.AbsoluteLocation);

        // Get the translation vector from the transformation
        TxVector translation = position_pick.LocationRelativeToWorkingFrame.Translation;
        var pick_pose_x = translation.X;
        var pick_pose_y = translation.Y;
        var pick_pose_z = translation.Z + z_offset;
        var pick_pose = new TxTransformation(new TxVector(pick_pose_x, pick_pose_y, pick_pose_z), TxTransformation.TxTransformationType.Translate);

        // First point: Pick position
        TxTransformation full_pose = new TxTransformation();
        full_pose.Translation = pick_pose.Translation;
        full_pose.RotationRPY_XYZ = position_pick.RotationRPY_XYZ;

        // Assign it back to the point
        FirstPoint.LocationRelativeToWorkingFrame = full_pose;

        // Define the place position
        var place_point_x = place_pose_recieved[0]; 
        var place_point_y = place_pose_recieved[1]; 
        var place_point_z = place_pose_recieved[2];
        var rotation = place_pose_recieved[3];
        var place_point = new TxTransformation(new TxVector(place_point_x, place_point_y, place_point_z), TxTransformation.TxTransformationType.Translate);

        // First point: Place position
        double rotZ = 0; // Nominal value
        if (rotation == 1)
        {
            rotZ = Math.PI / 2; // Rotate 90 degrees
        }

        // Second point: second overfly position
        SecondPoint.AbsoluteLocation = place_point;
        var SecondOverflyPosition = new TxTransformation(SecondPoint.AbsoluteLocation);
        SecondOverflyPosition.Translation = new TxVector(place_point_x, place_point_y, place_point_z + z_offset);
        SecondOverflyPosition.RotationRPY_XYZ = new TxVector(Math.PI, 0, rotZ);
        SecondPoint.AbsoluteLocation = SecondOverflyPosition;
    }

    // Configure the initial position of the first point
    private static void ConfigureInitialPosition(TxRoboticViaLocationOperation firstPoint, string tcp_ee_name)
    {
        TxFrame TCPpose1 = TxApplication.ActiveDocument.GetObjectsByName(tcp_ee_name)[0] as TxFrame;
        var TCP_pose1 = new TxTransformation(TCPpose1.LocationRelativeToWorkingFrame);
        firstPoint.LocationRelativeToWorkingFrame = TCP_pose1;
    }

    // Specify the parameters of the waypoint
    private static void SetWaypointValues(
        string point_name, 
        ITxOlpRobotControllerParametersHandler paramHandler, 
        string tcp
        )
    {
        TxRoboticViaLocationOperation Point = TxApplication.ActiveDocument.
        GetObjectsByName(point_name)[0] as TxRoboticViaLocationOperation;

        paramHandler.OnComplexValueChanged("Tool", tcp, Point);
        paramHandler.OnComplexValueChanged("Motion Type", new_motion_type, Point);
        paramHandler.OnComplexValueChanged("Speed", new_speed, Point);
        paramHandler.OnComplexValueChanged("Acc", new_accel, Point);
        paramHandler.OnComplexValueChanged("Blend", new_blend, Point);

    }

    // Activate suction cup
    public static void SuctionOn(
        string point_name,
        ITxObject tool,
        ITxObject considered_item
    )
    {
        TxRoboticViaLocationOperation Waypoint = TxApplication.ActiveDocument.
        GetObjectsByName(point_name)[0] as TxRoboticViaLocationOperation;

        // Create the OLP command for attachment
        ArrayList elements1 = new ArrayList();
        ArrayList elements2 = new ArrayList();
    
        var myCmd1 = new TxRoboticCompositeCommandStringElement("# Attach ");	
        var myCmd11 = new TxRoboticCompositeCommandTxObjectElement(considered_item);
        var myCmd111 = new TxRoboticCompositeCommandTxObjectElement(tool);

        // Append all the command	
        elements1.Add(myCmd1);  
        elements1.Add(myCmd11);  
        elements1.Add(myCmd111); 

        // Write the command 	
        TxRoboticCompositeCommandCreationData txRoboticCompositeCommandCreationData1 =
        new TxRoboticCompositeCommandCreationData(elements1);	
        Waypoint.CreateCompositeCommand(txRoboticCompositeCommandCreationData1);
    }

    // Deactivate suction cup
    public static void SuctionOff(
        string point_name,
        ITxObject considered_item
    )
    {
        TxRoboticViaLocationOperation Waypoint = TxApplication.ActiveDocument.
        GetObjectsByName(point_name)[0] as TxRoboticViaLocationOperation;

        // Create the OLP command for detachment
        ArrayList elements1 = new ArrayList();
        ArrayList elements2 = new ArrayList();
    
        var myCmd1 = new TxRoboticCompositeCommandStringElement("# Detach ");	
        var myCmd11 = new TxRoboticCompositeCommandTxObjectElement(considered_item);

        // Append all the command	
        elements1.Add(myCmd1);  
        elements1.Add(myCmd11);  

        // Write the command 	
        TxRoboticCompositeCommandCreationData txRoboticCompositeCommandCreationData1 =
        new TxRoboticCompositeCommandCreationData(elements1);	
        Waypoint.CreateCompositeCommand(txRoboticCompositeCommandCreationData1);
    }

    
}
