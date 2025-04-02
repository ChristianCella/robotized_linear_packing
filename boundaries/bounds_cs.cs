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

class Program
{
    // Private variables
    static TxSimulationPlayer Player;

    // Static variables
    static StringWriter m_output;
    static double determinantSum = 0;
    static double determinantCounter = 0;
    static int fitness_int = 0;
    static int port = 12345;
    static TxRobot Robot = GetRobot("GoFa12");
    static ITxObject tool = GetGripper("Suction cup");
    static ITxLocatableObject robot = Robot as ITxLocatableObject;
    static string new_tcp = "tgripper_tf";
    static string new_motion_type = "PTP";
    static string new_speed = "100%";
    static string new_accel = "100%";
    static string new_blend = "fine";
    static string operation_root = "RoboticProgram_";
    static string item_root = "Cube_";
    static int[] result = new int[2];
    static double[] place_pose = new double[4];

    // Main funtion
    static public void Main(ref StringWriter output)
    {
        m_output = output;
        TcpListener server = null;
        Player = TxApplication.ActiveDocument.SimulationPlayer;
        int N = 1000;
        
        try
        {
            int port = 12345;
            server = new TcpListener(IPAddress.Parse("127.0.0.1"), port);
            server.Start();
            TcpClient client = server.AcceptTcpClient();
            NetworkStream stream = client.GetStream();

            // Instantiate the item to be picked (never changes)
            ITxObject considered_item = GetItem("Cube_02");

            for (int i = 0; i < N; i++)
            {
                // Get the base and place poses
                var layout = ReceiveNumpyArray(stream);

                //move the base of the robot in the defined position 
                TxVector translation = new TxVector(layout[0, 0], 0, 0);
                TxVector orientation = new TxVector(0, 0, 0);
                TransformPose(robot, translation, orientation);
                
                // Re-initialize variables for the mean manipulability
                determinantCounter = 0;
                determinantSum = 0; 

                // Create the place pose
                place_pose[0] = layout[0, 1]; // x
                place_pose[1] = layout[0, 2]; // y
                place_pose[2] = layout[0, 3]; // z
                place_pose[3] = layout[0, 4]; // rotation
                              
                // Create the robotc operation
                TxContinuousRoboticOperation MyOp = RobotPickPlace(
                    Robot, 
                    considered_item, 
                    tool, 
                    "RobotProgram", 
                    "Cube_02", 
                    "TOOLFRAME", 
                    new_tcp, 
                    100, // pre-post offset
                    place_pose); 

                // Set the new operation to be simulated 
                TxApplication.ActiveDocument.CurrentOperation = MyOp;                                       
                Player.Rewind();
            
                // ! Core of the algorithm: Run the simulation
                if (!Player.IsSimulationRunning())
                {
                    m_output = output;        
                    Player.TimeIntervalReached += new TxSimulationPlayer_TimeIntervalReachedEventHandler(player_TimeIntervalReached);                      
                    Player.Play(); // Perform the simulation at the current time step 
                    Player.TimeIntervalReached -= new TxSimulationPlayer_TimeIntervalReachedEventHandler(player_TimeIntervalReached);
                }
            
                // Check if the simulation was successful
                int simulation_success = CheckSimulationSuccess();

                // 'Safety' rewind
                Player.Rewind();

                // ! Compute the metrics for this simulation (manipulability, operation time, xi)                                      
                double mean_determinant = Math.Round(determinantSum / determinantCounter, 5) * 100000;
                int single_fitness = (int)mean_determinant;
                result[0] = simulation_success;
                result[1] = single_fitness;
                                                       
                // Send the first array back to the client
                string result_vec = string.Join(",", result);
                byte[] result1 = Encoding.ASCII.GetBytes(result_vec);
                stream.Write(result1, 0, result1.Length);

                output.WriteLine("Simulation number: " + i);
                output.WriteLine("Robot base position: " + layout[0, 0]);
                output.WriteLine("Simulation success: " + simulation_success);
                output.WriteLine("Manipulability: " + single_fitness);

                // Delete the operation
                MyOp.Delete();

            }

            //close connection
            stream.Close();
            client.Close();
            server.Stop();
        }
        catch (Exception e)
        {
            output.Write("Exception: " + e.Message);
        }
    }

    /*
        Methods to transform the resources
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

    // get the gripper
    private static ITxObject GetGripper(string gripperName)
    {
        ITxObject Gripper = TxApplication.ActiveDocument.GetObjectsByName(gripperName)[0] as ITxObject;
        return Gripper;
    }

    // get the object
    private static ITxObject GetItem(string item_name)
    {
        ITxObject Item = TxApplication.ActiveDocument.GetObjectsByName(item_name)[0];
        return Item;
    }

    // Object transformation
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

    /*
        Methods to create the robotic program
    */

    // Initialize the operation
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
        string tcp_flange_name, // "TOOLFRAME"
        string tcp_ee_name, // new_tcp = "tgripper_tf"
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
        paramHandler.OnComplexValueChanged("Accel", new_accel, Point);
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

    /*
        Methods to receive data from Python
    */

    static int[,] ReceiveNumpyArray(NetworkStream stream)
    {
        // Receive the shape of the array
        byte[] shapeBuffer = new byte[8]; // Assuming the shape is of two int32 values
        stream.Read(shapeBuffer, 0, shapeBuffer.Length);
        int rows = BitConverter.ToInt32(shapeBuffer, 0);
        int cols = BitConverter.ToInt32(shapeBuffer, 4);

        // Receive the array data
        int arraySize = rows * cols * sizeof(int); // Assuming int32 values
        byte[] arrayBuffer = new byte[arraySize];
        stream.Read(arrayBuffer, 0, arrayBuffer.Length);

        // Convert byte array to int array
        int[,] array = new int[rows, cols];
        Buffer.BlockCopy(arrayBuffer, 0, array, 0, arrayBuffer.Length);

        return array;
    }

    static void PrintArray(int[,] array, StringWriter m_output)
    {
        int rows = array.GetLength(0);
        int cols = array.GetLength(1);
        for (int i = 0; i < rows; i++)
        {
            for (int j = 0; j < cols; j++)
            {
                m_output.Write(array[i, j] + " ");
            }
            m_output.Write("\n");
        }
    }

    /*
        Methods to calculate the Jacobian matrix and its determinant
    */

    static double[] CrossProduct(double[] a, double[] b)
    {
        return new double[] {
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]
        };
    }

    static double[] Subtract(double[] a, double[] b)
    {
        return new double[] {
            a[0] - b[0],
            a[1] - b[1],
            a[2] - b[2]
        };
    }

    static double[,] MultiplyMatrixByTranspose(double[,] matrix)
    {
        int rows = matrix.GetLength(0);
        int cols = matrix.GetLength(1);
        double[,] result = new double[rows, rows];

        for (int i = 0; i < rows; i++)
            for (int j = 0; j < rows; j++)
            {
                double sum = 0.0;
                for (int k = 0; k < cols; k++)
                    sum += matrix[i, k] * matrix[j, k];
                result[i, j] = sum;
            }

        return result;
    }

    // Recursive determinant calculator (Laplace expansion)
    static double CalculateDeterminant(double[,] matrix)
    {
        int size = matrix.GetLength(0);
        if (size != matrix.GetLength(1))
            throw new ArgumentException("Matrix must be square.");

        if (size == 1)
            return matrix[0, 0];

        double result = 0.0;

        for (int j = 0; j < size; j++)
        {
            double[,] minor = new double[size - 1, size - 1];

            for (int k = 1; k < size; k++)
            {
                for (int l = 0, m = 0; l < size; l++)
                {
                    if (l != j)
                        minor[k - 1, m++] = matrix[k, l];
                }
            }

            result += (j % 2 == 0 ? 1 : -1) * matrix[0, j] * CalculateDeterminant(minor);
        }

        return result;
    }

    // Define a method to display the value of the determinant during the simulation
    private static void player_TimeIntervalReached(object sender, TxSimulationPlayer_TimeIntervalReachedEventArgs args)
    {
        // Get transformation frames
        TxFrame DH0 = TxApplication.ActiveDocument.GetObjectsByName("BASEFRAME")[0] as TxFrame;
        var Frame0 = new TxTransformation(DH0.LocationRelativeToWorkingFrame);
        
        TxFrame DH1 = TxApplication.ActiveDocument.GetObjectsByName("fr1")[0] as TxFrame;
        var Frame1 = new TxTransformation(DH1.LocationRelativeToWorkingFrame);
        
        TxFrame DH2 = TxApplication.ActiveDocument.GetObjectsByName("fr2")[0] as TxFrame;
        var Frame2 = new TxTransformation(DH2.LocationRelativeToWorkingFrame);
        
        TxFrame DH3 = TxApplication.ActiveDocument.GetObjectsByName("fr3")[0] as TxFrame;
        var Frame3 = new TxTransformation(DH3.LocationRelativeToWorkingFrame);
        
        TxFrame DH4 = TxApplication.ActiveDocument.GetObjectsByName("fr4")[0] as TxFrame;
        var Frame4 = new TxTransformation(DH4.LocationRelativeToWorkingFrame);
        
        TxFrame DH5 = TxApplication.ActiveDocument.GetObjectsByName("fr5")[0] as TxFrame;
        var Frame5 = new TxTransformation(DH5.LocationRelativeToWorkingFrame);
        
        TxFrame DH6 = TxApplication.ActiveDocument.GetObjectsByName("TOOLFRAME")[0] as TxFrame;
        var Frame6 = new TxTransformation(DH6.LocationRelativeToWorkingFrame);
        
        // Get joint positions in meters
        var x1 = Frame1[0, 3] / 1000; var y1 = Frame1[1, 3] / 1000; var z1 = Frame1[2, 3] / 1000;
        var x2 = Frame2[0, 3] / 1000; var y2 = Frame2[1, 3] / 1000; var z2 = Frame2[2, 3] / 1000;
        var x3 = Frame3[0, 3] / 1000; var y3 = Frame3[1, 3] / 1000; var z3 = Frame3[2, 3] / 1000;
        var x4 = Frame4[0, 3] / 1000; var y4 = Frame4[1, 3] / 1000; var z4 = Frame4[2, 3] / 1000;
        var x5 = Frame5[0, 3] / 1000; var y5 = Frame5[1, 3] / 1000; var z5 = Frame5[2, 3] / 1000;
        var x6 = Frame6[0, 3] / 1000; var y6 = Frame6[1, 3] / 1000; var z6 = Frame6[2, 3] / 1000;
        
        // Define Z axes (rotation vectors)
        double[] Z0 = { Frame0[0, 2], Frame0[1, 2], Frame0[2, 2] };
        double[] Z1 = { Frame1[0, 2], Frame1[1, 2], Frame1[2, 2] };
        double[] Z2 = { Frame2[0, 2], Frame2[1, 2], Frame2[2, 2] };
        double[] Z3 = { Frame3[0, 2], Frame3[1, 2], Frame3[2, 2] };
        double[] Z4 = { Frame4[0, 2], Frame4[1, 2], Frame4[2, 2] };
        double[] Z5 = { Frame5[0, 2], Frame5[1, 2], Frame5[2, 2] };

        // Position vectors
        double[] p0 = { 0.0, 0.0, 0.0 };
        double[] p1 = { x1, y1, z1 };
        double[] p2 = { x2, y2, z2 };
        double[] p3 = { x3, y3, z3 };
        double[] p4 = { x4, y4, z4 };
        double[] p5 = { x5, y5, z5 };
        double[] p6 = { x6, y6, z6 }; // End-effector position
        double[] p = p6;

        // Compute linear part of Jacobian
        double[] result0 = CrossProduct(Z0, Subtract(p, p0));
        double[] result1 = CrossProduct(Z1, Subtract(p, p1));
        double[] result2 = CrossProduct(Z2, Subtract(p, p2));
        double[] result3 = CrossProduct(Z3, Subtract(p, p3));
        double[] result4 = CrossProduct(Z4, Subtract(p, p4));
        double[] result5 = CrossProduct(Z5, Subtract(p, p5));

        // Build Jacobian matrix J (6x6)
        double[,] J = {
            { result0[0], result1[0], result2[0], result3[0], result4[0], result5[0] },
            { result0[1], result1[1], result2[1], result3[1], result4[1], result5[1] },
            { result0[2], result1[2], result2[2], result3[2], result4[2], result5[2] },
            { Z0[0], Z1[0], Z2[0], Z3[0], Z4[0], Z5[0] },
            { Z0[1], Z1[1], Z2[1], Z3[1], Z4[1], Z5[1] },
            { Z0[2], Z1[2], Z2[2], Z3[2], Z4[2], Z5[2] }
        };

        // Compute standard determinant
        double detJ = CalculateDeterminant(J);

        // Compute Yoshikawa manipulability index: w = sqrt(det(J*J?)) ==> We use this
        double[,] JJt = MultiplyMatrixByTranspose(J);
        double detJJt = CalculateDeterminant(JJt);
        double w = Math.Sqrt(Math.Abs(detJJt));

        determinantSum = determinantSum + w;
        determinantCounter = determinantCounter + 1;

    }

    /*
        Methods to check if the simulation has given errors or not
    */
    static int CheckSimulationSuccess()
    {
        // Get errors and traces associated to the current simulation
        List<string> errors = TxApplication.ActiveDocument.SimulationPlayer.GetErrorsAndTraces(TxSimulationErrorType.Error);

        // Check if there are any errors and give a return value
        if (errors.Count > 0)
        {
            return 1; // Simulation failed
        }

        else // No error
        {
            return 0; // Simulation succeeded
        }
    }
}