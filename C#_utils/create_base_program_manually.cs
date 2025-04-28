/*
This snippet allows to create a device operation. This was used to move the robot with the slider of the
camozzi Line.
*/

using System.IO;
using Tecnomatix.Engineering;
public class MainScript
{
    public static void MainWithOutput(ref StringWriter output)
    {

        // Define the vector
        string[] item_names = new string[] { "Cube_02", "Cube_01", "Cube_00", "Cube_12", "Cube_11", "Cube_10" };
        string pose_root = "BasePose";
        string op_name = "MoveBase";

        // Get the line  	
        TxObjectList objects = TxApplication.ActiveDocument.GetObjectsByName("Line");
        var line = objects[0] as TxDevice;

        // Initialize start and end pose
        var start_pose = TxApplication.ActiveDocument.GetObjectsByName("MIDDLE")[0] as TxPose;
        var end_pose = TxApplication.ActiveDocument.GetObjectsByName("MIDDLE")[0] as TxPose;

        // Create all the operations
        for (int i = 0; i < item_names.Length; i++)
        { 
            // Get the pose
            if (i == 0)
            {
                TxObjectList start_poses = TxApplication.ActiveDocument.GetObjectsByName("MIDDLE");
                start_pose = start_poses[0] as TxPose;
                TxObjectList end_poses = TxApplication.ActiveDocument.GetObjectsByName(pose_root + item_names[i]);
                end_pose = end_poses[0] as TxPose;
            }
            else
            {
                TxObjectList start_poses = TxApplication.ActiveDocument.GetObjectsByName(pose_root + item_names[i-1]);
                start_pose = start_poses[0] as TxPose;
                TxObjectList end_poses = TxApplication.ActiveDocument.GetObjectsByName(pose_root + item_names[i]);
                end_pose = end_poses[0] as TxPose;
            }

            // Get the device by name
            TxDeviceOperationCreationData data = new TxDeviceOperationCreationData();
            data.Duration = 0;
            data.Name = op_name + item_names[i];
            TxApplication.ActiveDocument.OperationRoot.CreateDeviceOperation(data);

            // Get the created operation
            TxTypeFilter opFilter = new TxTypeFilter(typeof(TxContinuousRoboticOperation));
            TxOperationRoot opRoot = TxApplication.ActiveDocument.OperationRoot;

            TxObjectList operations = TxApplication.ActiveDocument.GetObjectsByName(data.Name);
            var MyOp = operations[0] as TxDeviceOperation;

            // Create the operation
            MyOp.Device = line;
            MyOp.SourcePose = start_pose;
            MyOp.TargetPose = end_pose;
            
            
        }

    }
}