// Copyright 2019 Siemens Industry Software Ltd.
using System;
using System.IO;
using System.Windows.Forms;
using Tecnomatix.Engineering;
using Tecnomatix.Engineering.Olp;

public class MainScript
{

    static StringWriter m_output;
    static string human_name = "Jack";
    static string test_name = "test3_ERP1_complete"; 
    static double dist = 0.0;
    static double dist_pick = 0.0;
    static double dist_place = 0.0;
    static int item_index = 0;
    static bool save_in_file = true;

    // Information on the pick-side objects
    static string[] item_names = new string[] { "Cube_01", "Cube_00", "Cube_02", "Cube_12", "Cube_11", "Cube_10" }; // ERP2 complete
    // static string[] item_names = new string[] { "Cube_01", "Cube_00", "Cube_02", "Cube_12", "Cube_11", "Cube_10" }; // ERP2 complete
    //static string[] item_names = new string[] { "Cube_02", "Cube_01", "Cube_00", "Cube_12", "Cube_11", "Cube_10" }; // ERP2 time
    
    // Static positions (constant for all the tests)
    static double[] place_poses_x = new double[] { -850.0, -765, -680, -340, -235, -340 };
    static double[] place_poses_y = new double[] { -447.5, -447.5, -447.5, -485.0, -485.0, -405.0 };

    // Durations

    static double[] pp_durations = new double[] { 13.41, 13.16, 8.53, 8.56, 8.79, 8.53 }; // ERP1 complete
    static double[] tvp_durations = new double[] { 0.83, 1.52, 2.21, 0.43, 1.63, 2.44 }; // ERP1 complete
    static double[] total_times = new double[] { 14.24, 28.92, 39.66, 48.65, 59.07, 70,04 }; // ERP1 complete

    /*
    static double[] pp_durations = new double[] { 13.4, 10.48, 8.49, 8.56, 8.79, 8.55 }; // ERP2 complete
    static double[] tvp_durations = new double[] { 0.96, 1.48, 2.23, 0.2, 1.65, 2.46 }; // ERP2 complete
    static double[] total_times = new double[] { 14.36, 26.32, 37.04, 45.8, 56.24, 67.25 }; // ERP2 complete
    */

    /*
    static double[] pp_durations = new double[] { 8.53, 10.55, 10.50, 8.56, 8.58, 8.54 }; // ERP2 time
    static double[] tvp_durations = new double[] { 1.31, 1.94, 0.63, 2.13, 1.63, 0.79 }; // ERP2 time
    static double[] total_times = new double[] { 9.84, 22.33, 33.46, 44.15, 54.36, 63.69 }; // ERP2 time
    */


    public static void MainWithOutput(ref StringWriter output)
    {

        // Access the simulation player
        TxSimulationPlayer player = TxApplication.ActiveDocument.SimulationPlayer;
        player.Rewind();

        // Run instant by instant
        if (!player.IsSimulationRunning())
        {
            m_output = output;
            player.TimeIntervalReached += new TxSimulationPlayer_TimeIntervalReachedEventHandler(player_TimeIntervalReached);       
            player.Play();
            player.TimeIntervalReached -= new TxSimulationPlayer_TimeIntervalReachedEventHandler(player_TimeIntervalReached);
           
        }

        // Just for safety
        player.Rewind();

        // Save output to file (if needed)
        if(save_in_file)
        {
        
			string fileName = "distance_" + test_name;
			string path = "C:/Users/chris/OneDrive - Politecnico di Milano/Politecnico di Milano/PhD - dottorato/GitHub repositories Lenovo/mics_project_predeployment/robotized_linear_packing/data/" + fileName + ".txt"; // You can customize the path here

        	//string path = "C:/Users/chris/OneDrive - Politecnico di Milano/Politecnico di Milano/PhD - dottorato/Works and suggestions by Marco/Final project Camozzi/Joint positions/robot_output.txt"; // You can customize the path here
        	File.WriteAllText(path, output.ToString());
        	MessageBox.Show("Output saved to: " + Path.GetFullPath(path));
        }
        
                 
    }

    private static void player_TimeIntervalReached(object sender, TxSimulationPlayer_TimeIntervalReachedEventArgs args)
    {
        // get the human
        TxHuman hum = TxApplication.ActiveDocument.GetObjectsByName(human_name)[0] as TxHuman;
        TxTransformation HumMat = hum.LocationRelativeToWorkingFrame;
        double hum_x = HumMat[0, 3];
		double hum_y = HumMat[1, 3];
		double hum_z = HumMat[2, 3];

        if (args.CurrentTime >= 0.0 && args.CurrentTime < total_times[0])
        {
            // Get the item to be picked
            item_index = 0;
            ITxLocatableObject Item = TxApplication.ActiveDocument.GetObjectsByName(item_names[item_index])[0] as ITxLocatableObject;
            TxTransformation ItemMat = Item.LocationRelativeToWorkingFrame;
            double item_x = ItemMat[0, 3];
            double item_y = ItemMat[1, 3];
            dist_pick = Math.Sqrt(Math.Pow(hum_x - item_x, 2) + Math.Pow(hum_y - item_y, 2));
            dist_place = Math.Sqrt(Math.Pow(hum_x - place_poses_x[item_index], 2) + Math.Pow(hum_y - place_poses_y[item_index], 2));
            dist = Math.Min(dist_pick, dist_place);
        }
        else if (args.CurrentTime >= total_times[0] && args.CurrentTime < total_times[1])
        {
            // Get the item to be picked
            item_index = 1;
            ITxLocatableObject Item = TxApplication.ActiveDocument.GetObjectsByName(item_names[item_index])[0] as ITxLocatableObject;
            TxTransformation ItemMat = Item.LocationRelativeToWorkingFrame;
            double item_x = ItemMat[0, 3];
            double item_y = ItemMat[1, 3];
            dist_pick = Math.Sqrt(Math.Pow(hum_x - item_x, 2) + Math.Pow(hum_y - item_y, 2));
            dist_place = Math.Sqrt(Math.Pow(hum_x - place_poses_x[item_index], 2) + Math.Pow(hum_y - place_poses_y[item_index], 2));
            dist = Math.Min(dist_pick, dist_place);
        }
        else if (args.CurrentTime >= total_times[1] && args.CurrentTime < total_times[2])
        {
            // Get the item to be picked
            item_index = 2;
            ITxLocatableObject Item = TxApplication.ActiveDocument.GetObjectsByName(item_names[item_index])[0] as ITxLocatableObject;
            TxTransformation ItemMat = Item.LocationRelativeToWorkingFrame;
            double item_x = ItemMat[0, 3];
            double item_y = ItemMat[1, 3];
            dist_pick = Math.Sqrt(Math.Pow(hum_x - item_x, 2) + Math.Pow(hum_y - item_y, 2));
            dist_place = Math.Sqrt(Math.Pow(hum_x - place_poses_x[item_index], 2) + Math.Pow(hum_y - place_poses_y[item_index], 2));
            dist = Math.Min(dist_pick, dist_place);
        }
        else if (args.CurrentTime >= total_times[2] && args.CurrentTime < total_times[3])
        {
            // Get the item to be picked
            item_index = 3;
            ITxLocatableObject Item = TxApplication.ActiveDocument.GetObjectsByName(item_names[item_index])[0] as ITxLocatableObject;
            TxTransformation ItemMat = Item.LocationRelativeToWorkingFrame;
            double item_x = ItemMat[0, 3];
            double item_y = ItemMat[1, 3];
            dist_pick = Math.Sqrt(Math.Pow(hum_x - item_x, 2) + Math.Pow(hum_y - item_y, 2));
            dist_place = Math.Sqrt(Math.Pow(hum_x - place_poses_x[item_index], 2) + Math.Pow(hum_y - place_poses_y[item_index], 2));
            dist = Math.Min(dist_pick, dist_place);
        }
        else if (args.CurrentTime >= total_times[3] && args.CurrentTime < total_times[4])
        {
            // Get the item to be picked
            item_index = 4;
            ITxLocatableObject Item = TxApplication.ActiveDocument.GetObjectsByName(item_names[item_index])[0] as ITxLocatableObject;
            TxTransformation ItemMat = Item.LocationRelativeToWorkingFrame;
            double item_x = ItemMat[0, 3];
            double item_y = ItemMat[1, 3];
            dist_pick = Math.Sqrt(Math.Pow(hum_x - item_x, 2) + Math.Pow(hum_y - item_y, 2));
            dist_place = Math.Sqrt(Math.Pow(hum_x - place_poses_x[item_index], 2) + Math.Pow(hum_y - place_poses_y[item_index], 2));
            dist = Math.Min(dist_pick, dist_place);
        }
        else if (args.CurrentTime >= total_times[4] && args.CurrentTime < total_times[5])
        {
            // Get the item to be picked
            item_index = 5;
            ITxLocatableObject Item = TxApplication.ActiveDocument.GetObjectsByName(item_names[item_index])[0] as ITxLocatableObject;
            TxTransformation ItemMat = Item.LocationRelativeToWorkingFrame;
            double item_x = ItemMat[0, 3];
            double item_y = ItemMat[1, 3];
            dist_pick = Math.Sqrt(Math.Pow(hum_x - item_x, 2) + Math.Pow(hum_y - item_y, 2));
            dist_place = Math.Sqrt(Math.Pow(hum_x - place_poses_x[item_index], 2) + Math.Pow(hum_y - place_poses_y[item_index], 2));
            dist = Math.Min(dist_pick, dist_place);
        }
        else if (args.CurrentTime >= total_times[5])
        {
            // Get the item to be picked
            item_index = 6;
        }

        // Also check if the human was in the safe zone
        if(hum_x == 3000.0 && hum_y == 3000.0)
        {
            dist = -1.0; // Overwrite this to a value usefule for the plots
        }

        // Display the distance
        m_output.Write("The distance is: " + dist.ToString() + ", the item is: " + item_index.ToString() + m_output.NewLine);
    }
   
}