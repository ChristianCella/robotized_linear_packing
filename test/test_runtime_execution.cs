/*
This snippet allows, at every temporal discretization, to:
    - check if some collisions happened
    - compute the manipulability index of the robot
*/

using System;
using System.IO;
using System.Windows.Forms;
using Tecnomatix.Engineering;

public class MainScript
{

    // Class-specific static variables   
    static StringWriter m_output;
    static int flag = 0;

    public static void Main(ref StringWriter output)
    {
    
        // Save the complete operation (human + robot) in a variable
        TxObjectList Operation = TxApplication.ActiveDocument.GetObjectsByName("RobotProgram");
        var op = Operation[0] as ITxCompoundOperation;

        // Check the collisions
        TxApplication.ActiveDocument.CollisionRoot.CheckCollisions = true;

        // Access the simulation player
        TxSimulationPlayer player = TxApplication.ActiveDocument.SimulationPlayer;
        player.SetOperation(op); // Set the operation
        player.Rewind();
        
        if (!player.IsSimulationRunning())
        {
        	m_output = output; // Display the time output
        	
            player.TimeIntervalReached += new TxSimulationPlayer_TimeIntervalReachedEventHandler(player_TimeIntervalReached);  
            player.TimeIntervalReached += new TxSimulationPlayer_TimeIntervalReachedEventHandler(player_ComputeJacobian);                    
            player.Play(); // Perform the simulation at the current time step 
            player.TimeIntervalReached -= new TxSimulationPlayer_TimeIntervalReachedEventHandler(player_TimeIntervalReached);
            player.TimeIntervalReached -= new TxSimulationPlayer_TimeIntervalReachedEventHandler(player_ComputeJacobian);
        }
        
        // Rewind the simulation
        player.Rewind();

        // Check the flag
        if (flag == 0)
        {
        	m_output.Write("No collision happened!" + m_output.NewLine);
        }

        player.Rewind();
    }

    /* 
        Methods
    */
    private static void player_TimeIntervalReached(object sender, TxSimulationPlayer_TimeIntervalReachedEventArgs args)
    {

        // Set some parameters for the collision check
        TxCollisionQueryParams collisionQueryParams = new TxCollisionQueryParams
        {
            UseNearMiss = false,
            UseAllowedPenetration = false
        };

        // Create a new context
        TxCollisionQueryResults results =
        TxApplication.ActiveDocument.CollisionRoot.GetCollidingObjects(collisionQueryParams, new
        TxCollisionQueryContext());

        // Analyze all the states
        foreach (TxCollisionState state in results.States)
        {
            switch (state.Type)
            {
                case TxCollisionState.TxCollisionStateType.Collision:

                    // Display a message on the screen: the time instant at which a possible collision happens
                    if (flag < 1) // 'flag' has not been incremented yet
                    {

      					// display the time instant
                        m_output.Write("A collision happened at: " + args.CurrentTime.ToString() + " seconds" + m_output.NewLine);

                        // Increase teh counter to avoid displaying all the collisions
                        flag++;
                    }
                    

                    break;

            }
        }
    }
    
    // Utility: Cross product of 3D vectors
    static double[] CrossProduct(double[] a, double[] b)
    {
        return new double[] {
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]
        };
    }

    // Utility: Vector subtraction
    static double[] Subtract(double[] a, double[] b)
    {
        return new double[] {
            a[0] - b[0],
            a[1] - b[1],
            a[2] - b[2]
        };
    }

    // Utility: Multiply matrix by its transpose: JJ? = J * J?
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
    private static void player_ComputeJacobian(object sender, TxSimulationPlayer_TimeIntervalReachedEventArgs args)
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
        //output.WriteLine("Determinant of Jacobian: det(J) = " + detJ.ToString("G6"));

        // Compute Yoshikawa manipulability index: w = sqrt(det(J*J?))
        double[,] JJt = MultiplyMatrixByTranspose(J);
        double detJJt = CalculateDeterminant(JJt);
        double w = Math.Sqrt(Math.Abs(detJJt));

        //output.WriteLine("Yoshikawa Manipulability Index: w = sqrt(det(J*Jt))");
        m_output.WriteLine("w = " + w.ToString("G6"));
       
    }
    

}




