/*
Test code to verify the data exchange with a socket-based architecture comprising nested loops.
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
using System.Collections.Generic;
using System.Linq;

class Program
{
    static public void Main(ref StringWriter output)
    {
        TcpListener server = null;
        try
        {
            // Define the number of simulations
            string address = "127.0.0.1";
            int port = 12345;

            // Initialize some variables
            int dim1 = 10;
            int dim2 = 20;
            double[] vec1 = new double[dim1];
            double[] vec2 = new double[dim2];

            // Start listening for incoming connections
            server = new TcpListener(IPAddress.Parse(address), port);
            server.Start();
            output.Write("Server started...");

            // Accept a client connection
            TcpClient client = server.AcceptTcpClient();
            NetworkStream stream = client.GetStream();

            // Receive the shared data
            var shared_data = ReceiveNumpyArray(stream);
            int Nsim = shared_data[0, 0];
            int trigger_end = shared_data[0, 1];
            int nested_idx = shared_data[0, 2];
            int loop_idx = shared_data[0, 3];
            output.Write("Shared data:");
            PrintArray(shared_data, output);

            // Loop for all the simulations
            for (int jj = trigger_end; jj < Nsim - 1; jj++)
            {

                // Receive sequence array
                var sequence = ReceiveNumpyArray(stream);
                output.Write("Sequence:");
                PrintArray(sequence, output);

                // Receive shared array
                var tasks = ReceiveNumpyArray(stream);
                output.Write("Tasks:");
                PrintArray(tasks, output);

                // Receive starting_times array
                var starting_times = ReceiveNumpyArray(stream);
                output.Write("Starting Times:");
                PrintArray(starting_times, output);

                // re-initialize the index
                nested_idx = shared_data[0, 2];

                // Inner loop
                while(nested_idx <= loop_idx)
                {
                    // Receive the shared data
                    var test_vec = ReceiveNumpyArray(stream);
                    output.Write("Nested iteration number: " + nested_idx.ToString() + "; received Test vector: ");
                    PrintArray(test_vec, output);

                    // Update the index
                    nested_idx++;

                    // Send the index back to the client
                    string updated_idx = (nested_idx).ToString();
                    byte[] updated_idx_ready = Encoding.ASCII.GetBytes(updated_idx);
                    stream.Write(updated_idx_ready, 0, updated_idx_ready.Length);

                }

                // Separate the display information on the terminal
                output.Write("\n");

                // Fake execution of the code
                for (int ii = 0; ii < dim2; ii++)
                {
                    if (ii < dim1)
                    {
                        vec1[ii] = (ii * 100) + jj;
                    }
                    vec2[ii] = (ii * 200) + jj;
                }

                // Display the results
                output.Write("First result: ");
                for (int ii = 0; ii < dim1; ii++)
                {
                    output.Write(vec1[ii] + ", ");
                }

                output.Write("\nSecond result: ");
                for (int ii = 0; ii < dim2; ii++)
                {
                    output.Write(vec2[ii] + ", ");
                }

                // Send the first array back to the client
                string result1_vec = string.Join(",", vec1);
                byte[] result1 = Encoding.ASCII.GetBytes(result1_vec);
                stream.Write(result1, 0, result1.Length);

                // Send the second array back to the client
                string result2_vec = string.Join(",", vec2);
                byte[] result2 = Encoding.ASCII.GetBytes(result2_vec);
                stream.Write(result2, 0, result2.Length);

            }

            // Close all the instances
            stream.Close();
            client.Close();
            server.Stop();
        }
        catch (Exception e)
        {
            output.Write("Exception: " + e.Message);
        }
    }

    // Definition of custom functions   
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
}