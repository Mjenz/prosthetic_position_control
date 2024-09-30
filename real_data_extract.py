import csv
import matplotlib.pyplot as plt
# Open the CSV file
data_directory = 'real_data'
csv_filename = ['s02_elbow_07.csv','s02_elbow_09.csv','s02_elbow_11.csv','s02_elbow_13.csv']  # Replace with your actual CSV file name
output_filename = 'src/data_profile_REAL.c'

# write the header of the .c file before parsing through the data files
with open(output_filename, 'w') as cfile:
    cfile.write('#include "data_profile.h"\n\n')

    # Loop through each file
    for i in range(len(csv_filename)):
        # Read the data from the CSV file
        angles = []
        with open(data_directory + '/' + csv_filename[i], 'r') as csvfile:
            csvreader = csv.reader(csvfile)
            for row in csvreader:
                if row:  # Ensure the row is not empty
                    angle = float(row[0])  # Assuming the angle is in the first column
                    angles.append(angle)

        # Plot
        plt.figure(i)
        plt.plot(angles)
        plt.title(f'Elbow Angle Trajectory at {(7+i*2)*0.1:.1f}m/s')
        plt.ylabel('elbow angle (rad)')
        plt.show()
        # Write the data to the C file
        cfile.write(f'float Angle_Profile_REAL_{7 + i*2}[{len(angles)}] = \n{{\n')

        for i, angle in enumerate(angles):
            if i != len(angles) - 1:
                cfile.write(f'    {angle:.5f},\n')
            else:
                cfile.write(f'    {angle:.5f}\n')
        cfile.write('};\n')