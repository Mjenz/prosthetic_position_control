% use MATLAB to crop the csv files to the desired GROUP of arm swings
% this is made easier by MATLABS interactive plots capability, otherwise
% this would be in python
data_directory = 'real_data';
csv_filename = {'s02_elbow_07.csv','s02_elbow_09.csv','s02_elbow_11.csv','s02_elbow_13.csv'}; % Replace with your actual CSV file name
output_filename = 'src/data_profile_REAL_adjusted.c';

% Write the header of the .c file before parsing through the data files
fid = fopen(output_filename, 'w');
fprintf(fid, '#include "data_profile.h"\n\n');

% Define the limits of cropping
lower = [994,959,15,527];
upper = [1425,1404,543,1125];

file_number = 1;

% Loop through each file
for i = 1:length(csv_filename)
    % Read the data from the CSV file
    angles = [];
    cropped = [];
    full_csv_path = fullfile(data_directory, csv_filename{i});
    csv_data = readmatrix(full_csv_path);
    
    if ~isempty(csv_data)
        angles = csv_data(:,1);  % Assuming the angle is in the first column
    end
    
    % Find local minimums
    TF = islocalmin(angles);

    % Create time var
    t = (1:1:length(angles))';
    

    % Plot
    % figure(i);
    % plot(t,angles,t(TF),angles(TF),'*');
    % title(sprintf('Elbow Angle Trajectory at %.1f m/s', (7 + (i-1)*2) * 0.1));
    % ylabel('elbow angle (rad)');
    % xlabel('Time');

    % Write the data to the C file
    fprintf(fid, 'float Angle_Profile_REAL_adjusted_%d[%d] = \n{\n', 7 + (i-1)*2, upper(file_number)-lower(file_number));
    
    for j = 1:length(angles)
        if j >= lower(file_number) && j <= upper(file_number)
            fprintf(fid, '    %.5f,\n', angles(j));
            cropped = [cropped, angles(j)];
        end
    end
    fprintf(fid, '};\n');

    % Plot after cropping
    figure(i);
    plot(cropped,'-',LineWidth=2);
    title(sprintf('Elbow Angle Trajectory at %.1f m/s', (7 + (i-1)*2) * 0.1));
    ylabel('elbow angle (rad)');
    xlabel('Time');
    
    file_number = file_number + 1;
end

fclose(fid);