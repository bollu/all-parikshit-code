classdef gyro_remote
    %GYRO_REMOTE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        xbee
    end
    
    properties (Hidden)
        cleanup
    end

    methods
        function gyro = gyro_remote(device_path, baud_rate)
            gyro.cleanup = onCleanup(@()delete(gyro));
            gyro.xbee = serial(device_path);
            if (~exist('baud_rate', 'var'))
                baud_rate = 9600
            end
            
            gyro.xbee.BaudRate = baud_rate;
            fopen(gyro.xbee);
        end
        
        function frames = read_frames(self)
            NUM_FRAMES_PER_READ = 3;
            % ask for data
            fwrite(self.xbee, 'r', 'uchar');
            

            frames = [read_frame_as_mat(self)];
            for i = 1:(NUM_FRAMES_PER_READ - 1)
                frames = [frames; read_frame_as_mat(self)];
            end
 
        end
        
        function frame_mat =  read_frame_as_mat(self)
          [gx, gy, gz, ax, ay, az, temp] = self.read_frame();
          frame_mat = [gx, gy, gz, ax, ay, az, temp];
        end

        %READS
        function [gx, gy, gz, ax, ay, az, temp] = read_frame(self)
            raw_data = fread(self.xbee, 7, 'int16');
            raw_gx = raw_data(1);
            raw_gy = raw_data(2);
            raw_gz = raw_data(3);
            raw_ax = raw_data(4);
            raw_ay = raw_data(5);
            raw_az = raw_data(6);
            raw_temp = raw_data(7);
            
            gx = raw_gx * 250.0 / double(pow2(15));
            gy = raw_gy * 250.0 / double(pow2(15));
            gz = raw_gz * 250.0 / double(pow2(15));
            
            ax = raw_ax * 2.0 / double(pow2(15));
            ay = raw_ay * 2.0 / double(pow2(15));
            az = raw_az * 2.0 / double(pow2(15));
            
            temp = (raw_temp / 340.0) + 36.53;
        end
        
        function delete(self)
            fclose(self.xbee);
        end
    end
    
end

