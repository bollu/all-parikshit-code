% NOTE - change this on re-plugging in serial device
% CLEARINT CODE: gyro.delete(); clear;
serial_dev = '/dev/tty.SLAB_USBtoUART';

gyro = gyro_remote(serial_dev, 115200)
rows = [];

figure;
hold on;

xlabel('time');
ylabel('gyro readings');

set(gca, 'ColorOrder', [1 0 0 ; 0 1 0; 0 0 1], 'NextPlot', 'replacechildren');

% open data file right now
data_filename = strcat(datestr(datetime('now')), '.data.csv');

num_rows = 0;
MAX_ROWS = 100;

while true
    frames  = gyro.read_frames();
    gyro_readings = frames(1:end, 1:3);
    
    dlmwrite(data_filename, gyro_readings, '-append');
    
    rows = [rows; gyro_readings];
    
    if (num_rows == MAX_ROWS)
        rows = rows(end-(MAX_ROWS - 1):end, :);
    else
        num_rows = num_rows + 1;
    end

    plot(rows);
    datetick('x', 'HH:MM:SS PM');

    legend('gx', 'gy', 'gz', 'Location','northwest');
    drawnow;
    
 
end

