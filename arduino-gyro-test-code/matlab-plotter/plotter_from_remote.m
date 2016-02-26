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
  
while true
    frames  = gyro.read_frames();
    gyro_readings = frames(1:end, 1:3);
    rows = [rows; gyro_readings];
    plot(rows);
    datetick('x', 'HH:MM:SS PM');

    legend('gx', 'gy', 'gz', 'Location','northwest');
    drawnow;
end

