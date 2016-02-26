#!/usr/bin/env python
import serial
import sys
import struct
import time
import math
import matplotlib.pyplot as plt
import datetime

from bokeh.plotting import figure, output_server, cursession, show


def unpack_int16(data):
    return struct.unpack('<h', data)[0]

def unpack_data_from_desc(desc, raw_data):
    unpacked = {}

    current_data = raw_data
    for (name, dtype) in desc:
        if dtype == 'int16':
            unpacked[name] = unpack_int16(current_data[0:2])
            current_data = current_data[2:]
        else:
            raise Exception("unknown data type for value: %s | dtype: %s" % 
                            (str(name), str(dtype)))

    return unpacked


mpu_data_desc = [('gx', 'int16'),
             ('gy', 'int16'),
             ('gz', 'int16'),
             ('ax', 'int16'),
             ('ay', 'int16'),
             ('az', 'int16'),
             ('temp', 'int16')]


def create_true_data(unpacked_data):
    true_data = {}
    true_data['gx'] = 250.0 * float(unpacked_data['gx']) / math.pow(2, 15)
    true_data['gy'] = 250.0 * float(unpacked_data['gy']) / math.pow(2, 15)
    true_data['gz'] = 250.0 * float(unpacked_data['gz']) / math.pow(2, 15)

    true_data['ax'] = 2.0 * float(unpacked_data['ax']) / math.pow(2, 15)
    true_data['ay'] = 2.0 * float(unpacked_data['ay']) / math.pow(2, 15)
    true_data['az'] = 2.0 * float(unpacked_data['az']) / math.pow(2, 15)

    true_data['temp'] = (float(unpacked_data['temp']) / 340.0) + 36.53

    return true_data

def read_data_from_mpu6050():
    dev.write(b'r');
    time.sleep(1);
    to_read = dev.inWaiting()
    raw_data = dev.read(to_read)

    # print("raw: %s" % raw_data)
    unpacked_data = unpack_data_from_desc(mpu_data_desc, raw_data)
    true_data = create_true_data(unpacked_data)
    return true_data

if __name__ == "__main__":
    device_path = sys.argv[1]
    dev = serial.Serial(device_path, 9600, bytesize=8, parity='N', stopbits=1)

    data_frames = []
    timestamps = []


    fig = plt.figure()
    ax = fig.add_subplot(111)

    gx_plot, = ax.plot([], [])
    gy_plot, = ax.plot([], [])
    gz_plot, = ax.plot([], [])
    plt.show(block=False)

    while True:
        try:
            true_data = read_data_from_mpu6050()
            data_frames.append(true_data)
            timestamps.append(datetime.datetime.now())
           
            gx_plot.set_xdata(timestamps)
            gy_plot.set_xdata(timestamps)
            gz_plot.set_xdata(timestamps)

            gx_plot.set_ydata(true_data['gx'])
            gy_plot.set_ydata(true_data['gy'])
            gz_plot.set_ydata(true_data['gz'])
            
            
            ax.relim()
            ax.autoscale_view(True, True, True)

            plt.gcf().autofmt_xdate()
            
            
            fig.canvas.draw()
            print("data:\n%s" % true_data)

        except KeyboardInterrupt:
            break
