#!/usr/bin/python
import sys
import usb.core
import usb.util
import numpy as np
import matplotlib.pyplot as plt
import time

# decimal vendor and product values
# Micelist
#   0 -
#   1 - nice mouse (CadMouse)
#   2 - mid mouse (from Jacobs)
#   3 - free mouse (from Chris)
#   4 - free mouse (from Etch bin)
#   5 - Amazon basics mouse
mouse = 5
idsVendor = [1133, 9583, 1578, 6700, 1203, 12538]
idsProduct = [49271, 50772, 22808, 66, 12555, 768]
dev = usb.core.find(idVendor=idsVendor[mouse], idProduct=idsProduct[mouse])
# or, uncomment the next line to search instead by the hexidecimal equivalent
#dev = usb.core.find(idVendor=0x45e, idProduct=0x77d)
# first endpoint
print(usb.core.show_devices())
interface = 0
endpoint = dev[0][(0,0)][0]
# if the OS kernel already claimed the device, which is most likely true
# thanks to http://stackoverflow.com/questions/8218683/pyusb-cannot-set-configuration
if dev.is_kernel_driver_active(interface) is True:
  # tell the kernel to detach
  dev.detach_kernel_driver(interface)
  # claim the device
  usb.util.claim_interface(dev, interface)
collected = 0
attempts = 100
x_vel = []
x_dir = []
y_vel = []
y_dir = []
x_pos = [0]
y_pos = [0]
t = []
#print(data_array)
i = 0

# Data collection
while collected < attempts :
    try:
        data = dev.read(endpoint.bEndpointAddress,endpoint.wMaxPacketSize)
        collected += 1
        print(data)
        if len(t) == 0:
            t0 = time.time_ns() // 1000000

        if (mouse == 0):
            # nothing yet
            mouse = 0           # nonsense
        elif (mouse == 1):
            # print(data[2:6])
            # #data_array[i] = np.array([data[1],data[2]])
            x_vel.append(data[2])
            # x_dir.append(data[3])
            y_vel.append(data[4])
            # y_dir.append(data[5])
            #y.append(data[2])
            #i = i+1
        elif (mouse == 2):
            # nothing yet
            mouse = 2       # nonsense
        elif (mouse == 3):
            t.append(time.time_ns() // 1000000)
            x_vel.append(np.interp(data[1], [0,255],[-1,1]))
            y_vel.append(np.interp(data[2], [0,255],[-1,1]))
            x_pos.append(x_pos[-1] + x_vel[-1])
            y_pos.append(y_pos[-1] + y_vel[-1])
        elif (mouse == 4):
            t.append(time.time_ns() // 1000000)
            x_vel.append(np.interp(data[1], [0,255],[-1,1]))
            y_vel.append(np.interp(data[2], [0,255],[-1,1]))
            x_pos.append(x_pos[-1] + x_vel[-1])
            y_pos.append(y_pos[-1] + y_vel[-1])
        elif (mouse == 4):
            t.append(time.time_ns() // 1000000)

    except usb.core.USBError as e:
        data = None
        if e.args == ('Operation timed out',):
            continue
# release the device
usb.util.release_interface(dev, interface)
# reattach the device to the OS kernel
dev.attach_kernel_driver(interface)

# Plot
# Plot 1
fig1 = plt.figure()
fig1.add_subplot(3, 1, 1)

#print(x_vel)

# fig1.axes[0].plot(x_vel, label=f'X velocity')
# fig1.axes[0].plot(y_vel, label=f'Y velocity')
fig1.axes[0].plot(x_pos, y_pos, label=f'Mouse position')
# fig1.axes[0].set_title('Rate Gyroscope Plotting')
# fig1.axes[0].set_xlabel('Time (s)')
# fig1.axes[0].set_ylabel('Rate Gyro (rad/s)')
# fig1.axes[0].legend(loc = 'upper left')

fig1.add_subplot(3, 1, 2)
fig1.axes[1].plot(t, x_vel)

fig1.add_subplot(3, 1, 3)
fig1.axes[2].plot(t, y_vel)

plt.show()
