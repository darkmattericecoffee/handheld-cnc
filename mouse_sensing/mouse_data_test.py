#!/usr/bin/python
import sys
import usb.core
import usb.util
import numpy as np
import matplotlib.pyplot as plt

# decimal vendor and product values
#dev = usb.core.find(idVendor=1133, idProduct=49271)
#dev = usb.core.find(idVendor=9583, idProduct=50772)         # nice mouse
dev = usb.core.find(idVendor=1578, idProduct=22808)             # mid mouse
# or, uncomment the next line to search instead by the hexidecimal equivalent
#dev = usb.core.find(idVendor=0x45e, idProduct=0x77d)
# first endpoint
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
#print(data_array)
i = 0
while collected < attempts :
    try:
        data = dev.read(endpoint.bEndpointAddress,endpoint.wMaxPacketSize)
        collected += 1
        #print(data)
        print(data[2:6])
        #data_array[i] = np.array([data[1],data[2]])
        x_vel.append(data[2])
        x_dir.append(data[3])
        y_vel.append(data[4])
        y_dir.append(data[5])
        #y.append(data[2])
        #i = i+1
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
fig1.add_subplot(1, 1, 1)

fig1.axes[0].plot(x_vel, label=f'X velocity')
fig1.axes[0].plot(y_vel, label=f'Y velocity')
# fig1.axes[0].set_title('Rate Gyroscope Plotting')
# fig1.axes[0].set_xlabel('Time (s)')
# fig1.axes[0].set_ylabel('Rate Gyro (rad/s)')
# fig1.axes[0].legend(loc = 'upper left')

plt.show()
