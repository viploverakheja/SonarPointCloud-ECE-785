import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

fig = plt.figure()
ax = fig.add_subplot(111, projection = '3d')
fp = open('co_ords','r')

all_pts = fp.readlines()
xval1 = []
yval1 = []
zval1 = []

#csp = all_pts[0].split(',')
#xval1.append(float(csp[2]))

for ap in range(0,len(all_pts)-1):
    csp = all_pts[ap].split(',')
    xval1.append(float(csp[0]))
    yval1.append(float(csp[1]))
    zval1.append(float(csp[2]))

#plt.xlim(-500,500)
#plt.ylim(-500,500)

xarr = np.array(xval1)
yarr = np.array(yval1)
zarr = np.array(zval1)

ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')
print "XVALMAX: " + str(max(xval1))
print "YVALMAX: " + str(max(yval1))
print "ZVALMAX: " + str(max(zval1))

ax.scatter(xarr,yarr,zarr,c='b',marker='o')
plt.show()
