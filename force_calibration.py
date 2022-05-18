import numpy as np
import numpy.matlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from numpy.linalg import eig, inv

import rospy
import rospkg
import rosbag

# ------------------- Ellispod fitting -------------------

def ls_ellipsoid(xx,yy,zz):                                  
    #finds best fit ellipsoid. Found at http://www.juddzone.com/ALGORITHMS/least_squares_3D_ellipsoid.html
    #least squares fit to a 3D-ellipsoid
    #  Ax^2 + By^2 + Cz^2 +  Dxy +  Exz +  Fyz +  Gx +  Hy +  Iz  = 1
    #
    # Note that sometimes it is expressed as a solution to
    #  Ax^2 + By^2 + Cz^2 + 2Dxy + 2Exz + 2Fyz + 2Gx + 2Hy + 2Iz  = 1
    # where the last six terms have a factor of 2 in them
    # This is in anticipation of forming a matrix with the polynomial coefficients.
    # Those terms with factors of 2 are all off diagonal elements.  These contribute
    # two terms when multiplied out (symmetric) so would need to be divided by two
    
    # change xx from vector of length N to Nx1 matrix so we can use hstack
    x = xx[:,np.newaxis]
    y = yy[:,np.newaxis]
    z = zz[:,np.newaxis]
    
    #  Ax^2 + By^2 + Cz^2 +  Dxy +  Exz +  Fyz +  Gx +  Hy +  Iz = 1
    J = np.hstack((x*x,y*y,z*z,x*y,x*z,y*z, x, y, z))
    K = np.ones_like(x) #column of ones
    
    #np.hstack performs a loop over all samples and creates
    #a row in J for each x,y,z sample:
    # J[ix,0] = x[ix]*x[ix]
    # J[ix,1] = y[ix]*y[ix]
    # etc.
    
    JT=J.transpose()
    JTJ = np.dot(JT,J)
    InvJTJ=np.linalg.inv(JTJ);
    ABC= np.dot(InvJTJ, np.dot(JT,K))

    # Rearrange, move the 1 to the other side
    #  Ax^2 + By^2 + Cz^2 +  Dxy +  Exz +  Fyz +  Gx +  Hy +  Iz - 1 = 0
    #    or
    #  Ax^2 + By^2 + Cz^2 +  Dxy +  Exz +  Fyz +  Gx +  Hy +  Iz + J = 0
    #  where J = -1
    eansa=np.append(ABC,-1)

    return (eansa)

def polyToParams3D(vec,printMe):                             
    #gets 3D parameters of an ellipsoid. Found at http://www.juddzone.com/ALGORITHMS/least_squares_3D_ellipsoid.html
    # convert the polynomial form of the 3D-ellipsoid to parameters
    # center, axes, and transformation matrix
    # vec is the vector whose elements are the polynomial
    # coefficients A..J
    # returns (center, axes, rotation matrix)
    
    #Algebraic form: X.T * Amat * X --> polynomial form
    
    if printMe: print('\npolynomial\n',vec)
    
    Amat=np.array(
    [
    [ vec[0],     vec[3]/2.0, vec[4]/2.0, vec[6]/2.0 ],
    [ vec[3]/2.0, vec[1],     vec[5]/2.0, vec[7]/2.0 ],
    [ vec[4]/2.0, vec[5]/2.0, vec[2],     vec[8]/2.0 ],
    [ vec[6]/2.0, vec[7]/2.0, vec[8]/2.0, vec[9]     ]
    ])
    
    if printMe: print('\nAlgebraic form of polynomial\n',Amat)
    
    #See B.Bartoni, Preprint SMU-HEP-10-14 Multi-dimensional Ellipsoidal Fitting
    # equation 20 for the following method for finding the center
    A3=Amat[0:3,0:3]
    A3inv=inv(A3)
    ofs=vec[6:9]/2.0
    center=-np.dot(A3inv,ofs)
    if printMe: print('\nCenter at:',center)
    
    # Center the ellipsoid at the origin
    Tofs=np.eye(4)
    Tofs[3,0:3]=center
    R = np.dot(Tofs,np.dot(Amat,Tofs.T))
    if printMe: print('\nAlgebraic form translated to center\n',R,'\n')
    
    R3=R[0:3,0:3]
    R3test=R3/R3[0,0]
    # print('normed \n',R3test)
    s1=-R[3, 3]
    R3S=R3/s1
    (el,ec)=eig(R3S)
    
    recip=1.0/np.abs(el)
    axes=np.sqrt(recip)
    if printMe: print('\nAxes are\n',axes  ,'\n')
    
    inve=inv(ec) #inverse is actually the transpose here
    if printMe: print('\nRotation matrix\n',inve)
    return (center,axes,inve)

def printAns3D(center,axes,R,xin,yin,zin,verbose):

      print("\nCenter at  %10.4f,%10.4f,%10.4f" % (center[0],center[1],center[2]))
      print("Axes gains %10.4f,%10.4f,%10.4f " % (axes[0],axes[1],axes[2]))
      print("Rotation Matrix\n%10.5f,%10.5f,%10.5f\n%10.5f,%10.5f,%10.5f\n%10.5f,%10.5f,%10.5f" % (
      R[0,0],R[0,1],R[0,2],R[1,0],R[1,1],R[1,2],R[2,0],R[2,1],R[2,2]))


      # Check solution
      # Convert to unit sphere centered at origin
      #  1) Subtract off center
      #  2) Rotate points so bulges are aligned with axes (no xy,xz,yz terms)
      #  3) Scale the points by the inverse of the axes gains
      #  4) Back rotate
      # Rotations and gains are collected into single transformation matrix M

      # subtract the offset so ellipsoid is centered at origin
      xc=xin-center[0]
      yc=yin-center[1]
      zc=zin-center[2]

      # create transformation matrix
      L = np.diag([1/axes[0],1/axes[1],1/axes[2]])
      M=np.dot(R.T,np.dot(L,R))
      print('\nTransformation Matrix\n',M)

      # apply the transformation matrix
      [xm,ym,zm]=np.dot(M,[xc,yc,zc])
      # Calculate distance from origin for each point (ideal = 1.0)
      rm = np.sqrt(xm*xm + ym*ym + zm*zm)

      print("\nAverage Radius  %10.4f (truth is 1.0)" % (np.mean(rm)))
      print("Stdev of Radius %10.4f\n " % (np.std(rm)))

      return M

# ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
# ------------------- Ellispod fitting ------------------





rospack = rospkg.RosPack()
bag = rosbag.Bag(rospack.get_path('rokubi_ft_driver') + '/log/log.bag')

wrenchData = list(bag.read_messages(topics=['/wrench_data']))

forceData = np.zeros((3, len(wrenchData)))
torqueData = np.zeros((3, len(wrenchData)))
time = np.zeros(len(wrenchData))

for i, msg in enumerate(wrenchData):

    newForce = msg.message.wrench.force
    newTorque = msg.message.wrench.torque

    forceData[:, i] = np.array([newForce.x, newForce.y, newForce.z ])
    torqueData[:, i] = np.array([newTorque.x, newTorque.y, newTorque.z ])

    time[i] = msg.message.header.stamp.to_sec()

time = time - time[0]



# Fit ellipsoid on convex hull
eansa            = ls_ellipsoid(forceData[0, :], forceData[1, :], forceData[2, :]) #get ellipsoid polynomial coefficients
center,axes,inve = polyToParams3D(eansa,False)   #get ellipsoid 3D parameters

# Get transform and check result
transformMatrix = printAns3D(center,axes,inve, forceData[0, :], forceData[1, :], forceData[2, :], 1)
plt.plot(np.linalg.norm(transformMatrix@((forceData.T-center.T).T), axis=0))
plt.title("Norm of vector of calibration (should be 1)")

print("-------------------------")
print("To transform raw data, use the formula F = mg * M * (raw_data-bias)")
print("with F the force in [N], mg the weight of your calibration mass in [N], M is the Transformation matrix, and bias is the center vector")
print("-------------------------")

# Plotting ellipsoid
u = np.linspace(0, 2 * np.pi, 100)
v = np.linspace(0, np.pi, 100)

x = np.outer(np.cos(u), np.sin(v))
y = np.outer(np.sin(u), np.sin(v))
z = np.outer(np.ones_like(u), np.cos(v))

ellipsoid = ( (inve@np.diag(axes) @ np.stack((x, y, z), 0).reshape(3, -1)).T + center.T).T.reshape(3, *x.shape)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot_surface(*ellipsoid, rstride=4, cstride=4, color='b', alpha=0.2)
ax.scatter(forceData[0, :], forceData[1, :], forceData[2, :])
ax.set_title("Fitted ellipsoid with raw data")


plt.show()