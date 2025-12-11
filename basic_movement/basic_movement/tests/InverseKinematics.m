mover6=importrobot("CPMOVER6.urdf")
mover6.show

%Task 3
tf1=[1 0 0 0;0 1 0 0;0 0 1 0.13;0 0 0 1]
tf2=[1 0 0 0;0 1 0 0;0 0 1 0.0625;0 0 0 1]
tf3=[cos(pi/2) 0 sin(pi/2) 0;0 1 0 0;-sin(pi/2) 0 cos(pi/2) 0.19;0 0 0 1]
tf4=[1 0 0 -0.06;0 1 0 0;0 0 1 0;0 0 0 1]
tf5=[1 0 0 0;0 1 0 0; 0 0 1 0.29;0 0 0 1]
tf6=[1 0 0 0;0 1 0 0;0 0 1 0.055;0 0 0 1]
tf1*tf2*tf3*tf4*tf5*tf6

%Task 4
randConfig=mover6.randomConfiguration
show(mover6,randConfig)
ik=inverseKinematics('RigidBodyTree',mover6)
weights=[0.25 0.25 0.25 1 1 1]
initialguess=mover6.randomConfiguration
[configSoln,solnInfo]=ik('link6',tform,weights,initialguess)
