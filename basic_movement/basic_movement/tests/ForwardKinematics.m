body=rigidBody('body1')
jnt1=rigidBodyJoint('jnt1','revolute')
jnt1.HomePosition=pi/4
tform=trvec2tform([0.25,0.25,0])
setFixedTransform(jnt1,tform)
body.Joint=jnt1
robot=rigidBodyTree
addBody(robot,body,'base')
robot


%Task 1
body2=rigidBody('body2')
jnt2=rigidBodyJoint('jnt2','revolute')
jnt2.HomePosition=pi/6
tform2=trvec2tform([1,0,0])
setFixedTransform(jnt2,tform2)
body2.Joint=jnt2
addBody(robot,body2,'body1')
body3=rigidBody('body3')
jnt3=rigidBodyJoint('jnt3','revolute')
jnt3.HomePosition=0
j3trf=[0 1 0 0.6;-1 0 0 -0.1;0 0 1 0;0 0 0 1]
setFixedTransform(jnt3,j3trf)
body3.Joint=jnt3
addBody(robot,body3,'body2')
body4=rigidBody('body4')
jnt4=rigidBodyJoint('jnt4','revolute')
jnt4.HomePosition=pi/4
tform4=trvec2tform([1,0,0])
setFixedTransform(jnt4,tform4)
body4.Joint=jnt4
addBody(robot,body4,'body3')

end_effector=rigidBody('end_effector')
addBody(robot,end_effector,'body4')

robot.show
task2a=getTransform(robot,homeConfiguration(robot),"end_effector", "base")
task2b=getTransform(robot,homeConfiguration(robot),"end_effector", "body1")
