node = ros2node("/french_bread", 44);
joint_pub = ros2publisher(node, "/joint_demands", "std_msgs/Float32MultiArray");
joint_msg = ros2message(joint_pub);

home = IKSolver(0.345, 0, 0.4425, 0, pi/2, 0);
config = IKSolver(0.4, 0, 0.25, 0, pi/2, 0);

for i = 1:6
    % joint_msg.data(i) = config(i).JointPosition;
    joint_msg.data(i) = home(i).JointPosition;
end

send(joint_pub, joint_msg);


function [configSoln] = IKSolver(X,Y,Z,A,B,C)
    mover6 = importrobot("CPMOVER6.urdf");

    tfT = trvec2tform([X Y Z]);
    tfR = eul2tform([A B C]);
    tform = tfT * tfR;

    ik = inverseKinematics("RigidBodyTree", mover6);
    weights = [0.25 0.25 0.25 1 1 1];
    initialguess = mover6.homeConfiguration;

    [configSoln, ~] = ik("link6", tform, weights, initialguess);
end