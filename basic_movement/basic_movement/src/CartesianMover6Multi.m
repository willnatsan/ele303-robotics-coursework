node = ros2node("/french_bread_multi", 44);
joint_trajectory_pub = ros2publisher(node, "/joint_trajectory", "trajectory_msgs/JointTrajectory");
joint_trajectory_msg = ros2message(joint_trajectory_pub);

home = IKSolver(0.345, 0, 0.4425, 0, pi/2, 0);
via1 = IKSolver(0.345, 0, 0.4, 0, pi/2, 0);
via2 = IKSolver(0.345, 0, 0.35, 0, pi/2, 0);
via3 = IKSolver(0.345, 0, 0.3, 0, pi/2, 0);
via4 = IKSolver(0.345, 0, 0.25, 0, pi/2, 0);
via5 = IKSolver(0.345, 0, 0.2, 0, pi/2, 0);

positions = [home, via1, via2, via3, via4, via5,
             via4, via3, via2, via1, home]

for i = 1:11
    for j = 1:6
        joint_trajectory_point_msg = ros2message("trajectory_msgs/JointTrajectoryPoint")
        joint_trajectory_point_msg.positions = positions(i)(j).JointPosition
        joint_trajectoy_msg.points = [joint_trajectory_msg.points, joint_trajectory_point_msg]
    end
end

send(joint_trajectory_pub, joint_trajectory_msg);


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