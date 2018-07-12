%% Initialization
clear all;
clc;
rosshutdown;
rosinit;
global node;

%% Define node
node = robotics.ros.Node('pointCloud_test');
sub = robotics.ros.Subscriber(node, '/points_raw', 'sensor_msgs/PointCloud2', @pointCloud_test_callback);


function pointCloud_test_callback(~, msg)
    global node;
    ptMsg = msg;

    plane_pub = robotics.ros.Publisher(node, '/filterd_points','sensor_msgs/PointCloud2');
    send(plane_pub, ptMsg);

end