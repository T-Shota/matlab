%rosinit 
node = robotics.ros.Node('listener');
sub = robotics.ros.Subscriber(node,'chatter','std_msgs/String',@chattercallback);
function chattercallback(~,msg)
    disp(msg.Data);
end