
node1 = robotics.ros.Node('talker5');
pub = robotics.ros.Publisher(node1, 'chatter','std_msgs/String');
msg = rosmessage(pub);
r=robotics.Rate(1);
for i=1:100
    s1 = 'Hello there! This is message ';
    s2 = num2str(i);
    ss = strcat(s1, s2);
    msg.Data = ss;
    send(pub,msg);
    waitfor(r);
end    