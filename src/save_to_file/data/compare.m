clear all;
close all;



pos=load('record_pose.txt');
joints=load('record_jts.txt');
%%

plot3(pos(:,1),pos(:,2),pos(:,3));


figure
plot(joints(:,1))
hold on
plot(joints(:,2))
plot(joints(:,3))
plot(joints(:,4))
plot(joints(:,5))
plot(joints(:,6))

figure

plot(diff(joints(:,1)));

    figure

for i = 1:6
    dq(:,i) = [0; (diff(joints(:,i)))];
    %dq(i,:) = data(:,i+1+DOF);

    dqs(:,i) = smooth(dq(:,i));
    
    hold on
    %plot(dq(i,:),'-r');
    plot(dqs(:,i),'-b');
     %plot(data(:,DOF+1+i),'-g');
    hold off
    
end