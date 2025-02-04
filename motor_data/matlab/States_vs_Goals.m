%https://la.mathworks.com/help/robotics/ref/importrobot.html

ndoc  = 1;
joint_goals   = table2array(readtable(strcat('motor_data/',num2str(ndoc),'_goal_position.txt')));
joint_states  = table2array(readtable(strcat('motor_data/',num2str(ndoc),'_pres_position.txt')));


diferencia = (joint_states - joint_goals);
diff_abs = abs(joint_states - joint_goals);

for j = 2:9

    figure(1)
    subplot(4,2,j-1)
    plot(diferencia(:,j),"LineWidth",2);grid on; grid minor;
    title("Joint " + (j-1));
    legend("Av. error: " + mean(diff_abs(:,j)))
end
xlabel("Iteración")
ylabel("Error (grados)")
for j = 2:9
    
    figure(2);
    subplot(4,2,j-1)
    plot(joint_states(:,j),"LineWidth",2);grid on;grid minor;
    title("Joint " + (j-1));hold on;
    plot(joint_goals(:,j),"LineWidth",2)
    legend('Real Position','Goal Position') 
end
xlabel("Iteración")
ylabel("Desplazamiento (grados)")


