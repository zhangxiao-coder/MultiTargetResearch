
clc;
clear;


while(1)
    cla;
    particle_state = load('data/particle_state.txt');
    uav_state = load('data/uav_state.txt');
    best_state = load('data/best_state.txt');
    figure(1);
    hold on
    for j =1:length(particle_state(:,1))
        plot(uav_state(1:j,1),uav_state(1:j,2),'ro');  
        hold on; 
        rectangle('Position',[uav_state(j,1)-5000,uav_state(j,2)-5000,10000,10000],'Curvature',[1,1]),axis equal;
        plot(best_state(j,1),best_state(j,2),'ko');
        for k=1:2:length(particle_state(j,:))
            plot(particle_state(j,k),particle_state(j,k+1),'g*');
        end
        grid on;
        text(30000,30000,'rader','color','g');
        rectangle('Position',[30000-5000,30000-5000,10000,10000],'Curvature',[1,1]),axis equal;
        rectangle('Position',[15000,10000,5000,5000]);
        hold off;
        pause(0.05);
    end
    
    title("particle position and target position")%БъЬт
    hold off;
    pause(1);
end