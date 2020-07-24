
clc;
clear;


while(1)
    cla;
    tarpoints = load('tarpoints.txt');
    dubins = load('dubins.txt');
    figure(1);
    hold on
    for j =1:length(dubins(:,1))
        
        plot(dubins(1:j,1),dubins(1:j,2),'r');
        hold on;     
        plot(tarpoints(j,1),tarpoints(j,2),'ro');     
        hold off;
        pause(0.0005);
    end
    
    title("particle position and target position")%БъЬт
    hold off;
    pause(1);
end




    
