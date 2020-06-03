
clc;
clear;


while(1)
    cla;
    target = load('data/target.txt');
    uav = load('data/uav.txt');
    Gbest_position=load('data/traj_Point.txt');
    figure(1);
    
     hold on;
    color ={'r','r','g','g','b','b','c','c','m','m','y','y','k','k'};
    for j =1:length(uav(:,1))
        plot (110000,110000);
        hold on;
        axis equal;
        plot (0,0);
        plot(uav(1:j,1),uav(1:j,2),'r');
        text(uav(j,1),uav(j,2)-2000,['(uav',num2str(0),')'],'color','r');
        text(uav(j,1),uav(j,2),['(target',num2str(uav(j,3)),')'],'color','r');
        plot(uav(1:j,4),uav(1:j,5),'g');
        text(uav(j,4),uav(j,5)-2000,['(uav',num2str(1),')'],'color','g');
        text(uav(j,4),uav(j,5),['(target',num2str(uav(j,6)),')'],'color','g');
        plot(uav(1:j,7),uav(1:j,8),'b');
        text(uav(j,7),uav(j,8)-2000,['(uav',num2str(2),')'],'color','b');
        text(uav(j,7),uav(j,8),['(target',num2str(uav(j,9)),')'],'color','b');
        plot(uav(1:j,10),uav(1:j,11),'c');
        text(uav(j,10),uav(j,11)-2000,['(uav',num2str(3),')'],'color','c');
        text(uav(j,10),uav(j,11),['(target',num2str(uav(j,12)),')'],'color','c');
        plot(uav(1:j,13),uav(1:j,14),'m');
        text(uav(j,13),uav(j,14)-2000,['(uav',num2str(4),')'],'color','m');
        text(uav(j,13),uav(j,14),['(target',num2str(uav(j,15)),')'],'color','m');
        plot(uav(1:j,16),uav(1:j,17),'y'); 
        text(uav(j,16),uav(j,17)-2000,['(uav',num2str(5),')'],'color','y');
        text(uav(j,16),uav(j,17),['(target',num2str(uav(j,18)),')'],'color','y');
        plot(uav(1:j,19),uav(1:j,20),'k');
        text(uav(j,19),uav(j,20)-2000,['(uav',num2str(6),')'],'color','k');
        text(uav(j,19),uav(j,20),['(target',num2str(uav(j,21)),')'],'color','k');
        plot(uav(1:j,22),uav(1:j,23),'r');
        text(uav(j,22),uav(j,23)-2000,['(uav',num2str(7),')'],'color','r');
        text(uav(j,22),uav(j,23),['(target',num2str(uav(j,24)),')'],'color','r');
        plot(uav(1:j,25),uav(1:j,26),'g');
        text(uav(j,25),uav(j,26)-2000,['(uav',num2str(8),')'],'color','g');
        text(uav(j,25),uav(j,26),['(target',num2str(uav(j,27)),')'],'color','g');
        
        plot(Gbest_position(1:j,1),Gbest_position(1:j,2),'ro');
        plot(Gbest_position(1:j,3),Gbest_position(1:j,4),'go');
        plot(Gbest_position(1:j,5),Gbest_position(1:j,6),'bo');
        plot(Gbest_position(1:j,7),Gbest_position(1:j,8),'co');
        plot(Gbest_position(1:j,9),Gbest_position(1:j,10),'mo');
        plot(Gbest_position(1:j,11),Gbest_position(1:j,12),'yo');
        plot(Gbest_position(1:j,13),Gbest_position(1:j,14),'ko');
        plot(Gbest_position(1:j,15),Gbest_position(1:j,16),'ro');
        plot(Gbest_position(1:j,17),Gbest_position(1:j,18),'go');
        %for i=1:2:6
           % plot(uav(1:j,i),uav(1:j,i+1));
        %end
        %for i=1:2:6
            %plot(Gbest_position(1:j,i),Gbest_position(1:j,i+1),'o');
        %end
        
        plot(target(1:j,1),target(1:j,2),'p');
        text(target(j,1),target(j,2),['(target',num2str(0),')'],'color','g');
        rectangle('Position',[target(j,1)-8000,target(j,2)-8000,16000,16000],'Curvature',[1,1]),axis equal;
        plot(target(1:j,3),target(1:j,4),'p');
        text(target(j,3),target(j,4),['(target',num2str(1),')'],'color','g');
        rectangle('Position',[target(j,3)-8000,target(j,4)-8000,16000,16000],'Curvature',[1,1]),axis equal;
        plot(target(1:j,5),target(1:j,6),'p');
        text(target(j,5),target(j,6),['(target',num2str(2),')'],'color','g');
        rectangle('Position',[target(j,5)-8000,target(j,6)-8000,16000,16000],'Curvature',[1,1]),axis equal;
        
        plot(50000,50000);
        text(50000,50000,'rader','color','g');
        rectangle('Position',[50000-10000,50000-10000,20000,20000],'Curvature',[1,1]),axis equal;
       
        hold off;
        pause(0.0000005);
    end
    
    title("particle position and target position")%БъЬт
    hold off;
    pause(10);
end




    
