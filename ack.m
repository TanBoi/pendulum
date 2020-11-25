function ack()


fclose(instrfind);
delete(instrfind);

state=1
g = serial('COM3','baudrate',115200);
fopen(g);


%on initialise les paramètres
l=1;
i=1;
pause=1;


%a changer
s=220;
y1= zeros(1);
y2= zeros(1);




%interface
btn = uicontrol('style', 'togglebutton');
set(btn, 'string', 'stop');
set(btn, 'callback', {@stopSim});
set(btn, 'position',[10 10 100 20]);

    function stopSim(hObj, event)
        %a changer
       l = 900; 
    end

btn_pause = uicontrol('style', 'togglebutton');
set(btn_pause, 'string', 'pause plot off');
set(btn_pause, 'callback', {@pauseSim});
set(btn_pause, 'position',[400 10 100 20]);

    function pauseSim(hObj, event)
       if pause == 1
           pause = 0;
           disp('Plot en pause');
           set(hObj, 'string', 'pause plot off');
       else
           pause = 1;
           set(hObj, 'string', 'pause plot on');
           disp('Reprise plot');
       end
    end

btn_rst = uicontrol('style', 'togglebutton');
set(btn_rst, 'string', 'reset plot');
set(btn_rst, 'callback', {@resetSim});
set(btn_rst, 'position',[200 10 100 20]);

    function resetSim(hObj, event)
       disp('Reset plot');
       y1= zeros(1);
       y2= zeros(1);
       i=1;fclose(instrfind);
       delete(instrfind);

       state=1
       g = serial('COM3','baudrate',115200);
       fopen(g);
    end

subplot(411);
title('Vitesse encodeur')
subplot(412);
title('Vitesse potar')
subplot(413);
title('Acceleration')
subplot(414);
title('Position pendule')

while l<s+3
    varc = fgets(g);
%     !disp(varc);
%     var = str2double(varc);
%     disp(str2num(varc)+str2num(varc));
    if state == 1
        %encodeur
        y1(i) = str2double(varc)*pi/180;
        x=1:1:length(y1);
        
        subplot(411);
        if pause
            plot(x,y1); 
            title('Vitesse encodeur')  
            grid on;    
        end
    else
        %potar
        y2(i) = str2double(varc);
        y2(i) = ((y2(i)-180000)/180000)*(pi/2)-pi/2;
        x=1:1:length(y2);

        subplot(412);
        if pause
            plot(x,y2);
            title('Position Potar')
            grid on;
        end
        subplot(413);
        
        %si pause off
        if pause
            plot(x,0);
            title('Acceleration')
            grid on;
        end
        %position 
        a = cos( y2(i));
        b = sin( y2(i));
        subplot(414);
        if pause
            p = plot(a, b, 'o', [0,a],[0,b]);
            title('Position pendule')
        end
        axis([-10 10 -1.5 1.5]);
        i=i+1;    
    end
    
    state= -state;
    drawnow;
    
end

fclose(g);

close all;

end