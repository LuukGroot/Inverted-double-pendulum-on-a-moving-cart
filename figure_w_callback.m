m1 = 0.1; m2 = 0.1; M = 1; L1 = 2; L2 = 2; g = 10; d = 1; %Constants

%time definements
tmax = 30; %Length of animation
tstep = 0.001; %time step of ODE solver
tdrawstep = 0.05; %time step of graphing framerate

%A and B matrices:
A = [0 1 0 0 0 0;
0 0 0 (L1*m1)/(M+m1) 0 0;
0 0 0 1 0 0;
0 0 g*(m1+m2)/(L1*m1) m1/(M+m1) -g*m2/(L1*m1) 0;
0 0 0 0 0 1;
0 0 -g*(m1+m2)/(L2*m1) 0 g*(m1+m2)/(L2*m1) 0];
B = [0; 1/(M+m1); 0; 1/(L1*M+L1*m1); 0; 0];

x0 = [-1; 0; pi+0.1; 0; pi+0.1; 0;]; % initial condition
wr = [1; 0; pi; 0; pi; 0]; % reference position

%control matrix K with lqr
Q = eye(6); % 6x6 identify matrix
R = .0001;
K = lqr(A,B,Q,R);
u=@(x) -K*(x - wr); % control law
umax = 208; %ulimit on controller values
umin = -208; %llimit on controller values



handles.is_called = false; % inital called checker for keypress is pressed
handles.cart_vstep = 2; % user input adjust speed step

f = figure; %define figure

for t = 0:tdrawstep:tmax
    
    %Evaluate ODE for tspan of one tstep.
    tspan = t:tstep:t+tdrawstep;
    sols = ode45(@(t,x)mydoublependcart(x,m1,m2,M,L1,L2,g,d,u(x),umax,umin),tspan,x0); 
    
    % update initial conditions x0 for next iteration
    for i = 1:6
        x0(i) = deval(sols,t+tdrawstep,i);
    end
    
   
    handles.cart_v = x0(2); %update cart_v 
    set(f,'KeyPressFcn', {@fig1_key_callback, handles}) %adjust cart speed based on user input
    data = guidata(f); %record change of cart_v in data
    %If key was pressed we update cart_v:
    if ~isempty(guidata(f)) && data.is_called
        handles.cart_v = data.cart_v;
    end
    handles.is_called = false; %reset the called checker for next iteration
    guidata(f,handles) %update the handles 
    x0(2) = handles.cart_v; %update cart_v for the next iteration
    
    %-------------------------------------------------------------------
    %define graphics:
    cart_heigth = 1;
    cart_width = 2;
    x_pos_cart = x0(1);
    y_pos_cart = cart_heigth;
    x_pos_pend1 = x0(1)+L1*sin(pi-x0(3));
    y_pos_pend1 = cart_heigth+L1*cos(pi-x0(3));
    x_pos_pend2 = x0(1)+L1*sin(pi-x0(3))+L2*sin(pi-x0(5));
    y_pos_pend2 = cart_heigth+L1*cos(pi-x0(3))+L2*cos(pi-x0(5));
    cart_rec = rectangle('Position',[x_pos_cart-cart_width/2, y_pos_cart-cart_heigth, cart_width, cart_heigth],'FaceColor','r');
    pend_L1 = line([x_pos_cart x_pos_pend1],[y_pos_cart y_pos_pend1],'LineWidth', 2,'Color','b');
    pend_L2 = line([x_pos_pend1 x_pos_pend2],[y_pos_pend1 y_pos_pend2],'LineWidth', 2,'Color','b');
    center_pend1 = [x_pos_pend1 y_pos_pend1];
    center_pend2 = [x_pos_pend2 y_pos_pend2];
    radius_pend1 = sqrt(cart_width*cart_heigth/(M/(m1)*pi)); %The radius is based on the relative masses of the object, so that it looks legit.
    radius_pend2 = sqrt(cart_width*cart_heigth/(M/(m2)*pi)); %The radius is based on the relative masses of the object, so that it looks legit.
    pend1 = rectangle('Position',[center_pend1-radius_pend1, 2*radius_pend1, 2*radius_pend1],'Curvature',[1 1],'FaceColor','g');
    pend2 = rectangle('Position',[center_pend2-radius_pend2, 2*radius_pend2, 2*radius_pend2],'Curvature',[1 1],'FaceColor','g');
    
    %insert velocity display in figure:
    
    
    xlim([-10 10]); %figure limits
    ylim([-10 10]); %figure limits
    
    drawnow;
    
    pause(tdrawstep);

    %delete all graphics for next iteration
    delete(cart_rec);
    delete(pend_L1);
    delete(pend_L2);
    delete(pend1);
    delete(pend2);   
    
end