function plotMAVStateVariables(uu)
%
% Modified:  4/5/2016 - Gang

    % process inputs to function
    pn          = uu(1);             % North position (meters)
    pe          = uu(2);             % East position (meters)
    h           = -uu(3);            % altitude (meters)
    u           = uu(4);             % velocity along body x-axis (meters/s)
    v           = uu(5);             % velocity along body y-axis (meters/s)
    w           = uu(6);             % velocity along body z-axis (meters/s)
    phi         = 180/pi*uu(7);      % roll angle (degrees)   
    theta       = 180/pi*uu(8);      % pitch angle (degrees)
    psi         = 180/pi*uu(9);      % yaw angle (degrees)
    p           = 180/pi*uu(10);     % body angular rate along x-axis (degrees/s)
    q           = 180/pi*uu(11);     % body angular rate along y-axis (degrees/s)
    r           = 180/pi*uu(12);     % body angular rate along z-axis (degrees/s)
    
    pn_c        = uu(13);            % commanded North position (meters)
    pe_c        = uu(14);            % commanded East position (meters)
    h_c         = -uu(15);            % commanded altitude (meters)
    u_c         = uu(16);            % commanded inertial x velocity (m/s)
    v_c         = uu(17);            % commanded inertial y velocity (m/s)
    w_c         = uu(18);            % commanded inertial z velocity (m/s)
    phi_c       = 180/pi*uu(19);     % commanded roll angle (degrees)   
    theta_c     = 180/pi*uu(20);     % commanded pitch angle (degrees)
    psi_c       = 180/pi*uu(21);     % commanded yaw angle (degrees)
    p_c         = 180/pi*uu(22);     % commanded body angular rate along x-axis (degrees/s)
    q_c         = 180/pi*uu(23);     % commanded body angular rate along y-axis (degrees/s)
    r_c         = 180/pi*uu(24);     % commanded body angular rate along z-axis (degrees/s)
    

    delta_f     = uu(25);     % delta_f 
    delta_r     = uu(26);     % delta_l
    delta_b     = uu(27);     % delta_b
    delta_l     = uu(28);     % delta_r
    t           = uu(29);     % simulation time
    
    % define persistent variables 
    persistent pn_handle
    persistent pe_handle
    persistent h_handle
    persistent u_handle
    persistent v_handle
    persistent w_handle
    persistent phi_handle
    persistent theta_handle
    persistent psi_handle
    persistent p_handle
    persistent q_handle
    persistent r_handle
    persistent delta_e_handle
    persistent delta_a_handle
    persistent delta_r_handle
    persistent delta_t_handle
    

  % first time function is called, initialize plot and persistent vars
    if t==0,
        figure(2), clf

        subplot(4,4,1)
        hold on
        pn_handle = graph_y_yd(t, pn, pn_c, 'p_n', []);
        
        subplot(4,4,2)
        hold on
        pe_handle = graph_y_yd(t, pe, pe_c, 'p_e', []);
        
        subplot(4,4,3)
        hold on
        h_handle = graph_y_yd(t, h, h_c, 'h', []);
        
        subplot(4,4,4)
        hold on
        u_handle = graph_y_yd(t, u, u_c, 'u', []);

        subplot(4,4,5)
        hold on
        v_handle = graph_y_yd(t, v, v_c, 'v', []);
        
        subplot(4,4,6)
        hold on
        w_handle = graph_y_yd(t, w, w_c, 'w', []);

        subplot(4,4,7)
        hold on
        phi_handle = graph_y_yd(t, phi, phi_c, '\phi', []);
        
        subplot(4,4,8)
        hold on
        theta_handle = graph_y_yd(t, theta, theta_c, '\theta', []);
        
        subplot(4,4,9)
        hold on
        psi_handle = graph_y_yd(t, psi, psi_c, '\psi', []);
        
        subplot(4,4,10)
        hold on
        p_handle = graph_y_yd(t, p, p_c, 'p', []);
        
        subplot(4,4,11)
        hold on
        q_handle = graph_y_yd(t, q, q_c, 'q', []);
          
        subplot(4,4,12)
        hold on
        r_handle = graph_y_yd(t, r, r_c, 'r', []);
        
        subplot(4,4,13)
        hold on
        delta_e_handle = graph_y(t, delta_f, [], 'b');
        ylabel('\delta_f')
        
        subplot(4,4,14)
        hold on
        delta_a_handle = graph_y(t, delta_r, [], 'b');
        ylabel('\delta_l')

        subplot(4,4,15)
        hold on
        delta_r_handle = graph_y(t, delta_b, [], 'b');
        ylabel('\delta_b')
        
        subplot(4,4,16)
        hold on
        delta_t_handle = graph_y(t, delta_l, [], 'b');
        ylabel('\delta_r')
        
    % at every other time step, redraw state variables
    else 
       graph_y_yd(t, pn, pn_c, 'p_n', pn_handle);
       graph_y_yd(t, pe, pe_c, 'p_e', pe_handle);
       graph_y_yd(t, h, h_c, 'h', h_handle);
       graph_y_yd(t, u, u_c, 'u_x', u_handle);
       graph_y_yd(t, v, v_c, 'u_y', v_handle);
       graph_y_yd(t, w, w_c, 'u_z', w_handle);
       graph_y_yd(t, phi, phi_c, '\phi', phi_handle);
       graph_y_yd(t, theta, theta_c, '\theta', theta_handle);
       graph_y_yd(t, psi, psi_c, '\psi', psi_handle);
       graph_y_yd(t, p, p_c, 'p', p_handle);
       graph_y_yd(t, q, q_c, 'q', q_handle);
       graph_y_yd(t, r, r_c, 'r', r_handle);
       graph_y(t, delta_f, delta_e_handle);
       graph_y(t, delta_r, delta_a_handle);
       graph_y(t, delta_b, delta_r_handle);
       graph_y(t, delta_l, delta_t_handle);
    end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% graph y with lable mylabel
function handle = graph_y(t, y, handle, color)
  
  if isempty(handle),
    handle    = plot(t,y,color);
  else
    set(handle,'Xdata',[get(handle,'Xdata'),t]);
    set(handle,'Ydata',[get(handle,'Ydata'),y]);
    %drawnow
  end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% graph y and yd with lable mylabel
function handle = graph_y_yd(t, y, yd, lab, handle)
  
  if isempty(handle),
    handle(1)    = plot(t,y,'b');
    handle(2)    = plot(t,yd,'r');
    ylabel(lab)
    set(get(gca, 'YLabel'),'Rotation',0.0);
  else
    set(handle(1),'Xdata',[get(handle(1),'Xdata'),t]);
    set(handle(1),'Ydata',[get(handle(1),'Ydata'),y]);
    set(handle(2),'Xdata',[get(handle(2),'Xdata'),t]);
    set(handle(2),'Ydata',[get(handle(2),'Ydata'),yd]);
    %drawnow
  end





