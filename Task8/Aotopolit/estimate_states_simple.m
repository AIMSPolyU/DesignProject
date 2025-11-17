% estimate_states_simple
%   - estimate the MAV states using gyros, accels, pressure sensors, and
%     GPS.
%   - performs similarly to standard EKF in the book, but GPS smoothing EKF
%     is simplified. Equations for north and east components are not
%     estimated, but rather calculated from estimates of Vg, psi, and chi,
%     and measurement of Va. EKF equations do not include wind states.
%
% Outputs are:
%   pnhat    - estimated North position, 
%   pehat    - estimated East position, 
%   hhat     - estimated altitude, 
%   Vahat    - estimated airspeed, 
%   alphahat - estimated angle of attack
%   betahat  - estimated sideslip angle
%   phihat   - estimated roll angle, 
%   thetahat - estimated pitch angel, 
%   chihat   - estimated course, 
%   phat     - estimated roll rate, 
%   qhat     - estimated pitch rate, 
%   rhat     - estimated yaw rate,
%   Vghat    - estimated ground speed, 
%   wnhat    - estimate of North wind, 
%   wehat    - estimate of East wind
%   psihat   - estimate of heading angle
% 
% 
% Modified:  3/15/2010 - RB
%            5/18/2010 - RB
%            2/20/2018 - TM
%

function xhat = estimate_states_simple(uu, UAV, SENSOR, SIM)

   % rename inputs
   y_gyro_x      = uu(1);
   y_gyro_y      = uu(2);
   y_gyro_z      = uu(3);
   y_accel_x     = uu(4);
   y_accel_y     = uu(5);
   y_accel_z     = uu(6);
   y_static_pres = uu(7);
   y_diff_pres   = uu(8);
   y_gps_n       = uu(9);
   y_gps_e       = uu(10);
   y_gps_h       = uu(11);
   y_gps_Vg      = uu(12);
   y_gps_course  = uu(13);
   t             = uu(14);
 
   
    % define persistent variables
    persistent alpha  % constant for low pass filter - only compute once
    persistent alpha1  % constant for low pass filter - only compute once
    persistent lpf_gyro_x   % low pass filter of x-gyro
    persistent lpf_gyro_y   % low pass filter of y-gyro
    persistent lpf_gyro_z   % low pass filter of z-gyro
    persistent lpf_static   % low pass filter of static pressure sensor
    persistent lpf_diff     % low pass filter of diff pressure sensor
    persistent lpf_accel_x  % low pass filter of x-accelerometer
    persistent lpf_accel_y  % low pass filter of y-accelerometer
    persistent lpf_accel_z  % low pass filter of z-accelerometer
    persistent xhat_a       % estimate of roll and pitch
    persistent P_a          % error covariance for roll and pitch angles
    persistent xhat_p       % estimate of pn, pe, Vg, wn, we, chi, psi
    persistent P_p          % error covariance for pn, pe, Vg, chi, psi
    persistent y_gps_n_old  % last measurement of gps_n - used to detect new GPS signal
    persistent y_gps_e_old  % last measurement of gps_e - used to detect new GPS signal
    persistent y_gps_Vg_old % last measurement of gps_Vg - used to detect new GPS signal
    persistent y_gps_course_old  % last measurement of gps_course - used to detect new GPS signal
    
   
    % initialize persistent variables
    lpf_a = 50;
    lpf_a1 = 20;
    if isempty(alpha)
        alpha = exp(-lpf_a*SIM.ts_control)
        alpha1 = exp(-lpf_a1*SIM.ts_control)
        lpf_gyro_x   = y_gyro_x;
        lpf_gyro_y   = y_gyro_y;
        lpf_gyro_z   = y_gyro_z;
        lpf_static   = y_static_pres;
        lpf_diff     = y_diff_pres;
        lpf_accel_x  = y_accel_x;
        lpf_accel_y  = y_accel_y;
        lpf_accel_z  = y_accel_z;
        xhat_a       = [0;0];
        P_a          = 0.1;
        xhat_p       = [y_gps_n; y_gps_e; y_gps_Vg; y_gps_course; 0; 0; 0];
        P_p          = 0.1;
        y_gps_n_old  = -9999;
        y_gps_e_old  = -9999;
        y_gps_Vg_old = -9999;
        y_gps_course_old  = -9999;
    end
    
    %------------------------------------------------------------------
    % low pass filter gyros to estimate angular rates
    lpf_gyro_x = alpha*lpf_gyro_x+(1-alpha)*y_gyro_x;
    lpf_gyro_y = alpha*lpf_gyro_y+(1-alpha)*y_gyro_y;
    lpf_gyro_z = alpha*lpf_gyro_z+(1-alpha)*y_gyro_z;
    phat = lpf_gyro_x;
    qhat = lpf_gyro_y;
    rhat = lpf_gyro_z;
    
    %------------------------------------------------------------------
    % low pass filter static pressure sensor and invert to estimate
    % altitude
    lpf_static = alpha*lpf_static+(1-alpha)*y_static_pres;
    hhat = lpf_static/UAV.rho/UAV.gravity;
    
    % low pass filter diff pressure sensor and invert to estimate Va
    lpf_diff = alpha*lpf_diff+(1-alpha)*y_diff_pres;
    Vahat = sqrt(2*lpf_diff/UAV.rho);
    
        
    %-------------------------------------------------------------------
    % implement continous-discrete EKF to estimate roll and pitch angles
    Q_a = SENSOR.accel_sigma;
    Q_gyro = SENSOR.gyro_sigma;
    R_accel = Q_a*Q_a;

    N = 5;
    Tp=SIM.ts_control/N;


    %-------------------------------------------------------------------
    % implement continous-discrete EKF to estimate pn, pe, Vg, chi, psi
    Q_p = 0.21;

    % prediction step 
    for i=1:N
        cp = cos(xhat_a(1));  % cos(phi)
        sp = sin(xhat_a(1));  % sin(phi)
        tt = tan(xhat_a(2));  % tan(theta)
        ct = cos(xhat_a(2));  % cos(theta)
        pnhat    = xhat_p(1);
        pehat    = xhat_p(2);
        Vghat    = xhat_p(3);
        chihat   = xhat_p(4); 
        wnhat    = xhat_p(5);
        wehat    = xhat_p(6);
        psihat   = xhat_p(7);   
        phihat   = xhat_a(1);
        thetahat = xhat_a(2);

        % TODO
        f_p    = 
        A_p    = 
        xhat_p = 
        P_p    = 
    end
    

     % measurement updates
    cc = cos(xhat_p(4));    % cos(chi)
    sc = sin(xhat_p(4));    % sin(chi)
    cpsi = cos(xhat_p(7));  % cos(psi)
    spsi = sin(xhat_p(7));  % sin(psi)
    cp = cos(xhat_a(1));    % cos(phi)
    sp = sin(xhat_a(1));    % sin(phi)
    ct = cos(xhat_a(2));    % cos(theta)
    st = sin(xhat_a(2));    % sin(theta)
    pnhat    = xhat_p(1);
    pehat    = xhat_p(2);
    Vghat    = xhat_p(3);
    chihat   = xhat_p(4); 
    wnhat    = xhat_p(5);
    wehat    = xhat_p(6);
    psihat   = xhat_p(7);

    phihat   = xhat_a(1);
    thetahat = xhat_a(2);
    % measurement updates
    threshold = 2;
    % x-axis accelerometer
    h_a = 
    C_a = 
    L_a = 
    P_a = 
    
    if norm([y_accel_x;y_accel_y;y_accel_z] - h_a)<threshold
        xhat_a = 
    end

    cp = cos(xhat_a(1));    % cos(phi)
    sp = sin(xhat_a(1));    % sin(phi)
    ct = cos(xhat_a(2));    % cos(theta)
    st = sin(xhat_a(2));    % sin(theta)
    
    % TODO
    if  (y_gps_n~=y_gps_n_old)...
        ||(y_gps_e~=y_gps_e_old)...
        ||(y_gps_Vg~=y_gps_Vg_old)...
        ||(y_gps_course~=y_gps_course_old)
        % gps North position
        h_p = 
        C_p = 
        L_p = 
        P_p = 
        xhat_p = 


    cc = cos(xhat_p(4));    % cos(chi)
    sc = sin(xhat_p(4));    % sin(chi)
    cpsi = cos(xhat_p(7));  % cos(psi)
    spsi = sin(xhat_p(7));  % sin(psi)
    cp = cos(xhat_a(1));    % cos(phi)
    sp = sin(xhat_a(1));    % sin(phi)
    ct = cos(xhat_a(2));    % cos(theta)
    st = sin(xhat_a(2));    % sin(theta)
    pnhat    = xhat_p(1);
    pehat    = xhat_p(2);
    Vghat    = xhat_p(3);
    chihat   = xhat_p(4); 
    wnhat    = xhat_p(5);
    wehat    = xhat_p(6);
    psihat   = xhat_p(7);

        % % gps course
        % % wrap course measurement
        while (y_gps_course - xhat_p(4))>pi, y_gps_course = y_gps_course - 2*pi; end
        while (y_gps_course - xhat_p(4))<-pi, y_gps_course = y_gps_course + 2*pi; end

        % update stored GPS signals
        y_gps_n_old      = y_gps_n;
        y_gps_e_old      = y_gps_e;
        y_gps_Vg_old     = y_gps_Vg;
        y_gps_course_old = y_gps_course;
    end


    pnhat    = xhat_p(1);
    pehat    = xhat_p(2);
    Vghat    = xhat_p(3);
    chihat   = xhat_p(4); 
    wnhat    = xhat_p(5);
    wehat    = xhat_p(6);
    psihat   = xhat_p(7);

    phihat   = xhat_a(1);
    thetahat = xhat_a(2);
    % not estimating these states 
    alphahat = thetahat;
    betahat  = 0;
    bxhat    = 0;
    byhat    = 0;
    bzhat    = 0;
    % wnhat    = Vghat*cos(chihat) - Vahat*cos(psihat);
    % wehat    = Vghat*sin(chihat) - Vahat*sin(psihat);
    velocity=Vahat*[ct*1;0;st*1];
    uhat=velocity(1);
    vhat=velocity(2);
    what=velocity(3);

      xhat = [...
        pnhat;...
        pehat;...
        hhat;...
        Vahat;...
        alphahat;...
        betahat;...
        phihat;...
        thetahat;...
        chihat;...
        phat;...
        qhat;...
        rhat;...
        Vghat;...
        wnhat;...
        wehat;...
        psihat;...
        bxhat;...
        byhat;...
        bzhat;...
        uhat;
        vhat;
        what;
        ];
end
