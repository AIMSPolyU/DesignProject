% path follow
%  - follow straight line path or orbit
%
% Modified:
%
% input is:
%   flag - if flag==1, follow waypoint path
%          if flag==2, follow orbit
%   
%   Va^d   - desired airspeed
%   r      - inertial position of start of waypoint path
%   q      - unit vector that defines inertial direction of waypoint path
%   c      - center of orbit
%   rho    - radius of orbit
%   lambda - direction of orbit (+1 for CW, -1 for CCW)
%   xhat   - estimated MAV states (pn, pe, pd, Va, alpha, beta, phi, theta, chi, p, q, r, Vg, wn, we, psi)
%
% output is:
%  Va_c - airspeed command
%  h_c  - altitude command
%  chi_c - heading command
%  phi_ff - feed forward roll command
%
function out = path_follow(in)
    % path following gains
    chi_infty =   pi/2;% approach angle for large distance from straight-line path
    k_path    =   0.005; % proportional gain for path following
    k_orbit   =   10;     % proportional gain for orbit following
    gravity   =   9.81;

    NN = 0;
    flag      = in(1+NN);
    Va_d      = in(2+NN);
    r_path    = [in(3+NN); in(4+NN); in(5+NN)];
    q_path    = [in(6+NN); in(7+NN); in(8+NN)];
    c_orbit   = [in(9+NN); in(10+NN); in(11+NN)];
    rho_orbit = in(12+NN);
    lam_orbit = in(13+NN);
    NN = NN + 13;
    pn        = in(1+NN);
    pe        = in(2+NN);
    h         = in(3+NN);
    Va        = in(4+NN);
    alpha   = in(5+NN);
    beta    = in(6+NN);
    phi       = in(7+NN);
    theta     = in(8+NN);
    chi       = in(9+NN);
    p       = in(10+NN);
    q       = in(11+NN);
    r       = in(12+NN);
    Vg      = in(13+NN);
    wn      = in(14+NN);
    we      = in(15+NN);
    psi     = in(16+NN);
    NN = NN + 16;
    t         = in(1+NN);
  
    switch flag
        case 1 % follow straight line path specified by r and q
          
            % compute wrapped version of path angle
            chi_q = atan2(q_path(2),q_path(1));
            while (chi_q - chi < -pi), chi_q = chi_q + 2*pi; end
            while (chi_q - chi > +pi), chi_q = chi_q - 2*pi; end
            Rpi=[cos(chi_q),sin(chi_q),0;-sin(chi_q),cos(chi_q),0;0,0,1];
            ep=Rpi*([pn;pe;-h]-r_path);
            n=cross(q_path,[0,0,1])/norm(cross(q_path,[0,0,1]));
            si=[pn;pe;-h]-r_path-([pn;pe;-h]-r_path)*n;

            % heading command
            chi_c =                    % TODO

            % commanded altitude
            h_c =                      % TODO
            % roll feedforward command
            phi_ff =                   % TODO
           
        case 2 % follow orbit specified by c, rho, lam

            d =                        % TODO
            % compute wrapped version of angular position on orbit
            varphi =                   % TODO
            while (varphi - chi < -pi), varphi = varphi + 2*pi; end
            while (varphi - chi > +pi), varphi = varphi - 2*pi; end
             % compute orbit error
            orbit_error =              % TODO
            % heading command
            chi_c =                    % TODO

            % commanded altitude is the height of the orbit
            h_c =                      % TODO

            % the roll angle feedforward command 
            phi_ff =                   % TODO
    end

    % command airspeed equal to desired airspeed
    Va_c = Va_d;
  
    % create output
    out = [Va_c; h_c; chi_c; phi_ff;d];
end


