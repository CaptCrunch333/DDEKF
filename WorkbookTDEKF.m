function [fat, fbsa, fa, fv, fp, fut_b, fR, fat_dot, fjerk, dFx, dFQ, hp, ha, dHpos, dHacc] = WorkbookTDEKF(varargin)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% OVERVIEW %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Estimates1 [p v a at ut_b]
    % Inputs1 [u]
    % Measurements1 [pos acc]
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Define Estimates
    p = sym('p', [3 1], 'real'); % drone position in inertial frame
    v = sym('v', [3 1], 'real'); % drone velocity in inertial frame
    bsa = sym('bsa', [3 1], 'real'); % drone acceleration in body frame including gravity (gives same quantity as the accelerometer)
    at = sym('at', 'real'); % Body frame generated acceleration in case of no drag
    ut_b = sym('ut_b', 'real'); % Body frame generated acceleration bias
    %% Define Inputs
    u = sym('u', 'real'); % Thrust Command
    q = sym('q', [4 1], 'real');
    %% Define Measurements
    pos = sym('pos', [3 1], 'real'); % Position Measurement in Inertial Frame
    acc = sym('acc', [3 1], 'real'); % Accelerometer Measurement in Body Frame
    %% Define Conversion & Intermediate Variables
    fR = convertQuatToRotationMatrix(q)'; % Body To Inertial
    R = sym('R', [3 3], 'real');
    at_dot = sym('at_dot', 'real');
    a = sym('a', [3 1], 'real');
    jerk = sym('jerk', [3 1], 'real');
    %% Define Parameters
    Tmz = sym('Tmz', 'real'); % Thrust time constant in the z direction (frameless)
    Kmz = sym('Kmz', 'real'); % Thrust gain (frameless) in the z direction
    Td = sym('Td', [3 1], 'real'); % Drag time constant in body frame [td_x td_y td_z]
    g = sym('g', [3 1], 'real'); % gravity estimate
    dt = sym('dt', 'real'); % step time
    %% Linear calculations
    % Generated Acceleration Update
    fat_dot = ((u - ut_b)*Kmz/Td(3) - at)*1/Tmz;
    fat = at + at_dot * dt;
    ffat = at + fat_dot * dt;
    dfat_dp = jacobian(ffat, p);
    dfat_dv = jacobian(ffat, v);
    dfat_dbsa = jacobian(ffat, bsa);
    dfat_dat = jacobian(ffat, at);
    dfat_dut_b = jacobian(ffat, ut_b);
    dfat_du = jacobian(ffat, u);
    dfat_dq = jacobian(ffat, q);
    % Drag-Acceleration term
    fbsa = bsa + dt*([0;0;at_dot] - diag(1./Td)*(bsa - R'*g));
    ffbsa = bsa + dt*([0;0;fat_dot] - diag(1./Td)*(bsa - fR'*g));
    dfbsa_dp = jacobian(ffbsa, p);
    dfbsa_dv = jacobian(ffbsa, v);
    dfbsa_dbsa = jacobian(ffbsa, bsa);
    dfbsa_dat = jacobian(ffbsa, at);
    dfbsa_dut_b = jacobian(ffbsa, ut_b);
    dfbsa_du = jacobian(ffbsa, u);
    dfbsa_dq = jacobian(ffbsa, q);
    fa = R*bsa - g;
    ffa = fR*bsa - g;
    fjerk = R*([0;0;at_dot] - diag(1./Td)*(bsa - R'*g));
    ffjerk = fR*([0;0;fat_dot] - diag(1./Td)*(bsa - R'*g));
    % Velocity Update Jacobians
    fv = v + dt*a + dt*dt/2*jerk; % [p v a q w g]
    ffv = v + dt*ffa + dt*dt/2*ffjerk; % [p v a q w g]
    dfv_dp = jacobian(ffv, p);
    dfv_dv = jacobian(ffv, v);
    dfv_dbsa = jacobian(ffv, bsa);
    dfv_dat = jacobian(ffv, at);
    dfv_dut_b = jacobian(ffv, ut_b);
    dfv_du = jacobian(ffv, u);
    dfv_dq = jacobian(ffv, q);
    % Position Update Jacobians
    fp = p + dt*v + dt*dt/2*a + ((dt^3)/6)*jerk;
    ffp = p + dt*v + dt*dt/2*ffa + ((dt^3)/6)*ffjerk;
    dfp_dp = jacobian(ffp, p);
    dfp_dv = jacobian(ffp, v);
    dfp_dbsa = jacobian(ffp, bsa);
    dfp_dat = jacobian(ffp, at);
    dfp_dut_b = jacobian(ffp, ut_b);
    dfp_du = jacobian(ffp, u);
    dfp_dq = jacobian(ffp, q);
    % Bias Update Jacobian
    fut_b = ut_b;
    dfut_b_dp = jacobian(fut_b, p);
    dfut_b_dv = jacobian(fut_b, v);
    dfut_b_dbsa = jacobian(fut_b, bsa);
    dfut_b_dat = jacobian(fut_b, at);
    dfut_b_dut_b = jacobian(fut_b, ut_b);
    dfut_b_du = jacobian(fut_b, u);
    dfut_b_dq = jacobian(fut_b, q);
    %% State Transition Jacobian
    dFx = [dfp_dp,      dfp_dv,     dfp_dbsa,      dfp_dat,     dfp_dut_b;
           dfv_dp,      dfv_dv,     dfv_dbsa,      dfv_dat,     dfv_dut_b;
           dfbsa_dp,    dfbsa_dv,   dfbsa_dbsa,    dfbsa_dat,   dfbsa_dut_b;
           dfat_dp,     dfat_dv,    dfat_dbsa,     dfat_dat,   dfat_dut_b;
           dfut_b_dp,   dfut_b_dv,  dfut_b_dbsa,   dfut_b_dat, dfut_b_dut_b];
    %% Control Transition Jacobian
    dFQ = [dfp_du,      dfp_dq;
           dfv_du,      dfv_dq;
           dfbsa_du,    dfbsa_dq;
           dfat_du,    dfat_dq;
           dfut_b_du,  dfut_b_dq;];
    %% Measurement Update Calculations
    % Position Measurement Jacobians
    hp = pos;
    dHpos = sym(zeros(3,11));
    dHpos(1:3, 1:3) = eye(3);
    % Linear Acceleration Measurement Jacobians
    ha = acc;
    dHacc = sym(zeros(3,11));
    dHacc(1:3, 7:9) = jacobian(ha, acc);
    %% Eval parameters
    if(~isempty(varargin))
        for i = 1:2:length(varargin)
            switch varargin{i}
                case {'Tmz'}
                    Tmz = varargin{i+1}(1);
                case {'Kmz'}
                    Kmz = varargin{i+1}(1);
                case {'Td'}
                    Td1 = varargin{i+1}(1);
                    Td2 = varargin{i+1}(2);
                    Td3 = varargin{i+1}(3);
                case {'g'}
                    g1 = varargin{i+1}(1);
                    g2 = varargin{i+1}(2);
                    g3 = varargin{i+1}(3);
                case {'dt'}
                    dt = varargin{i+1}(1);
            end
        end
        fat_dot = subs(fat_dot);
        fat = subs(fat);
        fut_b = subs(fut_b);
        fbsa = subs(fbsa);
        fa = subs(fa);
        fjerk = subs(fjerk);
        fv = subs(fv);
        fp = subs(fp);
        dFx = subs(dFx);
        dFQ = subs(dFQ);
        hp = subs(hp);
        ha = subs(ha);
        dHpos = subs(dHpos);
        dHacc = subs(dHacc);
    end
end