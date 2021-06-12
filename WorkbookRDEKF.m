function [fot_dot, fot, fo, fw, fq, fum_bias, dFx, dFQ, hq, hw, dHangles, dHgyro] = WorkbookRDEKF(varargin)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% OVERVIEW %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Estimates [q w o fmt u_mbias]
    % Inputs [MRc MPc]
    % Measurements [quat gyro]
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Define Estimates
    q = sym('q', [4 1], 'real'); % drone orientation in quaternion
    w = sym('w', [3 1], 'real'); % drone angular velocity in body frame
    o = sym('o', [2 1], 'real'); % drone angular acceleration in body frame
    ot = sym('ot', [2 1], 'real'); % Body frame generated angular acceleration in case of no drag
    um_bias = sym('um_bias', [2 1], 'real'); % Body frame generated angle command bias
    %% Define Inputs
    um = sym('um', [2 1],'real'); % Roll, and Pitch Angle Commands
    %% Define Measurements
    quat = sym('quat', [4 1], 'real'); % Angles Measurements in Quaternions
    gyro = sym('gyro', [2 1], 'real'); % Gyroscope Measurement in Body Frame
    %% Define Conversion & Intermediate Variables
    ot_dot = sym('ot_dot', [2 1], 'real');
    %% Define Parameters
    Tm = sym('Tma', [2 1], 'real'); % Moments time constants (frameless) [Tm_r, Tm_p, Tm_y]
    Km = sym('Kma', [2 1], 'real'); % Thrust gain (frameless) [Km_r, Km_p, Km_y]
    Td = sym('Tda', [2 1], 'real'); % Rotation Drag time constant in body frame [tdr_r, tdr_p];
    dt = sym('dt', 'real'); % step time
    %% Angular Calculations
    % Generated Angular Acceleration Update
    fot_dot = ((um - um_bias).*Km./Td - ot).*1./Tm;
    fot = ot + ot_dot*dt;
    ffot = ot + fot_dot*dt;
    dfa_aux_dq = jacobian(ffot, q);
    dfa_aux_dw = jacobian(ffot, w);
    dfa_aux_do = jacobian(ffot, o);
    dfa_aux_da_aux = jacobian(ffot, ot);
    dfa_aux_dum_bias = jacobian(ffot, um_bias);
    dfa_aux_dua = jacobian(ffot, um);
    % Angular Acceleration Update Jacobians
    fo = o + dt*(ot_dot - 1./Td.*o);
    ffo = o + dt*(fot_dot - 1./Td.*o);
    dfo_dq = jacobian(ffo, q);
    dfo_dw = jacobian(ffo, w);
    dfo_do = jacobian(ffo, o);
    dfo_da_aux = jacobian(ffo, ot);
    dfo_dum_bias = jacobian(ffo, um_bias);
    dfo_dua = jacobian(ffo, um);
    % Angular Velocity Update Jacobians
    fw = w + [(dt*o + dt*dt/2*(ot_dot - 1./Td.*o)); 0]; % o' for yaw is ignored
    ffw = w + [(dt*o + dt*dt/2*(fot_dot - 1./Td.*o)); 0]; % o' for yaw is ignored
    dfw_dq = jacobian(ffw, q);
    dfw_dw = jacobian(ffw, w);
    dfw_do = jacobian(ffw, o);
    dfw_da_aux = jacobian(ffw, ot);
    dfw_dum_bias = jacobian(ffw, um_bias);
    dfw_dua = jacobian(ffw, um);
    % Quaternion Update Jacobians
    q_dot = multiplyQuat(0.5*q, [0;w]); % its effect is in order of 1e-4 
    %q_ddot = 0.5*convertQuatToLeftMatrixForm(q_dot)*[0;w] + 0.5*convertQuatToLeftMatrixForm(q)*[0;o]; % its effect is in order of 1e-6
    q_ddot = [0 0 0 0]';
    fq = q + q_dot*dt + dt^2/2 * q_ddot;
    dfq_dq = jacobian(fq,q);
    dfq_dw = jacobian(fq,w);
    dfq_do = jacobian(fq,o);
    dfq_da_aux = jacobian(fq, ot);
    dfq_dum_bias = jacobian(fq, um_bias);
    dfq_dua = jacobian(fq, um);
    % Bias Update Jacobians
    fum_bias = um_bias;
    dfum_bias_dq = jacobian(fum_bias, q);
    dfum_bias_dw = jacobian(fum_bias, w);
    dfum_bias_do = jacobian(fum_bias, o);
    dfum_bias_da_aux = jacobian(fum_bias, ot);
    dfum_bias_dum_bias = jacobian(fum_bias, um_bias);
    dfum_bias_dua = jacobian(fum_bias, um);
    %% State Transition Jacobian
    dFx = [dfq_dq,         dfq_dw,         dfq_do,         dfq_da_aux,         dfq_dum_bias;
            dfw_dq          dfw_dw,         dfw_do,         dfw_da_aux,         dfw_dum_bias;
            dfo_dq,         dfo_dw,         dfo_do,         dfo_da_aux,         dfo_dum_bias;
            dfa_aux_dq      dfa_aux_dw,     dfa_aux_do,     dfa_aux_da_aux,     dfa_aux_dum_bias;
            dfum_bias_dq    dfum_bias_dw,   dfum_bias_do,   dfum_bias_da_aux,   dfum_bias_dum_bias;];
    %% Control Transition Jacobian
    dFQ = [dfq_dua;
            dfw_dua;
            dfo_dua;
            dfa_aux_dua;
            dfum_bias_dua;];
    %% Measurement Update Calculations
    % Angle Measurement Jacobian
    hq = quat;
    dHangles = sym(zeros(4,16));
    dHangles(1:4, 1:4) = eye(4);
    % Angular Velocity Measurement Jacobian
    hw = gyro;
    dHgyro = sym(zeros(3,16));
    dHgyro(1:3, 5:7) = eye(3);
    %% Eval parameters
    if(~isempty(varargin))
        for i = 1:2:length(varargin)
            switch varargin{i}
                case {'Tm'}
                    Tm1 = varargin{i+1}(1);
                    Tm2 = varargin{i+1}(2);
                case {'Km'}
                    Km1 = varargin{i+1}(1);
                    Km2 = varargin{i+1}(2);
                case {'Td'}
                    Td1 = varargin{i+1}(1);
                    Td2 = varargin{i+1}(2);
                case {'dt'}
                    dt = varargin{i+1}(1);
            end
        end
        fot_dot = subs(fot_dot);
        fot = subs(fot);
        fum_bias = subs(fum_bias);
        fo = subs(fo);
        fw = subs(fw);
        fq = subs(fq);
        dFx = subs(dFx);
        dFQ = subs(dFQ);
        hq = subs(hq);
        hw = subs(hw);
        dHangles = subs(dHangles);
        dHgyro = subs(dHgyro);
    end
end