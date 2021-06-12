%% 
% This line runs the workbook for the generation of RDEKF model from the
% identified parameters. The parameters are the result from the DNN-MRFT
% test. Tm is the identified actuation time cosntant for the roll and pitch
% channels (T1 from the DNN-MRFT). Km is the identified actuation gain. Td
% is the identified drag time constant for the roll and pitch channels ( T2
% from the DNN-MRFT). dt is the sought after time constant.

[fot_dot, fot, fo, fw, fq, fum_bias, dFx, dFQ, hq, hw, dHangles, dHgyro] = ...
                WorkbookRDEKF('Tm',[0.064 0.064], 'Km', [72.6454 75.8460 ], 'Td', [0.2494 0.2494], 'dt', 0.001);
%%
% These section of the code generates fuunctions for the RDEKF model. The
% function files are used to update the RDEKF block in the simulink file
% "RDEKF.slx"
mkdir RDEKF_files

func_dFxa = matlabFunction(dFx,'File', 'RDEKF_files/fdFx_rot');
func_dFQa = matlabFunction(dFQ,'File', 'RDEKF_files/fdFQ_rot');
func_fa_aux_dot = matlabFunction(fot_dot,'File', 'RDEKF_files/fot_dot');
func_fa_aux = matlabFunction(fot,'File', 'RDEKF_files/fot');
func_fo = matlabFunction(fo,'File', 'RDEKF_files/fo');
func_fw = matlabFunction(fw,'File', 'RDEKF_files/fw');
func_fq = matlabFunction(fq,'File', 'RDEKF_files/fq');