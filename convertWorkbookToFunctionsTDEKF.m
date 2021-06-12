%% 
% This line runs the workbook for the generation of TDEKF model from the
% identified parameters. The parameters are the result from the DNN-MRFT
% test. Tmz is the identified actuation time cosntant for the z channel (T1 
% from the DNN-MRFT). Kmz is the identified actuation gain. Td(1) is the
% identified drag time constant for the x channel, Td(2) for the y channel
% and Td(3) for the z channel (T2 from the DNN-MRFT). g is the gravity
% vector at the test location. dt is the sought after time constant.

[fat, fbsa, fa, fv, fp, fut_b, fR, fat_dot, fjerk, dFx, dFQ, hp, ha, dHpos, dHacc] = ...
    WorkbookTDEKF('Tmz',0.017707, 'Kmz', 0.294861, 'Td', [1.1629 1.1629 0.579374], 'g', [0;0;9.78909], 'dt', 0.001);
%%
% These section of the code generates fuunctions for the RDEKF model. The
% function files are used to update the RDEKF block in the simulink file
% "RDEKF.slx"
mkdir TDEKF_files

func_dFx = matlabFunction(dFx,'File', 'TDEKF_files/fdFx');
func_dFQ = matlabFunction(dFQ,'File', 'TDEKF_files/fdFQ');
func_fat_dot = matlabFunction(fat_dot,'File', 'TDEKF_files/fat_dot');
func_fat = matlabFunction(fat,'File', 'TDEKF_files/fat');
func_fbsa = matlabFunction(fbsa,'File', 'TDEKF_files/fbsa');
func_fa = matlabFunction(fa,'File', 'TDEKF_files/fa');
func_fv = matlabFunction(fv,'File', 'TDEKF_files/fv');
func_fjerk = matlabFunction(fjerk,'File', 'TDEKF_files/fjerk');
func_fp = matlabFunction(fp,'File', 'TDEKF_files/fp');