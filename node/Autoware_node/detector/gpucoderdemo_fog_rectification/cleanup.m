if ~exist('currentfigures') || isempty(currentFigures), currentFigures = []; end;
close(setdiff(findall(0, 'type', 'figure'), currentFigures))
clear mex
delete *.mexa64
[~,~,~] = rmdir('/home/handai/matlab/node/Autoware_node/ACF/gpucoderdemo_fog_rectification/codegen','s');
clear('/home/handai/matlab/node/Autoware_node/ACF/gpucoderdemo_fog_rectification/fog_rectification.m')
delete('/home/handai/matlab/node/Autoware_node/ACF/gpucoderdemo_fog_rectification/fog_rectification.m')
delete('/home/handai/matlab/node/Autoware_node/ACF/gpucoderdemo_fog_rectification/foggyInput.png')
clear
load old_workspace
delete old_workspace.mat
delete('/home/handai/matlab/node/Autoware_node/ACF/gpucoderdemo_fog_rectification/cleanup.m')
cd('/home/handai/matlab/node/Autoware_node/ACF')
rmdir('/home/handai/matlab/node/Autoware_node/ACF/gpucoderdemo_fog_rectification','s');
