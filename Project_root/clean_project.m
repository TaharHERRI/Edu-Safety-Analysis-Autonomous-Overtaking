%% Clean project directory
% clearvars
close all
clc

% Clear SDI
Simulink.sdi.clear

% Delete Simulink cache
if exist('slprj','dir')
    rmdir('slprj','s');
end
delete('*.slxc');

% Delete autosave files
delete('*.asv');

disp('Project cleaned successfully.');
