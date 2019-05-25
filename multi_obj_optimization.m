% clear all;
close all;
clc;

addpath('auxiliary','control','controllers','dynamics','physical_params',...
    'config','plot','trajectories','trajectory_planning','optimization_functions');

% Pick your config function
% [physics_p, control_p, traj_p, plot_p] = nested2d_config();
% [physics_p, control_p, traj_p, plot_p] = smc2d_config();
% [physics_p, control_p, traj_p, plot_p] = nested2d_slung_config();
% [physics_p, control_p, traj_p, plot_p] = smc2d_slung_config();
% [physics_p, control_p, traj_p, plot_p] = smc3d_slung_config();
% [physics_p, control_p, traj_p, plot_p] = smc2d_slung_config2();
[physics_p, control_p, traj_p, sim_p, plot_p] = full_smc2d_slung_config();
% optimization_function = @nested2d_xzerror_optfun;
% optimization_function = @smc2d_xzerror_optfun;
% optimization_function = @smc3d_multiobj_optfun;
% optimization_function = @smc2d_multiobj_optfun2;
optimization_function = @full_smc2d_multiobj_optfun;

% warnId = 'MATLAB:ode45:IntegrationTolNotMet';
% warnstate = warning('warning', warnId);

options = optimoptions('gamultiobj','Display','iter','UseParallel',true,'OutputFcn',@(op,s,fl)outputfun(op,s,fl),...%,seed),...
    'PopulationSize',60,'MaxGenerations',100,'MaxStallGenerations',100,...
    'PlotFcn',{@gaplotpareto,@gaplotstopping,@gaplotparetodistance});%'gaplotbestf' | 'gaplotbestindiv'

lb = ones(1,control_p.n_input)*.01;
ub = ones(1,control_p.n_input)*100;

% lb(4:5) = -100;

[xx,fval,exitflag,output,population,scores] = gamultiobj(@(x,nvar)optimization_function(x,physics_p,control_p,traj_p,sim_p),...
    control_p.n_input,[],[],[],[],lb,ub,[],options);

