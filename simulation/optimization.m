% clear all;
close all;
clc;

addpath('auxiliary','control','controllers','dynamics','physics',...
    'config','plot','trajectories','trajectory_planning','optimization_functions');

% Pick your config function
% [physics_p, control_p, traj_p, sim_p, plot_p] = shaped_flat_smc3d_slung_config();
% [physics_p, control_p, traj_p, sim_p, plot_p] = smc3d_slung_config();
[physics_p, control_p, traj_p, sim_p, plot_p] = shaped_smc3d_slung_config();
% optimization_function = @nested2d_xzerror_optfun;
% optimization_function = @smc2d_xzerror_optfun;
optimization_function = @final_smc3d_optfun;
% optimization_function = @smc2d_xzerror_optfun2; 

% warnId = 'MATLAB:ode45:IntegrationTolNotMet';
% warnstate = warning('warning', warnId);

options = optimoptions('ga','Display','iter','UseParallel',true,'OutputFcn',@(op,s,fl)outputfun(op,s,fl),...%,seed),...
    'PopulationSize',100,'MaxGenerations',100,'MaxStallGenerations',100,...
    'PlotFcn',{@gaplotbestf,@gaplotstopping,@gaplotbestindiv,@plot_opt_vars});%'gaplotbestf' | 'gaplotbestindiv'

n_input = 4;
lower = .1;
upper = 10.;
lb = ones(1,n_input)*lower;
ub = ones(1,n_input)*upper;

% lb([10,12]) = -upper;
% ub([10,12]) = -lower;

[xx,fval,exitflag,output,population,scores] = ga(@(x,nvar)optimization_function(x,physics_p,control_p,traj_p,sim_p),...
    n_input,[],[],[],[],lb,ub,[],options);

