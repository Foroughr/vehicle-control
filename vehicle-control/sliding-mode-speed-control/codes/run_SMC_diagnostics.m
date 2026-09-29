%% run_SMC_diagnostics
% SMC diagnostics under aerodynamic-drag uncertainty

clc;

%% Model and setup

modelName = 'sim4';

requiredVars = { ...
    't','t_end','dt','v_ref','Cd_sim', ...
    'Te_min','Te_max','phi_smc'};

for i = 1:numel(requiredVars)
    if ~evalin('base',sprintf('exist(''%s'',''var'')',requiredVars{i}))
        error('Variable "%s" is missing. Run vehicle_setup.m first.', ...
            requiredVars{i});
    end
end

t_ref       = evalin('base','t');
v_ref       = evalin('base','v_ref');
t_end       = evalin('base','t_end');
dt          = evalin('base','dt');
Cd_original = evalin('base','Cd_sim');

Te_min      = evalin('base','Te_min');
Te_max      = evalin('base','Te_max');
phi_smc     = evalin('base','phi_smc');

if evalin('base','exist(''lambda_smc'',''var'')')
    lambda_smc = evalin('base','lambda_smc');
else
    lambda_smc = NaN;
end

if evalin('base','exist(''k_smc'',''var'')')
    k_smc = evalin('base','k_smc');
elseif evalin('base','exist(''eta_smc'',''var'')')
    k_smc = evalin('base','eta_smc');
else
    k_smc = NaN;
end

t_ref = t_ref(:);
v_ref = v_ref(:);

%% Solver settings

if ~bdIsLoaded(modelName)
    load_system(modelName);
end

set_param(modelName,'FastRestart','off');

set_param(modelName, ...
    'SolverType','Fixed-step', ...
    'Solver','ode4', ...
    'FixedStep',num2str(dt), ...
    'StartTime','0', ...
    'StopTime',num2str(t_end));

%% Output folders

mainFolder = 'SMC_Project_Results';
stepFolder = fullfile(mainFolder,'SMC_Diagnostics');
figFolder  = fullfile(stepFolder,'Figures');
dataFolder = fullfile(stepFolder,'Data');
logFolder  = fullfile(stepFolder,'Logs');

folders = {mainFolder,stepFolder,figFolder,dataFolder,logFolder};

for i = 1:numel(folders)
    if ~exist(folders{i},'dir')
        mkdir(folders{i});
    end
end

oldFigures = dir(fullfile(figFolder,'*.png'));

for i = 1:numel(oldFigures)
    delete(fullfile(figFolder,oldFigures(i).name));
end

diaryFile = fullfile(logFolder,'SMC_Diagnostics_Command_Window.txt');

if exist(diaryFile,'file')
    delete(diaryFile);
end

diary(diaryFile);

fprintf('====================================================\n');
fprintf('SMC Diagnostics\n');
fprintf('Model          : %s\n',modelName);
fprintf('Solver         : ode4, fixed step %.4f s\n',dt);
fprintf('lambda_smc     : %.6f 1/s\n',lambda_smc);
fprintf('k_smc          : %.6f m/s^2\n',k_smc);
fprintf('phi_smc        : %.6f m/s\n',phi_smc);
fprintf('====================================================\n');

%% Simulation cases

Cd_cases = [0.2 0.3 0.4];
results = repmat(struct(),numel(Cd_cases),1);

restoreCd = onCleanup(@() assignin('base','Cd_sim',Cd_original));

for iCase = 1:numel(Cd_cases)

    Cd = Cd_cases(iCase);
    tag = strrep(sprintf('Cd_%.1f',Cd),'.','p');

    fprintf('\nRunning simulation for Cd = %.2f ...\n',Cd);

    assignin('base','Cd_sim',Cd);
    clearLoggedSignals();

    simOut = sim(modelName,'StopTime',num2str(t_end));
    tout = getSimulationTime(simOut);

    [t_v,v]   = readSignal(simOut,'sim_v_vehicle_mps',tout);
    [t_T,Te]  = readSignal(simOut,'sim_Te_Nm',tout);

    v_ref_sim = interp1(t_ref,v_ref,t_v,'linear','extrap');
    e = v - v_ref_sim;

    try
        [t_s,s] = readSignal(simOut,'sim_s',tout);
        hasSliding = true;
    catch
        t_s = t_v;
        s = nan(size(t_s));
        hasSliding = false;
    end

    %% Error metrics

    Tsim = t_v(end)-t_v(1);

    IAE  = trapz(t_v,abs(e));
    MAE  = IAE/Tsim;
    RMSE = sqrt(trapz(t_v,e.^2)/Tsim);

    MaxAbsError = max(abs(e));
    FinalError  = e(end);

    %% Torque metrics

    MinTorque = min(Te);
    MaxTorque = max(Te);

    satFlag = ...
        (Te >= Te_max-1e-6) | ...
        (Te <= Te_min+1e-6);

    TorqueSaturation = ...
        100*trapz(t_T,double(satFlag))/(t_T(end)-t_T(1));

    TV_Torque = sum(abs(diff(Te)));

    dt_T = diff(t_T);
    dTe  = diff(Te)./dt_T;
    t_rate = (t_T(1:end-1)+t_T(2:end))/2;

    if numel(t_rate) > 1
        RMS_TorqueRate = sqrt( ...
            trapz(t_rate,dTe.^2)/(t_rate(end)-t_rate(1)));
        MaxTorqueRate = max(abs(dTe));
    else
        RMS_TorqueRate = NaN;
        MaxTorqueRate = NaN;
    end

    %% Sliding metrics

    if hasSliding
        s_phi = s/phi_smc;

        MaxAbsS        = max(abs(s));
        MaxAbsSOverPhi = max(abs(s_phi));
        FinalS         = s(end);
        FinalSOverPhi  = s_phi(end);
    else
        s_phi = nan(size(s));

        MaxAbsS        = NaN;
        MaxAbsSOverPhi = NaN;
        FinalS         = NaN;
        FinalSOverPhi  = NaN;
    end

    %% Store results

    results(iCase).Cd = Cd;

    results(iCase).t_v = t_v;
    results(iCase).v_ref_kmh = v_ref_sim*3.6;
    results(iCase).v_kmh = v*3.6;
    results(iCase).e = e;

    results(iCase).t_T = t_T;
    results(iCase).Te = Te;
    results(iCase).t_rate = t_rate;
    results(iCase).dTe = dTe;

    results(iCase).t_s = t_s;
    results(iCase).s = s;
    results(iCase).s_phi = s_phi;
    results(iCase).hasSliding = hasSliding;

    results(iCase).RMSE = RMSE;
    results(iCase).MAE = MAE;
    results(iCase).MaxAbsError = MaxAbsError;
    results(iCase).FinalError = FinalError;
    results(iCase).IAE = IAE;

    results(iCase).MinTorque = MinTorque;
    results(iCase).MaxTorque = MaxTorque;
    results(iCase).TorqueSaturation = TorqueSaturation;

    results(iCase).TV_Torque = TV_Torque;
    results(iCase).RMS_TorqueRate = RMS_TorqueRate;
    results(iCase).MaxTorqueRate = MaxTorqueRate;

    results(iCase).MaxAbsS = MaxAbsS;
    results(iCase).MaxAbsSOverPhi = MaxAbsSOverPhi;
    results(iCase).FinalS = FinalS;
    results(iCase).FinalSOverPhi = FinalSOverPhi;

    %% Command-window summary

    fprintf('Cd = %.2f results:\n',Cd);
    fprintf('  RMSE error              = %.9e m/s\n',RMSE);
    fprintf('  MAE error               = %.9e m/s\n',MAE);
    fprintf('  Max absolute error      = %.9e m/s\n',MaxAbsError);
    fprintf('  Final error             = %.9e m/s\n',FinalError);
    fprintf('  IAE                     = %.9e m\n',IAE);
    fprintf('  Min torque              = %.6f Nm\n',MinTorque);
    fprintf('  Max torque              = %.6f Nm\n',MaxTorque);
    fprintf('  Torque saturation       = %.6f %%\n',TorqueSaturation);
    fprintf('  Torque total variation  = %.9e Nm\n',TV_Torque);
    fprintf('  RMS torque rate         = %.9e Nm/s\n',RMS_TorqueRate);
    fprintf('  Max torque rate         = %.9e Nm/s\n',MaxTorqueRate);
    fprintf('  Max |s|                 = %.9e\n',MaxAbsS);
    fprintf('  Max |s/phi|             = %.9e\n',MaxAbsSOverPhi);

    %% Individual figures

    fig = figure('Color','w');
    plot(t_v,v_ref_sim*3.6,'--','LineWidth',2); hold on;
    plot(t_v,v*3.6,'LineWidth',2);
    formatAxes();
    xlabel('Time [s]');
    ylabel('Speed [km/h]');
    title(sprintf('SMC Speed Tracking, C_d = %.2f',Cd));
    legend('Reference speed','Vehicle speed','Location','best');
    saveFigure(fig,figFolder,['Fig_Speed_Tracking_',tag]);

    fig = figure('Color','w');
    plot(t_v,e,'LineWidth',2);
    formatAxes();
    xlabel('Time [s]');
    ylabel('Speed Error [m/s]');
    title(sprintf('SMC Speed Tracking Error, C_d = %.2f',Cd));
    saveFigure(fig,figFolder,['Fig_Error_',tag]);

    fig = figure('Color','w');
    plot(t_T,Te,'LineWidth',2); hold on;
    yline(Te_max,'--','Upper torque limit');
    formatAxes();
    xlabel('Time [s]');
    ylabel('Engine Torque T_e [Nm]');
    title(sprintf('SMC Engine Torque, C_d = %.2f',Cd));
    saveFigure(fig,figFolder,['Fig_Torque_',tag]);

    fig = figure('Color','w');
    plot(t_rate,dTe,'LineWidth',1.5);
    formatAxes();
    xlabel('Time [s]');
    ylabel('Torque Rate [Nm/s]');
    title(sprintf('Engine Torque Rate, C_d = %.2f',Cd));
    saveFigure(fig,figFolder,['Fig_Torque_Rate_',tag]);

    if hasSliding

        fig = figure('Color','w');
        plot(t_s,s,'LineWidth',2);
        formatAxes();
        xlabel('Time [s]');
        ylabel('Sliding Surface s');
        title(sprintf('Sliding Surface, C_d = %.2f',Cd));
        saveFigure(fig,figFolder,['Fig_Sliding_Surface_',tag]);

        fig = figure('Color','w');
        plot(t_s,s_phi,'LineWidth',2); hold on;
        yline(1,'--');
        yline(-1,'--');
        yline(0,':');
        formatAxes();
        xlabel('Time [s]');
        ylabel('Normalized Sliding Variable s/\phi');
        title(sprintf('Normalized Sliding Variable, C_d = %.2f',Cd));
        legend('s/\phi','+1 boundary','-1 boundary','s = 0', ...
            'Location','best');
        saveFigure(fig,figFolder, ...
            ['Fig_Normalized_Sliding_Surface_',tag]);

    end

end

%% Comparison figures

fig = figure('Color','w');

plot(results(2).t_v,results(2).v_ref_kmh,'k--','LineWidth',2);
hold on;

for i = 1:numel(results)
    plot(results(i).t_v,results(i).v_kmh,'LineWidth',2);
end

formatAxes();
xlabel('Time [s]');
ylabel('Speed [km/h]');
title('SMC Robustness: Speed Tracking under C_d Uncertainty');
legend('Reference','C_d = 0.2','C_d = 0.3','C_d = 0.4', ...
    'Location','best');
saveFigure(fig,figFolder,'Fig_Robustness_Speed_Comparison');

fig = figure('Color','w');
hold on;

for i = 1:numel(results)
    plot(results(i).t_v,results(i).e,'LineWidth',2);
end

formatAxes();
xlabel('Time [s]');
ylabel('Speed Error [m/s]');
title('SMC Robustness: Speed Error under C_d Uncertainty');
legend('C_d = 0.2','C_d = 0.3','C_d = 0.4','Location','best');
saveFigure(fig,figFolder,'Fig_Robustness_Error_Comparison');

fig = figure('Color','w');
hold on;

for i = 1:numel(results)
    plot(results(i).t_T,results(i).Te,'LineWidth',2);
end

formatAxes();
xlabel('Time [s]');
ylabel('Engine Torque T_e [Nm]');
title('SMC Robustness: Engine Torque under C_d Uncertainty');
legend('C_d = 0.2','C_d = 0.3','C_d = 0.4','Location','best');
saveFigure(fig,figFolder,'Fig_Robustness_Torque_Comparison');

fig = figure('Color','w');
hold on;

for i = 1:numel(results)
    plot(results(i).t_rate,results(i).dTe,'LineWidth',1.5);
end

formatAxes();
xlabel('Time [s]');
ylabel('Torque Rate [Nm/s]');
title('SMC Robustness: Torque Rate under C_d Uncertainty');
legend('C_d = 0.2','C_d = 0.3','C_d = 0.4','Location','best');
saveFigure(fig,figFolder,'Fig_Robustness_Torque_Rate_Comparison');

if all([results.hasSliding])

    fig = figure('Color','w');
    hold on;

    for i = 1:numel(results)
        plot(results(i).t_s,results(i).s_phi,'LineWidth',2);
    end

    yline(1,'--');
    yline(-1,'--');
    yline(0,':');

    formatAxes();
    xlabel('Time [s]');
    ylabel('Normalized Sliding Variable s/\phi');
    title('SMC Boundary Layer Response under C_d Uncertainty');

    legend('C_d = 0.2','C_d = 0.3','C_d = 0.4', ...
        '+1 boundary','-1 boundary','s = 0','Location','best');

    saveFigure(fig,figFolder, ...
        'Fig_Robustness_Normalized_Sliding_Surface');

end

%% Save metrics

Cd = [results.Cd]';
RMSE_mps = [results.RMSE]';
MAE_mps = [results.MAE]';
MaxAbsError_mps = [results.MaxAbsError]';
FinalError_mps = [results.FinalError]';
IAE_m = [results.IAE]';

MinTorque_Nm = [results.MinTorque]';
MaxTorque_Nm = [results.MaxTorque]';
TorqueSaturationPercent = [results.TorqueSaturation]';

TV_Torque_Nm = [results.TV_Torque]';
RMS_TorqueRate_Nmps = [results.RMS_TorqueRate]';
MaxAbsTorqueRate_Nmps = [results.MaxTorqueRate]';

MaxAbsS = [results.MaxAbsS]';
MaxAbsSOverPhi = [results.MaxAbsSOverPhi]';
FinalS = [results.FinalS]';
FinalSOverPhi = [results.FinalSOverPhi]';

metricsTable = table( ...
    Cd,RMSE_mps,MAE_mps,MaxAbsError_mps,FinalError_mps,IAE_m, ...
    MinTorque_Nm,MaxTorque_Nm,TorqueSaturationPercent, ...
    TV_Torque_Nm,RMS_TorqueRate_Nmps,MaxAbsTorqueRate_Nmps, ...
    MaxAbsS,MaxAbsSOverPhi,FinalS,FinalSOverPhi);

writetable(metricsTable, ...
    fullfile(dataFolder,'SMC_Performance_Metrics.csv'));

save(fullfile(dataFolder,'SMC_Diagnostics_Workspace.mat'), ...
    'results','metricsTable','Cd_cases');

%% Save summary

summaryFile = fullfile(dataFolder,'SMC_Diagnostics_Summary.txt');
fid = fopen(summaryFile,'w');

fprintf(fid,'SMC Diagnostics Summary\n');
fprintf(fid,'=======================\n\n');
fprintf(fid,'Solver: ode4, fixed step %.6f s\n',dt);
fprintf(fid,'lambda_smc = %.6f 1/s\n',lambda_smc);
fprintf(fid,'k_smc      = %.6f m/s^2\n',k_smc);
fprintf(fid,'phi_smc    = %.6f m/s\n\n',phi_smc);

for i = 1:numel(results)

    fprintf(fid,'Cd = %.2f\n',results(i).Cd);
    fprintf(fid,'RMSE error              = %.9e m/s\n',results(i).RMSE);
    fprintf(fid,'MAE error               = %.9e m/s\n',results(i).MAE);
    fprintf(fid,'Max absolute error      = %.9e m/s\n', ...
        results(i).MaxAbsError);
    fprintf(fid,'Final error             = %.9e m/s\n', ...
        results(i).FinalError);
    fprintf(fid,'IAE                     = %.9e m\n',results(i).IAE);
    fprintf(fid,'Min torque              = %.9f Nm\n', ...
        results(i).MinTorque);
    fprintf(fid,'Max torque              = %.9f Nm\n', ...
        results(i).MaxTorque);
    fprintf(fid,'Torque saturation       = %.9f %%\n', ...
        results(i).TorqueSaturation);
    fprintf(fid,'Torque total variation  = %.9e Nm\n', ...
        results(i).TV_Torque);
    fprintf(fid,'RMS torque rate         = %.9e Nm/s\n', ...
        results(i).RMS_TorqueRate);
    fprintf(fid,'Max torque rate         = %.9e Nm/s\n', ...
        results(i).MaxTorqueRate);
    fprintf(fid,'Max |s|                 = %.9e\n',results(i).MaxAbsS);
    fprintf(fid,'Max |s/phi|             = %.9e\n\n', ...
        results(i).MaxAbsSOverPhi);

end

fclose(fid);

%% Save solver settings

fid = fopen(fullfile(dataFolder,'SMC_Solver_Settings.txt'),'w');
fprintf(fid,'Model: %s\n',modelName);
fprintf(fid,'Solver type: Fixed-step\n');
fprintf(fid,'Solver: ode4\n');
fprintf(fid,'Fixed-step size: %.6f s\n',dt);
fprintf(fid,'Start time: 0 s\n');
fprintf(fid,'Stop time: %.6f s\n',t_end);
fclose(fid);

clear restoreCd;

fprintf('\n====================================================\n');
fprintf('SMC Diagnostics Finished\n');
fprintf('Figures saved in:\n%s\n',figFolder);
fprintf('Data saved in:\n%s\n',dataFolder);
fprintf('====================================================\n');

diary off;

%% Local functions

function t = getSimulationTime(simOut)

    try
        t = simOut.get('tout');
    catch
        t = evalin('base','tout');
    end

    t = t(:);

end

function [t,y] = readSignal(simOut,name,tFallback)

    try
        raw = simOut.get(name);
    catch
        raw = evalin('base',name);
    end

    if isa(raw,'Simulink.SimulationData.Dataset')
        raw = raw.getElement(1);
    end

    if isa(raw,'Simulink.SimulationData.Signal')
        raw = raw.Values;
    end

    if isa(raw,'timeseries')

        t = raw.Time(:);
        y = squeeze(raw.Data);
        y = y(:);

    elseif isstruct(raw) && ...
            isfield(raw,'time') && ...
            isfield(raw,'signals')

        t = raw.time(:);
        y = squeeze(raw.signals.values);
        y = y(:);

    elseif isstruct(raw) && isfield(raw,'signals')

        y = squeeze(raw.signals.values);
        y = y(:);
        t = tFallback(:);

    elseif isnumeric(raw) && ...
            ~isvector(raw) && ...
            size(raw,2) >= 2

        t = raw(:,1);
        y = raw(:,end);

    elseif isnumeric(raw)

        y = raw(:);
        t = tFallback(:);

    else
        error('Unsupported format for signal "%s".',name);
    end

    if numel(t) ~= numel(y)
        error('Time and data lengths do not match for signal "%s".',name);
    end

    valid = isfinite(t) & isfinite(y);
    t = t(valid);
    y = y(valid);

    [t,index] = unique(t,'stable');
    y = y(index);

end

function clearLoggedSignals()

    names = { ...
        'sim_v_vehicle_mps', ...
        'sim_speed_error_mps', ...
        'sim_Te_Nm', ...
        'sim_s'};

    for i = 1:numel(names)
        if evalin('base',sprintf('exist(''%s'',''var'')',names{i}))
            evalin('base',sprintf('clear %s',names{i}));
        end
    end

end

function formatAxes()

    grid on;
    box on;

    set(gca, ...
        'FontName','Times New Roman', ...
        'FontSize',12);

end

function saveFigure(fig,folder,name)

    set(fig,'Color','w');
    drawnow;

    print(fig, ...
        fullfile(folder,[name,'.png']), ...
        '-dpng','-r300');

    close(fig);

end