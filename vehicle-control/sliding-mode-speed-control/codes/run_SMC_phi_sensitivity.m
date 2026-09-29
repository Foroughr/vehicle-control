%% run_SMC_phi_sensitivity
% Boundary-layer sensitivity analysis

clc;

%% Model and setup

modelName = 'sim4';

requiredVars = { ...
    't','t_end','dt','v_ref','Cd_sim', ...
    'phi_smc','Te_min','Te_max'};

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

Cd_original  = evalin('base','Cd_sim');
phi_original = evalin('base','phi_smc');

Te_min = evalin('base','Te_min');
Te_max = evalin('base','Te_max');

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
stepFolder = fullfile(mainFolder,'SMC_Phi_Sensitivity');
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

diaryFile = fullfile(logFolder,'SMC_phi_sensitivity_log.txt');

if exist(diaryFile,'file')
    delete(diaryFile);
end

diary(diaryFile);

%% Sensitivity cases

Cd_cases  = [0.2 0.3 0.4];
phi_cases = [0.05 0.075 0.10 0.15 0.20];

results = table();
allRuns = struct();
runNumber = 0;

restoreParameters = onCleanup(@() restoreValues( ...
    Cd_original,phi_original));

fprintf('====================================================\n');
fprintf('SMC Phi Sensitivity Analysis\n');
fprintf('Solver: ode4, fixed step %.4f s\n',dt);
fprintf('====================================================\n');

%% Simulations

for iPhi = 1:numel(phi_cases)

    phi = phi_cases(iPhi);
    assignin('base','phi_smc',phi);

    fprintf('\nTesting phi_smc = %.4f m/s\n',phi);

    for iCd = 1:numel(Cd_cases)

        Cd = Cd_cases(iCd);
        assignin('base','Cd_sim',Cd);

        fprintf('  Running Cd = %.2f ...\n',Cd);

        clearLoggedSignals();

        simOut = sim(modelName,'StopTime',num2str(t_end));
        tout = getSimulationTime(simOut);

        [t_v,v]  = readSignal(simOut,'sim_v_vehicle_mps',tout);
        [t_T,Te] = readSignal(simOut,'sim_Te_Nm',tout);

        v_ref_sim = interp1(t_ref,v_ref,t_v,'linear','extrap');
        e = v-v_ref_sim;

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
        FinalError = e(end);

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
        dTe = diff(Te)./dt_T;
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
            s_phi = s/phi;

            MaxAbsS = max(abs(s));
            MaxAbsSOverPhi = max(abs(s_phi));
            FinalS = s(end);
            FinalSOverPhi = s_phi(end);
        else
            s_phi = nan(size(s));

            MaxAbsS = NaN;
            MaxAbsSOverPhi = NaN;
            FinalS = NaN;
            FinalSOverPhi = NaN;
        end

        %% Results table

        newRow = table( ...
            phi,Cd,RMSE,MAE,MaxAbsError,FinalError,IAE, ...
            MinTorque,MaxTorque,TorqueSaturation, ...
            TV_Torque,RMS_TorqueRate,MaxTorqueRate, ...
            MaxAbsS,MaxAbsSOverPhi,FinalS,FinalSOverPhi, ...
            'VariableNames',{ ...
            'phi_smc','Cd','RMSE_error_mps','MAE_error_mps', ...
            'MaxAbsError_mps','FinalError_mps','IAE_m', ...
            'MinTorque_Nm','MaxTorque_Nm', ...
            'TorqueSaturation_percent','TV_Torque_Nm', ...
            'RMS_TorqueRate_Nmps','MaxTorqueRate_Nmps', ...
            'MaxAbs_s','MaxAbs_s_over_phi', ...
            'Final_s','Final_s_over_phi'});

        results = [results;newRow]; %#ok<AGROW>

        %% Save histories

        runNumber = runNumber+1;

        allRuns(runNumber).phi_smc = phi;
        allRuns(runNumber).Cd = Cd;

        allRuns(runNumber).t_v = t_v;
        allRuns(runNumber).v_ref = v_ref_sim;
        allRuns(runNumber).v = v;
        allRuns(runNumber).e = e;

        allRuns(runNumber).t_T = t_T;
        allRuns(runNumber).Te = Te;
        allRuns(runNumber).t_rate = t_rate;
        allRuns(runNumber).dTe = dTe;

        allRuns(runNumber).t_s = t_s;
        allRuns(runNumber).s = s;
        allRuns(runNumber).s_phi = s_phi;

        fprintf('    RMSE               = %.9e m/s\n',RMSE);
        fprintf('    MAE                = %.9e m/s\n',MAE);
        fprintf('    Max error          = %.9e m/s\n',MaxAbsError);
        fprintf('    IAE                = %.9e m\n',IAE);
        fprintf('    Max torque         = %.6f Nm\n',MaxTorque);
        fprintf('    Torque saturation  = %.6f %%\n',TorqueSaturation);
        fprintf('    Torque variation   = %.9e Nm\n',TV_Torque);
        fprintf('    RMS torque rate    = %.9e Nm/s\n',RMS_TorqueRate);
        fprintf('    Max |s/phi|        = %.9e\n',MaxAbsSOverPhi);

    end

end

%% Save data

writetable(results, ...
    fullfile(dataFolder,'SMC_phi_sensitivity_metrics.csv'));

save(fullfile(dataFolder,'SMC_phi_sensitivity_all_runs.mat'), ...
    'results','allRuns','Cd_cases','phi_cases');

%% Sensitivity figures

plotMetric(results,Cd_cases, ...
    'RMSE_error_mps', ...
    'RMSE Tracking Error [m/s]', ...
    'SMC Boundary-Layer Sensitivity: RMSE', ...
    figFolder,'Fig_SMC_Phi_RMSE');

plotMetric(results,Cd_cases, ...
    'MaxAbsError_mps', ...
    'Maximum Absolute Error [m/s]', ...
    'SMC Boundary-Layer Sensitivity: Peak Error', ...
    figFolder,'Fig_SMC_Phi_Max_Error');

plotMetric(results,Cd_cases, ...
    'IAE_m', ...
    'Integral Absolute Error [m]', ...
    'SMC Boundary-Layer Sensitivity: IAE', ...
    figFolder,'Fig_SMC_Phi_IAE');

plotMetric(results,Cd_cases, ...
    'MaxTorque_Nm', ...
    'Maximum Engine Torque [Nm]', ...
    'SMC Boundary-Layer Sensitivity: Maximum Torque', ...
    figFolder,'Fig_SMC_Phi_Max_Torque');

plotMetric(results,Cd_cases, ...
    'MaxAbs_s_over_phi', ...
    'Maximum |s/\phi|', ...
    'SMC Boundary-Layer Sensitivity: Sliding Motion', ...
    figFolder,'Fig_SMC_Phi_Max_Normalized_Sliding',1);

plotMetric(results,Cd_cases, ...
    'TorqueSaturation_percent', ...
    'Torque Saturation [%]', ...
    'SMC Boundary-Layer Sensitivity: Torque Saturation', ...
    figFolder,'Fig_SMC_Phi_Torque_Saturation');

plotMetric(results,Cd_cases, ...
    'TV_Torque_Nm', ...
    'Torque Total Variation [Nm]', ...
    'SMC Boundary-Layer Sensitivity: Torque Variation', ...
    figFolder,'Fig_SMC_Phi_Torque_Total_Variation');

plotMetric(results,Cd_cases, ...
    'RMS_TorqueRate_Nmps', ...
    'RMS Torque Rate [Nm/s]', ...
    'SMC Boundary-Layer Sensitivity: Torque Rate', ...
    figFolder,'Fig_SMC_Phi_RMS_Torque_Rate');

%% Summary file

summaryFile = fullfile(dataFolder,'SMC_phi_sensitivity_summary.txt');
fid = fopen(summaryFile,'w');

fprintf(fid,'SMC Phi Sensitivity Summary\n');
fprintf(fid,'===========================\n\n');
fprintf(fid,'Solver: ode4, fixed step %.6f s\n',dt);
fprintf(fid,'phi values: ');
fprintf(fid,'%.4f ',phi_cases);
fprintf(fid,'\nCd values: ');
fprintf(fid,'%.2f ',Cd_cases);
fprintf(fid,'\n\n');

for i = 1:height(results)

    fprintf(fid,'phi = %.4f, Cd = %.2f\n', ...
        results.phi_smc(i),results.Cd(i));

    fprintf(fid,'RMSE error              = %.9e m/s\n', ...
        results.RMSE_error_mps(i));
    fprintf(fid,'MAE error               = %.9e m/s\n', ...
        results.MAE_error_mps(i));
    fprintf(fid,'Max absolute error      = %.9e m/s\n', ...
        results.MaxAbsError_mps(i));
    fprintf(fid,'IAE                     = %.9e m\n', ...
        results.IAE_m(i));
    fprintf(fid,'Max torque              = %.9f Nm\n', ...
        results.MaxTorque_Nm(i));
    fprintf(fid,'Torque saturation       = %.9f %%\n', ...
        results.TorqueSaturation_percent(i));
    fprintf(fid,'Torque total variation  = %.9e Nm\n', ...
        results.TV_Torque_Nm(i));
    fprintf(fid,'RMS torque rate         = %.9e Nm/s\n', ...
        results.RMS_TorqueRate_Nmps(i));
    fprintf(fid,'Max |s/phi|             = %.9e\n\n', ...
        results.MaxAbs_s_over_phi(i));

end

fclose(fid);

%% Solver settings

fid = fopen(fullfile(dataFolder, ...
    'SMC_phi_sensitivity_Solver_Settings.txt'),'w');

fprintf(fid,'Model: %s\n',modelName);
fprintf(fid,'Solver type: Fixed-step\n');
fprintf(fid,'Solver: ode4\n');
fprintf(fid,'Fixed-step size: %.6f s\n',dt);
fprintf(fid,'Start time: 0 s\n');
fprintf(fid,'Stop time: %.6f s\n',t_end);

fclose(fid);

clear restoreParameters;

fprintf('\n====================================================\n');
fprintf('SMC Phi Sensitivity Analysis Finished\n');
fprintf('Figures saved in:\n%s\n',figFolder);
fprintf('Data saved in:\n%s\n',dataFolder);
fprintf('====================================================\n');

diary off;

%% Local functions

function restoreValues(Cd,phi)

    assignin('base','Cd_sim',Cd);
    assignin('base','phi_smc',phi);

end

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

function plotMetric(results,CdCases,columnName,yLabelText, ...
    titleText,folder,fileName,varargin)

    fig = figure('Color','w');
    hold on;

    for i = 1:numel(CdCases)

        index = abs(results.Cd-CdCases(i))<1e-9;

        plot(results.phi_smc(index), ...
            results.(columnName)(index), ...
            '-o','LineWidth',1.8,'MarkerSize',6, ...
            'DisplayName',sprintf('C_d = %.2f',CdCases(i)));

    end

    if ~isempty(varargin)
        yline(varargin{1},'k--','Boundary limit', ...
            'HandleVisibility','off');
    end

    grid on;
    box on;

    set(gca,'FontName','Times New Roman','FontSize',12);

    xlabel('\phi_{SMC} [m/s]');
    ylabel(yLabelText);
    title(titleText);
    legend('Location','best');

    print(fig,fullfile(folder,[fileName,'.png']), ...
        '-dpng','-r300');

    close(fig);

end