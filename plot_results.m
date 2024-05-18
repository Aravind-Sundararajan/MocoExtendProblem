clear 

simNames = {dir('output').name};

simDirs = {dir('output').folder};

results = struct();
path_ik = struct();
path_grf = struct();

for i = 1:length(simNames)
    if contains(simNames{i},'.') == 1
            %do nothing
    else
        path_ik.(simNames{i}) = simDirs{i} + "\" + simNames{i} + "\states.sto";
        path_grf.(simNames{i}) = simDirs{i} + "\" + simNames{i} + "\states_GRF.sto";
    end
end

results_ik = struct();
results_grf = struct();

fields = fieldnames(path_grf);

for name = fields'
    results_ik.(name{1}) = ReadOpenSimData(path_ik.(name{1}));
    results_grf.(name{1}) = ReadOpenSimData(path_grf.(name{1}));
end
sim_data = struct();
for name = fields'
    sim_data.(name{1}) = struct();

    for grf_label = {results_grf.track.labels{:}}
        good_label = strrep(grf_label{1},"/","_");
        i = find(strcmp(results_grf.track.labels, grf_label{1}));
        sim_data.(name{1}).(good_label) = results_grf.(name{1}).data(:,i);
    end

    for ik_label = {results_ik.track.labels{:}}
        i = find(strcmp(results_ik.track.labels, ik_label{1}));
        if contains(ik_label,"value") || contains(ik_label, "time")
            good_label = strrep(ik_label{1},"/","_");

            if startsWith(good_label, "_")
                tmp = char(good_label);
                good_label = string(tmp(2:end));
            end
            sim_data.(name{1}).(good_label) = results_ik.(name{1}).data(:,i);
        end
    end

end
%sim_data = ReadOpenSimData();



important_coordinates= {"jointset_groundPelvis_pelvis_tx_value" , ...
                        "jointset_groundPelvis_pelvis_ty_value" , ...
                        "jointset_groundPelvis_pelvis_tilt_value" , ...
                        "jointset_lumbar_lumbar_value", ...
                        "jointset_hip_r_hip_flexion_r_value",...
                        "jointset_knee_r_knee_angle_r_value", ...
                        "jointset_ankle_r_ankle_angle_r_value", ...
                        "ground_force_r_vx",...
                        "ground_force_r_vy"};

labels = {"Pelvis tx", "Pelvis ty", "Pelvis Tilt", "lumbar Flexion", "Hip Flexion", "Knee Angle", "Ankle Angle", "GRF Fx", "GRF Fy"};

figure(1);
T = tiledlayout(3,3);
for a = 1:9
    t = nexttile(T,a);
    if ~contains(important_coordinates{a}, 'tx') && ~contains(important_coordinates{a}, 'ty') && ~contains(important_coordinates{a}, 'force')
        plot(t,register(sim_data.track.time, ... 
            180.0/pi * sim_data.track.(important_coordinates{a})),...
            'displayname', 'tracking','lineWidth',2); hold on;
        plot(t,register(sim_data.effpred.time, ... 
            180.0/pi * sim_data.effpred.(important_coordinates{a})),...
            'displayname', 'effort','lineWidth',2); hold on;
        plot(t,register(sim_data.metpred.time, ... 
            180.0/pi * sim_data.metpred.(important_coordinates{a})),...
            'displayname', 'COT','lineWidth',2); hold on;    
        plot(t,register(sim_data.meppredStrain.time, ... 
            180.0/pi * sim_data.meppredStrain.(important_coordinates{a})),...
            'displayname', 'Volume Proxy','lineWidth',2); hold on;
    else
        plot(t, register(sim_data.track.time, ... 
            sim_data.track.(important_coordinates{a})),...
            'displayname', 'tracking','lineWidth',2); hold on;
        plot(t, register(sim_data.effpred.time, ... 
            sim_data.effpred.(important_coordinates{a})),...
            'displayname', 'effort','lineWidth',2); hold on;
        plot(t, register(sim_data.metpred.time, ... 
            sim_data.metpred.(important_coordinates{a})),...
            'displayname', 'COT','lineWidth',2); hold on;    
        plot(t, register(sim_data.meppredStrain.time, ... 
            sim_data.meppredStrain.(important_coordinates{a})),...
            'displayname', 'Volume Proxy','lineWidth',2); hold on;
    end
    title(t, labels(a), 'FontName', 'Times New Roman');
    xlabel(t,"Stride(%)", 'FontName', 'Times New Roman');
    ylabel(t,"Angle " + char(176)), 'FontName', 'Times New Roman';
end
legend();

figure(2);
T2 = tiledlayout(3,3);
for a = 1:9
    t = nexttile(T2,a);
    if ~contains(important_coordinates{a}, 'tx') && ~contains(important_coordinates{a}, 'ty') && ~contains(important_coordinates{a}, 'force')
            
        plot(t,register(sim_data.track.time, ... 
            180.0/pi * sim_data.track.(important_coordinates{a})),...
            'displayname','tracking' ,'lineWidth',2); hold on;
        plot(t,register(sim_data.meppredBOS.time, ... 
            180.0/pi * sim_data.meppredBOS.(important_coordinates{a})),...
            'displayname','BOS' ,'lineWidth',2); hold on;
        plot(t,register(sim_data.meppredZMP.time, ... 
            180.0/pi * sim_data.meppredZMP.(important_coordinates{a})),...
            'displayname','ZMP' ,'lineWidth',2); hold on;    
        plot(t,register(sim_data.meppredmarkerAccel.time, ... 
            180.0/pi * sim_data.meppredmarkerAccel.(important_coordinates{a})),...
            'displayname','head marker accel.' ,'lineWidth',2); hold on;
           else
        plot(t,register(sim_data.track.time, ... 
            sim_data.track.(important_coordinates{a})),...
            'displayname','tracking' ,'lineWidth',2); hold on;
        plot(t, register(sim_data.meppredBOS.time, ... 
            sim_data.meppredBOS.(important_coordinates{a})),...
            'displayname','BOS' ,'lineWidth',2); hold on;
        plot(t,register(sim_data.meppredZMP.time, ... 
            sim_data.meppredZMP.(important_coordinates{a})),...
            'displayname','ZMP' ,'lineWidth',2); hold on;    
        plot(t,register(sim_data.meppredmarkerAccel.time, ... 
            sim_data.meppredmarkerAccel.(important_coordinates{a})),...
            'displayname','head marker accel' ,'lineWidth',2); hold on;
    end
    title(t, labels(a), 'FontName', 'Times New Roman');
    xlabel(t,"Stride(%)", 'FontName', 'Times New Roman');
    ylabel(t,"Angle " + char(176)), 'FontName', 'Times New Roman';

end

legend();

function [yy] = register(x,y)
    xx = linspace(x(1), x(end), 101);
    yy = interp1(x,y,xx);
end