%clear 

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
        path_grf.(simNames{i}) = simDirs{i} + "\" + simNames{i} + "\GRF.sto";
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
%"jointset_lumbar_lumbar_value", ...

important_coordinates= {"jointset_hip_r_hip_flexion_r_value", ...
                        "jointset_knee_r_knee_angle_r_value", ...
                        "jointset_ankle_r_ankle_angle_r_value", ...
                        "ground_force_r_vx",...
                        "ground_force_r_vy"};

labels = {"Hip Flexion", "Knee Angle", "Ankle Angle", "GRF Fx   ", "GRF Fy   "};%"Pelvis tx", "Pelvis ty", "Pelvis Tilt","lumbar Flexion", 

% figure(1);
% T = tiledlayout(3,3);
% for a = 1:9
%     t = nexttile(T,a);
%     if ~contains(important_coordinates{a}, 'tx') && ~contains(important_coordinates{a}, 'ty') && ~contains(important_coordinates{a}, 'force')
%         plot(t,register(sim_data.track.time, ... 
%             180.0/pi * sim_data.track.(important_coordinates{a})),...
%             'displayname', 'tracking','lineWidth',5,'linestyle',"--"); hold on;
%         plot(t,register(sim_data.effpred.time, ... 
%             180.0/pi * sim_data.effpred.(important_coordinates{a})),...
%             'displayname', 'effort','lineWidth',5); hold on;
%         plot(t,register(sim_data.metpred.time, ... 
%             180.0/pi * sim_data.metpred.(important_coordinates{a})),...
%             'displayname', 'COT','lineWidth',5); hold on;    
%         plot(t,register(sim_data.meppredStrain.time, ... 
%             180.0/pi * sim_data.meppredStrain.(important_coordinates{a})),...
%             'displayname', 'Volume Proxy','lineWidth',5); hold on;
%     else
%         plot(t, register(sim_data.track.time, ... 
%             sim_data.track.(important_coordinates{a})),...
%             'displayname', 'tracking','lineWidth',5); hold on;
%         plot(t, register(sim_data.effpred.time, ... 
%             sim_data.effpred.(important_coordinates{a})),...
%             'displayname', 'effort','lineWidth',5); hold on;
%         plot(t, register(sim_data.metpred.time, ... 
%             sim_data.metpred.(important_coordinates{a})),...
%             'displayname', 'COT','lineWidth',5); hold on;    
%         plot(t, register(sim_data.meppredStrain.time, ... 
%             sim_data.meppredStrain.(important_coordinates{a})),...
%             'displayname', 'Volume Proxy','lineWidth',5); hold on;
%     end
%     title(t, labels(a), 'FontName', 'Times New Roman');
%     xlabel(t,"Stride(%)", 'FontName', 'Times New Roman');
%     ylabel(t,"Angle " + char(176)), 'FontName', 'Times New Roman';
% end
% legend();

fh = figure(2);
plot_color = 1- [1,1,1];
T2 = tiledlayout(2,3,'TileSpacing','Compact');

for a = 1:5
    t = nexttile(T2,a);
    if ~contains(important_coordinates{a}, 'tx') && ~contains(important_coordinates{a}, 'ty') && ~contains(important_coordinates{a}, 'force')
        plot(t,register(sim_data.track.time, ... 
            180.0/pi * sim_data.track.(important_coordinates{a})),...
            'displayname','tracking' ,'lineWidth',4,'linestyle',"--"); hold on;
        plot(t,register(sim_data.meppredBOS.time, ... 
            180.0/pi * sim_data.meppredBOS.(important_coordinates{a})),...
            'displayname','BOS' ,'lineWidth',5); hold on;
        plot(t,register(sim_data.meppredZMP.time, ... 
            180.0/pi * sim_data.meppredZMP.(important_coordinates{a})),...
            'displayname','ZMP' ,'lineWidth',5); hold on;    
%         plot(t,register(sim_data.meppredCOP.time, ... 
%             180.0/pi * sim_data.meppredCOP.(important_coordinates{a})),...
%             'displayname','COP' ,'lineWidth',5); hold on;
        plot(t,register(sim_data.meppredmarkerAccel.time, ... 
            180.0/pi * sim_data.meppredmarkerAccel.(important_coordinates{a})),...
            'displayname','head marker accel.' ,'lineWidth',5); hold on;
           else
        plot(t,register(sim_data.track.time, ... 
            sim_data.track.(important_coordinates{a})),...
            'displayname','tracking' ,'lineWidth',4,'linestyle',"--"); hold on;
        plot(t, register(sim_data.meppredBOS.time, ... 
            sim_data.meppredBOS.(important_coordinates{a})),...
            'displayname','BOS' ,'lineWidth',5); hold on;
        plot(t,register(sim_data.meppredZMP.time, ... 
            sim_data.meppredZMP.(important_coordinates{a})),...
            'displayname','ZMP' ,'lineWidth',5); hold on;    
%         plot(t,register(sim_data.meppredCOP.time, ... 
%             sim_data.meppredCOP.(important_coordinates{a})),...
%             'displayname','COP' ,'lineWidth',5); hold on;  
        plot(t,register(sim_data.meppredmarkerAccel.time, ... 
            sim_data.meppredmarkerAccel.(important_coordinates{a})),...
            'displayname','head accel.' ,'lineWidth',5); hold on;
    end
    box(t,'off')
    set(t, 'Color', 'none', 'XColor',plot_color, 'YColor',plot_color,'fontname','Times New Roman','FontSize', 28);
    title(t, labels(a) + "       "+ "(" + char('a'+ (a-1)) + ")", 'FontName', 'Times New Roman');
    
    xticks(t,[0 50 101]);
    H=gca;
    H.LineWidth=2;
    H.TitleHorizontalAlignment = 'right';
    if a > 2
        xlabel(t,"Stride (%)", 'FontName', 'Times New Roman');
    end
    if a == 1
        ylabel(t,"Angle (" + char(176) +")", 'FontName', 'Times New Roman');
    elseif a == 4
        ylabel(t,"Force (N)", 'FontName', 'Times New Roman');
    end
    switch (important_coordinates{a})
        case "pelvis_tx"
            yticks(t, [-3, 0, 3]);
        case "pelvis_ty"
            yticks(t, [0.4, 0.8, 1.2]);
        case "pelvis_tz"
            yticks(t, [-.15 0 .15]);
        case "pelvis_tilt"
            yticks(t, [-30 0 10.0]);
        case "pelvis_list"
            yticks(t, [-30 0 30]);
        case "pelvis_rotation"
            yticks(t, [-30 0 30]);
        case "lumbar_flexion"
            yticks(t, [-50, 0, 50]);
        case "lumbar_bending"
            yticks(t, [-20, 0, 20]);
        case "lumbar_rotation"
            yticks(t, [-35, 0, 35]);
            ylim(t,[-35 35]);
        case "jointset_hip_r_hip_flexion_r_value" 
            yticks(t, [-30, 0, 80]);
            ylim(t,[-30 80]);
            yticklabels(t, {" -30", "  0", " 80"} );
        case "hip_rotation"
            yticks(t, [-50, 0, 50]);
            ylim(t,[-50 50]);
        case "hip_adduction"
            yticks(t, [-40, 0, 40]);
            ylim(t,[-40 40]);
        case "jointset_knee_r_knee_angle_r_value" 
            yticks(t, [-100, -50, 0]);
            ylim(t,[-100 0]);
        case "jointset_ankle_r_ankle_angle_r_value"
            yticks(t, [-30, 0, 30]);
            ylim(t,[-30 30]);
        case "ground_force_r_vx"
            yticks(t, [-400 0 400]);
            ylim(t,[-400 400]);
        case "ground_force_r_vy"  
            yticks(t, [0 800 1600]);
            ylim(t,[0 1600]);
    end
    xlim(t, [1,101]);
    xticks(t, [1 50 101]);
    xticklabels(t, ["0" "50" "100"]);
    
end

lh =legend();
box(lh,'off')
%lh.Position = [0.5341    0.2267    0.5911    0.4321];
t = nexttile(T2,6);
%f_model= plot(t, linspace(0,100,101),'w'); hold on;

title(t,  "(" + char('a'+ (a)) + ")", 'FontName', 'Times New Roman','FontSize', 28)
ax = gca;
set(ax, 'Color', 'none');
box(ax,"off");
axis(t, "off");
xticks(t,[]);
yticks(t,[]);
H=gca;
H.TitleHorizontalAlignment = 'right';

%box(ax,'off');
% [img, map, alphachannel] = imread('.\JOSS\skelemens.png');
% h = imshow(img);%, 'AlphaData', alphachannel);
% set(h, 'AlphaData', alphachannel);

%ax = gca;
%ax.PositionConstraint = "outerposition"




%imshow(".\JOSS\skelemens.png");



function [yy] = register(x,z)
    y = z;%smoothdata(z,1,"gaussian",5); %smoothing N-D data (GRF noisy)
    xx = linspace(x(1), x(end), 101); %segmentation 
    yy = interp1(x,y,xx); %linear registration
end