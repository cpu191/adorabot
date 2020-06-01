%% Burger flipper
function [] = Flipper()
close all
clear all
%%%% TEST %%%%%%%
qn = deg2rad([0    100   -100     0    90     0])
robot = CR5;
robot.model.base = transl(0,0,0.5);
robot = robot.model;
robot.plot(qn,'floorlevel',3);
axis([-2 2 -2 2 0 3]);
ovenPose = transl(0.7,0,0)*trotz(pi/2);
pattyPose1 = transl(0.5,-0.7,0.7);
pattyPose2 = transl(0.5,-0.4,0.7);
pattyPose3 = transl(0.7,-0.4,0.7);
pattyPose4 = transl(0.7,-0.7,0.7);
fryPose = transl(0.3,0.3,0.7)*trotz(-pi/2);
spatPose = robot.fkine(qn)*trotz(pi)*troty(pi);
hold on; 
%% Loading objects
% Load Oven
[fOven,vOven,dataOven] = plyread('oven.ply','tri');
ovenVertexColours = [dataOven.vertex.red,dataOven.vertex.green,dataOven.vertex.blue] / 255;  
oven_h = trisurf(fOven,vOven(:,1),vOven(:,2),vOven(:,3),'FaceVertexCData',ovenVertexColours,'EdgeColor','interp','EdgeLighting','flat');
updatedOvenPosition = [ovenPose*[vOven,ones(size(vOven,1),1)]']';
oven_h.Vertices = updatedOvenPosition(:,1:3);

% Load Patties
[fPatty,vPatty,dataPatty] = plyread('patty.ply','tri');
pattyVertexColours = [dataPatty.vertex.red,dataPatty.vertex.green,dataPatty.vertex.blue] / 255;  
patty_h1 = trisurf(fPatty,vPatty(:,1),vPatty(:,2),vPatty(:,3),'FaceVertexCData',pattyVertexColours,'EdgeColor','interp','EdgeLighting','flat');
patty_h2 = trisurf(fPatty,vPatty(:,1),vPatty(:,2),vPatty(:,3),'FaceVertexCData',pattyVertexColours,'EdgeColor','interp','EdgeLighting','flat');
patty_h3 = trisurf(fPatty,vPatty(:,1),vPatty(:,2),vPatty(:,3),'FaceVertexCData',pattyVertexColours,'EdgeColor','interp','EdgeLighting','flat');
patty_h4 = trisurf(fPatty,vPatty(:,1),vPatty(:,2),vPatty(:,3),'FaceVertexCData',pattyVertexColours,'EdgeColor','interp','EdgeLighting','flat');
updatedPattyPosition1 = [pattyPose1*[vPatty,ones(size(vPatty,1),1)]']';
updatedPattyPosition2 = [pattyPose2*[vPatty,ones(size(vPatty,1),1)]']';
updatedPattyPosition3 = [pattyPose3*[vPatty,ones(size(vPatty,1),1)]']';
updatedPattyPosition4 = [pattyPose4*[vPatty,ones(size(vPatty,1),1)]']';
patty_h1.Vertices = updatedPattyPosition1(:,1:3);
patty_h2.Vertices = updatedPattyPosition2(:,1:3);
patty_h3.Vertices = updatedPattyPosition3(:,1:3);
patty_h4.Vertices = updatedPattyPosition4(:,1:3);
% Load Fry basket
[fFry,vFry,dataFry] = plyread('basket.ply','tri');
fryVertexColours = [dataFry.vertex.red,dataFry.vertex.green,dataFry.vertex.blue] / 255;  
fry_h = trisurf(fFry,vFry(:,1),vFry(:,2),vFry(:,3),'FaceVertexCData',fryVertexColours,'EdgeColor','interp','EdgeLighting','flat');
updatedFryPosition = [fryPose*[vFry,ones(size(vFry,1),1)]']';
fry_h.Vertices = updatedFryPosition(:,1:3);
% Load Spatula
[fSpat,vSpat,dataSpat] = plyread('spatula.ply','tri');
spatVertexColours = [dataSpat.vertex.red,dataSpat.vertex.green,dataSpat.vertex.blue] / 255;  
spat_h = trisurf(fSpat,vSpat(:,1),vSpat(:,2),vSpat(:,3),'FaceVertexCData',spatVertexColours,'EdgeColor','interp','EdgeLighting','flat');
updatedSpatPosition = [spatPose*[vSpat,ones(size(vSpat,1),1)]']';
spat_h.Vertices = updatedSpatPosition(:,1:3);
drawnow();
%% RMRC Parameters and setup
t = 5;                        % total time
deltaT = 0.05;                % Step frequency
steps =  t/deltaT;            % Number of simulation steps
q0 = zeros(1,6);              % Initial guess
qMatrix = zeros(steps,6);     % qMatrix Initialize
qDot = zeros(steps,6);   

%% Applying RMRC
for i= 1 : steps-1
    
end
end

% function [qMatrix] = RMRC(T1,T2,robot)
% 
% % T1 = T1 (1:3,4);              % First Transformation
% % T2 = T2 (1:3,4); 
% x = zeros(2,steps);
% s = lspb(0,1,steps);
% q0 = zeros(1,6);
% qMatrix = robot.ikcon(T1)
% for i = 1:steps
%  %   x(:,i) = 
% end
% end