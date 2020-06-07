%% Burger flipper
function [] = Flipper()
close all
clear all
set(0,'DefaultFigureWindowStyle','docked');
qn = deg2rad([0    0   -100     0    90     0]);
q0 = deg2rad([0    0   -100     90    90     0]);
robot = CR5;
Tc0 = robot.model.fkine(q0);
centerpnt = [0.5,0.5,0.5];
side = 0.5;
vertex = [0.75 0.5 0.8;0.75 -0.5 0.8;0.75 0 1.8];
% P = [0.75 0.75 5.75 0.75;
%     -0.2 0.2 -0.2 0.2;
%     1.2 1.2 0.8 0.8];
P = [1 1 1 1;
    -0.25 0.25 -0.25 0.25;
    1.3 1.3 0.8 0.8];
faces = [1 2 3];
facesP = [1 2 4 3];
%plotOptions.plotEdges = true;
% plotOptions.plotFaces = true;
% [vertex,faces,faceNormals] = RectangularPrism(centerpnt-side/2, centerpnt+side/2,plotOptions);
%
hold on
%%%%%% update position of rectangular
% updatedRec = [(transl(1,1,1)) * [vertex,ones(size(vertex,1),1)]']';
% updatedRec = updatedRec(:,1:3);
% patch('Faces',faces,'Vertices',updatedRec,'FaceVertexCData',tcolor,'FaceColor','flat','lineStyle','none');

%patch('Faces',faces,'Vertices',vertex,'FaceColor','green','lineStyle','none');
%patch('Faces',facesP,'Vertices',P','FaceColor','red','lineStyle','none');

%%%%%%%%%%


pStar = [662 362 362 662; 362 362 662 662];
cam = CentralCamera('focal', 0.08, 'pixel', 10e-5, ...
    'resolution', [1024 1024], 'centre', [512 512],'name', 'CR5camera');
cam.project(P);
fps = 25;

%Define values
%gain of the controler
lambda = 0.6;
%depth of the IBVS
depth = mean (P(1,:));
axis equal

hold on;
ovenPose = transl(0.7,0,0)*trotz(pi/2);
pattyPose1 = transl(0.55,-0.55,0.5)*trotz(-pi/2);
pattyPose2 = transl(0.55,-0.2,0.5)*trotz(-pi/2);
pattyPose3 = transl(0.7,-0.2,0.5)*trotz(-pi/2);
pattyPose4 = transl(0.7,-0.55,0.5)*trotz(-pi/2);
fryPose = transl(0.35,0.25,0.67)*trotz(-pi/2);
spatPose = transl(0.35,-0.08,0.52)*trotz(-pi/2);
EE2Patty = transl(0,0.08,0);
lightPose=transl(1.4,0,0);
lightPose1=transl(0,1.4,0)*trotz(pi/2);
concretePose1 = robot.model.base;
%[qMatrix] = RMRC(robot,pattyPose1*troty(pi),pattyPose2*troty(pi))


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
patty_h(1) = trisurf(fPatty,vPatty(:,1),vPatty(:,2),vPatty(:,3),'FaceVertexCData',pattyVertexColours,'EdgeColor','interp','EdgeLighting','flat');
patty_h(2) = trisurf(fPatty,vPatty(:,1),vPatty(:,2),vPatty(:,3),'FaceVertexCData',pattyVertexColours,'EdgeColor','interp','EdgeLighting','flat');
patty_h(3) = trisurf(fPatty,vPatty(:,1),vPatty(:,2),vPatty(:,3),'FaceVertexCData',pattyVertexColours,'EdgeColor','interp','EdgeLighting','flat');
patty_h(4) = trisurf(fPatty,vPatty(:,1),vPatty(:,2),vPatty(:,3),'FaceVertexCData',pattyVertexColours,'EdgeColor','interp','EdgeLighting','flat');
updatedPattyPosition1 = [pattyPose1*[vPatty,ones(size(vPatty,1),1)]']';
updatedPattyPosition2 = [pattyPose2*[vPatty,ones(size(vPatty,1),1)]']';
updatedPattyPosition3 = [pattyPose3*[vPatty,ones(size(vPatty,1),1)]']';
updatedPattyPosition4 = [pattyPose4*[vPatty,ones(size(vPatty,1),1)]']';
patty_h(1).Vertices = updatedPattyPosition1(:,1:3);
patty_h(2).Vertices = updatedPattyPosition2(:,1:3);
patty_h(3).Vertices = updatedPattyPosition3(:,1:3);
patty_h(4).Vertices = updatedPattyPosition4(:,1:3);
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

%Loading floor 
floorPose=transl(0,0,0);
[fFloor,vFloor,dataFloor] = plyread('floor.ply','tri');
floorVertexColours = [dataFloor.vertex.red,dataFloor.vertex.green,dataFloor.vertex.blue] / 255;
floor_h = trisurf(fFloor,vFloor(:,1),vFloor(:,2),vFloor(:,3),'FaceVertexCData',floorVertexColours,'EdgeColor','interp','EdgeLighting','flat');
floorVertexCount = size(vFloor,1);
updatedFloorPos = [floorPose * [vFloor,ones(floorVertexCount,1)]']';
floor_h.Vertices = updatedFloorPos(:,1:3);

%Loading Light Curtain
[fLight,vLight,dataLight] = plyread('lightC.ply','tri');
lightVertexColours = [dataLight.vertex.red,dataLight.vertex.green,dataLight.vertex.blue] / 255;
light_h = trisurf(fLight,vLight(:,1),vLight(:,2),vLight(:,3),'FaceVertexCData',lightVertexColours,'EdgeColor','interp','EdgeLighting','flat');
lightVertexCount = size(vLight,1);
updatedLightPos = [lightPose * [vLight,ones(lightVertexCount,1)]']';
light_h.Vertices = updatedLightPos(:,1:3);
light_h1 = trisurf(fLight,vLight(:,1),vLight(:,2),vLight(:,3),'FaceVertexCData',lightVertexColours,'EdgeColor','interp','EdgeLighting','flat');
updatedLightPos1 = [lightPose1 * [vLight,ones(lightVertexCount,1)]']';
light_h1.Vertices = updatedLightPos1(:,1:3);

%loading Robot Base
[fCube,vCube,dataCube] = plyread('concrete.ply','tri');
baseVertexColours = [dataCube.vertex.red,dataCube.vertex.green,dataCube.vertex.blue] / 255;
concrete_h1 = trisurf(fCube,vCube(:,1),vCube(:,2),vCube(:,3),'FaceVertexCData',baseVertexColours,'EdgeColor','interp','EdgeLighting','flat');
vertexCount = size(vCube,1);
updatedPosition1 = [concretePose1 * [vCube,ones(vertexCount,1)]']';
concrete_h1.Vertices = updatedPosition1(:,1:3);
view(3);
drawnow();

%% Compute qMatrices

%% qn to Fry
TrN = robot.model.fkine(qn);
qN2Fry = RMRC(robot,TrN,fryPose*troty(pi)*trotz(pi/2),qn);
%robot.model.plot(qN2Fry,'fps',300)

%% qFry to Down
Tdown = fryPose*troty(pi)*trotz(pi/2)*transl(0,0,0.15);
qFry2Down = RMRC(robot,robot.model.fkine(qN2Fry(end,:)),Tdown,qN2Fry(end,:)); % robot qMatrix
%robot.model.plot(qFry2Down,'fps',100)

%% q Down to qSpat
qDown2Spat = RMRC(robot,robot.model.fkine(qFry2Down(end,:)),spatPose*troty(pi),qFry2Down(end,:));
%robot.model.plot(qDown2Spat,'fps',100)

%% Flip 1
% Spat to p1
TrPickUpReady1 = pattyPose1*transl(0,-0.15,0)*troty(pi);
qSpat2p1 = RMRC(robot,robot.model.fkine(qDown2Spat(end,:)),TrPickUpReady1,qDown2Spat(end,:));
%robot.model.plot(qSpat2p1,'fps',200)
% p1 under
qUnder1 = RMRC(robot,robot.model.fkine(qSpat2p1(end,:)),pattyPose1*troty(pi)*transl(0,-0.08,-0.01),qSpat2p1(end,:));
%robot.model.plot(qUnder1,'fps',200);

toUnder1 = vertcat(qSpat2p1,qUnder1);

% p1 pickup
qPickUp1 = RMRC(robot,robot.model.fkine(qUnder1(end,:)),pattyPose1*troty(pi)*transl(0,-0.08,-0.1),qUnder1(end,:));
%robot.model.plot(qPickUp1,'fps',200);

% p1 Flip
qFlip1 = RMRC(robot,robot.model.fkine(qPickUp1(end,:)),robot.model.fkine(qPickUp1(end,:))*troty(-pi/4)*transl(0,-0.05,0),qPickUp1(end,:));
%robot.model.plot(qFlip1,'fps',200);

upFlip1 = vertcat(qPickUp1,qFlip1);

%% Flip 2
% p1 Flip to p2
TrPickUpReady2 = pattyPose2*transl(0,-0.15,0)*troty(pi);
qp12p2 = RMRC(robot,robot.model.fkine(qFlip1(end,:)),TrPickUpReady2,qFlip1(end,:));
%robot.model.plot(qp12p2,'fps',200)
% p2 under
qUnder2 = RMRC(robot,robot.model.fkine(qp12p2(end,:)),pattyPose2*troty(pi)*transl(0,-0.08,-0.01),qp12p2(end,:));
%robot.model.plot(qUnder2,'fps',200);

toUnder2 = vertcat(qp12p2,qUnder2);

% p2 pickup
qPickUp2 = RMRC(robot,robot.model.fkine(qUnder2(end,:)),pattyPose2*troty(pi)*transl(0,-0.08,-0.1),qUnder2(end,:));
%robot.model.plot(qPickUp2,'fps',200);
% p2 Flip

qFlip2 = repmat(qPickUp2(end,:),150,1);
prev = qPickUp2(end,:);
qFlip2(1,:) = prev;
for i = 2:150
    qFlip2(i,5) = qFlip2(i-1,5)-(deg2rad(90-74)/100);
end
%robot.model.plot(qFlip2,'fps',200);

upFlip2 = vertcat(qPickUp2,qFlip2);
%% Flip 3
% p2 Flip to p3
TrPickUpReady3 = pattyPose3*transl(0,-0.15,0)*troty(pi);
qp22p3 = RMRC(robot,robot.model.fkine(qFlip2(end,:)),TrPickUpReady3,qFlip2(end,:));
%robot.model.plot(qp22p3,'fps',200)
% p3 under
qUnder3 = RMRC(robot,robot.model.fkine(qp22p3(end,:)),pattyPose3*troty(pi)*transl(0,-0.08,-0.01),qp22p3(end,:));
%robot.model.plot(qUnder3,'fps',200);

toUnder3 = vertcat(qp22p3,qUnder3);

% p3 pickup
qPickUp3 = RMRC(robot,robot.model.fkine(qUnder3(end,:)),pattyPose3*troty(pi)*transl(0,-0.08,-0.1),qUnder3(end,:));
%robot.model.plot(qPickUp3,'fps',200);

% p3 Flip
qFlip3 = repmat(qPickUp3(end,:),150,1);
prev = qPickUp3(end,:);
qFlip3(1,:) = prev;
for i = 2:150
    qFlip3(i,5) = qFlip3(i-1,5)-(deg2rad(90-74)/100);
end
%robot.model.plot(qFlip3,'fps',200);

upFlip3 = vertcat(qPickUp3,qFlip3);
%% Flip 4
% p4 Flip to p4
TrPickUpReady4 = pattyPose4*transl(0,-0.15,0)*troty(pi);
qp32p4 = RMRC(robot,robot.model.fkine(qFlip3(end,:)),TrPickUpReady4,qFlip3(end,:));
%robot.model.plot(qp32p4,'fps',200)
% p4 under
qUnder4 = RMRC(robot,robot.model.fkine(qp32p4(end,:)),pattyPose4*troty(pi)*transl(0,-0.08,-0.01),qp32p4(end,:));
%robot.model.plot(qUnder4,'fps',200);

toUnder4 = vertcat(qp32p4,qUnder4);

% p4 pickup
qPickUp4 = RMRC(robot,robot.model.fkine(qUnder4(end,:)),pattyPose4*troty(pi)*transl(0,-0.08,-0.1),qUnder4(end,:));
%robot.model.plot(qPickUp4,'fps',200);
% p4 Flip
qFlip4 = repmat(qPickUp4(end,:),150,1);
prev = qPickUp4(end,:);
qFlip4(1,:) = prev;
for i = 2:150
    qFlip4(i,5) = qFlip4(i-1,5)-(deg2rad(90-74)/100);
end
%robot.model.plot(qFlip4,'fps',200);

upFlip4 = vertcat(qPickUp4,qFlip4);

%% Pickup Fry
% Drop Spatula
IspatPose = transl(0.35,-0.08,0.52)*trotz(-pi/2)*troty(pi);
q42Spat = RMRC(robot,robot.model.fkine(qFlip4(end,:)),IspatPose,qFlip4(end,:));

% Goto Fry
qSpat2Fry = RMRC(robot,robot.model.fkine(q42Spat(end,:)),Tdown,q42Spat(end,:));

% pick up fry
IfryPose = transl(0.35,0.25,0.67)*trotz(-pi/2);
qPickUpFry = RMRC(robot,robot.model.fkine(qSpat2Fry(end,:)),IfryPose*troty(pi)*trotz(pi/2),qSpat2Fry(end,:));

% Goback to qn
qNorm = RMRC(robot,robot.model.fkine(qPickUpFry(end,:)),robot.model.fkine(qn),qPickUpFry(end,:));

%% Run through the work
sUnit = 3; % Number of step every loop
%axis([-1.5 1.5 -1.5 1.5 -0.05 2]);

%Take the fry
for i = 1: size(qN2Fry,1)
robot.model.animate(qN2Fry(i,:));
pause(0.01);
end
%Put Fry down
for i = 1 :sUnit: size(qFry2Down,1)
    robot.model.animate(qFry2Down(i,:));
    fryPose = robot.model.fkine(qFry2Down(i,:))*trotz(-pi/2)*troty(pi);
    updatedFryPosition = [fryPose*[vFry,ones(size(vFry,1),1)]']';
    fry_h.Vertices = updatedFryPosition(:,1:3);
    drawnow();
    pause(0.01);
end

%Take spatula

for i = 1: size(qDown2Spat,1)
robot.model.animate(qDown2Spat(i,:));
pause(0.01);
end
% for i = 1 :sUnit: size(qSpat2p1,1)
%     robot.model.animate(qSpat2p1(i,:));
%     spatPose = robot.model.fkine(qSpat2p1(i,:))*troty(pi);
%     updatedSpatPosition = [spatPose*[vSpat,ones(size(vSpat,1),1)]']';
%     spat_h.Vertices = updatedSpatPosition(:,1:3);
%     drawnow();
%     pause(0.01);
% end

%Flip the meat1

 for i = 1 :sUnit: size(toUnder1,1)
    robot.model.animate(toUnder1(i,:));
    spatPose = robot.model.fkine(toUnder1(i,:))*troty(pi);
    updatedSpatPosition = [spatPose*[vSpat,ones(size(vSpat,1),1)]']';
    spat_h.Vertices = updatedSpatPosition(:,1:3);
    drawnow();
    pause(0.01);
end

for i=1:sUnit: size(upFlip1,1)
    robot.model.animate(upFlip1(i,:));
    spatPose = robot.model.fkine(upFlip1(i,:))*troty(pi);
    updatedSpatPosition = [spatPose*[vSpat,ones(size(vSpat,1),1)]']';
    pattyPose1 = robot.model.fkine(upFlip1(i,:))*EE2Patty;
    updatedPattyPosition1 = [pattyPose1*[vPatty,ones(size(vPatty,1),1)]']';
    patty_h(1).Vertices = updatedPattyPosition1(:,1:3);
    spat_h.Vertices = updatedSpatPosition(:,1:3);
    drawnow();
    pause(0.01);
end

% meat1 dropping
Tmeat = mTraj(pattyPose1,transl(0.55,-0.55,0.5)*trotz(-pi/2)*trotx(pi));
for i = 1:size(Tmeat,1)
    pattyPose1 = [ Tmeat(i,1,1),Tmeat(i,1,2),Tmeat(i,1,3),Tmeat(i,1,4);...
                 , Tmeat(i,2,1),Tmeat(i,2,2),Tmeat(i,2,3),Tmeat(i,2,4);...
                 , Tmeat(i,3,1),Tmeat(i,3,2),Tmeat(i,3,3),Tmeat(i,3,4);...
                 , Tmeat(i,4,1),Tmeat(i,4,2),Tmeat(i,4,3),Tmeat(i,4,4)];
    updatedPattyPosition1 = [pattyPose1*[vPatty,ones(size(vPatty,1),1)]']';
    patty_h(1).Vertices = updatedPattyPosition1(:,1:3);
    drawnow();
    pause(0.01);
end
%% Flip the meat2
% move to patty 2
for i = 1 :sUnit: size(toUnder2,1)
    robot.model.animate(toUnder2(i,:));
    spatPose = robot.model.fkine(toUnder2(i,:))*troty(pi);
    updatedSpatPosition = [spatPose*[vSpat,ones(size(vSpat,1),1)]']';
    spat_h.Vertices = updatedSpatPosition(:,1:3);
    drawnow();
    pause(0.01);
end

for i=1:sUnit: size(upFlip2,1)
    robot.model.animate(upFlip2(i,:));
    spatPose = robot.model.fkine(upFlip2(i,:))*troty(pi);
    updatedSpatPosition = [spatPose*[vSpat,ones(size(vSpat,1),1)]']';
    pattyPose2 = robot.model.fkine(upFlip2(i,:))*EE2Patty;
    updatedPattyPosition2 = [pattyPose2*[vPatty,ones(size(vPatty,1),1)]']';
    patty_h(2).Vertices = updatedPattyPosition2(:,1:3);
    spat_h.Vertices = updatedSpatPosition(:,1:3);
    drawnow();
    pause(0.01);
end

% meat2 dropping
Tmeat = mTraj(pattyPose2,transl(0.55,-0.2,0.5)*trotz(-pi/2)*trotx(pi));
for i = 1:size(Tmeat,1)
    pattyPose2 = [ Tmeat(i,1,1),Tmeat(i,1,2),Tmeat(i,1,3),Tmeat(i,1,4);...
                 , Tmeat(i,2,1),Tmeat(i,2,2),Tmeat(i,2,3),Tmeat(i,2,4);...
                 , Tmeat(i,3,1),Tmeat(i,3,2),Tmeat(i,3,3),Tmeat(i,3,4);...
                 , Tmeat(i,4,1),Tmeat(i,4,2),Tmeat(i,4,3),Tmeat(i,4,4)];
    updatedPattyPosition2 = [pattyPose2*[vPatty,ones(size(vPatty,1),1)]']';
    patty_h(2).Vertices = updatedPattyPosition2(:,1:3);
    drawnow();
    pause(0.01);
end


%% Flip the meat3

% move to patty 3
for i = 1 :sUnit: size(toUnder3,1)
    robot.model.animate(toUnder3(i,:));
    spatPose = robot.model.fkine(toUnder3(i,:))*troty(pi);
    updatedSpatPosition = [spatPose*[vSpat,ones(size(vSpat,1),1)]']';
    spat_h.Vertices = updatedSpatPosition(:,1:3);
    drawnow();
    pause(0.01);
end

for i=1:sUnit: size(upFlip3,1)
    robot.model.animate(upFlip3(i,:));
    spatPose = robot.model.fkine(upFlip3(i,:))*troty(pi);
    updatedSpatPosition = [spatPose*[vSpat,ones(size(vSpat,1),1)]']';
    pattyPose3 = robot.model.fkine(upFlip3(i,:))*EE2Patty;
    updatedPattyPosition3 = [pattyPose3*[vPatty,ones(size(vPatty,1),1)]']';
    patty_h(3).Vertices = updatedPattyPosition3(:,1:3);
    spat_h.Vertices = updatedSpatPosition(:,1:3);
    drawnow();
    pause(0.01);
end

% meat3 dropping
Tmeat = mTraj(pattyPose3,transl(0.7,-0.2,0.5)*trotz(-pi/2)*trotx(pi));
for i = 1:size(Tmeat,1)
    pattyPose3 = [ Tmeat(i,1,1),Tmeat(i,1,2),Tmeat(i,1,3),Tmeat(i,1,4);...
                 , Tmeat(i,2,1),Tmeat(i,2,2),Tmeat(i,2,3),Tmeat(i,2,4);...
                 , Tmeat(i,3,1),Tmeat(i,3,2),Tmeat(i,3,3),Tmeat(i,3,4);...
                 , Tmeat(i,4,1),Tmeat(i,4,2),Tmeat(i,4,3),Tmeat(i,4,4)];
    updatedPattyPosition3 = [pattyPose3*[vPatty,ones(size(vPatty,1),1)]']';
    patty_h(3).Vertices = updatedPattyPosition3(:,1:3);
    drawnow();
    pause(0.01);
end

%% Flip the meat4

% move to patty 4
for i = 1 :sUnit: size(toUnder4,1)
    robot.model.animate(toUnder4(i,:));
    spatPose = robot.model.fkine(toUnder4(i,:))*troty(pi);
    updatedSpatPosition = [spatPose*[vSpat,ones(size(vSpat,1),1)]']';
    spat_h.Vertices = updatedSpatPosition(:,1:3);
    drawnow();
    pause(0.01);
end

for i=1:sUnit: size(upFlip4,1)
    robot.model.animate(upFlip4(i,:));
    spatPose = robot.model.fkine(upFlip4(i,:))*troty(pi);
    updatedSpatPosition = [spatPose*[vSpat,ones(size(vSpat,1),1)]']';
    pattyPose4 = robot.model.fkine(upFlip4(i,:))*EE2Patty;
    updatedPattyPosition4 = [pattyPose4*[vPatty,ones(size(vPatty,1),1)]']';
    patty_h(4).Vertices = updatedPattyPosition4(:,1:3);
    spat_h.Vertices = updatedSpatPosition(:,1:3);
    drawnow();
    pause(0.01);
end

% meat4 dropping
Tmeat = mTraj(pattyPose4,transl(0.7,-0.55,0.5)*trotz(-pi/2)*trotx(pi));
for i = 1:size(Tmeat,1)
    pattyPose4 = [ Tmeat(i,1,1),Tmeat(i,1,2),Tmeat(i,1,3),Tmeat(i,1,4);...
                 , Tmeat(i,2,1),Tmeat(i,2,2),Tmeat(i,2,3),Tmeat(i,2,4);...
                 , Tmeat(i,3,1),Tmeat(i,3,2),Tmeat(i,3,3),Tmeat(i,3,4);...
                 , Tmeat(i,4,1),Tmeat(i,4,2),Tmeat(i,4,3),Tmeat(i,4,4)];
    updatedPattyPosition4 = [pattyPose4*[vPatty,ones(size(vPatty,1),1)]']';
    patty_h(4).Vertices = updatedPattyPosition4(:,1:3);
    drawnow();
    pause(0.01);
end

%Drop spatula
for i = 1 :sUnit: size(q42Spat,1)
    robot.model.animate(q42Spat(i,:));
    spatPose = robot.model.fkine(q42Spat(i,:))*troty(pi);
    updatedSpatPosition = [spatPose*[vSpat,ones(size(vSpat,1),1)]']';
    spat_h.Vertices = updatedSpatPosition(:,1:3);
    drawnow();
    pause(0.01);
end

%Go to fry
for i = 1: size(qSpat2Fry,1)
robot.model.animate(qSpat2Fry(i,:));
pause(0.01);
end

%Pickup Fry
for i = 1 :sUnit: size(qPickUpFry,1)
    robot.model.animate(qPickUpFry(i,:));
    fryPose = robot.model.fkine(qPickUpFry(i,:))*trotz(-pi/2)*troty(pi);
    updatedFryPosition = [fryPose*[vFry,ones(size(vFry,1),1)]']';
    fry_h.Vertices = updatedFryPosition(:,1:3);
    drawnow();
    pause(0.01);
end

%Go back to Ready Position
for i = 1: size(qNorm,1)
robot.model.animate(qNorm(i,:));
pause(0.01);
end

% qMatrix = RMRC(robot,transl(0.5,-0.2,0.7),transl(0.5,0.5,0.7),qn);
% for i = 1:size(qMatrix,1)
%     result = IsCollision(robot.model,qMatrix(i+2,:),faces,vertex,faceNormals);
%     if result
%         break;
%     end
%
%     robot.model.animate(qMatrix(i,:));
%     drawnow();
% end




end

%% Calculate the meat trajectory
function [qMeat] = mTraj(trans1,trans2)
mStep = 50;
s = lspb(0,1,mStep);
qMeat = nan(mStep,4,4);
%qMeat = repmat(nan,4,4)
for i= 1 : mStep
    qMeat(i,:,:) = (1-s(i)).*trans1 + s(i).*trans2;
end
end

%% LinePlaneIntersection(REF: LAB 5 SOLUTION ON UTSONLINE)
% Given a plane (normal and point) and two points that make up another line, get the intersection
% Check == 0 if there is no intersection
% Check == 1 if there is a line plane intersection between the two points
% Check == 2 if the segment lies in the plane (always intersecting)
% Check == 3 if there is intersection point which lies outside line segment
function [intersectionPoint,check] = LinePlaneIntersection(planeNormal,pointOnPlane,point1OnLine,point2OnLine)

intersectionPoint = [0 0 0];
u = point2OnLine - point1OnLine;
w = point1OnLine - pointOnPlane;
D = dot(planeNormal,u);
N = -dot(planeNormal,w);
check = 0; %#ok<NASGU>
if abs(D) < 10^-7        % The segment is parallel to plane
    if N == 0           % The segment lies in plane
        check = 2;
        return
    else
        check = 0;       %no intersection
        return
    end
end

%compute the intersection parameter
sI = N / D;
intersectionPoint = point1OnLine + sI.*u;

if (sI < 0 || sI > 1)
    check= 3;          %The intersection point  lies outside the segment, so there is no intersection
else
    check=1;
end
end
%% GET LINK POSES (REF: LAB 5 SOLUTION ON UTSONLINE)
% q - robot joint angles
% robot -  seriallink robot model
% transforms - list of transforms
function [ transforms ] = GetLinkPoses( q, robot)

links = robot.links;
transforms = zeros(4, 4, length(links) + 1);
transforms(:,:,1) = robot.base;

for i = 1:length(links)
    L = links(1,i);
    
    current_transform = transforms(:,:, i);
    
    current_transform = current_transform * trotz(q(1,i) + L.offset) * ...
        transl(0,0, L.d) * transl(L.a,0,0) * trotx(L.alpha);
    transforms(:,:,i + 1) = current_transform;
end
end
%% Is Collision (REF: LAB 5 SOLUTION ON UTSONLINE)
% This is based upon the output of questions 2.5 and 2.6
% Given a robot model (robot), and trajectory (i.e. joint state vector) (qMatrix)
% and triangle obstacles in the environment (faces,vertex,faceNormals)
function result = IsCollision(robot,qMatrix,faces,vertex,faceNormals,returnOnceFound)
if nargin < 6
    returnOnceFound = true;
end
result = false;

for qIndex = 1:size(qMatrix,1)
    % Get the transform of every joint (i.e. start and end of every link)
    tr = GetLinkPoses(qMatrix(qIndex,:), robot);
    
    % Go through each link and also each triangle face
    for i = 1 : size(tr,3)-1
        for faceIndex = 1:size(faces,1)
            vertOnPlane = vertex(faces(faceIndex,1)',:);
            [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)');
            if check == 1 && IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
                plot3(intersectP(1),intersectP(2),intersectP(3),'g*');
                display('Intersection');
                result = true;
                if returnOnceFound
                    return
                end
            end
        end
    end
end
end
%% IsIntersectionPointInsideTriangle (REF: LAB 5 SOLUTION ON UTSONLINE)
% Given a point which is known to be on the same plane as the triangle
% determine if the point is
% inside (result == 1) or
% outside a triangle (result ==0 )
function result = IsIntersectionPointInsideTriangle(intersectP,triangleVerts)

u = triangleVerts(2,:) - triangleVerts(1,:);
v = triangleVerts(3,:) - triangleVerts(1,:);

uu = dot(u,u);
uv = dot(u,v);
vv = dot(v,v);

w = intersectP - triangleVerts(1,:);
wu = dot(w,u);
wv = dot(w,v);

D = uv * uv - uu * vv;

% Get and test parametric coords (s and t)
s = (uv * wv - vv * wu) / D;
if (s < 0.0 || s > 1.0)        % intersectP is outside Triangle
    result = 0;
    return;
end

t = (uv * wu - uu * wv) / D;
if (t < 0.0 || (s + t) > 1.0)  % intersectP is outside Triangle
    result = 0;
    return;
end

result = 1;                      % intersectP is in Triangle
end

