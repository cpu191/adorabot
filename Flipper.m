%% Burger flipper
function [] = Flipper()
close all
clear all
qn = deg2rad([0    0   -100     0    90     0]);
q0 = deg2rad([0    0   -100     90    90     0]);
robot = CR5;
robot.model.animate(q0);
%robot.model.teach
Tc0 = robot.model.fkine(q0);

centerpnt = [0.5,0.5,0.5];
<<<<<<< HEAD
side = 0.5
vertex = [0.75 0.5 0.8;0.75 -0.5 0.8;0.75 0 1.8];
% P = [0.75 0.75 0.75 0.75;
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

patch('Faces',faces,'Vertices',vertex,'FaceColor','green','lineStyle','none');
patch('Faces',facesP,'Vertices',P','FaceColor','red','lineStyle','none');

%%%%%%%%%%
=======
side = 0.5;
plotOptions.plotFaces = true;
%[vertex,faces,faceNormals] = RectangularPrism(centerpnt-side/2, centerpnt+side/2,plotOptions);
>>>>>>> 237e332ec77bbd68c6b306604b2434accdd270ae
% axis([-2 2 -2 2 0 3]);
pStar = [662 362 362 662; 362 362 662 662];
cam = CentralCamera('focal', 0.08, 'pixel', 10e-5, ...
'resolution', [1024 1024], 'centre', [512 512],'name', 'CR5camera');
cam.project(P)
fps = 25;

%Define values
%gain of the controler
lambda = 0.6;
%depth of the IBVS
depth = mean (P(1,:));
axis equal
<<<<<<< HEAD
%%%%%%%%%%%%%%
% ovenPose = transl(0.7,0,0)*trotz(pi/2);
% pattyPose1 = transl(0.5,-0.7,0.7);
% pattyPose2 = transl(0.5,-0.4,0.7);
% pattyPose3 = transl(0.7,-0.4,0.7);
% pattyPose4 = transl(0.7,-0.7,0.7);
% fryPose = transl(0.3,0.3,0.7)*trotz(-pi/2);
% spatPose = robot.model.fkine(qn)*trotz(pi)*troty(pi);
%%%%
% plot camera and points
cam.T = Tc0;
cam.plot_camera('Tcam',Tc0, 'label','scale',0.15);
plot_sphere(P, 0.05, 'b');
lighting gouraud
light
p = cam.plot(P, 'Tcam', Tc0);
cam.plot(pStar, '*'); % create the camera view
cam.hold(true);
cam.plot(P, 'Tcam', Tc0, 'o'); % create the camera view
pause(2)

cam.hold(true);
cam.plot(P);    % show initial view


%Initialise display arrays
vel_p = [];
uv_p = [];
history = [];
%%%%%%%%%%%%%%%%
%[qMatrix] = RMRC(robot,pattyPose1*troty(pi),pattyPose2*troty(pi))
% qMatrix = RMRC(robot,transl(0.5,-0.2,0.7),transl(0.5,0.5,0.7));
% for i = 1:size(qMatrix,1)
%     result = IsCollision(robot.model,qMatrix(i+2,:),faces,vertex,faceNormals);
%     if result
%         break;
%     end
%     robot.model.animate(qMatrix(i,:));
%     drawnow();
% end
%robot.model.plot(qMatrix)
%%%%%%%%%%%%%%%
%% Trial
ksteps = 0;
 while true
        ksteps = ksteps + 1;
        
        % compute the view of the camera
        uv = cam.plot(P);
        
        % compute image plane error as a column
        e = pStar-uv;   % feature error
        e = e(:);
        Zest = [];
        
        % compute the Jacobian
        if isempty(depth)
            % exact depth from simulation (not possible in practice)
            pt = homtrans(inv(Tcam), P);
            J = cam.visjac_p(uv, pt(3,:) );
        elseif ~isempty(Zest)
            J = cam.visjac_p(uv, Zest);
        else
            J = cam.visjac_p(uv, depth );
        end

        % compute the velocity of camera in camera frame
        try
            v = lambda * pinv(J) * e;
        catch
            status = -1;
            return
        end
        fprintf('v: %.3f %.3f %.3f %.3f %.3f %.3f\n', v);

        %compute robot's Jacobian and inverse
        J2 = robot.model.jacobn(q0);
        Jinv = pinv(J2);
        % get joint velocities
        qp = Jinv*v;

         
         %Maximum angular velocity cannot exceed 180 degrees/s
         ind=find(qp>pi);
         if ~isempty(ind)
             qp(ind)=pi;
         end
         ind=find(qp<-pi);
         if ~isempty(ind)
             qp(ind)=-pi;
         end

        %Update joints 
        q = q0 + (1/fps)*qp;
        robot.model.animate(q');
=======
hold on;
ovenPose = transl(0.7,0,0)*trotz(pi/2);
pattyPose1 = transl(0.55,-0.55,0.5);
pattyPose2 = transl(0.55,-0.2,0.5);
pattyPose3 = transl(0.7,-0.2,0.5);
pattyPose4 = transl(0.7,-0.55,0.5);
fryPose = transl(0.35,0.25,0.67)*trotz(-pi/2);
spatPose = transl(0.35,-0.08,0.52)*trotz(-pi/2);
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
drawnow();

%% Compute qMatrices

%% qn to Fry
TrN = robot.model.fkine(qn);
qN2Fry = RMRC(robot,TrN,fryPose*troty(pi),qn);
%robot.model.plot(qN2Fry)

%% qFry to Down
Tdown = fryPose*troty(pi)*transl(0,0,0.2);
qFry2Down = RMRC(robot,fryPose*troty(pi),Tdown,qN2Fry(end,:)); % robot qMatrix
%robot.model.animate(qFry2Down)

%% q Down to qSpat
qDown2Spat = RMRC(robot,Tdown,spatPose*troty(pi),qFry2Down(end,:));
%robot.model.plot(qDown2Spat,'fps',100)

%% Flip 1
% Spat to p1
TrPickUpReady1 = pattyPose1*troty(pi)*transl(0.15,0,-0.1)*troty(pi/4);
qSpat2p1 = RMRC(robot,spatPose,TrPickUpReady1,qDown2Spat(end,:));
%robot.model.plot(qSpat2p1,'fps',100)
% p1 under
qUnder1 = RMRC(robot,TrPickUpReady1,pattyPose1*troty(pi)*transl(0.15,0,-0.01),qSpat2p1(end,:));
%robot.model.plot(qUnder1,'fps',100);
% p1 pickup
qPickUp1 = RMRC(robot,pattyPose1*troty(pi)*transl(0.15,0,-0.01),pattyPose1*troty(pi)*transl(0.15,0,-0.1),qUnder1(end,:));
%robot.model.plot(qPickUp1,'fps',60);
% p1 Flip
qFlip1 = RMRC(robot,pattyPose1*troty(pi)*transl(0.15,0,-0.1),robot.model.fkine(qPickUp1(end,:))*troty(pi/4)*transl(0,-0.05,0),qPickUp1(end,:));
%robot.model.plot(qFlip1,'fps',60);

%% Flip 2
% p1 Flip to p2
TrPickUpReady2 = pattyPose2*troty(pi)*transl(0.15,0,-0.1)*troty(pi/4);
qp12p2 = RMRC(robot,robot.model.fkine(qFlip1(end,:)),TrPickUpReady2,qFlip1(end,:));
%robot.model.plot(qp12p2)
% p2 under
qUnder2 = RMRC(robot,TrPickUpReady2,pattyPose2*troty(pi)*transl(0.15,0,-0.01),qp12p2(end,:));
%robot.model.plot(qUnder2,'fps',100);
% p2 pickup
qPickUp2 = RMRC(robot,pattyPose2*troty(pi)*transl(0.15,0,-0.01),pattyPose2*troty(pi)*transl(0.15,0,-0.1),qUnder2(end,:));
%robot.model.plot(qPickUp2,'fps',200);
% p2 Flip

qFlip2 = repmat(qPickUp2(end,:),150,1);
prev = qPickUp2(end,:);
qFlip2(1,:) = prev;
for i = 2:150
    qFlip2(i,5) = qFlip2(i-1,5)-(deg2rad(90-74)/100);
end
robot.model.plot(qFlip2,'fps',30);

%% Flip 3
% p2 Flip to p3
TrPickUpReady3 = pattyPose3*troty(pi)*transl(0.15,0,-0.1)*troty(pi/4);
qp22p3 = RMRC(robot,robot.model.fkine(qFlip2(end,:)),TrPickUpReady3,qFlip2(end,:));
%robot.model.plot(qp22p3)
% p3 under
qUnder3 = RMRC(robot,TrPickUpReady3,pattyPose3*troty(pi)*transl(0.15,0,-0.01),qp22p3(end,:));
%robot.model.plot(qUnder3,'fps',100);
% p3 pickup
qPickUp3 = RMRC(robot,pattyPose3*troty(pi)*transl(0.15,0,-0.01),pattyPose3*troty(pi)*transl(0.15,0,-0.1),qUnder3(end,:));
%robot.model.plot(qPickUp3,'fps',200);
% p3 Flip
qFlip3 = repmat(qPickUp3(end,:),150,1);
prev = qPickUp3(end,:);
qFlip3(1,:) = prev;
for i = 2:150
    qFlip3(i,5) = qFlip3(i-1,5)-(deg2rad(90-74)/100)
end
%robot.model.plot(qFlip3,'fps',30);
>>>>>>> 237e332ec77bbd68c6b306604b2434accdd270ae

        %Get camera location
        Tc = robot.model.fkine(q);
        cam.T = Tc;

<<<<<<< HEAD
        drawnow
        
        % update the history variables
        hist.uv = uv(:);
        vel = v;
        hist.vel = vel;
        hist.e = e;
        hist.en = norm(e);
        hist.jcond = cond(J);
        hist.Tcam = Tc;
        hist.vel_p = vel;
        hist.uv_p = uv;
        hist.qp = qp;
        hist.q = q;

        history = [history hist];

         pause(1/fps)

        if ~isempty(200) && (ksteps > 200)
            break;
        end
        
        %update current joint position
        q0 = q;
 end %loop finishes
%% Loading objects
% % Load Oven
% [fOven,vOven,dataOven] = plyread('oven.ply','tri');
% ovenVertexColours = [dataOven.vertex.red,dataOven.vertex.green,dataOven.vertex.blue] / 255;
% oven_h = trisurf(fOven,vOven(:,1),vOven(:,2),vOven(:,3),'FaceVertexCData',ovenVertexColours,'EdgeColor','interp','EdgeLighting','flat');
% updatedOvenPosition = [ovenPose*[vOven,ones(size(vOven,1),1)]']';
% oven_h.Vertices = updatedOvenPosition(:,1:3);
%
% % Load Patties
% [fPatty,vPatty,dataPatty] = plyread('patty.ply','tri');
% pattyVertexColours = [dataPatty.vertex.red,dataPatty.vertex.green,dataPatty.vertex.blue] / 255;
% patty_h1 = trisurf(fPatty,vPatty(:,1),vPatty(:,2),vPatty(:,3),'FaceVertexCData',pattyVertexColours,'EdgeColor','interp','EdgeLighting','flat');
% patty_h2 = trisurf(fPatty,vPatty(:,1),vPatty(:,2),vPatty(:,3),'FaceVertexCData',pattyVertexColours,'EdgeColor','interp','EdgeLighting','flat');
% patty_h3 = trisurf(fPatty,vPatty(:,1),vPatty(:,2),vPatty(:,3),'FaceVertexCData',pattyVertexColours,'EdgeColor','interp','EdgeLighting','flat');
% patty_h4 = trisurf(fPatty,vPatty(:,1),vPatty(:,2),vPatty(:,3),'FaceVertexCData',pattyVertexColours,'EdgeColor','interp','EdgeLighting','flat');
% updatedPattyPosition1 = [pattyPose1*[vPatty,ones(size(vPatty,1),1)]']';
% updatedPattyPosition2 = [pattyPose2*[vPatty,ones(size(vPatty,1),1)]']';
% updatedPattyPosition3 = [pattyPose3*[vPatty,ones(size(vPatty,1),1)]']';
% updatedPattyPosition4 = [pattyPose4*[vPatty,ones(size(vPatty,1),1)]']';
% patty_h1.Vertices = updatedPattyPosition1(:,1:3);
% patty_h2.Vertices = updatedPattyPosition2(:,1:3);
% patty_h3.Vertices = updatedPattyPosition3(:,1:3);
% patty_h4.Vertices = updatedPattyPosition4(:,1:3);
% % Load Fry basket
% [fFry,vFry,dataFry] = plyread('basket.ply','tri');
% fryVertexColours = [dataFry.vertex.red,dataFry.vertex.green,dataFry.vertex.blue] / 255;
% fry_h = trisurf(fFry,vFry(:,1),vFry(:,2),vFry(:,3),'FaceVertexCData',fryVertexColours,'EdgeColor','interp','EdgeLighting','flat');
% updatedFryPosition = [fryPose*[vFry,ones(size(vFry,1),1)]']';
% fry_h.Vertices = updatedFryPosition(:,1:3);
% % Load Spatula
% [fSpat,vSpat,dataSpat] = plyread('spatula.ply','tri');
% spatVertexColours = [dataSpat.vertex.red,dataSpat.vertex.green,dataSpat.vertex.blue] / 255;
% spat_h = trisurf(fSpat,vSpat(:,1),vSpat(:,2),vSpat(:,3),'FaceVertexCData',spatVertexColours,'EdgeColor','interp','EdgeLighting','flat');
% updatedSpatPosition = [spatPose*[vSpat,ones(size(vSpat,1),1)]']';
% spat_h.Vertices = updatedSpatPosition(:,1:3);
% drawnow();
=======
% p4 Flip to p4
TrPickUpReady4 = pattyPose4*troty(pi)*transl(0.15,0,-0.1)*troty(pi/4);
qp32p4 = RMRC(robot,robot.model.fkine(qFlip3(end,:)),TrPickUpReady4,qFlip3(end,:));
robot.model.plot3d(qp32p4)
% p4 under
qUnder4 = RMRC(robot,TrPickUpReady4,pattyPose4*troty(pi)*transl(0.15,0,-0.01),qp32p4(end,:));
robot.model.plot3d(qUnder4,'fps',100);
% p3 pickup
qPickUp4 = RMRC(robot,pattyPose4*troty(pi)*transl(0.15,0,-0.01),pattyPose4*troty(pi)*transl(0.15,0,-0.1),qUnder4(end,:));
robot.model.plot3d(qPickUp4,'fps',200);
% p3 Flip
qFlip4 = repmat(qPickUp4(end,:),150,1);
prev = qPickUp4(end,:);
qFlip4(1,:) = prev;
for i = 2:150
    qFlip4(i,5) = qFlip4(i-1,5)-(deg2rad(90-74)/100)
end
robot.model.plot3d(qFlip4,'fps',30);


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

>>>>>>> 237e332ec77bbd68c6b306604b2434accdd270ae


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

