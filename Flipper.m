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

        %Get camera location
        Tc = robot.model.fkine(q);
        cam.T = Tc;

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

