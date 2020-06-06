%% Dobot CR5 Class
classdef  CR5<handle
    properties
        model;
        workspace = [-2 2 -2 2 -0.2 2];
        qn = deg2rad([0    0   -100     0    90     0]);
        qMatrix;
        objPos;
        Dis;
%         cam = CentralCamera('focal', 0.08, 'pixel', 10e-5, ...
%             'resolution', [1024 1024], 'centre', [512 512],'name', 'UR10camera');

    end
    
    methods
        function self = CR5()
            self.GetCR5();
            self.PlotAndColourRobot();
        end
        
        function GetCR5(self)
            jlim = 360;
            L(1) = Link('d',0.147,'a',0,'alpha',pi/2,'offset',0,'qlim',[-deg2rad(jlim) deg2rad(jlim)]);
            L(2) = Link('d',0.025,'a',0.427,'alpha',0,'offset',pi/2,'qlim',[-deg2rad(360) deg2rad(360)]);
            L(3) = Link('d',0,'a',0.357,'alpha',0,'offset',0,'qlim',[-deg2rad(jlim) deg2rad(jlim)]);
            L(4) = Link('d',0.116,'a',0,'alpha',pi/2,'offset',pi/2,'qlim',[-deg2rad(jlim) deg2rad(jlim)]);
            L(5) = Link('d',0.116,'a',0,'alpha',-pi/2,'offset',0,'qlim',[-deg2rad(jlim) deg2rad(jlim)]);
            L(6) = Link('d',0.1,'a',0,'alpha',0,'offset',0,'qlim',[-deg2rad(jlim) deg2rad(jlim)]);
            

            self.model = SerialLink(L,'name',['CR5']);
            self.model.base = transl(-0.13,0,0.4);
            %transl(-0.13,0.02,0.38);
            %transl(-0.07,0.2,0.38);
            
        end

                function PlotAndColourRobot(self)%robot,workspace)
%                     self.model.plot(self.qn)
                    for linkIndex = 0:self.model.n
        %                 if self.useGripper && linkIndex == self.model.n
        %                     [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['UR5Link',num2str(linkIndex),'Gripper.ply'],'tri'); %#ok<AGROW>
        %                 else
                            [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['UR5Link',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
        %                 end
                        self.model.faces{linkIndex+1} = faceData;
                        self.model.points{linkIndex+1} = vertexData;
                    end
        
                    % Display robot
                    self.model.plot3d(zeros(1,self.model.n),'noarrow','workspace',self.workspace);
                    axis([-1.5 1.5 -1.5 1.5 0 3]);
                    if isempty(findobj(get(gca,'Children'),'Type','Light'))
                        camlight
                    end
                    self.model.delay = 0;
        
                    % Try to correctly colour the arm (if colours are in ply file data)
                    for linkIndex = 0:self.model.n
                        handles = findobj('Tag', self.model.name);
                        h = get(handles,'UserData');
                        try
                            h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                                , plyData{linkIndex+1}.vertex.green ...
                                , plyData{linkIndex+1}.vertex.blue]/255;
                            h.link(linkIndex+1).Children.FaceColor = 'interp';
                        catch ME_1
                            disp(ME_1);
                            continue;
                        end
                    end
                end
        
        %% Avoid Collision
        %         function AvoidCollison(self)
        %
        %         end
        %% LinePlaneIntersection
        
        %% Is Collision (REF: LAB 5 SOLUTION ON UTSONLINE)
        
        %% GET LINK POSES (REF: LAB 5 SOLUTION ON UTSONLINE)
        %% IsIntersectionPointInsideTriangle
        % Given a point which is known to be on the same plane as the triangle
        % determine if the point is
        % inside (result == 1) or
        % outside a triangle (result ==0 )
        
        %%%%%%%%%%%%%%%%
        function [qMatrix] = RMRC(self,T1,T2,qi)
            t = 1.5;                        % total time
            deltaT = 0.01;                % Step frequency
            steps =  t/deltaT;            % Number of simulation steps
            q0 = zeros(1,6);              % Initial guess
            qMatrix = zeros(steps,6);     % qMatrix Initialize
            qdot = zeros(steps,6);
            epsilon = 1.5 ;
            m = zeros(steps,1);           % Manipulability matrix
            position = zeros(3,steps);    % location of the transform
            theta = zeros(3,steps);       % Orientation of the transform
            
            % Extract the orientation of the poses
            orient1 = tr2rpy(T1);
            orient2 = tr2rpy(T2);
            %% Setup the trajectory
            s = lspb(0,1,steps);
            for i = 1:steps
                position(1,i) = (1-s(i))*T1(1,4) + s(i)*T2(1,4);    % X Coordinate
                position(2,i) = (1-s(i))*T1(2,4) + s(i)*T2(2,4);    % Y
                position(3,i) = (1-s(i))*T1(3,4) + s(i)*T2(3,4);    % Z
                theta(1,i) = (1-s(i))*orient1(1) + s(i)*orient2(1); % Roll
                theta(2,i) = (1-s(i))*orient1(2) + s(i)*orient2(2); % Pitch
                theta(3,i) = (1-s(i))*orient1(3) + s(i)*orient2(3); % Yaw
            end
            qMatrix(1,:) = qi;                      % Setup the first point
            %% Start RMRC (Reference UTSOnline Robotic Week 9)
            for i = 1:steps - 1
                T = self.model.fkine(qMatrix(i,:));
                deltaX = position(:,i+1) - T(1:3,4);
                Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1)); % Convert RPY angle to Rotation matrix
                Ra = T(1:3,1:3);
                Rdot = (1/deltaT)*(Rd-Ra);      %calculate rotation matrix
                S = Rdot*Ra';
                linearV = (1/deltaT)*deltaX;
                angularV = [S(3,2);S(1,3);S(2,1)];
                deltaTheta = tr2rpy(Rd*Ra');
                xDot = [linearV;angularV]; %Calculate velocity for next step
                J = self.model.jacob0(qMatrix(i,:));
                m(i) = sqrt(det(J*J'));         %Manipulability Matrix
                
                if m(i) < epsilon  % If manipulability is less than given threshold
                    lambda = (1 - m(i)/epsilon)*5E-2;
                else
                    lambda = 0;
                end
                
                invJ = inv(J'*J + lambda *eye(6))*J';                                   % DLS Inverse
                qdot(i,:) = (invJ*xDot)';
                % Check Joint limit
                for j = 1:6                                                             % Loop through joints 1 to 6
                    if qMatrix(i,j) + deltaT*qdot(i,j) < self.model.qlim(j,1)                     % If next joint angle is lower than joint limit...
                        qdot(i,j) = 0;
                    elseif qMatrix(i,j) + deltaT*qdot(i,j) > self.model.qlim(j,2)                 % If next joint angle is greater than joint limit ...
                        qdot(i,j) = 0;
                    end
                end
                qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);
                
            end
        end
        %%
        function CalDis(self) %%Calculate distance from robot to any object
            A = self.model.fkine(self.model.getpos());
            A = A(1:3,end);
            self.Dis = sqrt((self.objPos(1)-A(1))^2 + (self.objPos(2)-A(2))^2 + (self.objPos(3)-A(3))^2);
        end
        %% Set up the camera
        function SUCam(self)
            

        end

        end 

    end
  