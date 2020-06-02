%% Dobot CR5 Class
classdef  CR5<handle
    properties
        MountPos;
        model;
        workspace = [-2 2 -2 2 -0.2 2];
        qMatrix;
    end
    
    methods
        function self = CR5()
            self.GetCR5();
            self.PlotAndColourRobot();
        end
        
        function GetCR5(self)
            
            L(1) = Link('d',0.147,'a',0,'alpha',pi/2,'offset',0,'qlim',[-deg2rad(180) deg2rad(180)]);
            L(2) = Link('d',0.025,'a',0.427,'alpha',0,'offset',0,'qlim',[-deg2rad(180) deg2rad(180)]);
            L(3) = Link('d',0,'a',0.357,'alpha',0,'offset',0,'qlim',[-deg2rad(160) deg2rad(160)]);
            L(4) = Link('d',0.116,'a',0,'alpha',pi/2,'offset',pi/2,'qlim',[-deg2rad(180) deg2rad(180)]);
            L(5) = Link('d',0.116,'a',0,'alpha',-pi/2,'offset',0,'qlim',[-deg2rad(180) deg2rad(180)]);
            L(6) = Link('d',0.114,'a',0,'alpha',0,'offset',0,'qlim',[-deg2rad(360) deg2rad(360)]);
            
            self.model = SerialLink(L,'name',['CR5']);
            self.MountPos = input('Please type in CR5 base position: ');
            self.model.base = self.MountPos;
            
        end
        %     function PlotAndColour()
        %         for linkInd = 0:self.model.n
        %
        
        
        %% PlotAndColourRobot
        % Given a robot index, add the glyphs (vertices and faces) and
        % colour them in if data is available
        function PlotAndColourRobot(self)
            q= [0 0 0 0 0 0] ;
            self.model.plot(q);
        end
        %         function PlotAndColourRobot(self)%robot,workspace)
        %             for linkIndex = 0:self.model.n
        % %                 if self.useGripper && linkIndex == self.model.n
        % %                     [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['UR5Link',num2str(linkIndex),'Gripper.ply'],'tri'); %#ok<AGROW>
        % %                 else
        %                     [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['UR5Link',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
        % %                 end
        %                 self.model.faces{linkIndex+1} = faceData;
        %                 self.model.points{linkIndex+1} = vertexData;
        %             end
        %
        %             % Display robot
        %             self.model.plot3d(zeros(1,self.model.n),'noarrow','workspace',self.workspace);
        %             if isempty(findobj(get(gca,'Children'),'Type','Light'))
        %                 camlight
        %             end
        %             self.model.delay = 0;
        %
        %             % Try to correctly colour the arm (if colours are in ply file data)
        %             for linkIndex = 0:self.model.n
        %                 handles = findobj('Tag', self.model.name);
        %                 h = get(handles,'UserData');
        %                 try
        %                     h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
        %                         , plyData{linkIndex+1}.vertex.green ...
        %                         , plyData{linkIndex+1}.vertex.blue]/255;
        %                     h.link(linkIndex+1).Children.FaceColor = 'interp';
        %                 catch ME_1
        %                     disp(ME_1);
        %                     continue;
        %                 end
        %             end
        %         end
        %% Avoid Collision
        function AvoidCollison(self)
            
        end
        %% Is Collision (REF: LAB 5 SOLUTION ON UTSONLINE)
        function result = IsCollision(self,qMatrix,faces,vertex,faceNormals,returnOnceFound)
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
        %% GET LINK POSES (REF: LAB 5 SOLUTION ON UTSONLINE)
        function [ transforms ] = GetLinkPoses( q, self)
            links = self.model.links;
            transforms = zeros(4, 4, length(links) + 1);
            transforms(:,:,1) = self.model.base;
            for i = 1:length(links)
                L = links(1,i);
                current_transform = transforms(:,:, i);
                current_transform = current_transform * trotz(q(1,i) + L.offset) * ...
                    transl(0,0, L.d) * transl(L.a,0,0) * trotx(L.alpha);
                transforms(:,:,i + 1) = current_transform;
            end
        end
    end
end
