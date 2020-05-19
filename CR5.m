%% Dobot CR5 Class
classdef  CR5<handle
    properties
        model;
        workspace = [-2 2 -2 2 -0.2 2];
    end
    
    methods
    function self = CR5()     
    self.GetCR5();
  %  self.PlotAndColour();
    end
    
    function GetCR5(self)
        
    L(1) = Link('d',0.147,'a',0,'alpha',pi/2,'offset',0,'qlim',[-deg2rad(360) deg2rad(360)]);
    L(2) = Link('d',0.025,'a',0.427,'alpha',0,'offset',0,'qlim',[-deg2rad(360) deg2rad(360)]);
    L(3) = Link('d',0,'a',0.357,'alpha',0,'offset',0,'qlim',[-deg2rad(160) deg2rad(160)]);
    L(4) = Link('d',0.116,'a',0,'alpha',pi/2,'offset',-pi/2,'qlim',[-deg2rad(360) deg2rad(360)]);
    L(5) = Link('d',0.116,'a',0,'alpha',-pi/2,'offset',0,'qlim',[-deg2rad(360) deg2rad(360)]);
    L(6) = Link('d',0.114,'a',0,'alpha',0,'offset',0,'qlim',[-deg2rad(360) deg2rad(360)]);
    
    self.model = SerialLink(L,'name',['CR5_',datestr(now,'yyyymmddTHHMMSSFFF')])
    end
%     function PlotAndColour()
%         for linkInd = 0:self.model.n
%             
     end
end