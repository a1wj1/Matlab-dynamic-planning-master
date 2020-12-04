classdef OptimalFrenetPlanner
    %define the frenet plannar algorithm
    properties
    % 1.���ò���
    MAX_SPEED=50.0/3.6;         % ����ٶ� [m/s]������ͺ����ٶȵĳ�����
    MAX_ACCEL=2.0;                  % �����ٶ�[m/ss]
    MAX_CURVATURE=1.0;          % ������� [1/m]
    MAX_ROAD_WIDTH=7.0;       % ����·��� [m]
    MIN_ROAD_WIDTH= -7.0      % ��С��·��� [m]
    D_ROAD_W=1.0;                  % ��·��Ȳ������ [m]
    DT=0.2;                               % time tick [s]
    MAXT=5.0;                           % ���Ԥ��ʱ�� [s]
    MINT=4.0;                            %��СԤ��ʱ�� [s]
    TARGET_SPEED=30.0/3.6;    % Ŀ���ٶȣ���������ٶȱ��֣� [m/s]
    D_T_S=5/3.6;                      % Ŀ���ٶȲ������ [m/s]
    N_S_SAMPLE=1;                   % Ŀ���ٶȵĲ�������
    ROBOT_RADIUS=2.0;             % robot�뾶[m]

    % ��ʧ����Ȩ��
    KJ=0.1;                                  % Jerk     
    KT=0.1;                                 % time
    KD=1.0;                                 % Distance from reference path
    KV=1.0;                                 % Target speed
    KLAT=1.0;                              % Lateral
    KLON=1.0;                             % Longitudinal      

    numberObjects
    end
     
    methods ( Access = public)
       %% ����frenet�ռ��еĹ켣
        %ʹ�ø�������������ѧ�����ɹ켣�仯
        %-����λ��-lateral��ȣ�delta_road_��ȣ����·����
        %s0��������ʼλ��
        %ds0����ʼ�����ٶ�
        %d0��������ʼλ�ã��Ӳο�·��ƫ�ƣ�
        %dd0����ʼ�����ٶ�
        %ddd0����ʼ������ٶ�  
        function frenetTrajectories = CalcFrenetTrajectories(obj, s0, ds0,d0,dd0,ddd0)
            % Ϊÿ��ƫ��Ŀ������·��
            % Lateral sampling space�����ռ�
            sizeLatSampleSpace = length(obj.MIN_ROAD_WIDTH:obj.D_ROAD_W:obj.MAX_ROAD_WIDTH);%Lat������ƫ�ƣ�����ֵ
            sizeTimeSpace=length(obj.MINT:obj.DT:obj.MAXT);%ʱ�����ֵ
            %����Ŀ���ٶ�-����ֵX��������
            sizeLonSampleSpace=length((obj.TARGET_SPEED - obj.D_T_S * obj.N_S_SAMPLE):obj.D_T_S:(obj.TARGET_SPEED + obj.D_T_S * obj.N_S_SAMPLE));%Lon������ƫ��
            %�켣������
            numberTrajectories=sizeLatSampleSpace*sizeTimeSpace*sizeLonSampleSpace;
            %�켣����
            frenetTrajectories=cell(1, numberTrajectories);%,����һ���յ�1xnumberTrajectories��cell����.
            iTraj=1;
            %����������ÿһ��Ŀ���������ɹ켣
            for di=obj.MIN_ROAD_WIDTH: obj.D_ROAD_W:obj.MAX_ROAD_WIDTH%��ʵ���ǲ����ĸ�������d/t��s/tΪ����ȥ��
                %����滮
                for Ti=obj.MINT:obj.DT:obj.MAXT
                    %�ö���ѧ��������ƽ����ζ���ʽ
                    %d0����ʼλ��ƫ��
                    %dd0����ʼ�����ٶ�
                    %ddd0����ʼ������ٶ�
                    % di: Varoated lateral target lateral position
                    %ddT������Ŀ���ٶ�
                    %dddT������Ŀ����ٶ�
                    ddT=0;
                    dddT=0;
                    %���������Ŀ������di��Ti�ĺ������ʽ��5�ζ���ʽ
                    %d0, dd0, ddd0Ϊ��ʼ���ã� di, ddT, dddTΪĿ������
                    latPoly5 = QuinticPoly(d0, dd0, ddd0, di, ddT, dddT, Ti);     
                    %������s�����򣩺�d�����򣩶���ѧ��ɵ�frenetTrajectories������ʼ������d������
                    ft=FrenetTrajectory();%�����࣬FrenetTrajectory���涨����һϵ�в���
                    ft.t=0.0:obj.DT:Ti;%ʱ���������д����������
                    ft.d=latPoly5.X(ft.t);%ȡֵ
                    ft.dd=latPoly5.dX(ft.t);%ȡһ�׵���ֵ
                    ft.ddd=latPoly5.ddX(ft.t);%ȡ���׵���ֵ
                    ft.dddd=latPoly5.dddX(ft.t);%ȡ���׵���ֵ
                    %hold on
                    %plot(ft.t,ft.d,'*r')     
                    %�����ٶȹ滮 (�ٶȱ���)
                    for  tv = (obj.TARGET_SPEED - obj.D_T_S * obj.N_S_SAMPLE): obj.D_T_S: (obj.TARGET_SPEED + obj.D_T_S * obj.N_S_SAMPLE)
                          targetft=ft;
                          %�Ĵζ���ʽ
                          lonPoly4=QuarticPoly(s0, ds0, 0.0, tv, 0.0, Ti);            
                          targetft.s=lonPoly4.X(ft.t);%ȡֵ
                          targetft.ds=lonPoly4.dX(ft.t);%ȡһ�׵���ֵ
                          targetft.dds=lonPoly4.ddX(ft.t);%ȡ���׵���ֵ
                          targetft.ddds=lonPoly4.dddX(ft.t);%ȡ���׵���ֵ
                          % hold on
                          % plot(ft.t,targetft.s,'*g')
                          % Square of lateral jerk���Ż�����Ϊ�Ӽ��ٶȵ�ƽ��
                          Jd = sum(targetft.dddd.^2);
                          % Square of longitudinal jerk���Ż�����Ϊ�Ӽ��ٶȵ�ƽ��
                          Js = sum(targetft.ddds.^2);  
                          % Square of diff from target speed
                          dv = (obj.TARGET_SPEED - targetft.ds(end)).^2;
                          targetft.Jd = obj.KJ * Jd + obj.KT * Ti + obj.KD * targetft.d(end)^2 ;
                          targetft.Js = obj.KJ * Js + obj.KT * Ti + obj.KV * dv;%obj.KV * dv��ʾ�����ٶ���Ŀ���ٶ�ƫ�Ҫ̫��
                          targetft.J=obj.KLAT * targetft.Jd + obj.KLON * targetft.Js;%�ۺ�����J��ʽ   
                          frenetTrajectories{iTraj} = targetft;
                          iTraj = iTraj + 1;%��1����һ�α���
                    end
                end
            end
        end
        %% ��������ϵ�µĹ켣ת��
        function frenetTrajectories = CalcGlobalTrajectories(obj, frenetTrajectories, referencePath)
            for iTarj = 1: length(frenetTrajectories)
                 ft = frenetTrajectories{iTarj};
                 % ������������ϵ�µ�λ��x,y
                 for i = 1:(length(ft.s))%�������߳���
                     [ix, iy] = referencePath.calc_position(ft.s(i));
                     if isnan(ix)
                         break
                     end
                     iyaw = referencePath.calc_yaw(ft.s(i));
                     di = ft.d(i);
                     fx = ix + di * cos(iyaw + pi /2.0);
                     fy = iy + di * sin(iyaw + pi / 2.0);
                     ft.x(end+1)=fx;
                     ft.y(end+1)=fy;
                     
                 end
                 % plot(ft.x, ft.y, 'color', [1, 1, 1]*0.5)
                 % drawnow;
                 % calc theta and dL (running length)
                 for i = 1: (length(ft.x) - 1) 
                     dx = ft.x(i+1) - ft.x(i);
                     dy = ft.y(i+1) - ft.y(i);
                     ft.theta(end+1) = atan2(dy, dx);
                     ft.dL(end+1) = sqrt(dx^2 + dy^2);
                 end
                 ft.theta(end+1) = ft.theta(end);
                 ft.dL(end+1) = ft.dL(end);           
                 % calc curvature
                 for i = 1: (length(ft.theta) - 1)
                     ft.kappa(end+1) = (ft.theta(i+1) - ft.theta(i)) / ft.dL(i) ;
                 end
                 ft.kappa(end+1) = ft.kappa(end);           
                 frenetTrajectories{iTarj} = ft;           
            end
        end
       %% ��ײ���            
        function collision = CheckCollision(obj, ft, objects) 
                for i = 1:obj.numberObjects
                    ox = objects(i, 1);
                    oy = objects(i, 2);
                    d = zeros(length(ft.x), 1);
                    for idxPoint = 1:length(ft.x)
                        ix = ft.x(idxPoint);
                        iy = ft.y(idxPoint);
                        d(idxPoint) = ((ix - ox)^2 + (iy - oy)^2);
                    end
                    collision = any(d <= 1^2);
                    if collision
%                         plot(ft.x, ft.y, 'rx')
%                         plot(ox, oy, 'yo');
%                         drawnow;
                        return;
                    end
                end
                collision = 0;
        end
       %% ���켣�Ƿ���У�Ҫ�������������
        function okTrajectories = CheckTrajectories(obj, frenetTrajectories, objects)
            okTrajectories = {};
            for i = 1 : (length( frenetTrajectories))
                ft = frenetTrajectories{i};
                if any(ft.ds > obj.MAX_SPEED)  % Max speed check
                    continue
                elseif any(abs(ft.dds) > obj.MAX_ACCEL)     % Max accleration check
                    continue
                elseif any(abs(ft.kappa) > obj.MAX_CURVATURE)   % Max curvature check
                    continue
                elseif (obj.CheckCollision(ft, objects)==1)
                    continue
                end
                okTrajectories{end+1} = ft;
%                 plot(ft.x, ft.y, 'g');
%                 drawnow;
            end
        end
        %% Frenet�Ĺ켣���Ż�ѡ��       
        function bestpath = FrenetOptimalPlanning(obj, referencePath, s0, ds0, d0, dd0, ddd0, objects)
            % Initialization
            obj.numberObjects = size(objects, 1);%objects�����ϰ����б�
            
            frenetTrajectories = obj.CalcFrenetTrajectories(s0, ds0, d0, dd0, ddd0);%����Frenet����ϵ�µĹ켣
            frenetTrajectories = obj.CalcGlobalTrajectories(frenetTrajectories, referencePath);%������������ϵ�µĹ켣������Frenet�µĹ켣�滮�ú�ת�ɵѿ�����ʾ����
            frenetTrajectories = obj.CheckTrajectories(frenetTrajectories, objects);%�����������ϵ�µĹ켣�Ƿ�ϸ���������
            
            % frenetTrajectories�Ǹ��б��ҳ�cost��С����һ��
            mincost = inf;%infΪ�������+��,-infΪ����С��-��
            bestpath = NaN;
            for iTraj = 1: (length(frenetTrajectories))
                ft = frenetTrajectories{iTraj};
                if (mincost >= ft.J)%.J�����ۺ������ҳ�����С����������
                    mincost = ft.J;
                    bestpath = ft;
                end
            end
            
        end
        
        function referencePath = CalcReferencePath(x, y, ds)
            referencePath = CalcSplineCourse(x, y, ds);
        end
        
    end
end
    
    
    
    
    
    
    
    
    
    
    
    
    