classdef OptimalFrenetPlanner
    %define the frenet plannar algorithm
    properties
    % 1.设置参数
    MAX_SPEED=50.0/3.6;         % 最大速度 [m/s]，纵向和横向速度的尺量合
    MAX_ACCEL=2.0;                  % 最大加速度[m/ss]
    MAX_CURVATURE=1.0;          % 最大曲率 [1/m]
    MAX_ROAD_WIDTH=7.0;       % 最大道路宽度 [m]
    MIN_ROAD_WIDTH= -7.0      % 最小道路宽度 [m]
    D_ROAD_W=1.0;                  % 道路宽度采样间隔 [m]
    DT=0.2;                               % time tick [s]
    MAXT=5.0;                           % 最大预测时间 [s]
    MINT=4.0;                            %最小预测时间 [s]
    TARGET_SPEED=30.0/3.6;    % 目标速度（即纵向的速度保持） [m/s]
    D_T_S=5/3.6;                      % 目标速度采样间隔 [m/s]
    N_S_SAMPLE=1;                   % 目标速度的采样数量
    ROBOT_RADIUS=2.0;             % robot半径[m]

    % 损失函数权重
    KJ=0.1;                                  % Jerk     
    KT=0.1;                                 % time
    KD=1.0;                                 % Distance from reference path
    KV=1.0;                                 % Target speed
    KLAT=1.0;                              % Lateral
    KLON=1.0;                             % Longitudinal      

    numberObjects
    end
     
    methods ( Access = public)
       %% 计算frenet空间中的轨迹
        %使用给定的启动动力学并生成轨迹变化
        %-横向位置-lateral宽度：delta_road_宽度：最大路面宽度
        %s0：纵向起始位置
        %ds0：初始纵向速度
        %d0：横向起始位置（从参考路径偏移）
        %dd0：初始横向速度
        %ddd0：初始横向加速度  
        function frenetTrajectories = CalcFrenetTrajectories(obj, s0, ds0,d0,dd0,ddd0)
            % 为每个偏移目标生成路径
            % Lateral sampling space采样空间
            sizeLatSampleSpace = length(obj.MIN_ROAD_WIDTH:obj.D_ROAD_W:obj.MAX_ROAD_WIDTH);%Lat即横向偏移，采样值
            sizeTimeSpace=length(obj.MINT:obj.DT:obj.MAXT);%时间采样值
            %纵向目标速度-采样值X采样数量
            sizeLonSampleSpace=length((obj.TARGET_SPEED - obj.D_T_S * obj.N_S_SAMPLE):obj.D_T_S:(obj.TARGET_SPEED + obj.D_T_S * obj.N_S_SAMPLE));%Lon即纵向偏移
            %轨迹的数量
            numberTrajectories=sizeLatSampleSpace*sizeTimeSpace*sizeLonSampleSpace;
            %轨迹数量
            frenetTrajectories=cell(1, numberTrajectories);%,创建一个空的1xnumberTrajectories的cell矩阵.
            iTraj=1;
            %采样，并对每一个目标配置生成轨迹
            for di=obj.MIN_ROAD_WIDTH: obj.D_ROAD_W:obj.MAX_ROAD_WIDTH%其实就是采样的个数，以d/t，s/t为坐标去数
                %横向规划
                for Ti=obj.MINT:obj.DT:obj.MAXT
                    %用动力学方法生成平面五次多项式
                    %d0；起始位置偏移
                    %dd0：起始横向速度
                    %ddd0：开始横向加速度
                    % di: Varoated lateral target lateral position
                    %ddT：横向目标速度
                    %dddT：横向目标加速度
                    ddT=0;
                    dddT=0;
                    %计算出关于目标配置di，Ti的横向多项式，5次多项式
                    %d0, dd0, ddd0为初始配置， di, ddT, dddT为目标配置
                    latPoly5 = QuinticPoly(d0, dd0, ddd0, di, ddT, dddT, Ti);     
                    %创建由s（纵向）和d（横向）动力学组成的frenetTrajectories，并初始化侧向（d）部分
                    ft=FrenetTrajectory();%调用类，FrenetTrajectory里面定义了一系列参数
                    ft.t=0.0:obj.DT:Ti;%时间戳，这样写是生成数组
                    ft.d=latPoly5.X(ft.t);%取值
                    ft.dd=latPoly5.dX(ft.t);%取一阶导数值
                    ft.ddd=latPoly5.ddX(ft.t);%取二阶导数值
                    ft.dddd=latPoly5.dddX(ft.t);%取三阶导数值
                    %hold on
                    %plot(ft.t,ft.d,'*r')     
                    %纵向速度规划 (速度保持)
                    for  tv = (obj.TARGET_SPEED - obj.D_T_S * obj.N_S_SAMPLE): obj.D_T_S: (obj.TARGET_SPEED + obj.D_T_S * obj.N_S_SAMPLE)
                          targetft=ft;
                          %四次多项式
                          lonPoly4=QuarticPoly(s0, ds0, 0.0, tv, 0.0, Ti);            
                          targetft.s=lonPoly4.X(ft.t);%取值
                          targetft.ds=lonPoly4.dX(ft.t);%取一阶导数值
                          targetft.dds=lonPoly4.ddX(ft.t);%取二阶导数值
                          targetft.ddds=lonPoly4.dddX(ft.t);%取三阶导数值
                          % hold on
                          % plot(ft.t,targetft.s,'*g')
                          % Square of lateral jerk，优化对象为加加速度的平方
                          Jd = sum(targetft.dddd.^2);
                          % Square of longitudinal jerk，优化对象为加加速度的平方
                          Js = sum(targetft.ddds.^2);  
                          % Square of diff from target speed
                          dv = (obj.TARGET_SPEED - targetft.ds(end)).^2;
                          targetft.Jd = obj.KJ * Jd + obj.KT * Ti + obj.KD * targetft.d(end)^2 ;
                          targetft.Js = obj.KJ * Js + obj.KT * Ti + obj.KV * dv;%obj.KV * dv表示最后的速度与目标速度偏差不要太大
                          targetft.J=obj.KLAT * targetft.Jd + obj.KLON * targetft.Js;%综合两个J公式   
                          frenetTrajectories{iTraj} = targetft;
                          iTraj = iTraj + 1;%加1，下一次遍历
                    end
                end
            end
        end
        %% 世界坐标系下的轨迹转换
        function frenetTrajectories = CalcGlobalTrajectories(obj, frenetTrajectories, referencePath)
            for iTarj = 1: length(frenetTrajectories)
                 ft = frenetTrajectories{iTarj};
                 % 计算世界坐标系下的位置x,y
                 for i = 1:(length(ft.s))%遍历曲线长度
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
       %% 碰撞检测            
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
       %% 检查轨迹是否可行（要满足最大条件）
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
        %% Frenet的轨迹最优化选择       
        function bestpath = FrenetOptimalPlanning(obj, referencePath, s0, ds0, d0, dd0, ddd0, objects)
            % Initialization
            obj.numberObjects = size(objects, 1);%objects传入障碍物列表
            
            frenetTrajectories = obj.CalcFrenetTrajectories(s0, ds0, d0, dd0, ddd0);%计算Frenet坐标系下的轨迹
            frenetTrajectories = obj.CalcGlobalTrajectories(frenetTrajectories, referencePath);%计算世界坐标系下的轨迹，就是Frenet下的轨迹规划好后，转成笛卡尔显示出来
            frenetTrajectories = obj.CheckTrajectories(frenetTrajectories, objects);%检查世界坐标系下的轨迹是否合格限制条件
            
            % frenetTrajectories是个列表，找出cost最小的那一条
            mincost = inf;%inf为无穷大量+∞,-inf为无穷小量-∞
            bestpath = NaN;
            for iTraj = 1: (length(frenetTrajectories))
                ft = frenetTrajectories{iTraj};
                if (mincost >= ft.J)%.J即代价函数，找出它最小的那条就是
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
    
    
    
    
    
    
    
    
    
    
    
    
    