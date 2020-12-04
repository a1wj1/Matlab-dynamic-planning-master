%计算frenet坐标系下的曲线参数
classdef Spline2D

        properties(Access=public)%公有（public）的属性
            s%曲线长度
            sx
            sy
            ds
        end
        methods(Access=public)
            function obj=Spline2D(x, y)
            %initial the variable
            obj.s=obj.calc_s(x, y);%曲线长度
            obj.sx=Spline(obj.s, x);%s-x坐标系，
            obj.sy=Spline(obj.s, y);%s-y坐标系
            end
          %% calculate running length s  曲线长度,是一个数组，记录每一段的s长度   
            function s=calc_s(obj, x, y)
                 dx=diff(x);
                 dy=diff(y);
                 obj.ds=zeros(length(dx),1);
                 for i=1:length(dx)
                     idx=dx(i);
                     idy=dy(i);
                     obj.ds(i)=sqrt(idx^2+idy^2);
                 end
                 s=[0, cumsum(obj.ds)'];
            end
           %% calculate position     位置
            function [x, y]=calc_position(obj, s)
                x=obj.sx.calc(s);
                y=obj.sy.calc(s);
            end 
           %% calculate the curvature  曲率 
            function k=calc_curvature(obj, s)
               dx=obj.sx.calcd(s);
               ddx=obj.sx.calcdd(s);
               dy=obj.sy.calcd(s);
               ddy=obj.sy.calcdd(s);
               k=(ddy * dx - ddx * dy ) / (dx^2 + dy^2);
            end
           %% calculate yaw angle    航向角
            function yaw=calc_yaw(obj,s)      
                dx=obj.sx.calcd(s);
                dy=obj.sy.calcd(s);
                yaw=atan2(dy,dx);
            end     
        end
end



