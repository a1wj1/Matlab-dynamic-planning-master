%计算在参考曲线下的参考点r的参数rx,ry ,ryaw,rk,与曲线长度s和参考线objSpline
function [rx,ry ,ryaw,rk,s,objSpline]=calcSplineCourse(x, y, ds)
    objSpline=Spline2D(x,y);%参考线objSpline
    %表示以0为起点,以objSpline.s(end)为终点,以ds为步长的一维矩阵
    s=0:ds:objSpline.s(end);
    rx=[];
    ry=[];
    ryaw=[];
    rk=[];
    for i_s = s
        [ix,iy]=objSpline.calc_position(i_s);%在s-x和s-y坐标系下计算对应曲线长度下的x值和y值
        rx(end+1)=ix;%参考点r的x坐标
        ry(end+1)=iy;%参考点r的y坐标
        ryaw(end+1)=objSpline.calc_yaw(i_s);%参考点r的方位角
        rk(end+1)=objSpline.calc_curvature(i_s);%参考点r的曲率
    end
end 