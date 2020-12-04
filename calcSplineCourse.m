%�����ڲο������µĲο���r�Ĳ���rx,ry ,ryaw,rk,�����߳���s�Ͳο���objSpline
function [rx,ry ,ryaw,rk,s,objSpline]=calcSplineCourse(x, y, ds)
    objSpline=Spline2D(x,y);%�ο���objSpline
    %��ʾ��0Ϊ���,��objSpline.s(end)Ϊ�յ�,��dsΪ������һά����
    s=0:ds:objSpline.s(end);
    rx=[];
    ry=[];
    ryaw=[];
    rk=[];
    for i_s = s
        [ix,iy]=objSpline.calc_position(i_s);%��s-x��s-y����ϵ�¼����Ӧ���߳����µ�xֵ��yֵ
        rx(end+1)=ix;%�ο���r��x����
        ry(end+1)=iy;%�ο���r��y����
        ryaw(end+1)=objSpline.calc_yaw(i_s);%�ο���r�ķ�λ��
        rk(end+1)=objSpline.calc_curvature(i_s);%�ο���r������
    end
end 