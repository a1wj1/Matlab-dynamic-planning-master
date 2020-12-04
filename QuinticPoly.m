%��ζ���ʽ�Ĳ�������
classdef QuinticPoly
    % solve the quinticPoly 5th order equation
    properties
     % coefficients (c0, c1, c2, c3, c4, c5)
     c
    end
    
    methods (Access = public)
       %% ����ϵ��(c0, c1, c2, c3, c4, c5)   
        function obj = QuinticPoly (xi0, dxi0, ddxi0, xiT, dxiT, ddxiT, T)
           c012= [xi0; dxi0; ddxi0 / 2.0];%��t0=0,����c0, c1, c2
           %�� T=t1?t0������ c3, c4, c5
           M1=[1, T,  T*T;0, 1, 2*T;0, 0, 2 ];     
           M2=[T^3,T^4, T^5;3*T^2,4*T^3,5*T^4;6*T,12*T^2,20*T^3];        
           c345 = M2\([xiT; dxiT; ddxiT] - M1 * c012);
           obj.c = [c012; c345];
        end
       %% ����yֵ��y=c0+c1x+c2x2+c3x3+c4x4+c5x5     
        function x=X(obj, t)
            x=obj.c(1) + obj.c(2).*t + obj.c(3).*t.^2 + obj.c(4).*t.^3 + obj.c(5).*t.^4 + obj.c(6).*t.^5;
        end
       %% ����y'ֵ��y=c1+2c2x+3c3x2+4c4x3++5c5x4         
        function x=dX(obj, t)
            x=obj.c(2) + 2*obj.c(3).*t + 3*obj.c(4).*t.^2 + 4*obj.c(5).*t.^3 + 5* obj.c(6).*t.^4;
        end
       %% ����y''ֵ��y=2c2+6c3x+12c4x2++20c5x3 
        function x=ddX(obj, t)
            x= 2*obj.c(3) + 6*obj.c(4).*t + 12*obj.c(5).*t.^2 + 20* obj.c(6).*t.^3;
        end
       %% ����y'''ֵ��y=6c3+24c4x++60c5x2        
        function x=dddX(obj, t)
            x=  2 + 6*obj.c(4) + 24*obj.c(5).*t + 60* obj.c(6).*t.^2;
        end     
       
    end
end

            