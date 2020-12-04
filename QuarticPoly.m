%�Ĵζ���ʽ�Ĳ�������
classdef QuarticPoly
        % QuarticPoly Summary
        properties   %Ĭ��������ǹ��У�public��������
            %coefficients (c0, c1, c2, c3, c4)
            c%��һ�����飬5ά
        end
        methods (Access = public) 
         %% ����ϵ��(c0, c1, c2, c3, c4)
            function obj = QuarticPoly(xi0, dxi0, ddxi0, dxiT, ddxiT, T)
                c012= [xi0; dxi0 ; ddxi0 / 2.0];%��ʼʱ��t0=0,�ɵ�c0, c1, c2
                
                M1 = [0, 1, 2*T;0, 0, 2 ];
                      
                M2= [3*T^2,4*T^3;6*T,12*T^2];
                c34 = M2\([dxiT; ddxiT] - M1 * c012);
                obj.c = [c012; c34];
            end
           %% ����yֵ��y=c0+c1x+c2x2+c3x3+c4x4
            function x = X(obj, t)
                 x = obj.c(1) + obj.c(2).*t + obj.c(3).*t.^2 + obj.c(4).*t.^3 + obj.c(5).*t.^4;   
            end
           %% ����y'ֵ��y=c1+2c2x+3c3x2+4c4x3        
            function x = dX(obj, t)
                 x = obj.c(2) + 2*obj.c(3).*t + 3*obj.c(4).*t.^2 + 4*obj.c(5).*t.^3;   
            end
           %% ����y''ֵ��y=2c2+6c3x+12c4x2            
            function x = ddX(obj, t)
                 x = 2*obj.c(3) + 6*obj.c(4).*t + 12*obj.c(5).*t.^2;   
            end
           %% ����y'''ֵ��y=6c3+24c4x         
            function x = dddX(obj, t)
                 x = 2 + 6*obj.c(4) + 24*obj.c(5).*t;   
            end     
        end
end