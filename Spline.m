%��������y=ax^3+bx^2+cx+d
%��������������ֵ��ϵ��ai.bi.ci.di
classdef Spline
    properties(Access=public)%�ǹ��У�public��������
        a
        b
        c
        d
        w   
        x
        y     
        nx
    end
    methods(Access=public)%���г�Ա����
        %����Spline���캯��������������ֵ������������Կ��ǻ����Լ��Ĳ�ֵ����
        %ע�������x,y�������飬���㣨x0��y0��,(x1,y1),...,(xn,yn)
        function obj =Spline(x,y)
            %������ʼ��
            obj.b=[];
            obj.c=[];
            obj.d=[];
            obj.w=[];
            obj.x=x;
            obj.y=y;
            obj.nx=length(x)   %����x����
            %��������X��diff��X����[X��2��-X��1����X��3��-X��2��������X��n��-X��n-1��]��
            h=diff(x);%����x֮��Ĳ�   
            obj.a=y;%����ai,�����㷨��ai=yi
            %�������c
            A=obj.calc_A(h);%���ϵ������
            B=obj.calc_B(h);
            obj.c=A\B;%���̵Ľ�
            %ci=mi/2,�����obj.c=mi/2=A\B;��Ϊ�ڼ���B��ʱ��ϵ����3���Ѿ�����2����������ֱ�ӵ��ڡ�
            obj.c=(obj.c)';%�����ת�ã� %�������c
            
            %�������b��d
            for i = 1:(obj.nx-1)
                obj.d(end+1)=(obj.c(i+1)-obj.c(i)) / (3.0*h(i));
                tb= (obj.a(i+1) - obj.a(i) ) / h(i)- h(i)* (obj.c(i+1)+2.0*obj.c(i))/ 3.0;
                obj.b(end+1)=tb;    
            end
        end
       %% calculate matrix A for spline coefficient c            
        function A = calc_A(obj, h)%obj�൱��Spline���캯���Ķ��󣬿��Ե�������Ĳ���
            A= zeros(obj.nx, obj.nx);
            A(1, 1)= 1.0;
            for i = 1:(obj.nx - 1)
                if i ~= (obj.nx - 1)%~=�ǲ�����
                    A(i+1,i+1) = 2.0*(h(i) + h(i + 1));
                end
                A(i + 1,i) = h(i);
                A(i, i + 1)= h(i);
            end
            A(1, 2)=0.0;
            A(obj.nx, obj.nx-1) = 0.0;
            A(obj.nx, obj.nx) = 1.0;
        end
       %% calc matrix B for spline coefficient c           
        function B = calc_B(obj, h)

            B= zeros(obj.nx,1);
            for i = 1:(obj.nx-2)
                B(i+1)=3.0*(obj.a(i+2)-obj.a(i+1))/ ...
                      h(i+1)-3.0*(obj.a(i+1)-obj.a(i))/h(i) ;
            end         
        end
       %% calculate position   (t,result)     
        function result=calc(obj,t)
            %if t is outside of the input x, return None
            if (t<obj.x(1))
                result= NaN;
                return;
            elseif ( t>obj.x(end))
                result=NaN;
                return;
            end
            %���򷵻�xֵ��Ӧ�����������ϵ�ֵ���ѿ�������ϵ
             i = obj.research_index(t);
             dx=t-obj.x(i);
             result=obj.a(i) + obj.b(i)*dx + obj.c(i)*dx.^2.0 + obj.d(i)*dx.^3.0;
        end
       %% calculate frist derivative  ����  (t,dresult)              
        function result = calcd(obj, t)
            %if t is outside of the input x, return None
            if (t<obj.x(1))
                result= NaN;
                return;
            elseif ( t>obj.x(end))
                result=NaN;
                return;
            end
           i = obj.research_index(t);
             dx=t-obj.x(i);
             result=obj.b(i) +2.0* obj.c(i)*dx+3.0* obj.d(i)*dx.^2.0;
        end
       %% calculate second derivative ���׵���   (t,ddresult)       
        function result =calcdd(obj,t)        
            %if t is outside of the input x, return None
            if (t<obj.x(1))
                result= NaN;
                return;
            elseif ( t>obj.x(end))
                result=NaN;
                return;
            end
             i = obj.research_index(t);
             dx=t-obj.x(i);
             result=2.0* obj.c(i)+6.0* obj.d(i)*dx;
        end
       %% research date segment index
        function idx = research_index(obj, x) 
            idx=obj.bisect(obj.x, x, 1, length(obj.x))-1;
        end 
        function lo = bisect(obj, list, x, lo, hi)
          %�������б�a�в�����Ŀx������������a������
          %����ֵiʹ��a[��i]�е�����e����e<=x����������e����
          %a[i:]��e>x����ˣ����x�Ѿ��������б��У�a.insert��x����
          %�����ұߵ�x������롣
          %��ѡ����lo��Ĭ��ֵ0����hi��Ĭ��ֵlen��a������
          %Ҫ��������Ƭ��
            if lo < 1
                error('lo must be positive integer');
            end
            if isnan(hi)
                hi =length(list);
            end
            while lo < hi
                %floor�������书���ǡ�����ȡ������
                %����˵���������롱��������ȡ�ᡱ��
                %��ȡ������x������������롰�������롱��ͬ
                mid = floor((lo+hi)/2);
                if x < list(mid)
                    hi=mid;
                else 
                    lo=mid+1;
                end
            end
        end      
    end
end