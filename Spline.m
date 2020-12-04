%样条曲线y=ax^3+bx^2+cx+d
%返回三次样条插值的系数ai.bi.ci.di
classdef Spline
    properties(Access=public)%是公有（public）的属性
        a
        b
        c
        d
        w   
        x
        y     
        nx
    end
    methods(Access=public)%公有成员函数
        %调用Spline构造函数，三次样条插值函数，后面可以考虑换成自己的插值函数
        %注意这里的x,y都是数组，即点（x0，y0）,(x1,y1),...,(xn,yn)
        function obj =Spline(x,y)
            %参数初始化
            obj.b=[];
            obj.c=[];
            obj.d=[];
            obj.w=[];
            obj.x=x;
            obj.y=y;
            obj.nx=length(x)   %计算x长度
            %对于向量X，diff（X）是[X（2）-X（1），X（3）-X（2）。。。X（n）-X（n-1）]。
            h=diff(x);%计算x之间的差   
            obj.a=y;%计算ai,根据算法，ai=yi
            %计算参数c
            A=obj.calc_A(h);%求出系数矩阵
            B=obj.calc_B(h);
            obj.c=A\B;%方程的解
            %ci=mi/2,在这里，obj.c=mi/2=A\B;因为在计算B的时候，系数是3，已经除以2，所以这里直接等于。
            obj.c=(obj.c)';%矩阵的转置， %计算参数c
            
            %计算参数b和d
            for i = 1:(obj.nx-1)
                obj.d(end+1)=(obj.c(i+1)-obj.c(i)) / (3.0*h(i));
                tb= (obj.a(i+1) - obj.a(i) ) / h(i)- h(i)* (obj.c(i+1)+2.0*obj.c(i))/ 3.0;
                obj.b(end+1)=tb;    
            end
        end
       %% calculate matrix A for spline coefficient c            
        function A = calc_A(obj, h)%obj相当于Spline构造函数的对象，可以调用里面的参数
            A= zeros(obj.nx, obj.nx);
            A(1, 1)= 1.0;
            for i = 1:(obj.nx - 1)
                if i ~= (obj.nx - 1)%~=是不等于
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
            %否则返回x值对应于样条曲线上的值，笛卡尔坐标系
             i = obj.research_index(t);
             dx=t-obj.x(i);
             result=obj.a(i) + obj.b(i)*dx + obj.c(i)*dx.^2.0 + obj.d(i)*dx.^3.0;
        end
       %% calculate frist derivative  导数  (t,dresult)              
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
       %% calculate second derivative 二阶导数   (t,ddresult)       
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
          %返回在列表a中插入项目x的索引，假设a已排序。
          %返回值i使得a[：i]中的所有e都有e<=x，并且所有e都在
          %a[i:]的e>x。因此，如果x已经出现在列表中，a.insert（x）将
          %在最右边的x后面插入。
          %可选参数lo（默认值0）和hi（默认值len（a））绑定
          %要搜索的切片。
            if lo < 1
                error('lo must be positive integer');
            end
            if isnan(hi)
                hi =length(list);
            end
            while lo < hi
                %floor函数，其功能是“向下取整”，
                %或者说“向下舍入”、“向零取舍”，
                %即取不大于x的最大整数，与“四舍五入”不同
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