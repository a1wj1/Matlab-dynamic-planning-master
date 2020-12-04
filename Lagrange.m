%lagrane.m
%拉格朗日插值及其误差估计
%输入的量:X是n+1个节点(x_i,y_i)(i = 1,2, ... , n+1)横坐标向量，Y是纵坐标向量，
%x是以向量形式输入的m个插值点，M在[a,b]上满足｜f~(n+1)(x)｜≤M
%注：f~(n+1)(x)表示f(x)的n+1阶导数
%输出的量：y为m个插值构成的向量，R是误差限
function [y, R] = lagrange(X, Y, x, M)
n = length(X);m = length(x);
for i = 1:m
    z = x(i);
    s = 0.0;
    for k = 1:n
        p = 1.0; q1 = 1.0; c1 = 1.0;
        for j = 1:n
            if j~=k
                p = p * (z - X(j)) / (X(k) - X(j));
            end
            q1 = abs(q1 * (z - X(j)));
            c1 = c1 * j;
        end
        s = p * Y(k) + s;
    end
    y(i) = s;
    R(i) = M * q1 / c1;
end