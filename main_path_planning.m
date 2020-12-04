%% main algorithm for Frenet path planning 
clc%清除命令窗口的内容
clear %清除工作空间的所有变量
close all %关闭所有的Figure窗口
set(0,'DefaultLineLineWidth',1);%plot()画图的参数设置
%disp('Optimal Frenet Path Planning')%命令窗口输出

%% 2.建立轨迹（路径与速度）

% 2.1 建立道路线，用样条插值拟合
%上边界坐标
leftbound_x=[0.0, 10.0, 20.5, 30.0, 40.5, 50.0, 60.0];
leftbound_y=[15.0, 11.0, 16.0, 21.5, 24.0, 25.0, 21.0];
%道路中心线坐标
center_x=[0.0, 10.0, 20.5, 30.0, 40.5, 50.0, 60.0];
center_y=[5.0, 1.0, 6.0, 11.5, 14.0, 15.0, 11.0];
%右车道中心线，即参考曲线的坐标
w_x=[0.0, 10.0, 20.5, 30.0, 40.5, 50.0, 60.0];
w_y=[0.0, -4.0, 1.0, 6.5, 9.0, 10.0, 6.0];
%下边界坐标
rightbound_x=[0.0, 10.0, 20.5, 30.0, 40.5, 50.0, 60.0];
rightbound_y=[-5.0, -9.0, -4.0, 1.5, 4.0, 5.0, 1.0];


% figure(1) %创建一个新的窗口，参数采用1，即窗口名字为figure 1
% plot(w_x,w_y)%话出轨迹

% 2.2 设置障碍物坐标
ob=[20.0, 1.0 ;
      30.0, 6.0 ;
      30.0, 5.0 ;
      35.0, 7.0 ;
      50.0, 12.0 ];
 % hold on %保持窗口
 % plot(ob(:,1),ob(:,2),'*r')%'*r'画出来的线型就是星号连起来的线
  
 % 2.3 创建参考轨迹
  ds=0.1;    %discrete step size 
  GenerateTargetCourse = @(wx, wy) calcSplineCourse(wx, wy, ds);
  [RefX, RefY, RefYaw, RefCurvature, runningLength, referencePath]=GenerateTargetCourse(w_x, w_y);
  [RefX1, RefY1, RefYaw1, RefCurvature1, runningLength1, referencePath1]=GenerateTargetCourse(leftbound_x, leftbound_y);
  [RefX2, RefY2, RefYaw2, RefCurvature2, runningLength2, referencePath2]=GenerateTargetCourse(center_x, center_y);
  [RefX3, RefY3, RefYaw3, RefCurvature3, runningLength3, referencePath3]=GenerateTargetCourse(rightbound_x, rightbound_y);
 % hold on
 % plot(RefX(1,:),RefY(1,:),'*r')
 
 %% 主要的函数
 % Initial state
  s0_d=10.0 / 3.6;              % 初始的纵向速度[m/s]
  d0 = 2.0;                     % 初始的横向偏移值 [m]
  d0_d=0;                       % 初始的横向速度 [m/s]
  d0_dd = 0;                    % 初始的横向加速度 [m/s^2]
  s0= 0;                        % 初始的纵向值[m]
  %area=20;                      % animation area length[m]
  %创建对象objFrenetPlanner
  objFrenetPlanner = OptimalFrenetPlanner();
  show_animation=true;
  if show_animation
      figure(1)
  end
  writerObj=VideoWriter('test.avi'); %// 定义一个视频文件用来存动画
  open(writerObj); %// 打开该视频文件
  %开始仿真 
  T=500;
  for t = 1:T 
        trajectory = objFrenetPlanner.FrenetOptimalPlanning (referencePath, s0, s0_d, d0, d0_d, d0_dd, ob);
        
        %将计划轨迹的更新状态存储为初始状态
        %新轨迹下一次迭代的状态
        s0 = trajectory.s(2);
        s0_d= trajectory.ds(2);
        d0 = trajectory.d(2);
        d0_d = trajectory.dd(2);
        d0_dd=trajectory.ddd(2);
        
  if(show_animation)
      cla;
      %plot(RefX,RefY, '--b');
      plot(RefX1,RefY1, '-b');
      plot(RefX2,RefY2, '--b');
      plot(RefX3,RefY3, '-b');
      hold on
      axis equal
      plot(ob(:,1),ob(:,2),'xk')
      plot(trajectory.x(1:end),trajectory.y(1:end), '-or');
      %plot(trajectory.x(1), trajectory.y(1), 'vc');
      grid on 
      drawnow
      if(sqrt((trajectory.x(1)-RefX(end))^2+(trajectory.y(1)-RefY(end))^2)<2.0)
          print("complete goal")
          close(writerObj); %// 关闭视频文件句柄   
          break
      end
      
      frame = getframe; %// 把图像存入视频文件中
      frame.cdata = imresize(frame.cdata, [653 514]); %重新定义帧大小
      writeVideo(writerObj,frame); %// 将帧写入视频
      
  end
  end