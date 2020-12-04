%% main algorithm for Frenet path planning 
clc%�������ڵ�����
clear %��������ռ�����б���
close all %�ر����е�Figure����
set(0,'DefaultLineLineWidth',1);%plot()��ͼ�Ĳ�������
%disp('Optimal Frenet Path Planning')%��������

%% 2.�����켣��·�����ٶȣ�

% 2.1 ������·�ߣ���������ֵ���
%�ϱ߽�����
leftbound_x=[0.0, 10.0, 20.5, 30.0, 40.5, 50.0, 60.0];
leftbound_y=[15.0, 11.0, 16.0, 21.5, 24.0, 25.0, 21.0];
%��·����������
center_x=[0.0, 10.0, 20.5, 30.0, 40.5, 50.0, 60.0];
center_y=[5.0, 1.0, 6.0, 11.5, 14.0, 15.0, 11.0];
%�ҳ��������ߣ����ο����ߵ�����
w_x=[0.0, 10.0, 20.5, 30.0, 40.5, 50.0, 60.0];
w_y=[0.0, -4.0, 1.0, 6.5, 9.0, 10.0, 6.0];
%�±߽�����
rightbound_x=[0.0, 10.0, 20.5, 30.0, 40.5, 50.0, 60.0];
rightbound_y=[-5.0, -9.0, -4.0, 1.5, 4.0, 5.0, 1.0];


% figure(1) %����һ���µĴ��ڣ���������1������������Ϊfigure 1
% plot(w_x,w_y)%�����켣

% 2.2 �����ϰ�������
ob=[20.0, 1.0 ;
      30.0, 6.0 ;
      30.0, 5.0 ;
      35.0, 7.0 ;
      50.0, 12.0 ];
 % hold on %���ִ���
 % plot(ob(:,1),ob(:,2),'*r')%'*r'�����������;����Ǻ�����������
  
 % 2.3 �����ο��켣
  ds=0.1;    %discrete step size 
  GenerateTargetCourse = @(wx, wy) calcSplineCourse(wx, wy, ds);
  [RefX, RefY, RefYaw, RefCurvature, runningLength, referencePath]=GenerateTargetCourse(w_x, w_y);
  [RefX1, RefY1, RefYaw1, RefCurvature1, runningLength1, referencePath1]=GenerateTargetCourse(leftbound_x, leftbound_y);
  [RefX2, RefY2, RefYaw2, RefCurvature2, runningLength2, referencePath2]=GenerateTargetCourse(center_x, center_y);
  [RefX3, RefY3, RefYaw3, RefCurvature3, runningLength3, referencePath3]=GenerateTargetCourse(rightbound_x, rightbound_y);
 % hold on
 % plot(RefX(1,:),RefY(1,:),'*r')
 
 %% ��Ҫ�ĺ���
 % Initial state
  s0_d=10.0 / 3.6;              % ��ʼ�������ٶ�[m/s]
  d0 = 2.0;                     % ��ʼ�ĺ���ƫ��ֵ [m]
  d0_d=0;                       % ��ʼ�ĺ����ٶ� [m/s]
  d0_dd = 0;                    % ��ʼ�ĺ�����ٶ� [m/s^2]
  s0= 0;                        % ��ʼ������ֵ[m]
  %area=20;                      % animation area length[m]
  %��������objFrenetPlanner
  objFrenetPlanner = OptimalFrenetPlanner();
  show_animation=true;
  if show_animation
      figure(1)
  end
  writerObj=VideoWriter('test.avi'); %// ����һ����Ƶ�ļ������涯��
  open(writerObj); %// �򿪸���Ƶ�ļ�
  %��ʼ���� 
  T=500;
  for t = 1:T 
        trajectory = objFrenetPlanner.FrenetOptimalPlanning (referencePath, s0, s0_d, d0, d0_d, d0_dd, ob);
        
        %���ƻ��켣�ĸ���״̬�洢Ϊ��ʼ״̬
        %�¹켣��һ�ε�����״̬
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
          close(writerObj); %// �ر���Ƶ�ļ����   
          break
      end
      
      frame = getframe; %// ��ͼ�������Ƶ�ļ���
      frame.cdata = imresize(frame.cdata, [653 514]); %���¶���֡��С
      writeVideo(writerObj,frame); %// ��֡д����Ƶ
      
  end
  end