clear all;clc;close all;

set(0,'DefaultTextFontSize',22)
set(0,'DefaultLineLineWidth',2);
set(0,'DefaultTextInterpreter','latex')
set(0,'DefaultAxesFontSize',16)

load('rho_clear.mat');
load('Project_data.mat');
%% Define the localization scenario
% scenario settings

parameters.numberOfAP = 6;
parameters.positionAP = zeros(6,3); % 6 AP [x,y]
for i = 1:6
    parameters.positionAP(i,1) = AP(i,1);
    parameters.positionAP(i,2) = AP(i,2);
    parameters.positionAP(i,3) = AP(i,3);
end

 % 

%% parameters
parameters.simulationTime = 663;
parameters.samplingTime = 0.1; %s

%% Tracking by EKF
% for i=0:0.1:5
%     for j=0.5
%         for b=100
for p=1:4
    parameters.simulationTime = size(rho_clear{p,:}, 2); %s
    our_array = rho_clear{p,:};
   
    TYPE = 'TDOA';
     sigma_driving = 0;
    %initialization
    UE_init = [ 0, 0.5 ,0]; %ux uy uz 
    UE_init_COV = diag([ 0.001^2, 0.001^2, 0]);
    x_hat = NaN( parameters.simulationTime , 3);
    P_hat = NaN( 3,3,parameters.simulationTime );
    R = diag([0.1^2,0.1^2,0.1^2,0.1^2,0.1^2]);
    Q=diag([0.1^2, 0.1^2, 0.01^2]);
    F = [1 0 0; 0 1 0;0 0 1];

    %update over time
    for time = 1 : parameters.simulationTime

        %prediction
        if time == 1
            x_pred =  UE_init';
            P_pred =  UE_init_COV;
        else
            x_pred = F * x_hat(time-1,:)';
            P_pred = F * P_hat(:,:,time-1)*F' + Q;
        end
        H = buildJacobianMatrixH(parameters, x_pred' , AP , TYPE);
        %update
        G = P_pred * H' * inv( H*P_pred*H' + R);
        r = our_array(:,time);
      
        x_hat(time,:) = x_pred + G * ( r - H*x_pred ) ;
        P_hat(:,:,time) = P_pred - G * H* P_pred;

    end
    
     switch p
        case 1
             nls_1 = x_hat;
        case 2
             nls_2 = x_hat;
        case 3
             nls_3 = x_hat;
        case 4
             nls_4 = x_hat;
    end
end


figure;
 plot(nls_1(:,1),nls_1(:,2));
 hold on;
% 
%         end
%     end
% end
% % 

 %%



%   UE_init = [ 0 , 0, 0];
%     UE_init_COV = diag([ 10^2, 100^2, 1^2]);
%     x_hat = NaN( parameters.simulationTime , 3);
%     P_hat = NaN( 3,3,parameters.simulationTime );
% 
% 
%     figure(12); hold on
%     title('Time: ' , num2str(0) )
%     plot( UE(:,1) , UE(:,2) , 'o','MarkerSize',10,'MarkerEdgeColor',[0.30,0.75,0.93],'MarkerFaceColor',[0.30,0.75,0.93] )
%     plot( UE_init(1) , UE_init(2) , 'x','MarkerSize',10 )
%     plotCovariance( UE_init_COV(1:2,1:2) , UE_init(1,1) , UE_init(1,2)  , 3 ,'Initialization');
%     axis equal
%     legend('True UE','UE est = init.','Cov. prior')
% 
% 
%     H = [1 , 0 , 0 ,0 ; 0 , 1, 0, 0];
%     sigma_driving = 0;
%     L = [ 0.5*parameters.samplingTime^2 0 ; 0 0.5*parameters.samplingTime^2; parameters.samplingTime 0 ; 0 parameters.samplingTime];
%     Q = sigma_driving * L * L';
%     F = [1 , 0 , parameters.samplingTime , 0; 
%          0 , 1 , 0, parameters.samplingTime ;
%          0 , 0 , 1 , 0;
%          0 , 0 , 0 , 1];
%      sigma_driving = 0;
% 
%     %update over time
%     for time = 1 : parameters.simulationTime
% 
%         %prediction
%         if time == 1
%             x_pred = UE_init';
%             P_pred = UE_init_COV;
%         else
%             x_pred = F * x_hat(time-1,:)';
%             P_pred = F * P_hat(:,:,time-1) *F' + Q;
%         end
%         %update
%         G = P_pred * H' * inv( H*P_pred*H' + R);
%         x_hat(time,:) = x_pred + G * (rho( time , : )'  - H*x_pred );
%         P_hat(:,:,time) = P_pred - G * H * P_pred;
% 
% 
%         %plot evolution
%         figure(12);
%         plot( UE(:,1) , UE(:,2) , 'o','MarkerSize',10,'MarkerEdgeColor',[0.30,0.75,0.93],'MarkerFaceColor',[0.30,0.75,0.93] ), hold on
%         plot( UE(time,1) , UE(time,2) , 'o','MarkerSize',10,'MarkerEdgeColor',[0.30,0.75,0.93],'MarkerFaceColor',[0.50,0,0] ),
%         plotCovariance( P_pred(1:2,1:2)  , x_pred(1) , x_pred(2)  , 3 , 'Prior');
%         axis equal
%         xlim([parameters.xmin parameters.xmax]) , ylim([parameters.ymin parameters.ymax])
%         xlabel('[m]'), ylabel('[m]');
%         legend('True UE (all)','True UE (current)','Cov. pred.')
%         title('Time: ' , num2str(time) )
%         pause(.1)
%         plot( rho(time,1) , rho(time,2) , '^k')
%         plot( x_hat(:,1) , x_hat(:,2) , '-g','Marker','s')
%         plotCovariance( P_hat(1:2,1:2,time)  , x_hat(time,1) , x_hat(time,2)  , 3 , 'Update');
%         legend('True UE (all)','True UE (current)','Cov. pred.','meas.','KF est.','Cov. upd.')
% 
%         pause(.1)
%         hold off
% 
%     end
% 
% 
% 
% 
% 
% %% Tracking by EKF
% for i = 1:6
%     parameters.positionAP(i,1) = AP(i,1);
%     parameters.positionAP(i,2) = AP(i,2);
% end
% 
% AP = AP(:,1:2);
% for p=1:4
%     parameters.simulationTime = size(rho{p,:}, 2); %s
%     our_array = rho{p,:};
%     TYPE = 'TDOA';
%     %initialization
%     UE_init = [10, 20];
%     UE_init_COV = diag([100^2,100^2]);
%     x_hat = NaN( parameters.simulationTime , 2);
%     P_hat = NaN( 2 , 2 , parameters.simulationTime );
% 
%     R = diag([5,5,5,5,5]);
%     Q = diag([0.1, 0.1] );
%     F = [1 0 ; 0 1];
% 
%     %update over time
%     for time = 1 : parameters.simulationTime
% 
%         %prediction
%         if time == 1
%             x_pred =  UE_init';
%             P_pred =  UE_init_COV;
%         else
%             x_pred = F * x_hat(time-1,:)';
%             P_pred = F * P_hat(:,:,time-1)*F' + Q;
%         end
%         H = buildJacobianMatrixH(parameters, x_pred' , AP , TYPE);
%         %update
% 
%         G = P_pred * H' * inv( H*P_pred*H' + R);
%         m = measurementModel ( parameters , x_pred' , AP , TYPE)';
%         r = our_array(:,time);
%         x_hat(time,:) = x_pred + G * ( our_array(:,time) - measurementModel ( parameters , x_pred' , AP , TYPE)' ) ;
%         P_hat(:,:,time) = P_pred - G * H * P_pred;
% 
%     end
% 
%      switch p
%         case 1
%              nls_1 = x_hat;
%         case 2
%              nls_2 = x_hat;
%         case 3
%              nls_3 = x_hat;
%         case 4
%              nls_4 = x_hat;
%     end
% end
% 
