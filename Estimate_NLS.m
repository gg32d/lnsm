clc;clear;close all;
load('rho_clear.mat');
load('Project_data.mat');
parameters.numberOfAP = 6;
parameters.positionAP = zeros(6,3); % 6 AP [x,y]
for i = 1:6
    parameters.positionAP(i,1) = AP(i,1);
    parameters.positionAP(i,2) = AP(i,2);
    parameters.positionAP(i,3) = AP(i,3);
end
AP = AP(:,1:2);


%% IMPLEMENT NLS
% We initialize the sigma of the TDOA measurments to 0.1 meters and we decide
% to use 1000 iterations for each timestep
nls_1=zeros( size(rho_smooth{1,:}, 2) , 2 );
nls_2=zeros( size(rho_smooth{2,:}, 2) , 2 );
nls_3=zeros( size(rho_smooth{3,:}, 2) , 2 );
nls_4=zeros( size(rho_smooth{4,:}, 2) , 2 );
for p=1 : size(rho, 1)
    parameters.NiterMax = 1000; % number updates
    our_array = rho_clear{p,:};
    TYPE = 'TDOA';
     % R is not used since we are not in WLNS
    rho2 = our_array.';
    % nls_meas contains the position coordinates after each time interval
   
    % random initial position
  
    for i = 1:size(rho_clear{p,:}, 2)
        [ uHat , numberOfPerformedIterations,count ] = iterativeNLS( parameters , AP , TYPE , rho2(i,:)); 
        uHat = uHat( 1:numberOfPerformedIterations , : ); % this is the final estimate of NLS at the last iteration
    switch p
        case 1
             nls_1(i,:) = uHat(end,:);
        case 2
             nls_2(i,:) = uHat(end,:);
        case 3
             nls_3(i,:) = uHat(end,:);
        case 4
             nls_4(i,:) = uHat(end,:);
    end
        % We decide to start at the next timestep from the latest found
        % position.
      
    end
end
%%

% for i=length(rho2)
[ uHat , numberOfPerformedIterations, count] = iterativeNLS( parameters , AP , TYPE , rho2(1,:));

% end

%%
% 
  % plot(nls_1(:,1),nls_1(:,2),'.');
% 
% % % plot3(nls_2(:,1),nls_2(:,2),nls_2(:,3),'-');
% % % figure;
% % % plot3(nls_3(:,1),nls_3(:,2),nls_3(:,3),'-');
% % % figure;
% % % plot3(nls_4(:,1),nls_4(:,2),nls_4(:,3),'-');
% UE=[10,10,2];
% distanceUEAP=sqrt(sum((UE-AP).^2, 2));
% %direction cosine
% directionCosineX=(UE(1)-AP(:,1))./distanceUEAP;
% 
% distanceUEAP=sqrt(sum((UE-AP).^2, 2));
% %direction cosine
% directionCosineX=(UE(1)-AP(:,1))./distanceUEAP;
% directionCosineY=(UE(2)-AP(:,2))./distanceUEAP;
% directionCosineZ=(UE(3)-AP(:,3))./distanceUEAP;
% H = zeros( parameters.numberOfAP , 3);
% for a=1:1:6
% 
%             H(a,:,:) = [ directionCosineX(a) - directionCosineX(2), directionCosineY(a) - directionCosineY(2), directionCosineZ(a)-directionCosineZ(2)];% considering w as refAp
% 
% 
% end
% H(2,:,:) = []; 
% 
% z=pinv(H);
% 
% %%
% 
% distanceUEAP = sqrt( sum( [UE-AP].^2 , 2 ) ); 
% 
% %% build the vector/matrix of observation
% h = zeros( 1 , parameters.numberOfAP );
% for a = 1:parameters.numberOfAP
% 
%             refAP = 2;
%             h(a) = distanceUEAP( a ) - distanceUEAP( refAP );
% 
% end
% h(2)=[];
% 
% 
% cd=rho2(1,:) - h;
% 
% delta_u = z*cd';
% 
% %%
% 
 plot(nls_1(:,1),nls_1(:,2), '.');
 axis([0 7 0 10]);
figure;

plot(nls_2(:,1),nls_2(:,2), '.');
axis([0 7 0 10]);
figure;

plot(nls_3(:,1),nls_3(:,2), '.');
axis([0 7 0 10]);
figure;

plot(nls_4(:,1),nls_4(:,2), '.');
axis([0 7 0 10]);

% %%
% 


