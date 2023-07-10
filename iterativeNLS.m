function [uHat,numberOfPerformedIterations,count] = iterativeNLS( parameters , AP , TYPE , rho )

% NLS starting point - initial guess
%uHatInit = [ (rand-0.5)*parameters.xmax , (rand-0.5)*parameters.ymax ];
uHatInit = [3,5];
uHat = zeros( parameters.NiterMax , 2 );
delta=[];
H1=[];
delta1=[];
count=0;
for iter = 1:parameters.NiterMax-1
    %% Step 1 - initial guess
    if iter==1
        uHat(iter,:) = uHatInit;
    end
    %% Step 2 - compute Jacobian matrix
    H = buildJacobianMatrixH( parameters , uHat(iter,:) , AP , TYPE );
    jaco=H;
    %% Step 3 - compute the observation matrix and evaluate the difference delta rho
    h_uhat = measurementModel( parameters , uHat(iter,:) , AP , TYPE );
    delta_rho = rho - h_uhat;
    Ha=((H.')*H)+0.01*eye(2);
   
    %% Step 4 - compute the correction
    %% NLS
    
    % if det(Ha)==0
    %  return
    % 
    % end

    delta_u = pinv(Ha)*(H.')*delta_rho';
    
  
    %% Step 5 - update the estimate
    uHat( iter+1 , : ) = uHat( iter , :) + 0.1 * delta_u';

    numberOfPerformedIterations = iter + 1;
    
    %% stopping criterion
    if sum( delta_u.^2 ) < 1e-4
         return
     end       
    
end

end