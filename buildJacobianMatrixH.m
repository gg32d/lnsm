function H=buildJacobianMatrixH( parameters , UE , AP , TYPE )
distanceUEAP=sqrt(sum((UE-AP).^2, 2));
%direction cosine
directionCosineX=abs((UE(1)-AP(:,1))./distanceUEAP);
directionCosineY=abs((UE(2)-AP(:,2))./distanceUEAP);
directionCosineZ=abs((UE(3)-AP(:,3))./distanceUEAP);
H = zeros( parameters.numberOfAP , 3);
% H = zeros( parameters.numberOfAP , 2);
for a=1:1:6
    
            H(a,:) = [ directionCosineX(a) - directionCosineX(2), directionCosineY(a) - directionCosineY(2), directionCosineZ(a) - directionCosineZ(2)];% considering w as refAp
                        % H(a,:) = [ directionCosineX(a) - directionCosineX(2), directionCosineY(a) - directionCosineY(2)];
   
end
H(2,:)=[];