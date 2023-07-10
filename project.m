
clc;clear;close all;
load('Project_data.mat');



%%


for l=1:1:4
 for i=1:1:5
   for j=1:1:length(rho{l,1})
        if((0.1e-9<abs(rho{l,1}(i,j))&&(abs(rho{l,1}(i,j))<0.1e-2)))
            rho{l,1}(i,j)=NaN;
        end

    end
 end
end

%%
rho{1, 1}(:,1:2)=[];
rho{2, 1}(:,1:2)=[] ;
rho{3, 1}(:,1:2)=[] ;
rho{4, 1}(:,1:2)=[] ;
rho{3, 1}(:,664)=[];



% n=length(rho{1,1});
% plot(1:n,rho{1,1}(1,:),'bo',.....
%     1:n,rho{1,1}(2,:),'bo',.....
%     1:n,rho{1,1}(3,:),'bo',.....
%     1:n,rho{1,1}(4,:),'bo',.....
%     1:n,rho{1,1}(5,:),'bo');


%% we can use smoothdata and apply the sgolayfilter

for i=1:1:length(rho)
    for j=1:1:size(rho{i,1})



    end
end
%%
for l=1:1:4
for i=1:1:5
rho_clear{l,1}(i,:)=fillmissing(rho{l,1}(i,:),'makima');
  % rho_smooth{l,1}(i,:)=sgolayfilt(rho_clear{l,1}(i,:),10,19);
 end
end
a=rho_clear{1,1};
 n=length(a);

 % plot(1:n,a,'LineWidth',1);
 % figure;
 % plot(1:n,rho_smooth{1,1},'LineWidth',1);

%%

 

