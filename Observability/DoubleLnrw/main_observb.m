C = eye(12,12); %Create output matrix C = dh/dx
W_emp=zeros(12,12,12); % create matrix to store data for each sensors 
Ts=0.01; % sampling time of the data
epsi=0.001; % perturbation value
t = Ts/10;  %sampling time for imperical gramian
model =@(x,u)nonlin_eq_DL(x,u,sys); %model of the system 
y=zeros(12,1622); % initialization matrix for MPC data
un = zeros(4,1622); % initialization of matrix for inputs of MPC
 [tall,y(1,:),y(2,:),y(3,:),y(4,:),y(5,:),y(6,:),y(7,:),y(8,:),y(9,:),y(10,:),y(11,:),y(12,:)]= ... 
               textread('mpc.txt','%f %f %f %f %f %f %f %f %f %f %f %f %f');
               
 [tall,un(1,:),un(2,:),un(3,:),un(4,:)]= textread('u_log.txt','%f %f %f %f %f');

%observability gramian for each individual sensor
for i=1:12
     W_emp(:,:,i) =  observGramEmp(C(i,:),t,Ts,y,epsi,un,model); %observGramLin(C(i,:),Ts,y,un',sys);%
end
%%
k={ [1];
    [2];
    [5];
    [6];
    [7];
    [8];
    
    [1,2];
    [1,5];
    [1,6];
    [1,7];
    [1,8];    
    [2,5];
    [2,6];
    [2,7];
    [2,8];
    [5,6];
    [5,7];
    [5,8];
    [6,7];
    [6,8];
    [7,8];
    
    [1,2,5];
    [1,2,6];
    [1,2,7];
    [1,2,8];    
    [1,5,6];
    [1,5,7];
    [1,5,8];
    [1,6,7];
    [1,6,8];
    [1,7,8];   
    [2,5,6];
    [2,5,7];
    [2,5,8];
    [2,6,7];
    [2,6,8];
    [2,7,8];    
    [5,6,7];
    [5,6,8];
    [5,7,8];   
    [6,7,8];
    
    [1,2,5,6];
    [1,2,5,7];
    [1,2,5,8];
    [1,2,6,7];
    [1,2,6,8];
    [1,2,7,8];
    [1,5,6,7];
    [1,5,6,8];
    [1,5,7,8];
    [1,6,7,8];  
    [2,5,6,7];
    [2,5,6,8];
    [2,5,7,8];    
    [2,6,7,8];  
    [5,6,7,8];
    
    [1,2,5,6,7];
    [1,2,5,6,8];
    [1,2,5,7,8];        
    [1,2,6,7,8];   
    [1,5,6,7,8];
    [2,5,6,7,8];
    
    [1,2,5,6,7,8];
  }  ;

m = 1e90; %to find minimum
ind=1;    % first index
obm = zeros(length(k),1); 
for j=1:length(k)
    W=zeros(12,12);
    
    for i=1:length(k{j})
        W=W+W_emp(:,:,k{j}(i));
    end
    obm(j)=log(det(W^(-1)));
    if(obm(j)<m)
        m = obm(j);
        ind=j;
    end
end