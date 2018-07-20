C = eye(12,12);
W_emp=zeros(12,12,12);

Ts=0.01;
epsi=0.001;
t = Ts/10;
model =@(x,u)nonlin_eq_DL(x,u,sys);
y=zeros(12,1622);
un = zeros(4,1622);
 [tall,y(1,:),y(2,:),y(3,:),y(4,:),y(5,:),y(6,:),y(7,:),y(8,:),y(9,:),y(10,:),y(11,:),y(12,:)]= textread('mpc.txt','%f %f %f %f %f %f %f %f %f %f %f %f %f');
 [tall,un(1,:),un(2,:),un(3,:),un(4,:)]= textread('u_log.txt','%f %f %f %f %f');

for i=1:12
     W_emp(:,:,i) =  observGramEmp(C(i,:),t,Ts,y,epsi,un,model); %observGramLin(C(i,:),Ts,y,un',sys);%
end
%%
k = valid_sensor_conf([1,2,5,6,7,8],6);
m = 1e90;
ind=1;
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
