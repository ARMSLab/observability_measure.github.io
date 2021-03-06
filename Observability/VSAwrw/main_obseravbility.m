%%
load('data.mat');
C = eye(7,7);
t = Ts/10;
epsi=0.001;

model =@(x,u)nonlin_eq_VSA(x,u,sys);
W_emp=zeros(7,7,7);
for i=1:7
     W_emp(:,:,i) = observGramEmp(C(i,:),t,Ts,y,epsi,un',model); %observGramLin(C(i,:),Ts,y,un',sys);%
end

k=valid_sensor_conf([1,3,4,5],4);
m = 1e90;
ind=1;
obm = zeros(length(k),1);
for j=1:length(k)
    W=zeros(7,7);
    C=zeros(length(k{j}),7);
    for i=1:length(k{j})
        C(i,k{j}(i))=1;
    end
    for i=1:length(k{j})
        W=W+W_emp(:,:,k{j}(i));%W=W+observGramEmp(C(i,:),t,Ts,y,epsi,un',model);
    end
    obm(j)=log(det(W^(-1)));
    if(obm(j)<m)
        m = obm(j);
        ind=j;
    end
end

k{ind}
m
