%%
C = eye(7,7);
W_emp=zeros(7,7,7);
epsi=0.001;
t = 5.0000e-04;
for i=1:7
     W_emp(:,:,i) = observGramEmp(C(i,:),t,Ts,y,epsi,un',model); %observGramLin(C(i,:),Ts,y,un',sys);%
end
%%
k={ [1];
    [3];
    [4];
    [5];
    [1,3];
    [1,4];
    [1,5];
    [3,4];
    [3,5];
    [4,5];
    [1,3,4];
    [1,3,5];
    [1,4,5];
    [3,4,5];
    [1,3,4,5];
  }  ;
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
ind
m