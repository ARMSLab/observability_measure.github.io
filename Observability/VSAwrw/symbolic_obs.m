   U= sym('U',[3,7]);
    x =sym('x',[7,1]);
dy = nonlin_eq_VSA(x,U(:,1), sys) ;     
C=zeros(4,7);
C(1,1)=1;
C(2,3)=1;
C(3,4)=1;
C(4,5)=1;
C=sym(C);
Y=sym(zeros(7,4));
for ind=1:4
    Y(:,ind) = C(ind,:)*x;
end
for ind1=1:4
    for ind2=2:7
        Y(ind2,ind1) = lder(Y(ind2-1,ind1),dy,x);
        for ind3=1:ind2
            Y(ind2,ind1) =  Y(ind2,ind1) + lder(Y(ind2-1,ind1),U(:,ind2),U(:,ind2-1));
        end
    end
end
%%
K=sym(zeros(7,7,4));
for ind1=1:4
    K(:,:,ind1) = jacobian(Y(:,ind1),x);
end