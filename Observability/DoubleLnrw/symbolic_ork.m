 U= sym('U',[4,11]);
 x =sym('x',[12,1]);
 
 dy = nonlin_eq_DL(x,U(:,1),sys);
 
 C = zeros(6,12);
 C(1,1) =1 ;
 C(2,2) =1 ;
 C(3,5) =1 ;
 C(4,6) =1 ;
 C(5,7) =1 ;
 C(6,8) =1 ;
 
C=sym(C);
Y=sym(zeros(12,6));
for ind=1:4
    Y(:,ind) = C(ind,:)*x;
end
for ind1=1:6
    for ind2=2:12
        Y(ind2,ind1) = lder(Y(ind2-1,ind1),dy,x);
        for ind3=1:ind2
            Y(ind2,ind1) =  Y(ind2,ind1) + lder(Y(ind2-1,ind1),U(:,ind2),U(:,ind2-1));
        end
    end
end
%%
K=sym(zeros(12,12,6));
for ind1=1:6
    K(:,:,ind1) = jacobian(Y(:,ind1),x);
end