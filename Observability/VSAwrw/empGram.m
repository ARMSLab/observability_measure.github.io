function W= empGram(ind,C,x,u,epsi,Ts,t,model)
    
    W=zeros(size(x,1));
    Y=zeros(size(C,1),floor(Ts/t),size(x,1));
    for i =1:size(x,1)
        x0=x(:,ind);
        e=zeros(size(x0));
        e(i)=epsi;
        x0p = x0+e;
        x0m = x0-e;
        for ind2=1:floor(Ts/t)
            x0p=RK4(x0p,u(:,ind),t,model);
            x0m=RK4(x0m,u(:,ind),t,model);
            Y(:,ind2,i)= C*(x0p-x0m);
        end
    end
    for i = 1:size(W,1)
        for j = 1:size(W,1)
            for ind2 = 1:floor(Ts/t)
               a= t*(Y(:,ind2,i)'*Y(:,ind2,j));
               W(i,j) =W(i,j) + (1/(4*epsi^2))*a;
            end
            
        end
    end
end