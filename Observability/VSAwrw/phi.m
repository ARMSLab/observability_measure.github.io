function Phi = phi(ind,x,u,Ts,sys)
    Phi=eye(size(x,1));
        for ind2=1:ind
            dx = nonlin_eq_VSA(x(:,ind2),u(:,ind2),sys);
            [A,B,K] = linearize_model_VSA(x(:,ind2),dx,u(:,ind2),sys);
            Phi = Phi + expm(A*Ts);
        end
end