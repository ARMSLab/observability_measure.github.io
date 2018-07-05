function W_lin = observGramLin(C,Ts,x,u,sys)
    W_lin = zeros(size(x,1));
    for ind=1:length(x)
        Phi = phi(ind,x,u,Ts,sys);
        L=C*Phi;
        W_lin = W_lin + (L'*L)*Ts;
    end
end