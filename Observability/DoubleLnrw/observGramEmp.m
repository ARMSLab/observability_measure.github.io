function W_emp = observGramEmp(C,t,Ts,x,epsi,u,model)
    W_emp = zeros(size(x,1));
    for ind=1:length(x)
        
        W= empGram(ind,C,x,u,epsi,Ts,t,model);
        W_emp = W_emp + W;
    end
end