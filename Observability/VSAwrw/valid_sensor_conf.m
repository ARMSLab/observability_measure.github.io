function K=valid_sensor_conf(M,l)
    n=length(M);
    K=cell(factorial(n)/(factorial(l)*factorial(n-l)),1);
    kk=1;
    for ind1=1:l
        C=combnk(M,ind1);
        for ind2=1:size(C,1)
            K{kk}=C(ind2,:);
            kk=kk+1;
        end
    end
end