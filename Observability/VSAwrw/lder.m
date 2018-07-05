function k = lder(h,f,x)
    k= jacobian(h,x)*f;
end