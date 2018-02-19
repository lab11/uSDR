
function[out] = cal_p(n,x,p_single);

out = nchoosek(n, x)*((p_single)^x)*((1-p_single)^(n-x));
