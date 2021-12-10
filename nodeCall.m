function [n_o,t_record] = nodeCall(n,t_record)
N = size(n,1);
n_o = n;
for i = 1:N
    random = rand(1);
    if (random > 0.99 && n_o(i,3) == 0 && t_record(i) == 0)
        n_o(i,3) = 1;
        t_record(i) = 1;
    elseif (random > 0.99 && n_o(i,3) == 0 && t_record(i) ~= 0)
        t_record(i) = 0;
    
    end
end
