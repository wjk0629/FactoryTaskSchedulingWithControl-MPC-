function [x_k] = integratePosi(x_0, u)
dist = u(1, :); phi = u(2, :);
x_k = zeros(4, length(dist)+1);
x_k(:,1) = x_0;

for i=2:(length(dist)+1)
    x_k(1,i) = x_k(1,i-1) + dist(i-1)*cos(x_k(3,i-1)); %
    x_k(2,i) = x_k(2,i-1) + dist(i-1)*sin(x_k(3,i-1)); %
    x_k(3,i) = x_k(3,i-1) + phi(i-1);
    %x_k(3,i) = atan2(x_k(2,i),x_k(1,i));
%     
%     if (x_k(3,i) > -pi && x_k(3,i) < -pi/2) && (x_k(3,i) > pi/2 && x_k(3,i) < pi)
%         x_k(3,i) = (pi + x_k(3,i));
%     elseif (x_k(3,i) > 0 && x_k(3,i) < pi/2) && (x_k(3,i) > - pi/2 && x_k(3,i) < 0)
%         x_k(3,i) = (-pi + x_k(3,i));
%     end
%     if x_k(3,i) > pi && x_k(3,i) <= (pi + pi/2)
%         x_k(3,i) = x_k(3,i) - pi;
%     elseif x_k(3,i) > (pi + pi/2) && x_k(3,i) <= 2*pi
%         x_k(3,i) = x_k(3,i) - pi;
%     end
    if x_k(3,i) > pi 
        x_k(3,i) = x_k(3,i) - pi;
    elseif x_k(3,i) < -pi
        x_k(3,i) = x_k(3,i) + pi;
    end

end