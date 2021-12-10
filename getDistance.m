function d = getDistance(x, y)
    d = sqrt(sum((x-y).^2,2));
end