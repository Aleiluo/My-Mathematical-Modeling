function d = Dist(x, y)
    d = sqrt((x(:,1)-y(:,1)).^2 + (x(:,2)-y(:,2)).^2 + (x(:,3)-y(:,3)).^2);
end
