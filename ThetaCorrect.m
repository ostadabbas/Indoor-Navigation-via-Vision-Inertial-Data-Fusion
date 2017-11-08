function thetaC = ThetaCorrect(theta)
    theta = [0; theta(:)];
    tDiff = theta(2:end) - theta(1:end-1);
    tDiff(tDiff > pi) = tDiff(tDiff > pi) - 2*pi;
    tDiff(tDiff < -pi) = tDiff(tDiff < -pi) + 2*pi;
    thetaC = cumsum(tDiff);
end