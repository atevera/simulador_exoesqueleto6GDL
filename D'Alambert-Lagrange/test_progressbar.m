f = waitbar(0, 'Starting');
n = 100;
for i= 1:n
    % write your code here
    waitbar(i/n, f, sprintf('Progress: %d %%', floor(i/n*100)));
    pause(0.1);
end
close(f)