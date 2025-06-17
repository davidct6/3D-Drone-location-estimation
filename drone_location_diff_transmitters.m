%% Parameters
N_values = [4, 8];
T = 480; 
speed = 25; 
r_initial = 10; 
r_final = 1000; 
area_size = 5000; 
alt_min = 2000; 
alt_max = 5000; 
noise_std = 10; 

for N = N_values
    %% Generation of the transmitter location Matrix P
    P = [area_size * rand(1, N);
         area_size * rand(1, N);
         alt_min + (alt_max-alt_min) * rand(1, N)];
    
    %% Generation of drone trajectory
    
    theta = linspace(0, 10*pi, T); % Spiraling angle
    r = linspace(r_initial, r_final, T); % Spiral radius
    [x, y] = pol2cart(theta, r);
    z = 500 + 1000 * rand; % Random altitude
    drone_traj = [x; y; z * ones(1, T)];
    
    %% Compute Distances Matrix D (WITH NOISE)
    D = zeros(T, N);
    for t = 1:T
        for n = 1:N
            D(t, n) = norm(drone_traj(:, t) - P(:, n)) + noise_std * randn; % Add Gaussian noise
        end
    end
    
    %% Estimate Drone Position Using lsqnonlin
    estimated_traj = zeros(3, T);
    options = optimoptions('lsqnonlin', 'Display', 'off'); % Quiet solver
    
    for t = 1:T
        f = @(pos) vecnorm(pos - P, 2, 1) - D(t, :);
        initial_guess = mean(P, 2) + 50 * randn(3, 1); % Random perturbation
        estimated_traj(:, t) = lsqnonlin(f, initial_guess, [], [], options);
    end
    %% Plot Results
    figure; hold on; grid on; axis equal;
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    title('3D Drone Trajectory and Transmitter Positions');
    
    % Plot Transmitters
    scatter3(P(1, :), P(2, :), P(3, :), 100, 'r', 'filled', 'DisplayName', 'Transmitters');
    
    % Plot Actual and Estimated Trajectories
    plot3(drone_traj(1, :), drone_traj(2, :), drone_traj(3, :), 'b', 'LineWidth', 1.5, 'DisplayName', 'Actual Trajectory');
    
    plot3(estimated_traj(1, :), estimated_traj(2, :), estimated_traj(3, :), 'g', 'LineWidth', 1.5, 'DisplayName', 'Estimated Trajectory');
    
    legend;
    view(3);
    drawnow;
    hold off;
end