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
    %% Generate initial transmitter positions
    P0 = [area_size * rand(1, N);
          area_size * rand(1, N);
          alt_min + (alt_max - alt_min) * rand(1, N)];

    %% Random direction for all transmitters (horizontal movement only)
    direction = rand(2, 1); 
    direction = direction / norm(direction); % normalize
    speeds = 5 + (15 - 5) * rand(1, N); % different speed per transmitter

    %% Build 3D matrix P of transmitter positions over time
    P = zeros(3, N, T);
    for t = 1:T
        displacements = direction .* (speeds * (t - 1)); % 1xN horizontal displacement
        P(1, :, t) = P0(1, :) + displacements(1, :); % X
        P(2, :, t) = P0(2, :) + displacements(2, :); % Y
        P(3, :, t) = P0(3, :);                       % Z stays fixed
    end

    %% Generate spiral drone trajectory (on a tilted fixed plane)
    theta = linspace(0, 10*pi, T); % Spiral angle
    r = linspace(r_initial, r_final, T); % Spiral radius
    [x, y] = pol2cart(theta, r);
    z_plane = 500 + 1000 * rand; % Base altitude
    drone_traj = [x; y; z_plane * ones(1, T)];

    % Rotate the plane randomly in 3D (fixed during flight)
    [az, el] = deal(rand * 2*pi, rand * pi); 
    Rz = [cos(az), -sin(az), 0; sin(az), cos(az), 0; 0, 0, 1];
    Rx = [1, 0, 0; 0, cos(el), -sin(el); 0, sin(el), cos(el)];
    R = Rz * Rx;
    drone_traj = R * drone_traj;

    %% Compute Distances Matrix D (WITH NOISE)
    D = zeros(T, N);
    for t = 1:T
        for n = 1:N
            D(t, n) = norm(drone_traj(:, t) - P(:, n, t)) + noise_std * randn;
        end
    end

    %% Estimate Drone Position Using lsqnonlin (dynamic P)
    estimated_traj = zeros(3, T);
    options = optimoptions('lsqnonlin', 'Display', 'off');

    for t = 1:T
        Pt = P(:, :, t); % transmitter positions at time t
        f = @(pos) vecnorm(pos - Pt, 2, 1) - D(t, :);
        initial_guess = mean(Pt, 2) + 50 * randn(3, 1);
        estimated_traj(:, t) = lsqnonlin(f, initial_guess, [], [], options);
    end

    %% Plot Results
    figure; hold on; grid on; axis equal;
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    title(['3D Trajectory (Moving Transmitters), N = ', num2str(N)]);

    % Plot transmitter trajectories
    for n = 1:N
        plot3(squeeze(P(1, n, :)), squeeze(P(2, n, :)), squeeze(P(3, n, :)), 'r--');
    end

    % Plot drone trajectory
    plot3(drone_traj(1, :), drone_traj(2, :), drone_traj(3, :), 'b', 'LineWidth', 1.5, 'DisplayName', 'Actual Trajectory');
    plot3(estimated_traj(1, :), estimated_traj(2, :), estimated_traj(3, :), 'g', 'LineWidth', 1.5, 'DisplayName', 'Estimated Trajectory');

    legend;
    view(3);
    hold off;
    drawnow;
end
