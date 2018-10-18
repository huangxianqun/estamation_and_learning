function myPose = particleLocalization(ranges, angles, map, param)

% occupancy value of unexplored pixels
unknown = mode(reshape(map, size(map,1)*size(map,2), 1));

N = size(ranges, 2); % number of poses to calculate
myPose = zeros(3, N); % initialize return value

resol = param.resol; % map resolution
origin = param.origin; % origin in pixel

sig = [0.08, 0, 0; 0, 0.08, 0; 0, 0, 0.08]; % noise for particle movement

myPose(:,1) = param.init_pose; % init position

M = 200; % number of particles

P = repmat(myPose(:,1), [1, M]);

thr = ceil(3.5/5*size(angles,1)); % set the score threshold as 70%

for j = 2:N
    maxscore = 0;
    while maxscore < thr
        Q=P+(randn(size(P,2),3)*sig)'; % particles movement  Ëæ»ú²úÉú
        score = zeros(size(Q,2), 1); % scores
        for k = 1:size(Q,2) % calculate score for each particle
            occ_x = ceil( (ranges(:,j) .* cos(angles+Q(3,k)) + Q(1,k) )  * resol + origin(1) );
            occ_y = ceil( (-ranges(:,j) .* sin(angles+Q(3,k)) + Q(2,k) ) * resol + origin(2) );
            ids = occ_x > 0 & occ_x <= size(map,2) & occ_y > 0 & occ_y <= size(map,1);
            score(k) = size( map( map (sub2ind(size(map), occ_y(ids), occ_x(ids)) ) > unknown ), 1);
        end
        [maxscore, index] = max(score); % select particle with maximum score
    end
    myPose(:,j) = Q(:,index); % set pose(j) as the optimal particle

    Q = Q(:,score >= thr); % select particles with high score
    P = repmat(Q, 1, ceil(M/size(Q,2)) ); % regenerate particles
end

end