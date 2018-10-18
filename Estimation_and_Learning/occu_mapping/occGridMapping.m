function myMap = occGridMapping(ranges, scanAngles, pose, param)

resol = param.resol; % the number of grids for 1 meter.
myMap = zeros(param.size); % the initial map size in pixels
origin = param.origin; % the origin of the map in pixels

% Log-odd parameters 
lo_occ = param.lo_occ;
lo_free = param.lo_free;
lo_max = param.lo_max;
lo_min = param.lo_min;

lidarn = size(scanAngles,1); % number of rays per timestamp
N = size(ranges,2); % number of timestamp

for i = 1:N % for each timestamp
    theta = pose(3,i); % orientation of robot
    % coordinate of robot in real world
    x = pose(1,i);
    y = pose(2,i);

    % local coordinates of occupied points in real world
    local_occs = [ranges(:,i).*cos(scanAngles+theta), -ranges(:,i).*sin(scanAngles+theta)];

    % coordinate of robot in metric map
    grid_rob = ceil(resol * [x; y]);

    % calc coordinates of occupied and free points in metric map
    for j=1:lidarn
        real_occ = local_occs(j,:) + [x, y]; % global coordinate of occ in real world
        grid_occ = ceil(resol * real_occ); % coordinate of occ in metric map

        % coordinates of free in metric map (by breshnham's algorithm)
        [freex, freey] = bresenham(grid_rob(1),grid_rob(2),grid_occ(1),grid_occ(2));

        % convert coordinate to offset to array
        free = sub2ind(size(myMap),freey+origin(2),freex+origin(1));
        occ = sub2ind(size(myMap), grid_occ(2)+origin(2), grid_occ(1)+origin(1));

        % update metric map
        myMap(free) = myMap(free) - lo_free;
        myMap(occ) = myMap(occ) + lo_occ;
    end
end

% reassign value if out of range
myMap(myMap < lo_min) = lo_min;
myMap(myMap > lo_max) = lo_max;