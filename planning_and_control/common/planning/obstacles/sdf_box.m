function sd = sdf_box(pt, center, shape)
    pt = pt - center;
    shape_2 = shape / 2;
    
    fq = abs(pt); % transform to first quadrant (simplify by symmetry)
    trv = fq - shape_2; % vector from top right vertex
    
    outer = vecnorm(max(trv, 0)); % if any(trv > 0) then we're outside the box
    inner = min(max(trv), 0); % if any(trv < 0) then we're inside the box
    sd = outer + inner; % only one can be != 0
end