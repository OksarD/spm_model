% get the projected vector of a vector to a plane, given the normal vector of that plane
function proj = project_to_plane(n, k)
    proj_k = (dot(k,n)/(norm(n)^2))*n;
    proj = k - proj_k;
end