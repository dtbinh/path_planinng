function D2 =ray_similarity_metric(ZI,ZJ)
length_weight = 1.0;       
% ZI = [ray_x ray_y ray_z]
D2 = zeros(size(ZJ,1),1);
       for i = 1:size(ZJ,1)
           v1 = ZI; v2 = ZJ(i,:);
           D2(i) = length_weight * abs(norm(ZI)-norm(ZJ(i,:))) +  acos(dot(v1, v2) / (norm(v1) * norm(v2)));
       end

   
end

%   taking as arguments a 1-by-N vector ZI containing a single observation
%   from X or Y, an M2-by-N matrix ZJ containing multiple observations from
%   X or Y, and returning an M2-by-1 vector of distances D2, whose Jth
%   element is the distance between the observations ZI and ZJ(J,:).