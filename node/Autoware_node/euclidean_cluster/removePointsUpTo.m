function removePointsUpTo(in_cloud_ptr, out_cloud_ptr, in_distance)
	out_cloud_ptr.points = [];
	for i = 1:length(in_cloud_ptr.points)
		origin_distance = sqrt(power(in_cloud_ptr.points(i).x, 2) + power(in_cloud_ptr.points(i).y, 2));
		if origin_distance > in_distance
			push(out_cloud_ptr.points, in_cloud_ptr.points(i));
		end
	end
end