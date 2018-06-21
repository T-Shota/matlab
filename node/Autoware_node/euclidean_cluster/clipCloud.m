function clipCloud(in_cloud_ptr, out_cloud_ptr, in_min_height, in_max_height)
	out_cloud_ptr.points = [];
	for i = 1:length(in_cloud_ptr.points)
		if (in_cloud_ptr.points(i).z >= in_min_height) && (in_cloud_ptr.point(i) <= in_max_height)
			push(out_cloud_ptr.points, in_cloud_ptr.points(i));
		end
	end
end