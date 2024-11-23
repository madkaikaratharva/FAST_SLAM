function [particles] = slam_resample(particles, init_weight)
	
	particles_count = size(particles, 2);
	weight_total = 0;
	
    for i = 1:particles_count
		weight_total = weight_total + particles(i).weight;
    end
    
    % Create a copy of particles
    fields = fieldnames(particles);
    new_particles = struct();

    % Copy fields from the original particles to the new_particles
    for i = 1:length(fields)
        field = fields{i};
        new_particles.(field) = particles.(field);
    end

	for i = 1:particles_count


        % Initialize a copy of the total weight
        weight_copy = weight_total;
       
        % Generate a random weight value within the total weight range
        W_random = (weight_total - 0) * rand;
        
        for j = 1:particles_count
            weight_copy = weight_copy - particles(j).weight;
          
            % Check if the cumulative weight is within the random weight range
            if weight_copy <= W_random
                 % Assign the selected particle to the new_particles
                new_particles(i) = particles(j);

                % Set the weight of the new particle to the initial weight
                new_particles(i).weight = init_weight;
                break;
            end
        end
     
    end
   
    % Return the new set of particles after resampling
    particles = new_particles;
end