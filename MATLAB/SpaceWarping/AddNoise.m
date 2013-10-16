function noisy_z = AddNoise( z, t_error, r_error )
    noisy_z = z;
    length = size(z,2);
    noisy_z(1,:) = noisy_z(1,:)+(t_error * rand(1,length));
    noisy_z(2,:) = noisy_z(2,:)+(r_error*pi/180 * rand(1,length));
end