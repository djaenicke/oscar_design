fileID = fopen('DATA_LOG.BIN');
i = 1;

while ~feof(fileID)
    cnt(i,1)     = fread(fileID, 1, 'uint32');
    r_raw(i,1)  = fread(fileID, 1, 'float32');
    l_raw(i,1)  = fread(fileID, 1, 'float32');
    r_filt(i,1) = fread(fileID, 1, 'float32');
    l_filt(i,1) = fread(fileID, 1, 'float32');
    meas_vbatt(i,1) = fread(fileID, 1, 'float');
    max_vbatt(i,1)  = fread(fileID, 1, 'float');
    max_speed(i,1)  = fread(fileID, 1, 'float');
    min_speed(i,1)  = fread(fileID, 1, 'float');
    r_speed_sp(i,1) = fread(fileID, 1, 'float');
    l_speed_sp(i,1) = fread(fileID, 1, 'float');
    r_error(i,1)    = fread(fileID, 1, 'float');
    l_error(i,1)    = fread(fileID, 1, 'float');
    r_integral(i,1) = fread(fileID, 1, 'float');
    l_integral(i,1) = fread(fileID, 1, 'float');
    u_r(i,1)        = fread(fileID, 1, 'float');
    u_l(i,1)        = fread(fileID, 1, 'float');
    u_r_dc(i,1)     = fread(fileID, 1, 'uint8');
    u_l_dc(i,1)     = fread(fileID, 1, 'uint8');
    pad_byte        = fread(fileID, 1, 'uint8');
    pad_byte        = fread(fileID, 1, 'uint8');
    end_pattern = dec2hex(fread(fileID, 1, 'uint32'));
    i=i+1;
end

fclose(fileID);
