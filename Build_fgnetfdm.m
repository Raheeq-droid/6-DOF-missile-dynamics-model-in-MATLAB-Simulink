function pkt = build_fgnetfdm(lat_deg, lon_deg, alt_ft, phi, theta, psi)
% Returns a 408-byte FGNetFDM v24 UDP packet for FlightGear.
% Called from run_GNC_3d — do not run directly.
 
pkt = zeros(408, 1, 'uint8');
 
pkt = wu32(pkt,  1, 24);
pkt = wf64(pkt,  9, lat_deg * pi/180);
pkt = wf64(pkt, 17, lon_deg * pi/180);
pkt = wf64(pkt, 25, alt_ft);
pkt = wf32(pkt, 33, phi);
pkt = wf32(pkt, 37, theta);
pkt = wf32(pkt, 41, psi);
 
end
 
function p = wu32(p, o, v)
p(o:o+3) = typecast(swapbytes(uint32(v)), 'uint8');
end
 
function p = wf64(p, o, v)
p(o:o+7) = typecast(swapbytes(double(v)), 'uint8');
end
 
function p = wf32(p, o, v)
p(o:o+3) = typecast(swapbytes(single(v)), 'uint8');
end
 