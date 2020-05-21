function p0cx = getCOMSym(T0x,pxc)
% Returns the center of mass location in the inertial frame {0}
% T0x is the transformation of a vector in frame x to {0}
% pxc is the center of mass expressed in frame x

p0cx = T0x*[pxc(1); pxc(2); pxc(3); 1]; 
p0cx = p0cx(1:3,:);
end

