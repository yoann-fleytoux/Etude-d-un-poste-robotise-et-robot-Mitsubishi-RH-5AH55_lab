% Calcul du MGI
function [Qtot] = mgi_mitsu(matPostObjetDansR0)
	% Qtot = ensemble des configurations solutions. Qtot est une matrice
	% dont les colonnes sont définies par les 2 configurations solutions.
	Qtot = zeros(2,4);
    a1 = 0.4; % Distance entre L1 et L2 en m
    a2 = 0.4; % Distance entre L2 et L3 en m
	% On identifie T04 (matrice 'MGD' contenant les inconnues qi) à T0P4
	T0P4 = matPostObjetDansR0
	EPS2 = 1;
	cq2 = (T0P4(1,4)^2 + T0P4(2,4)^2 - a1^2 -a2^2)/(2*a1*a2); 

	for i=1:2
    	Qtot(3,i) = T0P4(3,4); % q3 
    	sq2 = EPS2*sqrt(1-cq2^2);
    	Qtot(2,i) = atan2(sq2,cq2); % q2
    	B1 = a1 + a2*cq2;
    	B2 = a2*sq2;
    	sq1 = (B1*T0P4(2,4) - B2*T0P4(1,4))/(B1^2 + B2^2);
    	cq1 = (B1*T0P4(1,4) + B2*T0P4(2,4))/(B1^2 + B2^2);
    	Qtot(1,i) = atan2(sq1,cq1); % q1
    	Qtot(4,i) = atan2(T0P4(2,1),T0P4(1,1)) - Qtot(1,i) - Qtot(2,i); % q4 
    	EPS2=-1; 
	end
end