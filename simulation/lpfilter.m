function output = lpfilter(input,sample_freq,cutoff_freq)

output = zeros(length(input(:,1)),length(input(1,:)));

fr = sample_freq / cutoff_freq;
ohm = tan(pi / fr);
c = 1.0 + 2.0 * cos(pi / 4.0) * ohm + ohm * ohm;
b0 = ohm * ohm / c;
b1 = 2.0 * b0;
b2 = b0;
a1 = 2.0 * (ohm * ohm - 1.0) / c;
a2 = (1.0 - 2.0 * cos(pi / 4.0) * ohm + ohm * ohm) / c;

zf = zeros(length(input(:,1)),3);

for i = 1:length(input(1,:))
    zf(:,1) = input(:,i) - zf(:,2) * a1 - zf(:,3) * a2;
    output(:,i) = zf(:,1) * b0 + zf(:,2) * b1 + zf(:,3) * b2;

    zf(:,3) = zf(:,2);
    zf(:,2) = zf(:,1);
end