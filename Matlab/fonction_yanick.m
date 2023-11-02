sx = 0.3;
sy = 0.1;
Tsup = 1;
increments = 0.1;
pas = 5;
temps = [0:increments:7];
pos= zeros(1,size(temps, 2));

premier_pas = 2.5;

a = find(temps == premier_pas);

for current_pas = 1 : pas
   for current_time = a + ((current_pas - 1) * Tsup): a + current_pas  * Tsup / increments
       pos(current_time) = current_pas * sx;
   end
end

plot(temps, pos)