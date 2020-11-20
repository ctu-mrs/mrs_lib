pkg load symbolic;
%syms p1 p2 p3 dt;
syms p1 p2 p3 dt dtsqhalf;
sym("p1", "real");
sym("p2", "real");
sym("p3", "real");
sym("dt", "real");
sym("dtsqhalf", "real");
%syms p11 p12 p13 p14 p15 p16;
%syms p21 p22 p23 p24 p25 p26;
%syms p31 p32 p33 p34 p35 p36;
%syms p41 p42 p43 p44 p45 p46;
%syms p51 p52 p53 p54 p55 p56;
%syms p61 p62 p63 p64 p65 p66;

A = [
  1, dt, dtsqhalf, 0, 0, 0;
  0, 1, dt, 0, 0, 0;
  0, 0, 0, sym('1'), 1, 0;
  0, 0, 0, 0, 0, p3;
  0, 0, 0, 0, sym('1'), 0;
  0, 0, 0, 0, 0, dt*p1
  ];

P = sym('P', [6 6]);
%P = [
%p11 p12 p13 p14 p15 p16;
%p21 p22 p23 p24 p25 p26;
%p31 p32 p33 p34 p35 p36;
%p41 p42 p43 p44 p45 p46;
%p51 p52 p53 p54 p55 p56;
%p61 p62 p63 p64 p65 p66
%  ];

Pn = simplify(A*P*A');

for i = 1:6
  printf("const double");
  for j = 1:6
    pstr = char(P(i, j));
    printf(" &%s = P(%d, %d)", pstr, i-1, j-1);
    if j < 6
      printf(",");
    endif
  endfor
  printf(";\n");
endfor
printf("double dtsqhalf = dt*dt/2.0;\n");
for i = 1:6
  for j = 1:6
    pnstr = char(Pn(i, j));
    pnstr = strrep(pnstr, "conjugate", "");
    printf("P_ret(%d, %d) = %s + Q(%d, %d);\n", i-1, j-1, pnstr, i-1, j-1);
  endfor
endfor
