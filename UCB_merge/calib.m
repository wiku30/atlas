[A(1),E(1)]=gen_map('A/A1');
[A(2),E(2)]=gen_map('A/A2');
[A(3),E(3)]=gen_map('A/A3');
[A(4),E(4)]=gen_map('A/A4');
[A(5),E(5)]=gen_map('A/A5');

origin = A(3).gps_origin;

for i=1:5
    AA(i)=change_origin(A(i),origin);
    save_map(AA(i),['A/mapsT_unified/A',num2str(i)]);
end