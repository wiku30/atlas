[A{1},E(1)]=gen_map('A/A1');
[A{2},E(2)]=gen_map('A/A2');
[A{3},E(3)]=gen_map('A/A3');
[A{4},E(4)]=gen_map('A/A4');
[A{5},E(5)]=gen_map('A/A5');

AA=A{3};

for i=1:5
    if i~=3
        i
        tic
        AA=merge_map(AA,A{i},-1);
        toc
    end
end

showmap(AA);

