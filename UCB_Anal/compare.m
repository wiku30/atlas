function r=compare(res,truth)
    n=size(truth,1);
    m=size(res,1);
    for i=1:m
        if(floor(i/100)==i/100)
            i
        end
        r.err(i)=1e+243;
        for j=1:n
            tmp=res(i,:)-truth(j,:);
            e=norm(tmp,2);
            if(e<r.err(i))
                r.corr_ind(i)=j;
                r.err(i)=e;
            end
        end
        j=r.corr_ind(i);
        r.corr(i,:)=truth(j,:);
        r.dev(i,:)=res(i,:)-r.corr(i,:);
    end
    r.meanerr=mean(r.err);
    r.rmserr=rms(r.err);
end