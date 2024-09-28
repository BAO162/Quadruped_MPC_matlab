function mpctable=getMpcTable(iteration,nIterations,offsets,durations)
mpctable=zeros(4*nIterations,1);

for i=1:nIterations
    iter=mod((i+iteration-1) , nIterations);
    progress=iter*[1;1;1;1]-offsets;
    for j=1:4
        if(progress(j) < 0)
            progress(j) = progress(j)+ nIterations;
        end
        if(progress(j) < durations(j))
            mpctable((i-1)*4 + j) = 1;
        else
            mpctable((i-1)*4 + j) = 0;
        end
    end

end
