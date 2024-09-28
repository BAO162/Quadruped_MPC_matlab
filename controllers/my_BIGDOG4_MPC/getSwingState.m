function swingstate=getSwingState(phase,offsetsFloat,durationsFloat)
 swing_offset = offsetsFloat + durationsFloat;
  for i=1:4
    if(swing_offset(i) > 1) 
        swing_offset(i) = swing_offset(i)-1;
    end
  end
  swing_duration = [1;1;1;1] - durationsFloat;

   swingstate = phase - swing_offset;

 for i=1:4
  
    if(swingstate(i) < 0) 
        swingstate(i) =swingstate(i)+ 1;
    end
    if(swingstate(i) > swing_duration(i))
     swingstate(i) = 0;
    else
     swingstate(i) = swingstate(i) / swing_duration(i);
    end
 end
