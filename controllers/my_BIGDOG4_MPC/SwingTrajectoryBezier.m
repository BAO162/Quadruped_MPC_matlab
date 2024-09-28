function  [pout,p_v,p_a]=SwingTrajectoryBezier(pf_init,pf_final,phase,swingtime,h)

        pout=cubicBezier(pf_init,pf_final,phase);
        p_v=cubicBezier_v(pf_init,pf_final,phase)/swingtime;
        p_a=cubicBezier_a(pf_init,pf_final,phase)/(swingtime * swingtime);

if phase<0.5
    zp=cubicBezier(pf_init(3),pf_init(3)+h,phase*2);
   zv=cubicBezier_v(pf_init(3),pf_init(3)+h,phase*2)*2/swingtime;
    za=cubicBezier_a(pf_init(3),pf_init(3)+h,phase*2)*4/(swingtime * swingtime);
else
    zp=cubicBezier(pf_init(3)+h,pf_final(3),phase*2-1);
    zv=cubicBezier_v(pf_init(3)+h,pf_final(3),phase*2-1)*2/swingtime;
    za=cubicBezier_a(pf_init(3)+h,pf_final(3),phase*2-1)*4/(swingtime * swingtime);
end
pout(3)=zp;
p_v(3)=zv;
p_a(3)=za;


end

function pout=cubicBezier(p0,pf,t)
pout=p0+(t^3+3*t^2*(1-t))*(pf-p0);
end

function pout=cubicBezier_v(p0,pf,t)
pout=6*t*(1-t)*(pf-p0);
end

function pout=cubicBezier_a(p0,pf,t)
pout=(6-12*t)*(pf-p0);
end

