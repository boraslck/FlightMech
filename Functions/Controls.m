function [delta_t,delta_e,delta_a,delta_r] = Controls(delta_td,delta_ed,delta_ad,delta_rd)
    delta_t = deg2rad(delta_td);
    delta_e = deg2rad(delta_ed);
    delta_a = deg2rad(delta_ad);
    delta_r = deg2rad(delta_rd);
end