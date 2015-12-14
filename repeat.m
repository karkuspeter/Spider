rep = 100;
spider
wh_vec = w_hist;
Rh_vec = R_hist;
for i=2:rep
    spider;
    wh_vec = wh_vec + w_hist;
    Rh_vec = Rh_vec + R_hist;
end