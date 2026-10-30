#____________________________________________________________________________________________
# servol_rt
# 
#____________________________________________________________________________________________

float64[6] pos               # position  
float64[6] vel               # velocity
float64[6] acc               # acceleration
float64    time              # target time [s]; 0 means bound by vel/acc, whose first element must then be positive