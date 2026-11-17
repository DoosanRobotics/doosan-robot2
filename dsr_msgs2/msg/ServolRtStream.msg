#____________________________________________________________________________________________
# servol_rt
# 
#____________________________________________________________________________________________

float64[6] pos               # position  
float64[6] vel               # velocity
float64[6] acc               # acceleration
float64    time              # target time [s]; 0 means bound by vel/acc, so vel[0] and acc[0] must then be positive