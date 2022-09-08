import numpy as np 

L_range = np.array([1.2412,1.28315,0.324234,0.432562363,0.3244,1.2752,0.123,0.23423,1.214,3.342,6.345,0.123])
R_range = np.array([1.2412,1.28315,0.324234,0.432562363,0.3244,1.2752,0.123,0.23423,1.214,3.342,6.345,0.123])

def L_speed_set(L_range):

    D_M = 1.25
    D_m = 0.1

    TF = [(L_range>D_m)*(L_range<D_M)]

    ID = np.where((L_range>D_m)*(L_range<D_M))

    if sum(np.where((L_range>D_m)*(L_range<D_M), True, False)) == 0:

        return 0

    for i in TF:

        VL = ((D_M-L_range[i])*ID*0.01)

    return (np.sum(VL))

def R_speed_set(R_range):

    D_M = 1.25
    D_m = 0.1

    TF = [(R_range>D_m)*(R_range<D_M)]

    ID = np.where((R_range>D_m)*(R_range<D_M))

    ID_R = (ID[0]-359)

    if sum(np.where((R_range>0.1)*(R_range<D_M), True, False)) == 0 :

        return 0

    for i in TF:

        VL = ((D_M-R_range[i])*ID_R*-0.01)

    return (np.sum(VL))

print(L_speed_set(L_range))
print(R_speed_set(R_range))