import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import pickle
with open('look.pkl', 'rb') as archivo: 
    matriz= pickle.load(archivo)


offset=matriz[0,7]
offset1=offset[2]
offset2=offset[3]

for i in range(4):
    for j in range(8):
        if type(matriz[i,j])!=int:
            lista=matriz[i,j]

            lista[2]=lista[2]-offset1
            lista[3]=lista[3]-offset2
            matriz[i,j]=np.array(lista)


with open('look.pkl', 'wb') as archivo:
    pickle.dump(matriz, archivo)