#!/usr/bin/env python
# -*- coding: utf-8 -*-
import pickle
import numpy as np
import pandas as pd 
import math as m

with open('matriz_valores.pkl', 'rb') as archivo: 
    matriz= pickle.load(archivo)


for i in range(matriz.shape[0]-2):
    for j in range(matriz.shape[1]-2):
        if type(matriz[i+1,j+1])==int:
            #verticales
            a=matriz[i,j+1]
            b=matriz[i+2,j+1]
            #horizontales
            c=matriz[i+1,j]
            d=matriz[i+1,j+2]

            if (type(a)!=int) and (type(b)!=int):
                ang1=(a[2]+b[2])/2
                ang2=(a[3]+b[3])/2
                ang3=(a[4]+b[4])/2
                matriz[i+1,j+1]=[i+1,j+1,ang1,ang2,ang3]
            elif (type(c)!=int) and (type(d)!=int):
                ang1=(c[2]+d[2])/2
                ang2=(c[3]+d[3])/2
                ang3=(c[4]+d[4])/2
                matriz[i+1,j+1]=[i+1,j+1,ang1,ang2,ang3]

ruta_archivo_csv = 'matriz_prueba.csv'
df = pd.DataFrame(matriz)

# Guardar el DataFrame en el archivo CSV sin incluir índices y encabezados
df.to_csv(ruta_archivo_csv, index=False, header=False)      

with open('matriz_prueba.pkl', 'wb') as archivo:
    pickle.dump(matriz, archivo)