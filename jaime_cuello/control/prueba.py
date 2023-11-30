import numpy as np
import pandas as pd


a = np.loadtxt('datos.txt')


a.to_csv('matrix.csv', index=False, header=False)