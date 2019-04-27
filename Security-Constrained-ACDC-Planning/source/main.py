# 
#!/usr/bin/python
#
# Copyright 2019, Southeast University, Liu Pengxiang
#
# Code for the paper (IEEE Transactions on Sustainable Energy):
#
# Security-Constrained Planning of AC-DC Hybrid Distribution System 
# with High Penetration of Renewables
#


import sys
import csv
import math
import xlrd
import time
import numpy as np
import matplotlib.pyplot as plt

from gurobipy import *

# This function input data from Excel files.
#
def ReadData(filename,num):
    Data = []
    readbook = xlrd.open_workbook(filename)
    # Data preprocessing
    for i in range(num):  # sheet number
        sheet = readbook.sheet_by_index(i)
        n_row = sheet.nrows
        n_col = sheet.ncols
        if i < num-1:  # coordinate of slice
            Coordinate = [1,n_row,0,n_col]
        else:
            Coordinate = [2,n_row,1,n_col]
        Temp = sheet._cell_values       # data in the Excel file
        Data.append(np.array(Matrix_slice(Temp,Coordinate)))
    return Data


# This function slice the matrix for easy operation
#
def Matrix_slice(Matrix,Coordinate):
    row_start = Coordinate[0]
    row_end   = Coordinate[1]
    col_start = Coordinate[2]
    col_end   = Coordinate[3]
    Matrix_partitioned = []  # A partitioned matrix
    for i in range(row_end-row_start):
        Matrix_partitioned.append([])
        for j in range(col_end-col_start):
            Matrix_partitioned[i].append(Matrix[row_start+i][col_start+j])
    return Matrix_partitioned

if __name__ == "__main__":

    # Input parameter
    filename = "data/Data-Ninghai.xlsx"  # file name
    Data = ReadData(filename,7)  # Data
    
    n = 1