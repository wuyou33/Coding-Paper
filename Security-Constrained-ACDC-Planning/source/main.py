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


# This class creates the parameter class
#
class Parameter(object):
    def __init__(self,Data):
        # System
        self.N_stage  = 1     # number of stage
        self.N_year   = 10    # number of year in each stage
        self.N_scene  = 16    # number of reconfiguration scenario
        self.N_hour   = 6     # number of hour in each scenario
        self.N_time   = 90    # number of times
        self.I_rate   = 0.05  # interest rate
        self.Big_M    = 2500  # a sufficient large number M
        self.Factor   = 1.25  # power factor
        self.Volt_AC  = 110   # base voltage of AC system
        self.Volt_DC  = 50    # base voltage of DC system
        self.Volt_low = 0.95  # lower limit of bus valtage
        self.Volt_upp = 1.05  # upper limit of bus valtage


# This function inputs data from Excel files
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
        Temp = sheet._cell_values  # data in the Excel file
        Data.append(np.array(Matrix_slice(Temp,Coordinate)))
    return Data


# This function slices the matrix based on given coordinate
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