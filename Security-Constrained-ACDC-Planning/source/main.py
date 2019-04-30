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
        # Bus
        self.Bus  = Data[0]
        self.Bus_AC = self.Bus[np.where(self.Bus[:,4] == 0)]  # AC bus
        self.Bus_DC = self.Bus[np.where(self.Bus[:,4] == 1)]  # DC bus
        self.N_bus = len(self.Bus)  # number of bus
        self.N_bus_AC = len(self.Bus_AC)  # number of AC bus
        self.N_bus_DC = len(self.Bus_DC)  # number of DC bus
        # Load
        self.Load = Data[1]
        self.Cost_load_buy = 83   # cost of load purchasing
        self.Cost_load_cut = 150  # cost of load curtailment
        # Line
        self.Line = Data[2]
        self.Line_AC = self.Line[np.where(self.Line[:,6] == 0)]  # AC line
        self.Line_DC = self.Line[np.where(self.Line[:,6] == 1)]  # DC line
        self.N_line = len(self.Line)  # number of line
        self.N_line_AC = len(self.Line_AC)  # number of AC line
        self.N_line_DC = len(self.Line_DC)  # number of DC line
        self.Dep_line = Depreciation(25,self.I_rate)
        # Conv
        self.Conv = Data[3]
        self.N_conv = len(self.Conv)
        # Candidate Equipments
        self.Cdd_line  = np.array([[120, 0.358, 0.13, 129232],
                                   [ 30, 0.373, 0.21, 38524 ],
                                   [ 90, 0.365, 0.00, 70286 ]])
        


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
        Coordinate = [1,n_row,0,n_col]
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


# This function creates a depreciation calculator
#
def Depreciation(life,rate):
    recovery = rate*((1+rate)**life)/((1+rate)**life-1)
    return recovery


if __name__ == "__main__":

    # Input parameter
    filename = "data/Data-Ninghai.xlsx"  # file name
    Data = ReadData(filename,7)  # Data
    Para = Parameter(Data)  # System parameter
    
    n = 1