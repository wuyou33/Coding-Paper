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
        self.N_year   = 5     # number of year
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
        # Converter
        self.Conv = Data[3]
        self.N_conv = len(self.Conv)  # number of converter station
        # Substation
        self.Sub = Data[4]
        self.N_sub = len(self.Sub)  # number of substation
        # Renewables
        self.Gen = Data[5]
        self.N_gen = len(self.Gen)  # number of renewables
        self.Cost_gen_out = np.array([2.2,1.7,6.7])
        self.Cost_gen_cut = np.array([0.0,0.0,0.0])
        # Typical day
        self.Day = Data[6]
        # Depreciation
        self.Dep_line = Depreciation(20,self.I_rate)  # line
        self.Dep_conv = Depreciation(30,self.I_rate)  # converter
        self.Dep_sub = Depreciation(30,self.I_rate)  # substation
        self.Dep_gen  = Depreciation(25,self.I_rate)  # renewables
        # Candidate line (capacity/investment/resistance/reactance/current/type)
        self.Cdd_line = np.array([
            [120, 29232, 0.358, 0.13, 1.091, 0],
            [ 30, 38524, 0.373, 0.21, 0.857, 0],
            [ 90, 70286, 0.365, 0.00, 1.800, 1]]
            )
        self.N_type_line = len(self.Cdd_line)
        # Candidate converter (capacity/investment)
        self.Cdd_conv = np.array([
            [100, 152767],
            [ 50, 152767]]
            )
        self.N_type_conv = len(self.Cdd_conv)
        # Candidate subststion (capacity/investment)
        self.Cdd_sub = np.array([
            [120, 36934],
            [ 60, 36934]]
            )
        self.N_type_sub = len(self.Cdd_sub)


# This class builds the infomation for each bus i, including the 
# set of line and converter station with a head or tail end of bus i
# 
class BusInfo(object):
    def __init__(self,Para):
        # Set of lines whose head-end/tail-end is bus i
        Line_head = [[] for i in range(Para.N_bus)]
        Line_tail = [[] for i in range(Para.N_bus)]
        for i in range(Para.N_line):
            head = Para.Line[i][1]
            tail = Para.Line[i][2]
            Line_head[int(round(head))].append(i)
            Line_tail[int(round(tail))].append(i)
        self.Line_head = Line_head
        self.Line_tail = Line_tail
        # Set of converter station whose head-end/tail-end is bus i
        Conv_head = [[] for i in range(Para.N_bus)]
        Conv_tail = [[] for i in range(Para.N_bus)]
        for i in range(Para.N_conv):
            head = Para.Conv[i][1]
            tail = Para.Conv[i][2]
            Conv_head[int(round(head))].append(i)
            Conv_tail[int(round(tail))].append(i)
        self.Conv_head = Conv_head
        self.Conv_tail = Conv_tail

# This class restores the results of planning problem
class Result_Planning(object):
    # Initialization
    def __init__(self,model,Para):
        self.x_line = self.value(model._x_line, 'int')
        self.x_conv = self.value(model._x_conv, 'int')
        self.x_sub  = self.value(model._x_sub , 'int')
        self.y_line = self.value(model._y_line, 'int')
        self.v_flow = self.value(model._v_flow, 'con')
        self.V_bus  = (self.v_flow)[N_V_bus  : N_V_bus  + Para.N_bus ,:,:,:]
        self.I_line = (self.v_flow)[N_I_line : N_I_line + Para.N_line,:,:,:]
        self.P_line = (self.v_flow)[N_P_line : N_P_line + Para.N_line,:,:,:]
        self.Q_line = (self.v_flow)[N_Q_line : N_Q_line + Para.N_line,:,:,:]
        self.P_conv = (self.v_flow)[N_P_conv : N_P_conv + Para.N_conv,:,:,:]
        self.Q_conv = (self.v_flow)[N_Q_conv : N_Q_conv + Para.N_conv,:,:,:]
        self.P_sub  = (self.v_flow)[N_P_sub  : N_P_sub  + Para.N_sub ,:,:,:]
        self.Q_sub  = (self.v_flow)[N_Q_sub  : N_Q_sub  + Para.N_sub ,:,:,:]
        self.C_load = (self.v_flow)[N_C_load : N_C_load + Para.N_bus ,:,:,:]
        self.S_gen  = (self.v_flow)[N_S_gen  : N_S_gen  + Para.N_gen ,:,:,:]
        self.C_gen  = (self.v_flow)[N_C_gen  : N_C_gen  + Para.N_gen ,:,:,:]
    # Convert gurobi tuplelist to array
    def value(self,v_flow,string):
        # Get value
        key = v_flow.keys()
        val = v_flow.copy()
        for i in range(len(key)):
            if string == 'int':
                val[key[i]] = int(round(v_flow[key[i]].x))
            if string == 'con':
                val[key[i]] = v_flow[key[i]].x
        # Convert dictionary to numpy array
        dim = tuple([item + 1 for item in max(key)])  # dimention
        arr = np.zeros(dim)
        for i in range(len(val)):
            arr[key[i]] = val[key[i]]
        return arr

            

        
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
        d_cel = sheet._cell_values  # data in the Excel cell
        if i < num-1:
            Coordinate = [1,n_row,0,n_col]
        else:
            Coordinate = [2,n_row,0,n_col]
        Data.append(np.array(Matrix_slice(d_cel,Coordinate)))
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


# Global variables
def GlobalVar(Para):
    # Name
    global N_V_bus, N_I_line, N_P_line, N_Q_line, N_P_conv, N_Q_conv
    global N_P_sub, N_Q_sub , N_C_load, N_S_gen , N_C_gen
    global N_N_var
    # Initialization
    N_V_bus  = 0                       # square of bus voltage
    N_I_line = N_V_bus  + Para.N_bus   # square of line current
    N_P_line = N_I_line + Para.N_line  # active power flow
    N_Q_line = N_P_line + Para.N_line  # reactive power flow
    N_P_conv = N_Q_line + Para.N_line  # active power flow
    N_Q_conv = N_P_conv + Para.N_conv  # reactive power compensation
    N_P_sub  = N_Q_conv + Para.N_conv  # power injection at substation
    N_Q_sub  = N_P_sub  + Para.N_sub   # power injection at substation
    N_C_load = N_Q_sub  + Para.N_sub   # Load shedding
    N_S_gen  = N_C_load + Para.N_bus   # renewables generation
    N_C_gen  = N_S_gen  + Para.N_gen   # renewables curtailment
    N_N_var  = N_C_gen  + Para.N_gen   # Number of all variables
    # Return
    return 0


# This function creates the master planning model
# 
def Func_Planning(Para,Info):

    # Import gurobi model
    model = Model()

    # investment variables
    x_line = model.addVars(Para.N_line, Para.N_type_line, vtype = GRB.BINARY)
    x_conv = model.addVars(Para.N_conv, Para.N_type_conv, vtype = GRB.BINARY)
    x_sub  = model.addVars(Para.N_sub,  Para.N_type_sub , vtype = GRB.BINARY)
    # Reconfiguration variables
    y_line = model.addVars(Para.N_line, Para.N_scene, Para.N_year, vtype = GRB.BINARY)
    # Fictitious power flow variables
    f_line = model.addVars(Para.N_line, Para.N_scene, Para.N_year, lb = -1e2)
    f_conv = model.addVars(Para.N_conv, Para.N_scene, Para.N_year, lb = -1e2)
    f_sub  = model.addVars(Para.N_sub,  Para.N_scene, Para.N_year, lb = -1e2)
    f_load = model.addVars(Para.N_bus,  Para.N_scene, Para.N_year, lb = -1e2)
    f_gen  = model.addVars(Para.N_gen,  Para.N_scene, Para.N_year, lb = -1e2)
    # Operating variable
    v_flow = model.addVars(N_N_var, Para.N_hour, Para.N_scene, Para.N_year, lb = -1e2)
    # Update in model
    model._x_line = x_line
    model._x_conv = x_conv
    model._x_sub  = x_sub
    model._y_line = y_line
    model._v_flow = v_flow
    
    # Set objective
    obj_normal = model.addVar()
    obj_fault  = model.addVar()
    # Investment cost
    inv = LinExpr()
    for t in range(Para.N_year):
        RR = (1+Para.I_rate) ** (-(t+1))  # Recovery rate
        for n in range(Para.N_line):  # line
            for k in range(Para.N_type_line):
                cof = Para.Cdd_line[k,1] * Para.Line[n,3]  # coefficient
                inv = inv + RR * x_line[n,k] * cof * Para.Dep_line
        for n in range(Para.N_conv):  # converter
            for k in range(Para.N_type_conv):
                cof = Para.Cdd_conv[k,0] * Para.Cdd_conv[k,1]  # coefficient
                inv = inv + RR * x_conv[n,k] * cof * Para.Dep_conv
        for n in range(Para.N_sub):  # substation
            for k in range(Para.N_type_sub):
                cof = Para.Cdd_sub [k,0] * Para.Cdd_sub [k,1]  # coefficient
                inv = inv + RR * x_sub [n,k] * cof * Para.Dep_sub 
    # Operating cost
    opr = LinExpr()
    for t in range(Para.N_year):
        for s in range(Para.N_scene):
            for h in range(Para.N_hour):
                for n in range(Para.N_sub):  # power purchasing
                    opr = opr + v_flow[N_P_sub  + n,h,s,t] * Para.Cost_load_buy
                for n in range(Para.N_bus):   # load shedding
                    opr = opr + v_flow[N_C_load + n,h,s,t] * Para.Cost_load_cut
                for n in range(Para.N_gen):   # renewables generation and curtailment
                    gen_type = int(Para.Gen[n,3])  # type: wind/solar/hydro
                    opr = opr + v_flow[N_S_gen  + n,h,s,t] * Para.Cost_gen_out[gen_type]
                    opr = opr + v_flow[N_C_gen  + n,h,s,t] * Para.Cost_gen_cut[gen_type]
    # Total cost under normal condition
    model.addConstr(obj_normal == inv + opr * Para.N_time)
    
    # Constraint 0 (installation)
    for t in range(Para.N_year):
        for s in range(Para.N_scene):
            for n in range(Para.N_line):
                model.addConstr(x_line.sum(n,'*') <= 1)
                if Para.Line[n,7] == 0:  # AC line
                    model.addConstr(x_line[n,2] == 0)
                if Para.Line[n,7] == 1:  # DC line
                    model.addConstr(x_line[n,1] == 0)
                    model.addConstr(x_line[n,0] == 0)
    
    # Constraint 1 (reconfiguration)
    for t in range(Para.N_year):
        for s in range(Para.N_scene):
            for n in range(Para.N_line):
                if Para.Line[n,5] > 0:
                    model.addConstr(y_line[n,s,t] <= 1)
                else:
                    model.addConstr(y_line[n,s,t] <= x_line.sum(n,'*'))
    
    # Constraint 2 (fictitious power flow)
    for t in range(Para.N_year):
        for s in range(Para.N_scene):
            for n in range(Para.N_bus):
                if Para.Load[n,t+1] == 0:
                    model.addConstr(f_load[n,s,t] == 0)
                else:
                    model.addConstr(f_load[n,s,t] == 1)
            for n in range(Para.N_line):
                model.addConstr(f_line[n,s,t] >= -1e2 * y_line[n,s,t])
                model.addConstr(f_line[n,s,t] <=  1e2 * y_line[n,s,t])
            for n in range(Para.N_conv):
                model.addConstr(f_conv[n,s,t] >= -1e2 * x_conv.sum(n,'*'))
                model.addConstr(f_conv[n,s,t] <=  1e2 * x_conv.sum(n,'*'))
            for n in range(Para.N_sub):
                model.addConstr(f_sub[n,s,t] >=  0)
                model.addConstr(f_sub[n,s,t] <=  1e2)
            for n in range(Para.N_gen):
                model.addConstr(f_gen [n,s,t] == -1)
    
    # Constraint 3 (connectivity)
    for t in range(Para.N_year):
        for s in range(Para.N_scene):
            for n in range(Para.N_bus):
                line_head = Info.Line_head[n]
                line_tail = Info.Line_tail[n]
                conv_head = Info.Conv_head[n]
                conv_tail = Info.Conv_tail[n]
                expr = LinExpr()
                expr = expr - f_load[n,s,t]
                expr = expr - quicksum(f_line[i,s,t] for i in line_head)
                expr = expr + quicksum(f_line[i,s,t] for i in line_tail)
                expr = expr - quicksum(f_conv[i,s,t] for i in conv_head)
                expr = expr + quicksum(f_conv[i,s,t] for i in conv_tail)
                if n in Para.Sub[:,1]:
                    i = int(np.where(n == Para.Sub[:,1])[0])
                    expr = expr + f_sub[i,s,t]
                if n in Para.Gen[:,1]:
                    i = int(np.where(n == Para.Gen[:,1])[0])
                    expr = expr + f_gen[i,s,t]
                model.addConstr(expr == 0)
    
    # Constraint 4 (radial topology)
    for t in range(Para.N_year):
        for s in range(Para.N_scene):
            expr = LinExpr()
            expr = expr + Para.N_bus_AC
            expr = expr - Para.N_sub
            expr = expr - quicksum(y_line[i,s,t] for i in Para.Line_AC[:,0])
            model.addConstr(expr == 0)

    # Constraint 5 (optimal power flow)
    for t in range(Para.N_year):
        for s in range(Para.N_scene):
            for h in range(Para.N_hour):
                # 0.data of unit
                n_row = math.floor(s%4) * 6 + h
                n_col = math.floor(s/4) * 4
                punit = Para.Day[n_row, n_col:n_col+4]

                # 1.nodal active power balance
                for n in range(Para.N_bus):
                    # Bus-Branch information
                    if Para.Bus[n,4] == 0: factor = math.sin(Para.Factor)  # AC bus
                    if Para.Bus[n,4] == 1: factor = 1                      # DC bus
                    line_head = Info.Line_head[n]
                    line_tail = Info.Line_tail[n]
                    conv_head = Info.Conv_head[n]
                    conv_tail = Info.Conv_tail[n]
                    # Formulate expression
                    expr = LinExpr()
                    expr = expr - quicksum(v_flow[N_P_line + i,h,s,t] for i in line_head)
                    expr = expr + quicksum(v_flow[N_P_line + i,h,s,t] for i in line_tail)
                    expr = expr - quicksum(v_flow[N_P_conv + i,h,s,t] for i in conv_head)
                    expr = expr + quicksum(v_flow[N_P_conv + i,h,s,t] for i in conv_tail)
                    expr = expr + v_flow[N_C_load + n,h,s,t] * factor
                    for i in line_tail:  # line loss
                        coef = Para.Cdd_line[0,2] * Para.Line[i,3]
                        expr = expr - v_flow[N_I_line + i,h,s,t] * coef
                    if n in Para.Sub[:,1]:  # active power input from substation
                        i = int(np.where(n == Para.Sub[:,1])[0])
                        expr = expr + v_flow[N_P_sub  + i,h,s,t]
                    if n in Para.Gen[:,1]:  # active power input from renewables
                        i = int(np.where(n == Para.Gen[:,1])[0])
                        expr = expr + v_flow[N_S_gen  + i,h,s,t] * factor
                    model.addConstr(expr == Para.Load[n,t] * punit[0] * factor)

                # 2.nodal reactive power balance
                for n in range(Para.N_bus):
                    # Bus-Branch information
                    if Para.Bus[n,4] == 0: factor = math.cos(Para.Factor)  # AC bus
                    if Para.Bus[n,4] == 1: factor = 0                      # DC bus
                    line_head = Info.Line_head[n]
                    line_tail = Info.Line_tail[n]
                    conv_head = Info.Conv_head[n]
                    conv_tail = Info.Conv_tail[n]
                    # Formulate expression
                    expr = LinExpr()
                    expr = expr - quicksum(v_flow[N_Q_line + i,h,s,t] for i in line_head)
                    expr = expr + quicksum(v_flow[N_Q_line + i,h,s,t] for i in line_tail)
                    expr = expr - quicksum(v_flow[N_Q_conv + i,h,s,t] for i in conv_head)
                    expr = expr + quicksum(v_flow[N_Q_conv + i,h,s,t] for i in conv_tail)
                    expr = expr + v_flow[N_C_load + n,h,s,t] * factor
                    for i in line_tail:  # line loss
                        coef = Para.Cdd_line[0,3] * Para.Line[i,3]
                        expr = expr - v_flow[N_I_line + i,h,s,t] * coef
                    if n in Para.Sub[:,1]:  # reactive power input from substation
                        i = int(np.where(n == Para.Sub[:,1])[0])
                        expr = expr + v_flow[N_Q_sub  + i,h,s,t]
                    if n in Para.Gen[:,1]:  # reactive power input from renewables
                        i = int(np.where(n == Para.Gen[:,1])[0])
                        expr = expr + v_flow[N_S_gen  + i,h,s,t] * factor
                    model.addConstr(expr == Para.Load[n,t] * punit[0] * factor)
                
                # 3.Voltage balance on line
                for n in range(Para.N_line):
                    bus_head = Para.Line[n,1]
                    bus_tail = Para.Line[n,2]
                    R = Para.Cdd_line[0,3] * Para.Line[i,2]
                    X = Para.Cdd_line[0,3] * Para.Line[i,3]
                    expr = LinExpr()
                    expr = expr + v_flow[N_V_bus + bus_head,h,s,t]
                    expr = expr - v_flow[N_V_bus + bus_tail,h,s,t]
                    expr = expr - v_flow[N_P_line + n,h,s,t] * 2 * R
                    expr = expr - v_flow[N_Q_line + n,h,s,t] * 2 * X
                    expr = expr + v_flow[N_I_line + n,h,s,t] * (R**2 + X**2)
                    model.addConstr(expr >= -Para.Big_M * (1 - y_line[n]))
                    model.addConstr(expr <=  Para.Big_M * (1 - y_line[n]))
                
                # 4.Renewable generation
                for n in range(Para.N_gen):
                    expr = LinExpr()
                    expr = expr + v_flow[N_S_gen + n,h,s,t]
                    expr = expr + v_flow[N_C_gen + n,h,s,t]
                    gen_type = int(Para.Gen[n,3])
                    model.addConstr(expr == Para.Gen[n,2] * punit[gen_type + 1])

                # 5.Lower and upper bounds
                for n in range(Para.N_bus):   # 1) voltage
                    V_low = (Para.Bus[n,1] * Para.Volt_low) ** 2
                    V_upp = (Para.Bus[n,1] * Para.Volt_upp) ** 2
                    model.addConstr(v_flow[N_V_bus  + n,h,s,t] >= V_low)
                    model.addConstr(v_flow[N_V_bus  + n,h,s,t] <= V_upp)
                for n in range(Para.N_line):  # 2) current
                    expr = LinExpr()
                    expr = expr + Para.Line[n,4] * Para.Line[n,5] / Para.Line[n,6]
                    for k in range(Para.N_type_line):
                        expr = expr + x_line[n,k] * Para.Cdd_line[k,4]
                    model.addConstr(v_flow[N_I_line + n,h,s,t] >= -expr)
                    model.addConstr(v_flow[N_I_line + n,h,s,t] <=  expr)
                for n in range(Para.N_line):  # 3) active power
                    expr = LinExpr()
                    expr = expr + Para.Line[n,4] * Para.Line[n,5]
                    for k in range(Para.N_type_line):
                        expr = expr + x_line[n,k] * Para.Cdd_line[k,0]
                    model.addConstr(v_flow[N_P_line + n,h,s,t] >= -expr)
                    model.addConstr(v_flow[N_P_line + n,h,s,t] <=  expr)
                    model.addConstr(v_flow[N_P_line + n,h,s,t] >= -y_line[n,s,t] * Para.Big_M)
                    model.addConstr(v_flow[N_P_line + n,h,s,t] <=  y_line[n,s,t] * Para.Big_M)
                for n in range(Para.N_line):  # 4) reactive power
                    expr = LinExpr()
                    expr = expr + Para.Line[n,4] * Para.Line[n,5]
                    for k in range(Para.N_type_line):
                        expr = expr + x_line[n,k] * Para.Cdd_line[k,0]
                    model.addConstr(v_flow[N_Q_line + n,h,s,t] >= -expr)
                    model.addConstr(v_flow[N_Q_line + n,h,s,t] <=  expr)
                    model.addConstr(v_flow[N_Q_line + n,h,s,t] >= -y_line[n,s,t] * Para.Big_M)
                    model.addConstr(v_flow[N_Q_line + n,h,s,t] <=  y_line[n,s,t] * Para.Big_M)
                    if Para.Line[n,7] == 1:
                        model.addConstr(v_flow[N_Q_line + n,h,s,t] == 0)
                for n in range(Para.N_conv):  # 5) converter
                    expr = LinExpr()
                    for k in range(Para.N_type_conv):
                        expr = expr + x_conv[n,k] * Para.Cdd_conv[k,0]
                    model.addConstr(v_flow[N_P_conv + n,h,s,t] >= -expr)
                    model.addConstr(v_flow[N_P_conv + n,h,s,t] <=  expr)
                    model.addConstr(v_flow[N_Q_conv + n,h,s,t] >= -expr)
                    model.addConstr(v_flow[N_Q_conv + n,h,s,t] <=  expr)
                for n in range(Para.N_sub):
                    expr = LinExpr()
                    expr = expr + Para.Sub[n,2]
                    for k in range(Para.N_type_sub):
                        expr = expr + x_sub[n,k] * Para.Cdd_sub[k,0]
    

    model.addConstr(opr == 0)
    model.setObjective(obj_normal, GRB.MINIMIZE)
    model.setParam("MIPGap", 0.025)
    model.optimize()
    if model.status == GRB.Status.OPTIMAL:
        result = Result_Planning(model,Para)
        Plot.Reconfig(Para,result,0,0)
        return result
    else:
        return 0
    

# This class restores the 'plot' function
class PlotFunc(object):
    def __init__(self,Para):
        pass
    def Planning(self,Para,result):
        x_line = (result.x_line).sum(axis = 1)
        x_conv = (result.x_conv).sum(axis = 1)
        for n in range(Para.N_line):
            if Para.Line[n,5] > 0:  # existing line
                x_line[n] = 1
        self.Plot_Figure(Para,x_line,x_conv)
    def Reconfig(self,Para,result,s,t):
        y_line = result.y_line[:,s,t]
        x_conv = (result.x_conv).sum(axis = 1)
        self.Plot_Figure(Para,y_line,x_conv)
    def Plot_Figure(self,Para,Line,Conv):
        x = (Para.Bus[:,2]).copy()
        y = (Para.Bus[:,3]).copy()
        for n in range(Para.N_bus):  # Bus
            if Para.Bus[n,4] == 0:
                plt.text(x[n]+2,y[n]+2, '%s'%n)
                plt.plot(x[n],y[n],'r.')
            if Para.Bus[n,4] == 1:
                x[n] = x[n] + 200
                y[n] = y[n] + 25
                plt.text(x[n]+2,y[n]+2, '%s'%n)
                plt.plot(x[n],y[n],'b.')
        for n in range(Para.N_line):  # lines
            x1 = x[int(round(Para.Line[n,1]))]
            y1 = y[int(round(Para.Line[n,1]))]
            x2 = x[int(round(Para.Line[n,2]))]
            y2 = y[int(round(Para.Line[n,2]))]
            if Para.Line[n,7] == 0:
                if Line[n] == 1: plt.plot([x1,x2],[y1,y2],'r-' )
                if Line[n] == 0: plt.plot([x1,x2],[y1,y2],'r--')
            if Para.Line[n,7] == 1:
                if Line[n] == 1: plt.plot([x1,x2],[y1,y2],'b-' )
                if Line[n] == 0: plt.plot([x1,x2],[y1,y2],'b--')
        for n in range(Para.N_conv):  # converters
            head = int(round(Para.Conv[n,1]))
            tail = int(round(Para.Conv[n,2]))
            if Conv[n] == 1:
                plt.plot(x[head],y[head],'rs')
                plt.plot(x[tail],y[tail],'bs')
        
        plt.axis('equal')
        plt.show()


if __name__ == "__main__":

    # Input parameter
    Name = "data/Data-Ninghai.xlsx"  # file name
    Data = ReadData (Name,7)  # Data
    Para = Parameter(Data)    # System parameter
    Info = BusInfo  (Para)    # Bus information
    Plot = PlotFunc (Para)

    # Algorithm parameter
    Ret0 = GlobalVar(Para)  # global variables
    Iter = 10  # max number of iteration
    Cont = []  # set of contingency
    lb   = []  # lower bound
    ub   = []  # upper bound

    # Main function
    for k in range(Iter):
        Res_Planning = Func_Planning(Para,Info)

    
    n = 1