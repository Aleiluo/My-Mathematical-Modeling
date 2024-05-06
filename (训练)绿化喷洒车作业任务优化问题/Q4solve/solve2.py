import sys
import gurobipy as gp
from gurobipy import GRB
import time

def mycallback(model, where):

    if where == GRB.Callback.MIP:
        # MIP solution callback
        currentTime = time.time()
        solName = int((currentTime - model._startTime) / model._interval) * model._interval
        
        # Statistics
        objbnd = model.cbGet(GRB.Callback.MIP_OBJBND)
        obj = model.cbGet(GRB.Callback.MIP_OBJBST)
        gap = abs(obj - objbnd) / obj
        
        # Export statistics
        msg = str(currentTime-model._startTime)+" --- " + " obj: " + str(obj) + " objbnd: " + str(objbnd) + " Gap: " + str(gap) + "\n"
        model._reportFile.write(msg)
        model._reportFile.flush()



model = gp.read('ModelQ4-J21J27.lp')
model.read('q4Solve-J21J27 obj 11.2659.sol')
# 求解参数
#model.Params.NoRelHeurTime = 6 * 3600
model.Params.MIPGap = 0.01
model.Params.TimeLimit = 6 * 3600 + 30 * 60

reportFile = open('report2.txt', 'w')

model._startTime = time.time()
model._reportFile = reportFile
model._interval = 1  #每隔一段时间输出当前可行解，单位秒

model.optimize(mycallback)

model.write(f'q4Solve obj {round(model.ObjVal,4)}.sol')
