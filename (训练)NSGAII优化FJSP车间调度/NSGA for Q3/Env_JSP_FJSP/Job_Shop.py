from Env_JSP_FJSP.Job import Job
from Env_JSP_FJSP.Machine import Machine

class Job_shop:
    def __init__(self,args):
        
        self.parallel_dict = args.parallel_dict
        
        self.n= args.n
        self.m=args.m
        self.O_num=args.O_num
        self.PM = args.Processing_Machine
        self.PT = args.Processing_Time
        self.reset()

    def reset(self):
        self.C_max = 0      # makespan
        self.load=0         # Total load of machines
        self.max_EndM=None  # the last end machine
        self.mac_load=[0]*self.m    # load of each machine
        self.Jobs=[]
        for i in range(self.n):
            Ji=Job(i,self.PM[i],self.PT[i],self.parallel_dict)
            self.Jobs.append(Ji)
        self.Machines=[]
        for j in range(self.m):
            Mi=Machine(j)
            self.Machines.append(Mi)

    # 对染色体的第i个基因
    def decode(self,Job,Machine):
        Ji=self.Jobs[Job]
        # obtain processing time/start time/processing machine of current operation
        # 首先，通过当前工件的加工情况获得该工序的"加工时间","开始时间","加工所用机器"
        o_pt, s, M_idx = Ji.get_next_info(Machine)
        Mi=self.Machines[M_idx-1]
        start=Mi.find_start(s,o_pt)     # obtatin real start time on machine
        # o_pt：处理过程时间
        end=start+o_pt
        self.load+=o_pt
        # 增加当前机器的负载
        self.mac_load[Mi.idx]+=o_pt
        Mi.update(start, end, [Ji.idx, Ji.cur_op])  # update machine state
        Ji.update(start, end, Mi.idx)     #update Job state
        if end>self.C_max:  # update makespan
            self.C_max=end
            self.max_EndM=Mi
        self.max_load = max(self.mac_load)  #update max_load of machine