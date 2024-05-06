from Env_JSP_FJSP.Job_Shop import Job_shop


class Popi:
    def __init__(self, args, CHS, J_site=None, len1=0, len2=0):
        self.args = args
        self.J_site = J_site
        self.l1 = len1
        self.CHS = CHS
        self.JS = Job_shop(args)
        if self.args.means_m > 1:
            self.fitness = list(self.decode1())
        else:
            self.fitness = list(self.decode0())

    def decode0(self):
        for i in self.CHS:
            self.JS.decode(i, 0)
        return self.JS.C_max, self.JS.load, self.JS.max_load

    def decode1(self):
        # 对于每一个工序：
        for i in range(self.l1):
            # cur_op是该数字第几次出现
            O_num = self.JS.Jobs[self.CHS[i]].cur_op
            m_idx = self.J_site.index((self.CHS[i], O_num))
            # 传入工件索引和该次加工所使用的机器
            self.JS.decode(self.CHS[i], self.CHS[m_idx + self.l1])
            
        return self.JS.C_max, \
            sum([self.JS.Machines[i].end[-1] - self.JS.mac_load[i] \
                 for i in range(len(self.JS.Machines)) if len(self.JS.Machines[i].end) > 0]), \
                1e19
                
        # return self.JS.C_max, \
        #     sum([self.JS.Machines[i].end[-1] for i in range(len(self.JS.Machines)) if len(self.JS.Machines[i].end) > 0]), \
        #         1e19

        # return self.JS.C_max, \
        #     sum([self.JS.Machines[i].end[-1] - self.JS.mac_load[i] for i in range(len(self.JS.Machines)) if len(self.JS.Machines[i].end) > 0]), \
        #         1e19

        # 返回加急工件最大耗时尽可能少，加急工件总耗时尽可能小，最大机器耗时尽可能小

        # 当前工序使用的机器的序号
        # J_site的(i,j)表示第i个工件的第j个工序
        # self.CHS[i]就是工件的序号
        # O_num就是当前工件执行到第几个工序
        # m_idx就是“机器部分染色体”的位置，机器部分染色体的含义就是这个
        # 那么m_idx就要加上工序总数self.l1才行
