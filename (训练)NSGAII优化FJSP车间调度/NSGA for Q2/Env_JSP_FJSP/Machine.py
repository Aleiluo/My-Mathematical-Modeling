

class Machine:
    def __init__(self, idx):
        self.idx = idx
        self.start = []
        self.end = []
        self._on = []

    def update(self, s, e, Job):
        # 添加当前机器的开始时间s
        self.start.append(s)
        self.start.sort()
        # 添加当前机器的终止时间e
        self.end.append(e)
        self.end.sort()
        # 查找元素s的索引
        idx = self.start.index(s)
        # idx就是当前工件在该机器的第几个工作时间区间
        self._on.insert(idx, Job)

    def find_start(self, s, o_pt):

        ## 需要重写
        
        if self.end == []:
            return max(s, 0)
        else:
            if s > self.end[-1]:
                # 大于最后一个区间的右侧
                return s
            else:
                # 否则优先向前搜索，找到一个可行的空位插入，如果找不到，就在最后一个区间后面加工
                
                # 初始化o_s为最劣情况
                o_s = self.end[-1]
                # .|...........|....|.............|......|...........|.........
                #  倒数第三个区间  一开始指向的位置：^       倒数第一个区间
                l = len(self.end) - 2
                while l >= 0:
                    if s + o_pt > self.start[l + 1]:
                        break
                    if self.end[l] > s and self.end[l] + o_pt <= self.start[l + 1]:
                        o_s = self.end[l]
                    elif self.end[l] < s and s + o_pt <= self.start[l + 1]:
                        o_s = s
                    l -= 1
                return o_s