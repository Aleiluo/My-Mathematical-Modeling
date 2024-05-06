# 一个Job就是一个工件而不是工序


class Job:
    def __init__(self, idx, processing_machine, processing_time,
                 parallel_dict):

        self.parallel_dict = parallel_dict

        self.idx = idx
        self.processing_machine = processing_machine
        self.processing_time = processing_time
        # 第几次出现
        self.cur_op = 0
        self.cur_pt = None
        self._on = []
        self.start = []
        self.end = []
        # 最后一次终止时间
        self.endt = 0

    def get_next_info(self, Machine):

        # 验证当前工序和上一个工序的关系
        # 如果允许并行，返回的开始时间就是上上个工件的结束时间
        # 否则返回上个工件的结束时间

        m_idx = self.processing_machine[self.cur_op][Machine]
        self.cur_pt = self.processing_time[self.cur_op][Machine]

        # 判断返回的时间
        job_start = self.endt
        if self.cur_op >= 2:
            # 问题中只有这种情况
            _, parallel_work = self.parallel_dict.get((self.idx, self.cur_op),
                                                      (-1, -1))
            if parallel_work != -1:
                if parallel_work < self.cur_op:
                    # 如果允许并行的是上一个工序
                    job_start = self.end[-2]
                else:
                    job_start = self.end[-1]
            else:
                # 并行后的下一个工序要等待所有的并行都完成的了才能开始
                job_start = max(self.end[-1], self.end[-2])

        # 返回处理时间，最后一次作业的结束时间，机器真实编号
        return self.processing_time[self.cur_op][Machine], job_start, m_idx

    def update(self, s, e, m_idx):  # s:工序开始时间，e:工序结束时间，m_idx:机器序号
        self.endt = max(e, self.endt)
        # 该工件加工阶段 + 1
        self.cur_op += 1
        # 添加当前工序的起止时间到工件
        self.start.append(s)
        self.end.append(e)
        # 添加当前工序的加工机器序号
        self._on.append(m_idx)