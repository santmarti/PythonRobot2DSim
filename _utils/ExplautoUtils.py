import numpy as np

from explauto import Environment
from explauto import SensorimotorModel
from explauto import InterestModel

class myEnvironment(Environment):

    def __init__(self, m_mins, m_maxs, s_mins, s_maxs):
        Environment.__init__(self, m_mins, m_maxs, s_mins, s_maxs)
        
    def compute_motor_command(self, m_values):
        return bounds_min_max(m_values, self.conf.m_mins, self.conf.m_maxs)

    def compute_sensori_effect(self, m_values):
        return []

    def getRandomInput(self):
        m_mins = self.conf.m_mins
        m_maxs = self.conf.m_maxs
        l = len(m_mins)
        m = [round(r*(m_maxs[i]-m_mins[i])+m_mins[i],2) for i,r in enumerate(np.random.rand(l))]
        return m

    def getRandomOutput(self):
        s_mins = self.conf.s_mins
        s_maxs = self.conf.s_mins
        l = len(s_mins)
        s = [r*(s_maxs[i]-s_mins[i])+s_mins[i] for i,r in enumerate(np.random.rand(l))]
        return s


class mySMmodel(object):

    def __init__(self,env):
        self.fms = SensorimotorModel.from_configuration(env.conf, 'nearest_neighbor')

    def update(self,m,s):
        self.fms.update(m,s)

    def fwd_prediction(self,m):
    	fms.mode = "exploit" 
        res = fms.forward_prediction(m)
    	fms.mode = "explore" 
        return res

    def inv_prediction(self,s):
    	fms.mode = "exploit" 
        res = fms.inverse_prediction(s)
    	fms.mode = "explore" 
        return res







