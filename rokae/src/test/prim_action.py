import os
import numpy as np

#define prim
class PrimAction:
    def __init__(self, prim):
        self.pre={}
        self.eff={}
        self.prim=prim
        if self.prim=='move':
            self.pre={'have_coarse_pose':True, 'above_bolt':False}
            self.eff={'above_bolt':True}
        elif self.prim=='mate':
            self.pre={'target_aim':False,'above_bolt':True}
            self.eff={'target_aim':True}
        elif self.prim=='push':
            self.pre={'target_clear':False,'above_bolt':True}
            self.eff={'target_clear':True}
        elif self.prim=='insert':
            self.pre={'target_aim':True,'target_clear':True,'cramped':False}
            self.eff={'cramped':True}
        elif self.prim=='disassemble':
            self.pre={'cramped':True,'disassembled':False}
            self.eff={'disassembled':True}
    

    #change stage
    def action(self,stage):
        new_stage=stage
        for x in self.eff:
            new_stage[x]=self.eff[x]
        return new_stage
    

    #verify pre of prim
    def able(self,stage):
        for x in self.pre:
            if not stage[x]==self.pre[x]:
                return False
        return True