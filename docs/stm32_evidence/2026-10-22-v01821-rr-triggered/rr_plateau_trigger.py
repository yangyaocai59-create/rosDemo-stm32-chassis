"""Pure host trigger logic. No I/O or motor commands."""
class RrPlateauTrigger:
    def __init__(self, threshold_ms=100, minimum_elapsed_s=1.0):
        self.threshold_ms=threshold_ms
        self.minimum_elapsed_s=minimum_elapsed_s
        self.previous=None
        self.anchor=None
        self.confirmed_changes=0
        self.trigger=None

    def feed(self, wheel, elapsed_s, active=True):
        if self.trigger is not None:return self.trigger
        if not active:
            self.previous=self.anchor=None
            self.confirmed_changes=0
            return None
        t,count,cps=wheel[0],wheel[4],wheel[8]
        if self.previous is not None:
            dt=(t-self.previous[0])&0xffffffff
            if dt==0:return None # duplicate source timestamp is not evidence
            if dt>200:raise ValueError('WHEEL source timestamp discontinuity/stale')
            if count!=self.previous[1]:
                if count>self.previous[1] and cps>=50:
                    self.confirmed_changes=min(2,self.confirmed_changes+1)
                self.anchor=(t,count,cps,elapsed_s)
            elif (self.anchor is not None and self.confirmed_changes>=2 and
                  elapsed_s>=self.minimum_elapsed_s and
                  ((t-self.anchor[0])&0xffffffff)>=self.threshold_ms):
                self.trigger=dict(kind='RR_SAME_NET_COUNT_AFTER_CONFIRMED_MOTION',
                    first_source_ms=self.anchor[0],last_source_ms=t,
                    same_net_count=count,span_ms=(t-self.anchor[0])&0xffffffff,
                    first_filtered_cps=self.anchor[2],last_filtered_cps=cps,
                    first_host_elapsed_s=self.anchor[3],detected_host_elapsed_s=elapsed_s,
                    minimum_confirmed_count_changes=2,threshold_ms=self.threshold_ms,
                    note='Feedback evidence only, not proof of exact physical wheel stall')
        else:self.anchor=(t,count,cps,elapsed_s)
        self.previous=(t,count)
        return self.trigger
