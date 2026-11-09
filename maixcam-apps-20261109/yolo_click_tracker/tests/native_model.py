"""Rate-limited synthetic firmware target following, no hardware inertia claim."""


def make_native_model(base_type, clock, positions):
    class Model(base_type):
        def __init__(self):
            super().__init__(positions)
            self.rates={1:40.,2:30.}
            self.last_update=clock.monotonic()

        def integrate(self):
            now=clock.monotonic();dt=max(0.,now-self.last_update);self.last_update=now
            if self.stuck:return
            for mid in (1,2):
                if self.enabled[mid]:
                    error=(self.target[mid]-self.physical[mid]+180.)%360.-180.
                    step=max(-self.rates[mid]*6.*dt,min(self.rates[mid]*6.*dt,error))
                    self.physical[mid]+=step

        def write(self,data):
            self.integrate()
            if data[2]==1:self.rates[data[1]]=abs(int.from_bytes(data[3:5],'big',signed=True))
            if data[2]==3:
                stuck=self.stuck;self.stuck=True
                try:return super().write(data)
                finally:self.stuck=stuck
            return super().write(data)

        def read(self,*args,**kwargs):
            self.integrate()
            return super().read(*args,**kwargs)
    return Model()
