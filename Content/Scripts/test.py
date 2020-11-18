print("TEST")
import unreal_engine as ue
class PyTester:
    def begin_play(self):
        ue.log('Begin Play on visualizer class')
    
    def tick(self, delta_time):
        ue.log('pytick')