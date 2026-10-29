"""Actual COMMAND -> PHYSICS -> SENSOR timing of the gyro consumer."""
from types import SimpleNamespace
from Basilisk.architecture import messaging,sysModel
from Basilisk.utilities import SimulationBaseClass
from simulation.assembly import AssemblyContext,SessionConfig,Slot
from simulation.components.clock import ClockComponent
from simulation.components.gyroscope import GyroscopeComponent
from simulation.own_motion import OwnMotionFeedback


def test_capture_consumes_previous_sensor_tick_and_preserves_50hz_clock():
    class Source(sysModel.SysModel):
        def __init__(self):
            super().__init__();self.ModelTag='gyroPhysicsTest';self.message=messaging.SCStatesMsg()
        def UpdateState(self,ns):
            if int(ns)%20_000_000==0:
                self.message.write(messaging.SCStatesMsgPayload(omega_BN_B=[0.,0.,.001]),ns,self.moduleID)
    class Consumer(sysModel.SysModel):
        def __init__(self,gyro):
            super().__init__();self.ModelTag='gyroCaptureTest';self.gyro=gyro;self.observer=OwnMotionFeedback();self.rows=[]
        def UpdateState(self,ns):
            reading=self.gyro.sensor.latest
            if reading is None:return
            self.observer.update(int(ns),reading,[0.,0.,0.],epoch=('s','r'))
            self.rows.append((int(ns),int(reading['sample_time_ns'])))
    source=Source();sim=SimulationBaseClass.SimBaseClass()
    process=sim.CreateNewProcess('graspProcess');process.addTask(sim.CreateNewTask('graspTask',2_500_000))
    ctx=AssemblyContext(sim,None,config=SessionConfig(publish_stride=8),physics_ports=SimpleNamespace(body_state=lambda _:source.message))
    ctx.install([ClockComponent()]);ctx.add(source,Slot.PHYSICS)
    gyro=GyroscopeComponent(dict(white_noise_std_rad_s=0.,initial_bias_std_rad_s=0.,bias_random_walk_rad_s_sqrt_s=0.),seed=2)
    ctx.install([gyro]);consumer=Consumer(gyro);ctx.add(consumer,Slot.COMMAND)
    sim.InitializeSimulation();sim.ConfigureStopTime(100_000_000);sim.ExecuteSimulation()
    assert consumer.rows[0]==(2_500_000,0)
    assert all(0<now-stamp<=20_000_000 for now,stamp in consumer.rows)
    assert consumer.observer.diagnostics()['unique_samples']==5
    assert consumer.rows[-1]==(100_000_000,80_000_000)
