"""촬영 시각 TF를 사용해 /perception/pucks를 B0 고정12칸 JSON으로 발행."""
import json
import math
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from tf2_ros import Buffer,TransformListener,TransformException
from std_msgs.msg import String
from std_srvs.srv import Trigger
from robot_interfaces.msg import PuckArray
from .core import Survey,CAMERAS

def rotation(q):
    x,y,z,w=q.x,q.y,q.z,q.w
    norm=math.sqrt(x*x+y*y+z*z+w*w)
    if not math.isfinite(norm) or norm<1e-9:raise ValueError('유효하지 않은 쿼터니언')
    x,y,z,w=[v/norm for v in (x,y,z,w)]
    return ((1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)),
            (2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)),
            (2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)))

def mv(r,v):return [sum(r[i][j]*v[j] for j in range(3)) for i in range(3)]

class SurveyNode(Node):
    def __init__(self):
        super().__init__('puck_survey')
        frame=self.declare_parameter('world_frame','map').value
        self.core=Survey(frame,self.declare_parameter('merge_gate_m',.035).value,self.declare_parameter('min_observations',2).value,self.declare_parameter('tagged_min_confidence',.6).value)
        self.age=self.declare_parameter('max_age_s',.5).value
        self.output=self.declare_parameter('output_path','').value
        self.enabled=False
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub=self.create_publisher(String,'/perception/puck_survey',qos)
        self.sub=self.create_subscription(PuckArray,'/perception/pucks',self.receive,10)
        self.create_service(Trigger,'/perception/survey/start',self.start)
        self.create_service(Trigger,'/perception/survey/reset',self.reset)
        self.create_service(Trigger,'/perception/survey/finish',self.finish)
        self.create_timer(1.,self.publish)
        self.publish()

    def publish(self):
        report=self.core.snapshot();report['enabled']=self.enabled
        self.pub.publish(String(data=json.dumps(report,ensure_ascii=False,allow_nan=False)))

    def start(self,req,res):
        self.core.reset();self.enabled=True;self.publish();res.success=True;res.message='새 퍽 조사 시작';return res

    def reset(self,req,res):
        self.core.reset();self.enabled=False;self.publish();res.success=True;res.message='퍽 등록부 초기화';return res

    def finish(self,req,res):
        try:
            if self.core.rejected.get('audit_io_error'):raise ValueError('관측 이력 저장 실패, 조사 다시 시작 필요')
            if not self.output:raise ValueError('output_path를 지정해야 완료 파일을 저장할 수 있음')
            self.core.finish(self.output);self.enabled=False;self.publish();res.success=True;res.message=self.output
        except (ValueError,OSError) as e:res.success=False;res.message=str(e)
        return res

    def receive(self,msg):
        if not self.enabled:return
        try:stamp=Time.from_msg(msg.header.stamp)
        except ValueError:
            self.core.reject('invalid_stamp');self.publish();return
        age=(self.get_clock().now()-stamp).nanoseconds*1e-9
        if stamp.nanoseconds<=0 or age<-.02 or age>self.age:
            self.core.reject('stale_or_future');self.publish();return
        if not msg.header.frame_id:
            self.core.reject('missing_frame');self.publish();return
        try:
            tf=self.buffer.lookup_transform(self.core.frame,msg.header.frame_id,stamp,timeout=Duration(seconds=.05)).transform
            r=rotation(tf.rotation)
        except (TransformException,ValueError):
            self.core.reject('missing_tf');self.publish();return
        for p in msg.pucks:
            if p.color==3: self.core.reject('blue_obstacle');continue
            xyz=mv(r,[p.pose.position.x,p.pose.position.y,p.pose.position.z])
            xyz=[v+t for v,t in zip(xyz,(tf.translation.x,tf.translation.y,tf.translation.z))]
            pose=None;yaw=None
            try:
                if p.orientation_valid:
                    pr=rotation(p.pose.orientation)
                    axis=mv(r,[pr[i][2] for i in range(3)])
                    pose='standing' if abs(axis[2])>=math.cos(math.radians(20)) else 'lying' if abs(axis[2])<=math.sin(math.radians(20)) else None
                    a=axis if pose=='lying' else mv(r,[pr[i][0] for i in range(3)])
                    yaw=math.degrees(math.atan2(a[1],a[0]))
            except ValueError:pass
            observation={'x':xyz[0],'y':xyz[1],'id':p.tag_id,'color':{1:'red',2:'green'}.get(p.color),
               'shape':{1:'cylinder',2:'cuboid'}.get(p.shape),'pose':pose,'yaw_deg':yaw,
               'camera':CAMERAS.get(p.camera,'unknown'),'confidence':float(p.confidence)}
            accepted=self.core.observe(observation,stamp.nanoseconds*1e-9)
            if accepted and self.output:
                try:
                    path=Path(self.output).with_suffix('.observations.jsonl');path.parent.mkdir(parents=True,exist_ok=True)
                    with path.open('a') as f:
                        f.write(json.dumps({'survey_id':self.core.survey_id,'stamp_ns':stamp.nanoseconds,'input_frame':msg.header.frame_id,
                            'world_frame':self.core.frame,'observation_m':observation},allow_nan=False)+'\n')
                except OSError:
                    self.core.reject('audit_io_error');self.enabled=False;self.get_logger().error('관측 이력 파일 저장 실패')
        self.publish()

def main():
    rclpy.init();node=SurveyNode();ex=MultiThreadedExecutor(num_threads=2);ex.add_node(node)
    try:ex.spin()
    finally:ex.shutdown();node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
