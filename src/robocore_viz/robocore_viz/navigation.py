"""Configured Nav2 goal/cancel bridge used by the browser dashboard."""
import math
import threading
import time
from urllib.parse import urlparse
import json
import tornado.web


class Navigation:
    def __init__(self,node,config,allowlist):
        self.config=config
        self.enabled=bool(config.get('action')) and 'nav2_msgs/action/NavigateToPose' in allowlist
        self.lock=threading.RLock();self.handle=None;self.cancel_requested=False
        self.status={'enabled':self.enabled,'state':'idle','message':'Choose a map goal'}
        self.client=None
        if self.enabled:
            from rclpy.action import ActionClient
            from nav2_msgs.action import NavigateToPose
            self.client=ActionClient(node,NavigateToPose,config['action'])
            self.node=node
    def read(self):
        with self.lock:
            result=dict(self.status)
            result['available']=bool(self.client and self.client.server_is_ready())
            return result
    def goal(self,payload):
        from nav2_msgs.action import NavigateToPose
        values=[float(payload.get(key,0)) for key in ('x','y','yaw')]
        if not all(math.isfinite(v) for v in values):raise ValueError('Goal coordinates must be finite')
        with self.lock:
            if not self.enabled:raise ValueError('Navigation is not enabled in this profile')
            if not self.client.server_is_ready():raise ValueError('Nav2 action server is unavailable')
            if self.status['state'] in ('pending','active','canceling'):raise ValueError('Cancel or finish the active goal first')
            goal=NavigateToPose.Goal();goal.pose.header.frame_id=self.config.get('frame','map')
            goal.pose.header.stamp=self.node.get_clock().now().to_msg()
            goal.pose.pose.position.x,goal.pose.pose.position.y=values[:2]
            goal.pose.pose.orientation.z=math.sin(values[2]/2);goal.pose.pose.orientation.w=math.cos(values[2]/2)
            self.cancel_requested=False;self.handle=None
            self.status={'enabled':True,'state':'pending','message':'Sending goal','goal':dict(zip(('x','y','yaw'),values))}
            self.client.send_goal_async(goal,feedback_callback=self.feedback).add_done_callback(self.accepted)
            return self.read()
    def feedback(self,message):
        with self.lock:
            feedback=message.feedback
            self.status['distance_remaining']=float(feedback.distance_remaining)
    def accepted(self,future):
        with self.lock:
            try:
                self.handle=future.result()
                if not self.handle.accepted:
                    self.status.update(state='rejected',message='Nav2 rejected the goal');return
                self.status.update(state='active',message='Navigating')
                self.handle.get_result_async().add_done_callback(self.completed)
                if self.cancel_requested:self._cancel()
            except Exception as error:self.status.update(state='failed',message=str(error))
    def completed(self,future):
        with self.lock:
            try:
                response=future.result()
                state={4:'succeeded',5:'canceled',6:'failed'}.get(response.status,'failed')
                self.status.update(state=state,message='Goal reached' if state=='succeeded' else state.capitalize(),error_code=int(response.result.error_code))
            except Exception as error:self.status.update(state='failed',message=str(error))
            self.handle=None
    def _cancel(self):
        self.status.update(state='canceling',message='Canceling navigation')
        self.handle.cancel_goal_async()
    def cancel(self):
        with self.lock:
            self.cancel_requested=True
            if self.handle is not None:self._cancel()
            elif self.status['state']=='pending':self.status.update(message='Cancel requested')
            return self.read()


class NavigationHandler(tornado.web.RequestHandler):
    def initialize(self,node):self.node=node
    def get(self):self.write(self.node.navigation.read())
    def post(self):
        origin=self.request.headers.get('Origin')
        if origin and urlparse(origin).netloc!=self.request.host:
            raise tornado.web.HTTPError(403,'Origin does not match dashboard')
        if not self.request.headers.get('Content-Type','').startswith('application/json'):
            raise tornado.web.HTTPError(415,'Expected application/json')
        try:
            payload=json.loads(self.request.body)
            result=self.node.navigation.cancel() if payload.get('cancel') is True else self.node.navigation.goal(payload)
            self.write(result)
        except (ValueError,TypeError) as error:
            self.set_status(400);self.write({'error':str(error)})
