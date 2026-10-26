#!/usr/bin/env python3
"""场景02只采集验收；不启动/停止Gazebo或PX4。动作仅由显式action-file触发。"""
import argparse
import bisect
from collections import Counter, deque
import copy
import gzip
import hashlib
import json
import math
import os
from pathlib import Path
import socket
import sys
import threading
import time
import xml.etree.ElementTree as ET

DEFAULT_ASSETS_ROOT=Path(__file__).resolve().parents[3]/'px4-gazebo-assets'

def rotate(q,v):
    norm=math.sqrt(sum(x*x for x in q))
    w,x,y,z=(a/norm for a in q)
    # q*(0,v)*conj(q)，独立于生产C++旋转函数。
    tx,ty,tz=2*(y*v[2]-z*v[1]),2*(z*v[0]-x*v[2]),2*(x*v[1]-y*v[0])
    return [v[0]+w*tx+y*tz-z*ty,v[1]+w*ty+z*tx-x*tz,v[2]+w*tz+x*ty-y*tx]

def rpy_quat(r,p,y):
    cr,sr,cp,sp,cy,sy=math.cos(r/2),math.sin(r/2),math.cos(p/2),math.sin(p/2),math.cos(y/2),math.sin(y/2)
    return [cr*cp*cy+sr*sp*sy,sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy]

def nominal_cg(model):
    mass,moment=0.,[0.,0.,0.]
    for link in model.findall('link'):
        lp,ip=link.find('pose'),link.find('inertial/pose')
        if any(e is not None and e.get('relative_to') for e in (lp,ip)):
            raise ValueError('独立CG检查要求展开的绝对link pose')
        pose=list(map(float,link.findtext('pose','0 0 0 0 0 0').split()))
        inertial=list(map(float,link.findtext('inertial/pose','0 0 0 0 0 0').split()))
        m=float(link.findtext('inertial/mass'))
        offset=rotate(rpy_quat(*pose[3:]),inertial[:3])
        for i in range(3):moment[i]+=m*(pose[i]+offset[i])
        mass+=m
    return [x/mass for x in moment]

def relative(row,model,target,cg):
    observer=row['poses'][model]
    offset=rotate(observer['q'],cg)
    v=[row['poses'][target]['p'][i]-observer['p'][i]-offset[i] for i in range(3)]
    return [v[1],v[0],-v[2]]  # 本场景heading=0，ENU转NED。

def angles(r):
    return math.atan2(r[1],r[0]),math.atan2(-r[2],math.hypot(r[0],r[1]))

def omega_rates(r,v):
    """独立位置差分得到r/v，以r×v/|r|²计算NED向量；不使用两角导数。"""
    if len(r)!=3 or len(v)!=3 or not all(math.isfinite(x) for x in (*r,*v)):
        raise ValueError('独立三维LOS需要有限三维r/v')
    norm2=sum(x*x for x in r)
    if norm2<1e-12:raise ValueError('零距离视线无定义')
    return [math.degrees((r[1]*v[2]-r[2]*v[1])/norm2),
            math.degrees((r[2]*v[0]-r[0]*v[2])/norm2),
            math.degrees((r[0]*v[1]-r[1]*v[0])/norm2)]

def interpolate(rows,stamps,stamp,model,target):
    index=bisect.bisect_left(stamps,stamp)
    if index<len(rows) and abs(stamps[index]-stamp)<1e-8:
        return rows[index]
    if not 0<index<len(rows):return None
    a,b=rows[index-1],rows[index]
    dt=b['stamp']-a['stamp']
    if not 0<dt<=.12:return None
    alpha=(stamp-a['stamp'])/dt
    result={'stamp':stamp,'poses':{}}
    for name in (model,target):
        aa,bb=a['poses'][name],b['poses'][name]
        sign=1 if sum(x*y for x,y in zip(aa['q'],bb['q']))>=0 else -1
        result['poses'][name]={'p':[x+alpha*(y-x) for x,y in zip(aa['p'],bb['p'])],
                               'q':[x+alpha*(sign*y-x) for x,y in zip(aa['q'],bb['q'])]}
    return result

def frequency(rows):
    stamps=[r['mono'] for r in rows]
    if len(stamps)<2:return {'samples':len(stamps),'hz':0.}
    intervals=[b-a for a,b in zip(stamps,stamps[1:])]
    bins=Counter(int(t-stamps[0]) for t in stamps)
    complete=[v for k,v in bins.items() if 1<=k<int(stamps[-1]-stamps[0])]
    return {'samples':len(stamps),'hz':(len(stamps)-1)/(stamps[-1]-stamps[0]),
            'mean_complete_real_second_hz':sum(complete)/len(complete) if complete else None,
            'per_real_second':dict(sorted(bins.items())), 'max_interval_s':max(intervals)}

class Capture:
    def __init__(self,args):
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.msgs10.stringmsg_pb2 import StringMsg
        from gz.transport13 import Node
        from ground_station import UdpClient
        from telemetry_forward import TelemetryForwarder,aisheng_message
        self.a,self.lock,self.rows,self.running=args,threading.RLock(),[],True
        self.plugin,self.sources,self.packets,self.poses,self.errors=[],[],[],[],[]
        self.expected=deque()
        self.pose_batches={}
        self.pose_sources=Counter()
        self.pose_duplicate_same_stamp=0
        self.pose_conflicts_same_stamp=0
        self.px4_copies=[]
        self.px4_socket=socket.socket(socket.AF_INET,socket.SOCK_DGRAM) if args.px4_udp_port else None
        self.phase='passive'
        self.node=Node()
        self.node.subscribe(StringMsg,args.prefix+'/telemetry',self.on_plugin)
        for suffix in ('pose/info','dynamic_pose/info'):
            topic=f'/world/{args.world}/{suffix}'
            self.node.subscribe(Pose_V,topic,lambda msg, source=topic:self.on_pose(msg,source))
        self.convert=aisheng_message
        self.receiver=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.receiver.bind(('127.0.0.1',0));self.receiver.settimeout(.2)
        self.forward=TelemetryForwarder();self.forward.start(*self.receiver.getsockname())
        self.client=UdpClient(args.udp_host,args.udp_port,aircraft_id=args.aircraft_id,
                              on_log=lambda s:self.record('client_log',s))
        if self.client.bind_error:raise RuntimeError(self.client.bind_error)
        self.client.on_telemetry=self.on_udp_source
        self.worker=threading.Thread(target=self.receive,daemon=True)
        self.worker.start();self.client.start()
        self.monitor_worker=threading.Thread(target=self.monitor,daemon=True)
        self.monitor_worker.start()

    def record(self,kind,value):
        row={'kind':kind,'mono':time.monotonic(),'phase':self.phase,'value':value}
        with self.lock:self.rows.append(row)
        return row

    def on_plugin(self,msg):
        try:
            value=json.loads(msg.data)
            row=self.record('plugin',value)
            with self.lock:self.plugin.append(row)
        except Exception as e:self.record('error',repr(e))

    def on_pose(self,msg,source):
        # full pose/info包含static实体；dynamic流仅作同刻补充，绝不沿用旧位置。
        if not msg.HasField('header') or not msg.header.HasField('stamp'):
            self.record('pose_rejected',{'source':source,'reason':'missing_sim_stamp'})
            return
        stamp_ns=msg.header.stamp.sec*1000000000+msg.header.stamp.nsec
        if stamp_ns<=0:return
        poses={}
        for p in msg.pose:
            if p.name==self.a.model or p.name in self.a.targets:
                poses[p.name]={'p':[p.position.x,p.position.y,p.position.z],
                               'q':[p.orientation.w,p.orientation.x,p.orientation.y,p.orientation.z]}
        if not poses:return
        with self.lock:
            self.pose_sources[source]+=1
            batch=self.pose_batches.setdefault(stamp_ns,{'poses':{},'sources':set(),'name_sources':{}})
            if batch['sources']:self.pose_duplicate_same_stamp+=1
            for name,pose in poses.items():
                previous=batch['poses'].get(name)
                if previous and max(abs(a-b) for a,b in zip(previous['p'],pose['p']))>1e-6:
                    self.pose_conflicts_same_stamp+=1
                old_source=batch['name_sources'].get(name,'')
                # full pose流优先，同stamp的dynamic补充不得覆盖已有full实体。
                if name not in batch['poses'] or '/dynamic_pose/' not in source or '/dynamic_pose/' in old_source:
                    batch['poses'][name]=pose;batch['name_sources'][name]=source
            batch['sources'].add(source)
            merged=copy.deepcopy(batch['poses'])
            sources=sorted(batch['sources'])
            for old in list(self.pose_batches):
                if old<stamp_ns-5000000000:del self.pose_batches[old]
        self.record('pose_packet',{'stamp_ns':stamp_ns,'source':source,'poses':poses})
        if self.a.model not in merged:return
        row={'stamp':stamp_ns*1e-9,'stamp_ns':stamp_ns,'poses':merged,'sources':sources}
        self.record('world_pose',row)
        with self.lock:self.poses.append(row)

    def on_udp_source(self,raw):
        value=json.loads(raw)
        row=self.record('gateway_udp',value)
        with self.lock:self.sources.append(row)
        try:
            packet=self.convert(value)
            data=json.dumps(packet,separators=(',',':'),allow_nan=False).encode()
            with self.lock:self.expected.append((data,value))
            self.forward.offer(raw)
        except Exception as e:
            with self.lock:self.errors.append('转换失败: '+repr(e))

    def receive(self):
        while self.running:
            try:
                data,_=self.receiver.recvfrom(65535)
            except socket.timeout:continue
            except OSError:break
            try:
                value=json.loads(data)
                row=self.record('aisheng_udp',value)
                with self.lock:
                    self.packets.append(row)
                    if self.px4_socket:
                        self.px4_socket.sendto(data,('127.0.0.1',self.a.px4_udp_port))
                        self.px4_copies.append(self.record('px4_udp_copy',value))
                    expected=self.expected.popleft() if self.expected else None
                    if expected is None or data!=expected[0]:
                        self.errors.append('实际UDP与按收到遥测构造的生产转换不一致')
                    if expected is not None:
                        source=expected[1]
                        for axis in ('pitch','yaw'):
                            gyro=max(-20.,min(20.,round(source['attitude_deg'][axis]['vel'],6)))
                            los=source['target_los']
                            target_rate=max(-20.,min(20.,round(los[axis+'_rate_dps'],6))) if los['valid'] else 0.
                            if value['gyro_'+axis+'_rate']!=gyro or value[axis+'_los_rate']!=target_rate:
                                self.errors.append('独立gyro/LOS字段源或限幅检查失败')
                        los=source['target_los']
                        if type(los.get('omega_valid')) is not bool or not isinstance(los.get('omega_reason'),str):
                            self.errors.append('源遥测缺少三维LOS有效性，不能伪造零向量')
                        else:
                            omega=los.get('omega_ned_dps')
                            if los['omega_valid']:
                                if not isinstance(omega,list) or len(omega)!=3:
                                    self.errors.append('有效三维LOS缺少NED向量');continue
                                expected_omega=[round(x,6) for x in omega]
                            else:
                                if omega is not None:self.errors.append('无效三维LOS向量未明确为null')
                                expected_omega=[0.,0.,0.]
                            for name,expected_value in zip(('los_omega_n','los_omega_e','los_omega_d'),expected_omega):
                                if name not in value or value[name]!=expected_value:
                                    self.errors.append('独立三维LOS字段来源/无倍率/不限幅检查失败')
            except Exception as e:
                with self.lock:self.errors.append(repr(e))

    def finish_requested(self):
        return bool(self.a.finish_file and self.a.finish_file.exists())

    def monitor(self):
        self.a.output.mkdir(parents=True,exist_ok=True)
        started=time.monotonic();next_print=started+20
        while self.running:
            try:
                with self.lock:
                    latest={'phase':self.phase,'elapsed_s':time.monotonic()-started,
                            'counts':{'plugin':len(self.plugin),'gateway_udp':len(self.sources),
                                      'aisheng_udp':len(self.packets),'world_pose':len(self.poses),
                                      'px4_udp_copy':len(self.px4_copies),'errors':len(self.errors)},
                            'telemetry':copy.deepcopy(self.sources[-1]['value']) if self.sources else None}
                temporary=self.a.output/'latest.json.tmp'
                temporary.write_text(json.dumps(latest,ensure_ascii=False,indent=2)+'\n')
                temporary.replace(self.a.output/'latest.json')
                if time.monotonic()>=next_print:
                    print('CAPTURE_PROGRESS',json.dumps({'phase':latest['phase'],'elapsed_s':round(latest['elapsed_s'],1),
                                                        'counts':latest['counts'],
                                                        'los':(latest['telemetry'] or {}).get('target_los')},ensure_ascii=False),flush=True)
                    next_print=time.monotonic()+20
            except Exception as exc:
                with self.lock:self.errors.append('进度快照异常: '+repr(exc))
            time.sleep(1.)

    def manual_for(self,yaw,pitch,duration):
        if not all(type(v) in (int,float) and math.isfinite(v) for v in (yaw,pitch,duration)):
            raise ValueError('手动速度和时长必须是有限数值')
        if max(abs(yaw),abs(pitch))>20 or not 0<duration<=30:
            raise ValueError('手动速度每轴±20以内，持续时长(0,30]秒')
        started=time.monotonic()
        try:
            while time.monotonic()-started<duration and not self.finish_requested():
                if self.client.send_manual(yaw=yaw,pitch=pitch)<0:raise RuntimeError('手动速度请求被生产客户端拒绝')
                time.sleep(.05)
        finally:self.client.send_manual()

    def aim(self,yaw,pitch,timeout=20.):
        # 生产协议没有绝对位置指令；仅用实测机械角反馈和既有manual流对准。
        targets={'yaw':yaw,'pitch':pitch}
        for axis,value in targets.items():
            low,high=(-60.,60.) if axis=='yaw' else (-80.,30.)
            if type(value) not in (int,float) or not math.isfinite(value) or not low<=value<=high:
                raise ValueError('绝对对准角超出机械限位')
        deadline=time.monotonic()+timeout
        settled=0
        try:
            while time.monotonic()<deadline and not self.finish_requested():
                if not self.client.public_angles_ready:
                    time.sleep(.05);continue
                actual=self.client.snapshot()['attitude_deg']
                error={axis:target-actual[axis]['pos'] for axis,target in targets.items()}
                settled=settled+1 if max(abs(v) for v in error.values())<.25 else 0
                if settled>=4:return
                rates={axis:max(-16.,min(16.,3.*v)) if abs(v)>.12 else 0. for axis,v in error.items()}
                if self.client.send_manual(**rates)<0:raise RuntimeError('对准速度请求被生产客户端拒绝')
                time.sleep(.05)
            if not self.finish_requested():raise TimeoutError('机械角反馈对准超时')
        finally:self.client.send_manual()

    def poll_action(self):
        if not self.a.action_file or not self.a.action_file.exists():return
        for line in self.a.action_file.read_text().splitlines()[self.action_count:]:
            self.action_count+=1
            if not line.strip():continue
            action=json.loads(line)
            if 'phase' in action:self.phase=str(action.pop('phase'))
            cmd=action.pop('cmd',None)
            if cmd is None:
                self.record('phase',self.phase);continue
            self.record('control_action',{'cmd':cmd,**action})
            if cmd=='aim':
                self.aim(**action);continue
            if cmd=='manual':
                self.manual_for(action.pop('yaw',0.),action.pop('pitch',0.),action.pop('duration',.5));continue
            if cmd not in ('set_vision_mode','track','stop','home','reset','reset_axis'):
                raise ValueError('action-file仅允许吊舱验收控制指令')
            seq=self.client.send_reliable(cmd,**action)
            self.record('control_request',{'cmd':cmd,'seq':seq,**action})
            print('动作',cmd,seq,flush=True)

    def close(self):
        # 生产UdpClient用Event命名_stop，不能调用Thread.join/is_alive，避免触发Thread._stop()。
        operations=[('client_stop',self.client.stop),('client_socket_close',self.client.sock.close)]
        for label,operation in operations:
            try:operation()
            except Exception as exc:self.errors.append(label+': '+repr(exc))
        time.sleep(.3)  # recv循环有.2s超时；不调用生产客户端的Thread内部方法。
        try:self.forward.close()
        except Exception as exc:self.errors.append('forward_close: '+repr(exc))
        time.sleep(.15);self.running=False
        for label,operation in [('receiver_close',self.receiver.close),
                                ('receiver_join',lambda:self.worker.join(timeout=2)),
                                ('monitor_join',lambda:self.monitor_worker.join(timeout=2))]:
            try:operation()
            except Exception as exc:self.errors.append(label+': '+repr(exc))
        if self.px4_socket:
            try:self.px4_socket.close()
            except Exception as exc:self.errors.append('px4_socket_close: '+repr(exc))
        try:
            for topic in self.node.subscribed_topics():self.node.unsubscribe(topic)
        except Exception as exc:self.errors.append('unsubscribe: '+repr(exc))

    def save_raw(self):
        self.a.output.mkdir(parents=True,exist_ok=True)
        with self.lock:data=json.dumps(self.rows,ensure_ascii=False,separators=(',',':')).encode()
        (self.a.output/'samples.json.gz').write_bytes(gzip.compress(data,mtime=0))

    def report(self,cg,world_fingerprint):
        valid_sources=[r for r in self.sources if r['value'].get('target_los',{}).get('valid')]
        invalid_sources=[r for r in self.sources if r['value'].get('target_los',{}).get('valid') is False]
        stopped=[r for r in invalid_sources if r['value'].get('mode')!='TRACK']
        valid_omega_sources=[r for r in self.sources if r['value'].get('target_los',{}).get('omega_valid') is True]
        invalid_omega_sources=[r for r in self.sources if r['value'].get('target_los',{}).get('omega_valid') is False]
        scores,omega_scores,skipped=[],[],Counter()
        # 按整数纳秒排序去重；每个目标一次构建缓存，防止两路重复stamp和大样本反复排序。
        pose_cache={}
        for target in self.a.targets:
            by_stamp={}
            for row in self.poses:
                if self.a.model in row['poses'] and target in row['poses']:
                    by_stamp[row.get('stamp_ns',round(row['stamp']*1e9))]=row
            rows=[by_stamp[key] for key in sorted(by_stamp)]
            pose_cache[target]=(rows,[row['stamp'] for row in rows])
        for source in valid_sources:
            t=source['value'];los=t['target_los'];target=los.get('target_name');stamp=los.get('sim_stamp')
            if target not in self.a.targets or not isinstance(stamp,(int,float)):
                skipped['unknown_target_or_time']+=1;continue
            rows,stamps=pose_cache[target]
            delta=self.a.difference_window
            points=[interpolate(rows,stamps,stamp+d,self.a.model,target) for d in (-delta,0.,delta)]
            if any(r is None for r in points):skipped['pose_coverage']+=1;continue
            before,current,after=points
            ra,rc,rb=(relative(r,self.a.model,target,cg) for r in points)
            if math.hypot(rc[0],rc[1])<1e-6:skipped['singular']+=1;continue
            aa,ab=angles(ra),angles(rb)
            truth=[math.degrees(math.remainder(ab[0]-aa[0],2*math.pi))/(2*delta),math.degrees(ab[1]-aa[1])/(2*delta)]
            reported=[los['yaw_rate_dps'],los['pitch_rate_dps']]
            scores.append({'stamp':stamp,'phase':source['phase'],'truth_dps':truth,'reported_dps':reported,
                           'rate_error_dps':max(abs(a-b) for a,b in zip(truth,reported)),
                           'position_error_m':max(abs(a-b) for a,b in zip(rc,los['relative_position_ned_m']))})
        for source in valid_omega_sources:
            los=source['value']['target_los'];target=los.get('target_name');stamp=los.get('sim_stamp')
            if target not in self.a.targets or not isinstance(stamp,(int,float)):
                skipped['omega_unknown_target_or_time']+=1;continue
            rows,stamps=pose_cache[target];delta=self.a.difference_window
            points=[interpolate(rows,stamps,stamp+d,self.a.model,target) for d in (-delta,0.,delta)]
            if any(row is None for row in points):skipped['omega_pose_coverage']+=1;continue
            ra,rc,rb=(relative(row,self.a.model,target,cg) for row in points)
            velocity=[(b-a)/(2*delta) for a,b in zip(ra,rb)]
            try:truth=omega_rates(rc,velocity)
            except ValueError:skipped['omega_zero_range_or_nonfinite']+=1;continue
            reported=los.get('omega_ned_dps')
            if not isinstance(reported,list) or len(reported)!=3:
                skipped['omega_missing_vector']+=1;continue
            omega_scores.append({'stamp':stamp,'phase':source['phase'],'truth_ned_dps':truth,
                'reported_ned_dps':reported,'omega_error_dps':max(abs(a-b) for a,b in zip(truth,reported)),
                'position_error_m':max(abs(a-b) for a,b in zip(rc,los['relative_position_ned_m']))})
        errors=list(self.errors)
        if any(not {'omega_ned_dps','omega_valid','omega_reason'}<=r['value'].get('target_los',{}).keys() for r in self.sources):
            errors.append('实际源遥测缺少三维LOS字段，不能以零向量代替')
        if len(omega_scores)<20:errors.append(f'独立三维LOS位姿比对样本不足: {len(omega_scores)}')
        if omega_scores and max(row['omega_error_dps'] for row in omega_scores)>self.a.rate_tolerance:
            errors.append('独立r×v三维LOS角速度误差超限')
        if omega_scores and max(row['position_error_m'] for row in omega_scores)>self.a.position_tolerance:
            errors.append('独立三维LOS目标相对位置误差超限')
        plugin_by_stamp={r['value'].get('ts_mono'):r['value'] for r in self.plugin}
        forwarded_matches=0
        for row in self.sources:
            value=row['value']
            if value.get('protocol_version')!=8 or value.get('telemetry_clock')!='steady' or value.get('telemetry_hz')!=50:
                errors.append('实际网关协议或遥测时基/配置不是v8/steady/50Hz')
                break
            direct=plugin_by_stamp.get(value.get('ts_mono'))
            if direct is not None:
                if any(direct.get(key)!=value.get(key) for key in ('target_los','attitude_deg')):
                    errors.append('网关没有原样转发对应插件样本')
                else:forwarded_matches+=1
        if forwarded_matches<20:errors.append('插件到网关对应样本不足')
        if not self.plugin:errors.append('没有真实Gz插件遥测')
        if not valid_sources:errors.append('没有有效目标LOS样本')
        if valid_sources and max(abs(r['value']['target_los'][axis+'_rate_dps']) for r in valid_sources for axis in ('yaw','pitch'))<.05:
            errors.append('目标惯性LOS全为近零，未覆盖运动目标角速度')
        if not stopped:errors.append('没有停止后LOS无效样本')
        if len(scores)<20:errors.append(f'独立位姿比对样本不足: {len(scores)}')
        if scores and max(r['position_error_m'] for r in scores)>self.a.position_tolerance:errors.append('独立目标相对位置误差超限')
        if scores and max(r['rate_error_dps'] for r in scores)>self.a.rate_tolerance:errors.append('独立bearing/elevation差分误差超限')
        if not self.packets:errors.append('没有收到实际艾晟UDP')
        if self.expected:errors.append('存在未收到的艾晟报文')
        fields={'track_status','pitch_angle','yaw_angle','pitch_misalignment','yaw_misalignment','tracking_confidence','target_confidence','tracking_gate_w','tracking_gate_h','gyro_pitch_rate','gyro_yaw_rate','pitch_los_rate','yaw_los_rate','los_omega_n','los_omega_e','los_omega_d'}
        if any(set(r['value'])!=fields for r in self.packets):errors.append('艾晟字段集合不是固定16字段')
        if any(abs(r['value'][field])>20 for r in self.packets for field in ('gyro_pitch_rate','gyro_yaw_rate','pitch_los_rate','yaw_los_rate')):errors.append('艾晟角速度限幅超限')
        by_phase={}
        for phase in sorted(set(r['phase'] for r in self.sources)):
            ss=[r for r in scores if r['phase']==phase]
            by_phase[phase]={'sources':sum(r['phase']==phase for r in self.sources),'scored':len(ss),'max_rate_error_dps':max((r['rate_error_dps'] for r in ss),default=None),
                'omega_scored':sum(row['phase']==phase for row in omega_scores),
                'max_omega_error_dps':max((row['omega_error_dps'] for row in omega_scores if row['phase']==phase),default=None)}
        self.a.output.mkdir(parents=True,exist_ok=True)
        self.save_raw()
        (self.a.output/'independent_scores.json').write_text(json.dumps(scores,ensure_ascii=False,indent=2)+'\n')
        (self.a.output/'independent_omega_scores.json').write_text(json.dumps(omega_scores,ensure_ascii=False,indent=2)+'\n')
        summary={'result':'passed' if not errors else 'failed','errors':errors,'world':self.a.world,'model':self.a.model,
                 'assets_root':str(self.a.assets_root),'nominal_cg_independently_mass_derived_m':cg,'world_sha256':world_fingerprint,
                 'plugin_rate':frequency(self.plugin),'gateway_udp_rate':frequency(self.sources),'aisheng_udp_rate':frequency(self.packets),
                 'valid_omega_samples':len(valid_omega_sources),'invalid_omega_samples':len(invalid_omega_sources),
                 'independent_omega_samples':len(omega_scores),
                 'max_omega_error_dps':max((row['omega_error_dps'] for row in omega_scores),default=None),
                 'valid_los_samples':len(valid_sources),'invalid_los_samples':len(invalid_sources),'stopped_invalid_samples':len(stopped),
                 'plugin_to_gateway_exact_samples':forwarded_matches,'independent_samples':len(scores),'max_position_error_m':max((r['position_error_m'] for r in scores),default=None),
                 'max_rate_error_dps':max((r['rate_error_dps'] for r in scores),default=None),'skipped':dict(skipped),'phases':by_phase,
                 'forwarder':self.forward.snapshot(),'client_stats':self.client.stats(),'actual_aisheng_endpoint':self.receiver_endpoint,
                 'pose_message_sources':dict(self.pose_sources),'pose_duplicate_same_stamp':self.pose_duplicate_same_stamp,
                 'pose_conflicts_same_stamp':self.pose_conflicts_same_stamp,'px4_udp_copy_rate':frequency(self.px4_copies),'px4_udp_port':self.a.px4_udp_port,
                 'target_los_examples':[r['value']['target_los'] for r in valid_sources[:2]]}
        (self.a.output/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
        return summary

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--assets-root',type=Path,default=DEFAULT_ASSETS_ROOT,help='资产仓目录；默认PX4仓同级px4-gazebo-assets')
    p.add_argument('--world-file',type=Path,default=Path('/tmp/px4-scenarios/02_rocket_two_axis_pod/world.sdf'))
    p.add_argument('--world',default='geo_32_217857_119_105878')
    p.add_argument('--partition',default='px4_scenario_02_rocket_two_axis_pod')
    p.add_argument('--model',default='rocket_two_axis_pod_1');p.add_argument('--prefix',default='/uav_1/pod')
    p.add_argument('--udp-host',default='127.0.0.1');p.add_argument('--udp-port',type=int,default=8770);p.add_argument('--aircraft-id',type=int,default=1)
    p.add_argument('--targets',nargs='+',default=['xiaomotuo','xiaobianfu','rocket_two_axis_pod_2'])
    p.add_argument('--px4-udp-port',type=int,help='将实际艾晟UDP原始字节复制到本机PX4端口，例9000；默认不开')
    p.add_argument('--finish-file',type=Path,help='文件出现即正常结束采集并保存报告')
    p.add_argument('--duration',type=float,default=45);p.add_argument('--action-file',type=Path)
    p.add_argument('--difference-window',type=float,default=.04)
    p.add_argument('--position-tolerance',type=float,default=.1);p.add_argument('--rate-tolerance',type=float,default=.8)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();args.assets_root=args.assets_root.resolve()
    station=args.assets_root/'models/gimbals/two_axis_pod/ground_station'
    if not (station/'ground_station.py').is_file():p.error('assets-root下缺少两轴吊舱生产地面站')
    sys.path.insert(0,str(station))
    os.environ['GZ_PARTITION']=args.partition;os.environ['GZ_IP']='127.0.0.1'
    root=ET.parse(args.world_file).find('world')
    assert root.get('name')==args.world
    assert abs(float(root.findtext('spherical_coordinates/heading_deg','0')))<1e-9,'独立NED检查当前仅允许heading=0场景'
    model=root.find(f"model[@name='{args.model}']");assert model is not None
    cg=nominal_cg(model)
    configured=list(map(float,model.findtext("plugin[@name='two_axis::GimbalController']/los_observer_cg_model",'').split()))
    assert len(configured)==3 and max(abs(a-b) for a,b in zip(configured,cg))<1e-8,('固定名义CG与零位质量重心不一致',configured,cg)
    fingerprint=hashlib.sha256(args.world_file.read_bytes()).hexdigest()
    cap=Capture(args);cap.action_count=0;cap.receiver_endpoint=cap.receiver.getsockname()
    print('CAPTURE_READY',json.dumps({'forward_endpoint':cap.receiver_endpoint,'actions':str(args.action_file),'duration':args.duration,'cg':cg}),flush=True)
    started=time.monotonic()
    try:
        while time.monotonic()-started<args.duration and not cap.finish_requested():
            cap.poll_action();time.sleep(.05)
    except BaseException as exc:
        cap.errors.append('采集或显式验收动作异常: '+repr(exc))
    finally:
        try:cap.close()
        except BaseException as exc:cap.errors.append('采集关闭异常: '+repr(exc))
        # 原始证据先落盘，即使后续score或summary失败也可恢复分析。
        cap.save_raw()
    try:summary=cap.report(cg,fingerprint)
    except BaseException as exc:
        summary={'result':'failed','errors':cap.errors+['报告生成异常: '+repr(exc)],
                 'raw_evidence':'samples.json.gz','counts':{'plugin':len(cap.plugin),'gateway_udp':len(cap.sources),
                 'aisheng_udp':len(cap.packets),'world_pose':len(cap.poses)}}
        (args.output/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(summary,ensure_ascii=False,indent=2),flush=True)
    return 0 if summary['result']=='passed' else 1
if __name__=='__main__':raise SystemExit(main())
