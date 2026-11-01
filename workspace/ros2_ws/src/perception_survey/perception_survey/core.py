"""B0 고정 12칸 등록부. ROS·정답 월드와 독립인 관측 누적부(좌표 입력 m, 출력 mm)."""
import copy
import json
import math
import os
import tempfile
import uuid
from pathlib import Path

BOX_CENTERS = ((1.468,.950),(3.670,3.152),(2.936,5.354),(.734,5.354),(2.202,5.354),(3.670,3.886))
CAMERAS = {0:'front_d435',1:'rear_d435',2:'arm'}

class Survey:
    def __init__(self, frame='map', gate_m=.035, min_observations=2, tagged_min_confidence=.6):
        if not 0 < gate_m < .04 or min_observations < 1:
            raise ValueError('결합 문턱은 0~40mm 사이, 최소 관측은 1 이상')
        if not math.isfinite(tagged_min_confidence) or not 0 <= tagged_min_confidence <= 1:
            raise ValueError('태그 관측 색띠 점수 문턱은 0~1의 유한값이어야 함')
        self.tagged_min_confidence = tagged_min_confidence
        self.frame, self.gate, self.minimum = frame, gate_m, min_observations
        self.reset()

    def reset(self):
        self.survey_id = uuid.uuid4().hex
        self.stamp = -1.
        self.frozen = False
        self.rejected = {}
        self.boxes = [{'box':i+1,'center_mm':[x*1000,y*1000], 'yaw_deg':0.,'yaw_measured':False} for i,(x,y) in enumerate(BOX_CENTERS)]
        self.tracks = []
        for b,(x,y) in enumerate(BOX_CENTERS,1):
            for side in (-1,1):
                self.tracks.append({'box':b,'side':side,'x_mm':(x+side*.04)*1000,'y_mm':y*1000,
                  'measured':False,'present':None,'color':None,'shape':None,'pose':None,'yaw_deg':None,'id':None,
                  'conf':{'color':0.,'shape':0.,'id':0.},'seen_by':[],'n_obs':0,'last_seen_stamp':None,
                  '_points':[],'_values':{k:{} for k in ('color','shape','pose','id')},'_seen':set(),'_yaw':[]})

    def reject(self, reason):
        self.rejected[reason] = self.rejected.get(reason,0)+1
        return False

    def observe(self, o, stamp):
        if self.frozen: return self.reject('frozen')
        if not math.isfinite(stamp) or stamp < 0: return self.reject('invalid_stamp')
        # 동일 영상에서 여러 퍽을 넣는 것은 허용하지만 과거 시각으로 되돌리지 않음.
        if stamp < self.stamp-1e-6: return self.reject('out_of_order')
        try:
            x,y=float(o['x']),float(o['y']); score=float(o.get('confidence',0.))
            yaw=o.get('yaw_deg')
            if yaw is not None and not math.isfinite(float(yaw)):return self.reject('invalid_yaw')
            tid=o.get('id',-1); color=o.get('color'); shape=o.get('shape'); pose=o.get('pose')
            if not all(math.isfinite(v) for v in (x,y,score)) or not 0<=score<=1: return self.reject('invalid_value')
            if isinstance(tid,bool) or not isinstance(tid,int) or tid < -1 or tid > 11: return self.reject('not_puck_id')
            if color not in (None,'red','green') or shape not in (None,'cylinder','cuboid') or pose not in (None,'standing','lying'):
                return self.reject('invalid_class')
        except (KeyError,ValueError,TypeError): return self.reject('invalid_value')
        ds=sorted((math.hypot(x-t['x_mm']/1000,y-t['y_mm']/1000),i) for i,t in enumerate(self.tracks))
        if ds[0][0]>self.gate: return self.reject('outside_slot')
        if ds[1][0]-ds[0][0]<.005: return self.reject('ambiguous_slot')
        t=self.tracks[ds[0][1]]; camera=o.get('camera','unknown')
        if camera not in CAMERAS.values(): return self.reject('invalid_camera')
        key=(round(stamp,9),camera)
        if key in t['_seen']: return self.reject('duplicate_frame')
        t['_seen'].add(key);t['n_obs']+=1;t['measured']=True;t['present']=True;t['last_seen_stamp']=f'{stamp:.9f}'
        t['_points'].append((x,y));t['_points']=t['_points'][-30:]
        t['x_mm']=sum(p[0] for p in t['_points'])/len(t['_points'])*1000
        t['y_mm']=sum(p[1] for p in t['_points'])/len(t['_points'])*1000
        if camera not in t['seen_by']:t['seen_by'].append(camera)
        for field,value in (('color',color),('shape',shape),('pose',pose),('id',tid if tid>=0 else None)):
            if value is None:continue
            # Puck.confidence는 색/검출 신뢰도. 태그 ID를 이 값으로 버리지 않음.
            # 태그 색띠 점유율과 D435 검출 점수는 같은 수치 의미가 아니다.
            threshold = self.tagged_min_confidence if tid >= 0 and camera == 'arm' else .6
            if field in ('color','shape') and score < threshold:continue
            votes=t['_values'][field];votes[value]=votes.get(value,0)+1
            t[field]=next(iter(votes)) if len(votes)==1 else None
            if field in t['conf']:t['conf'][field]=(1. if field=='id' else score) if len(votes)==1 else 0.
        yaw=o.get('yaw_deg')
        if yaw is not None and math.isfinite(float(yaw)):
            t['_yaw'].append(math.radians(float(yaw)))
            t['yaw_deg']=math.degrees(math.atan2(sum(math.sin(v) for v in t['_yaw']),sum(math.cos(v) for v in t['_yaw'])))
        self.stamp=stamp
        # 초기 ±40mm 배치에서 두 관측 중심으로 박스 축 추정. 퍽 이동 전 조사 전용.
        for b in self.boxes:
            a,z=[v for v in self.tracks if v['box']==b['box']]
            if a['measured'] and z['measured']:
                dx,dy=z['x_mm']-a['x_mm'],z['y_mm']-a['y_mm']
                center=((a['x_mm']+z['x_mm'])/2,(a['y_mm']+z['y_mm'])/2)
                if 60<=math.hypot(dx,dy)<=100 and math.hypot(center[0]-b['center_mm'][0],center[1]-b['center_mm'][1])<=20:
                    b['yaw_deg']=math.degrees(math.atan2(dy,dx));b['yaw_measured']=True;b['yaw_source']='puck_pair'
        return True

    def snapshot(self):
        slots=[{k:copy.deepcopy(v) for k,v in t.items() if not k.startswith('_')} for t in self.tracks]
        errors=[];ids={}
        for t in self.tracks:
            name=f"{t['box']}:{t['side']}"
            if t['n_obs']<self.minimum:errors.append(f'{name}:insufficient_observations')
            for field in ('color','shape','pose','id'):
                votes=t['_values'][field]
                if len(votes)!=1 or sum(votes.values())<self.minimum:errors.append(f'{name}:{field}_unknown_or_conflict')
            if t['shape']=='cylinder' and t['pose']=='lying':errors.append(f'{name}:lying_cylinder')
            if t['id'] is not None:ids.setdefault(t['id'],[]).append(name)
        for tid,names in ids.items():
            if len(names)>1:errors.append(f'duplicate_id:{tid}')
        if len(ids)!=12:errors.append('id_set_incomplete')
        return {'survey_id':self.survey_id,'stamp':f'{max(self.stamp,0):.9f}','frame':f'{self.frame}(mm, 발제 원점)',
                'boxes':copy.deepcopy(self.boxes),'slots':slots,'complete':not errors,
                'errors':errors,'rejected':dict(self.rejected),'frozen':self.frozen}

    def finish(self,path):
        report=self.snapshot()
        if not report['complete']:raise ValueError(';'.join(report['errors']))
        p=Path(path);p.parent.mkdir(parents=True,exist_ok=True)
        self.frozen=True;report=self.snapshot()
        fd,name=tempfile.mkstemp(prefix='.puck_survey_',dir=p.parent)
        try:
            with os.fdopen(fd,'w') as f:json.dump(report,f,ensure_ascii=False,indent=2);f.flush();os.fsync(f.fileno())
            os.replace(name,p)
        except BaseException:
            self.frozen=False
            if os.path.exists(name):os.unlink(name)
            raise
        return report
