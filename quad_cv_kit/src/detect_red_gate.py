"""红管门框检测与最近门短时跟踪；demo/demo_video_cv_improved.py 为批量入口。"""
from pathlib import Path
import argparse
import json
import subprocess
import cv2
import numpy as np
if __package__:
    from .gate_line_geometry import endpoints, intersection, samples
    from . import gate_models
else:
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from src.gate_line_geometry import endpoints, intersection, samples
    from src import gate_models


def red_mask(frame):
    h, s, v = cv2.split(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV))
    b, g, r = cv2.split(frame.astype(np.float32))
    # 周围背景的红绿比例：比直接平均色相更适合眩光下的蓝紫色偏。
    signal = np.log((r + 10) / (g + 10))
    contrast = signal - cv2.GaussianBlur(signal, (0, 0), 9)
    local = ((h < 30) | (h > 115)) & (s > 20) & (v > 20) & (v < 250) & (signal > -.24) & (contrast > .055)
    red = ((h < 25) | (h > 145)) & (s > 50) & (v > 20) & (signal > .1)
    mask = np.uint8(local | red) * 255
    mask=cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,np.ones((3,3),np.uint8))
    count,labels,stats,_=cv2.connectedComponentsWithStats(mask)
    keep=(stats[:,cv2.CC_STAT_AREA]>=35)&(np.maximum(stats[:,2],stats[:,3])>=25)
    keep[0]=False
    return np.uint8(keep[labels])*255


def prepare_detection_frame(frame):
    height,width=frame.shape[:2]
    ratio=min(640/width,360/height)
    resized=cv2.resize(frame,(round(width*ratio),round(height*ratio)),interpolation=cv2.INTER_AREA)
    offset=np.array([(640-resized.shape[1])//2,(360-resized.shape[0])//2],float)
    canvas=np.zeros((360,640,3),np.uint8)
    x,y=offset.astype(int)
    canvas[y:y+resized.shape[0],x:x+resized.shape[1]]=resized
    return canvas,ratio,offset


def line_from_segment(mask, ends, min_support=.6):
    a, b = ends.reshape(2, 2).astype(float)
    d = b-a
    length = np.linalg.norm(d)
    if length < 32:
        return None
    vertical = abs(d[0]) < abs(d[1])*.65
    horizontal = abs(d[1]) < abs(d[0])*.7
    if not (vertical or horizontal):
        return None
    if (vertical and d[1] < 0) or (horizontal and d[0] < 0):
        a, b = b, a
    p, z, normal = samples(mask, a, b, 10)
    good = z.any(axis=1)
    if good.mean() < min_support:
        return None
    # 每个截面选离种子最近的红色连通段，避免拉向另一根管。
    ids=np.flatnonzero(good)[::2]
    stripes=z[ids]
    positions=np.arange(21)[None,:]
    nearest=np.argmin(np.where(stripes,np.abs(positions-10),100),axis=1)[:,None]
    low=np.max(np.where((~stripes)&(positions<nearest),positions,-1),axis=1)+1
    high=np.min(np.where((~stripes)&(positions>nearest),positions,21),axis=1)-1
    widths=high-low+1
    pts=np.asarray(p[ids]+((low+high)/2-10)[:,None]*normal,np.float32)
    if len(pts) < 10:
        return None
    vx, vy, x, y = cv2.fitLine(pts, cv2.DIST_L2, 0, .01, .01).reshape(-1)
    direction = np.array([vx, vy], float)
    if (vertical and vy < 0) or (horizontal and vx < 0):
        direction *= -1
    normal = np.array([-direction[1], direction[0]])
    origin = np.array([x, y], float)
    t = pts @ direction
    residual=np.abs(pts@normal-origin@normal)
    if np.percentile(residual,90)>5:
        return None
    if (t.max()-t.min())/max(1,np.median(widths))<5:
        return None
    return dict(d=direction, n=normal, b=float(origin@normal), lo=float(t.min()), hi=float(t.max()),
                width=float(np.median(widths)), support=float(good.mean()), vertical=vertical)


def elbow_mask(frame):
    h,s,v=cv2.split(cv2.cvtColor(frame,cv2.COLOR_BGR2HSV))
    white=np.uint8((((h>18)&(h<102)&(s<110)&(v>70))|((s<18)&(v>130))))*255
    white=cv2.morphologyEx(white,cv2.MORPH_OPEN,np.ones((2,2),np.uint8))
    _,labels,stats,_=cv2.connectedComponentsWithStats(white)
    keep=(stats[:,4]>=18)&(stats[:,2]>=3)&(stats[:,3]>=3);keep[0]=False
    return np.uint8(keep[labels])*255


def tube_contrast(frame,line):
    b,g,r=cv2.split(frame.astype(np.float32))
    signal=np.log((r+10)/(g+10))
    ends=endpoints(line)
    p=np.linspace(ends[0],ends[1],max(20,int(line['hi']-line['lo'])))
    width=line['width']
    offsets=np.array([-width*1.1-3,-width*.2,0,width*.2,width*1.1+3])
    q=p[:,None,:]+offsets[None,:,None]*line['n']
    x,y=np.rint(q[...,0]).astype(int),np.rint(q[...,1]).astype(int)
    valid=(x>=0)&(x<frame.shape[1])&(y>=0)&(y<frame.shape[0])
    z=signal[np.clip(y,0,frame.shape[0]-1),np.clip(x,0,frame.shape[1]-1)]
    delta=np.mean(z[:,1:4],axis=1)-(z[:,0]+z[:,4])/2
    delta=delta[valid.all(axis=1)]
    if len(delta)<20:return 0,0
    return float(np.median(delta)),float(np.mean(delta>.035))


def get_lines(frame, mask, reference_frame=None):
    lsd = cv2.createLineSegmentDetector(cv2.LSD_REFINE_STD)
    segments = []
    reference_frame = frame if reference_frame is None else reference_frame
    chroma = cv2.cvtColor(reference_frame, cv2.COLOR_BGR2LAB)[:, :, 1]
    for img in (mask, cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), cv2.createCLAHE(2., (8, 8)).apply(chroma)):
        found = lsd.detect(img)[0]
        if found is not None:
            segments.extend(found.reshape(-1, 4))
    hough=cv2.HoughLinesP(mask,1,np.pi/720,35,minLineLength=55,maxLineGap=10)
    if hough is not None:
        segments.extend(hough.reshape(-1,4))
    lines = [q for s in segments if (q := line_from_segment(mask, s, .48)) is not None]
    lines.sort(key=lambda q: (q['hi']-q['lo'])*q['support'], reverse=True)
    merged = []
    for line in lines:
        group = None
        for old in merged:
            if old['vertical'] != line['vertical'] or np.dot(old['d'], line['d']) < .988:
                continue
            mid = endpoints(line).mean(axis=0)
            distance = abs(mid@old['n']-old['b'])
            projected = endpoints(line)@old['d']
            gap = max(old['lo']-projected.max(), projected.min()-old['hi'], 0)
            if (distance < max(2.5, .45*min(old['width'], line['width']))
                    and gap < min(34, max(12, 2.5*old['width']))):
                group = old
                break
        if group is None:
            merged.append(line)
        else:
            q = endpoints(line)@group['d']
            group['lo'] = min(group['lo'], float(q.min()))
            group['hi'] = max(group['hi'], float(q.max()))
            group['width'] = (group['width']+line['width'])/2
    split=[]
    for line in merged:
        a,b=endpoints(line)
        p,z,_=samples(mask,a,b,max(2,int(line['width']*.4)))
        occupancy=z.any(axis=1)
        joined=cv2.morphologyEx(np.uint8(occupancy)[None,:],cv2.MORPH_CLOSE,np.ones((1,7),np.uint8))[0]>0
        ids=np.flatnonzero(joined)
        runs=np.split(ids,np.flatnonzero(np.diff(ids)>1)+1)
        for run in runs:
            if len(run)<35:continue
            lo,hi=p[run[[0,-1]]]@line['d']
            if (hi-lo)/max(line['width'],1)<5:continue
            item=dict(line,lo=float(lo),hi=float(hi),support=float(occupancy[run].mean()))
            contrast, fraction = max((tube_contrast(source, item) for source in (reference_frame, frame)),
                                     key=lambda measurement: measurement[0])
            if contrast>.035 and fraction>.48:
                item['contrast']=contrast
                split.append(item)
    merged=sorted(split,key=lambda q:(q['hi']-q['lo'])*q['support'],reverse=True)
    # 长线优先；不让浮尘短线导致组合数失控。
    vertical = [q for q in merged if q['vertical']][:10]
    horizontal = [q for q in merged if not q['vertical']][:10]
    return vertical, horizontal


def pipe_groups(vertical,horizontal,frame,reference_frame=None):
    """管子端部组成连通图，缺边的近框和完整框使用同一个选择规则。"""
    lines=vertical+horizontal
    parent=list(range(len(lines)))
    links=[]
    white=elbow_mask(frame if reference_frame is None else reference_frame)
    def root(i):
        while parent[i]!=i:
            parent[i]=parent[parent[i]];i=parent[i]
        return i
    for i,v in enumerate(vertical):
        for j,h in enumerate(horizontal,len(vertical)):
            point=intersection(v,h)
            if point is None:continue
            if abs(float(v['d']@h['d']))>.707:
                continue
            ev,eh=endpoints(v),endpoints(h)
            dv=np.linalg.norm(ev-point,axis=1).min()
            dh=np.linalg.norm(eh-point,axis=1).min()
            gap=max(18,min(26,1.8*max(v['width'],h['width'])))
            x,y=np.rint(point).astype(int)
            patch=white[max(0,y-7):min(white.shape[0],y+8),max(0,x-7):min(white.shape[1],x+8)]
            # 白色弯头给出真正的管端；同一直线后方的管不能把这个端点吃掉。
            if dv>gap or dh>gap:
                if dv>55 or dh>55 or np.count_nonzero(patch)<25:continue
            if max(v['width'],h['width'])>3*min(v['width'],h['width']):continue
            parent[root(i)]=root(j)
            links.append((i,j,point))
    groups=[]
    for r in set(root(i) for i in range(len(lines))):
        indices=[i for i in range(len(lines)) if root(i)==r]
        members=[lines[i] for i in indices]
        width=float(np.median([l['width'] for l in members]))
        length=sum(l['hi']-l['lo'] for l in members)
        if width<3 or length<70:continue
        joints=[(i,j,p) for i,j,p in links if i in indices and j in indices]
        # 以最粗最长的可见管为锚点，不因近框缺边切换到后面的完整框。
        strengths=[(l['hi']-l['lo'])*l['width']**1.5 for l in members]
        anchor=indices[int(np.argmax(strengths))]
        if len(members)>4:
            selected=[anchor]
            while len(selected)<4:
                neighbors=[i for i in indices if i not in selected and
                           sum(lines[j]['vertical']==lines[i]['vertical'] for j in selected)<2 and
                           any((a==i and b in selected) or (b==i and a in selected) for a,b,_ in joints)]
                if not neighbors:break
                selected.append(max(neighbors,key=lambda i:(lines[i]['hi']-lines[i]['lo'])*lines[i]['width']**1.5))
            indices=selected;members=[lines[i] for i in indices]
            joints=[(i,j,p) for i,j,p in joints if i in indices and j in indices]
            width=float(np.median([l['width'] for l in members]))
        score=max((l['hi']-l['lo'])*l['width']**1.5 for l in members)
        score*=1+.04*(len(members)-1)
        consistent=True
        for orientation in (True,False):
            same=[l for l in members if l['vertical']==orientation]
            if len(same)!=2:continue
            a,b=same
            if abs(float(a['d']@b['d']))<.866:
                consistent=False;break
            if max(a['width'],b['width'])/min(a['width'],b['width'])>2.3:
                consistent=False;break
            def fully_visible(l):
                ends=endpoints(l)
                return np.all((ends[:,0]>10)&(ends[:,0]<630)&(ends[:,1]>10)&(ends[:,1]<350))
            if fully_visible(a) and fully_visible(b):
                ra=(a['hi']-a['lo'])/a['width'];rb=(b['hi']-b['lo'])/b['width']
                if max(ra,rb)/min(ra,rb)>2.2:
                    consistent=False;break
        if not consistent:continue
        # 蓝紫色池底纹理只有局部色差；框管仍应有至少一根显著偏红。
        chroma=[]
        for l in members:
            a,b=endpoints(l);p=np.linspace(a,b,60);x,y=np.rint(p).astype(int).T
            valid=(x>=0)&(x<640)&(y>=0)&(y<360)
            pixels=frame[np.clip(y,0,359),np.clip(x,0,639)].astype(float)
            if valid.any():chroma.append(float(np.median(np.log((pixels[valid,2]+10)/(pixels[valid,1]+10)))))
        if not chroma or max(chroma)<.15:continue
        if len(members)==4 and sum(l['vertical'] for l in members)==2:
            vs=sorted([l for l in members if l['vertical']],key=lambda l:endpoints(l).mean(axis=0)[0])
            hs=sorted([l for l in members if not l['vertical']],key=lambda l:endpoints(l).mean(axis=0)[1])
            quad=np.array([intersection(vs[0],hs[0]),intersection(vs[1],hs[0]),
                           intersection(vs[1],hs[1]),intersection(vs[0],hs[1])],np.float32)
            if not cv2.isContourConvex(quad) or cv2.contourArea(quad)<1200:continue
        if len(members)==1:
            l=members[0]
            # 缺少弯头连接时，单根管必须仍有明确红色；蓝色池底纹理不能冒充。
            a,b=endpoints(l);p=np.linspace(a,b,80)
            x,y=np.rint(p).astype(int).T
            valid=(x>=0)&(x<frame.shape[1])&(y>=0)&(y<frame.shape[0])
            hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)[np.clip(y,0,frame.shape[0]-1),np.clip(x,0,frame.shape[1]-1)]
            red=((hsv[:,0]<30)|(hsv[:,0]>143))&(hsv[:,1]>35)
            if np.mean(red[valid])<.4:continue
            pixels=frame[np.clip(y,0,frame.shape[0]-1),np.clip(x,0,frame.shape[1]-1)].astype(float)
            redness=np.median(np.log((pixels[valid,2]+10)/(pixels[valid,1]+10)))
            if redness<.15:continue
        corners=[]
        for _,_,p in joints:
            if all(np.linalg.norm(p-q)>15 for q in corners):corners.append(p)
        segments=[]
        for i,l in zip(indices,members):
            l['observed_segment'] = endpoints(l).copy()
            ends=l['observed_segment'].copy()
            for a,b,p in joints:
                if i in (a,b):
                    ends[np.argmin(np.linalg.norm(ends-p,axis=1))]=p
            if len(joints):
                span=np.sort(ends@l['d'])
                clipped=dict(l,lo=float(span[0]),hi=float(span[1]))
                segments.append(endpoints(clipped))
            else:segments.append(ends)
        groups.append(dict(score=float(score),width=width,lines=members,segments=segments,
                           corners=corners,complete=len(corners)==4 and len(members)==4,
                           quad=None))
    groups.sort(key=lambda q:q['score'],reverse=True)
    return groups


def detect(frame, search_bbox=None, anchor_bbox=None, reference_frame=None, valid_mask=None):
    """Detect in a masked ROI, retaining the full-frame processing scale."""
    small,ratio,offset=prepare_detection_frame(frame)
    if reference_frame is not None and reference_frame.shape != frame.shape:
        raise ValueError('Reference and enhanced frames must have identical dimensions')
    reference = small if reference_frame is None else prepare_detection_frame(reference_frame)[0]
    mask, color_score = gate_models.combined_evidence(small, reference)
    width, height = round(frame.shape[1]*ratio), round(frame.shape[0]*ratio)
    bounds = (int(offset[0]), int(offset[1]), int(offset[0])+width, int(offset[1])+height)
    small_valid = None
    if valid_mask is not None:
        small_valid = np.zeros_like(mask, bool)
        small_valid[bounds[1]:bounds[3], bounds[0]:bounds[2]] = cv2.resize(
            np.uint8(valid_mask), (width, height), interpolation=cv2.INTER_NEAREST) > 0
        mask[~small_valid] = 0
    if search_bbox is not None:
        points=np.asarray(search_bbox,float).reshape(2,2)*ratio+offset
        low=np.floor(points[0]).astype(int);high=np.ceil(points[1]).astype(int)
        low=np.clip(low,[0,0],[640,360]);high=np.clip(high,[0,0],[640,360])
        roi_mask=np.zeros_like(mask)
        roi_mask[low[1]:high[1],low[0]:high[0]]=255
        mask=cv2.bitwise_and(mask,roi_mask)
        small=small.copy();small[roi_mask==0]=0
        reference=reference.copy();reference[roi_mask==0]=0
        bounds = (max(bounds[0], low[0]), max(bounds[1], low[1]),
                  min(bounds[2], high[0]), min(bounds[3], high[1]))
    vertical,horizontal=get_lines(small,mask,reference)
    recovered = gate_models.trim_lines(vertical+horizontal, mask, color_score, bounds)
    vertical = [line for line in recovered if line['vertical']][:10]
    horizontal = [line for line in recovered if not line['vertical']][:10]
    complete = gate_models.complete_models(vertical, horizontal, mask, bounds, small_valid)
    # The weak LAB evidence is safe only with four-side geometric validation.
    # Partial chains have no enclosing model to stop a warm white support leg.
    partial_mask = cv2.bitwise_and(mask, red_mask(reference))
    partial_lines = gate_models.trim_lines(vertical+horizontal, partial_mask, color_score, bounds)
    # Preserve the old partial detector's minimum rod length after re-trimming.
    # Tiny red patches on a white foot must not add an incompatible fourth rod.
    partial_lines = [line for line in partial_lines if line['hi']-line['lo'] >= 35
                     and (line['hi']-line['lo'])/max(1, line['width']) >= 5]
    partial = pipe_groups([line for line in partial_lines if line['vertical']],
                          [line for line in partial_lines if not line['vertical']], small, reference)
    # A supported complete model wins over competing fragments of those same rods.
    partial = [candidate for candidate in partial if not any(
        sum(any(abs(line['d']@member['d']) > .99 and
                    abs(endpoints(line).mean(axis=0)@member['n']-member['b']) < max(3, member['width']*.55)
                    for member in model['lines']) for line in candidate['lines']) >= 2
        for model in complete)]
    candidates = complete+partial
    if anchor_bbox is not None:
        anchor=np.asarray(anchor_bbox,float).reshape(2,2)*ratio+offset
        def anchored(candidate):
            for segment in candidate['segments']:
                points=np.linspace(segment[0],segment[1],40)
                if np.count_nonzero(np.all((points>=anchor[0]-3)&(points<=anchor[1]+3),axis=1))>=2:
                    return True
            return False
        candidates=[c for c in candidates if anchored(c)]
    return candidates, (vertical,horizontal), mask


def apparent_pipe_width(candidate):
    """Length-weighted median tube width in the 640x360 detection canvas.

    For gates built from the same tubing, apparent tube width is a depth
    proxy even when most of the nearer gate has left the image.
    """
    measurements = []
    for line, segment in zip(candidate['lines'], candidate['segments']):
        width = float(line['width'])
        length = float(np.linalg.norm(segment[1] - segment[0]))
        if np.isfinite(width) and width > 0 and length > 0:
            measurements.append((width, length * float(line.get('support', 1))))
    if not measurements:
        return float(candidate['width'])
    measurements.sort()
    half = sum(weight for _, weight in measurements) * .5
    total = 0.0
    for width, weight in measurements:
        total += weight
        if total >= half:
            return width
    return measurements[-1][0]


def select_nearest(candidates, previous=None, width_tolerance=.12):
    """Select thickest supported gate; use continuity only within a depth tie.

    Normalize the connected edge chains before ranking, so unrelated pipes
    discarded by polygon_edges cannot determine which gate is nearest.
    """
    usable = [polygon_edges(c) for c in candidates]
    usable = [c for c in usable if c is not None and c['segments']]
    if not usable:
        return None
    for candidate in usable:
        candidate['apparent_width'] = apparent_pipe_width(candidate)
    thickest = max(c['apparent_width'] for c in usable)
    tied = [c for c in usable
            if c['apparent_width'] >= thickest * (1 - width_tolerance)]
    if previous is not None:
        matching = [c for c in tied if overlap(c, previous)]
        if matching:
            tied = matching
    # Visible length helps break width ties; completeness alone earns no bonus.
    return max(tied, key=lambda c: (sum(np.linalg.norm(s[1]-s[0])
                                      for s in c['segments']),
                                   c['apparent_width']))


class RedGateTracker:
    """Nearest red gate, with bounded image-supported tracking between detections."""

    def __init__(self, fps=30, detect_every=3, hold_seconds=.2):
        if not np.isfinite(fps) or fps <= 0 or detect_every < 1 or hold_seconds < 0:
            raise ValueError('Invalid frame rate or tracking settings')
        self.detect_every = int(detect_every)
        self.hold_frames = int(round(fps * hold_seconds))
        self.previous = None
        self.last_gray = None
        self.last_observed = None
        self.index = 0
        self.last_status = {}

    def reset(self):
        """Discard old target motion when switching to a different YOLO door."""
        self.previous = None
        self.last_gray = None
        self.last_observed = None

    def update(self, frame, search_bbox=None, anchor_bbox=None, allow_detect=True,
               prefer_previous_width=False, reference_frame=None, valid_mask=None):
        small, ratio, offset = prepare_detection_frame(frame)
        reference = prepare_detection_frame(reference_frame)[0] if reference_frame is not None else small
        if reference_frame is not None and reference_frame.shape != frame.shape:
            raise ValueError('Reference frame must match detection frame dimensions')
        gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
        moved = None
        age = self.index - self.last_observed if self.last_observed is not None else 0
        if self.previous is not None and age <= self.hold_frames:
            current_mask, _ = gate_models.combined_evidence(small, reference)
            if valid_mask is not None:
                valid_small = np.zeros_like(current_mask)
                h,w = frame.shape[:2]
                x,y = offset.astype(int)
                resized = cv2.resize(np.uint8(valid_mask), (round(w*ratio),round(h*ratio)), interpolation=cv2.INTER_NEAREST)
                valid_small[y:y+resized.shape[0],x:x+resized.shape[1]] = resized
                current_mask[valid_small == 0] = 0
            moved = move_with_image(self.previous, self.last_gray, gray, current_mask, small)
        fresh = None
        lines = ([], [])
        candidate_count = None
        if allow_detect and (self.index % self.detect_every == 0 or moved is None):
            if search_bbox is None:
                candidates, lines, _ = detect(frame, reference_frame=reference_frame, valid_mask=valid_mask)
            else:
                candidates, lines, _ = detect(frame, search_bbox, anchor_bbox, reference_frame, valid_mask)
            candidate_count = len(candidates)
            fresh = select_nearest(candidates, moved)
            if (prefer_previous_width and fresh is not None and moved is not None
                    and apparent_pipe_width(fresh) < apparent_pipe_width(moved)*.7):
                # A thin background gate must not replace a still-supported near pipe.
                fresh = None
        selected = fresh if fresh is not None else polygon_edges(moved)
        if fresh is not None:
            selected = dict(selected, tracked=False)
            self.last_observed = self.index
        elif selected is not None:
            selected = dict(selected, tracked=True)
        if selected is not None:
            selected['apparent_width'] = apparent_pipe_width(selected)
        self.last_status = dict(frame=self.index, candidate_count=candidate_count,
                                detection_ran=candidate_count is not None,
                                observation=('tracked' if selected.get('tracked') else 'detected')
                                if selected else 'missing',
                                age_frames=self.index-self.last_observed
                                if selected and self.last_observed is not None else None,
                                ratio=ratio, offset=offset.tolist())
        self.previous, self.last_gray = selected, gray
        self.index += 1
        return selected, lines


def original_geometry(frame, candidate):
    """Return serializable geometry in source-image pixels, including padding offset."""
    if candidate is None:
        return None
    _, ratio, offset = prepare_detection_frame(frame)
    segments = [(np.asarray(s)-offset)/ratio for s in candidate['segments']]
    points = np.concatenate(segments)
    low, high = points.min(axis=0), points.max(axis=0)
    height, width = frame.shape[:2]
    bbox = [float(np.clip(low[0], 0, width-1)), float(np.clip(low[1], 0, height-1)),
            float(np.clip(high[0], 0, width-1)), float(np.clip(high[1], 0, height-1))]
    return dict(bbox=bbox, segments=[s.tolist() for s in segments],
                observed_segments=[((np.asarray(line.get('observed_segment', segment))-offset)/ratio).tolist()
                                   for line, segment in zip(candidate['lines'], candidate['segments'])],
                corners=[((np.asarray(p)-offset)/ratio).tolist() for p in candidate['corners']],
                complete=bool(candidate['complete']),
                pipe_width_px=apparent_pipe_width(candidate)/ratio,
                observation='tracked' if candidate.get('tracked') else 'detected',
                model=candidate.get('model', 'endpoint-chain'),
                side_support=candidate.get('side_support'))


def project_gate_geometry(geometry, point_mapper, image_size):
    """Map corrected edges as sampled curves; never bridge invalid raw pixels."""
    if geometry is None:
        return None
    width, height = image_size

    def valid(points):
        return (np.isfinite(points).all(axis=1)
                & (points[:, 0] >= 0) & (points[:, 0] <= width-1)
                & (points[:, 1] >= 0) & (points[:, 1] <= height-1))

    paths = []
    for segment in geometry['segments']:
        a, b = np.asarray(segment, dtype=float)
        count = max(2, int(np.ceil(np.linalg.norm(b-a)/8)) + 1)
        samples = a + np.linspace(0, 1, count)[:, None] * (b-a)
        mapped = np.asarray(point_mapper(samples), dtype=float)
        run = []
        for point, keep in zip(mapped, valid(mapped)):
            if keep:
                run.append(point.tolist())
            else:
                if len(run) >= 2:
                    paths.append(run)
                run = []
        if len(run) >= 2:
            paths.append(run)
    corners = []
    if geometry['corners']:
        mapped = np.asarray(point_mapper(geometry['corners']), dtype=float)
        corners = [p.tolist() for p, keep in zip(mapped, valid(mapped)) if keep]
    return dict(edge_paths=paths, corners=corners, complete=geometry['complete'],
                source_edge_count=len(geometry['segments']),
                observation=geometry['observation'])


def draw_gate_geometry(frame, geometry, index=0, fps=30, view=''):
    """Draw supported straight or mapped curved edges, without a bounding box."""
    result = frame.copy()
    if geometry is not None:
        paths = geometry.get('edge_paths', geometry.get('segments', []))
        for path in paths:
            points = np.rint(path).astype(np.int32)
            cv2.polylines(result, [points], False, (0, 255, 0), 3, cv2.LINE_AA)
        for point in geometry['corners']:
            cv2.circle(result, tuple(np.rint(point).astype(int)), 5,
                       (0, 255, 0), -1, cv2.LINE_AA)
        edges = geometry.get('source_edge_count', len(paths))
        caption = (f'Nearest gate | {geometry["observation"]} | '
                   f'{edges} edges | {index/fps:.1f}s')
    else:
        caption = f'No supported gate | {index/fps:.1f}s'
    if view:
        caption = f'{view} | {caption}'
    cv2.putText(result, caption, (12, 26), cv2.FONT_HERSHEY_SIMPLEX, .6,
                (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(result, caption, (12, 26), cv2.FONT_HERSHEY_SIMPLEX, .6,
                (0, 255, 0) if geometry else (255, 255, 255), 1, cv2.LINE_AA)
    return result


def draw_nearest(frame, candidate, index=0, fps=30):
    """Draw only supported edges of the single nearest gate, in source pixels."""
    return draw_gate_geometry(frame, original_geometry(frame, candidate), index, fps)


def move_with_image(previous,gray,next_gray,mask,next_frame):
    """仅跨相邻帧用图像运动移动标注，每三帧重新检测所有管子。"""
    if previous is None:return None
    band=np.zeros_like(gray)
    for segment in previous['segments']:
        a,b=np.rint(segment).astype(int)
        cv2.line(band,tuple(a),tuple(b),255,16)
    for q in previous['corners']:
        cv2.circle(band,tuple(np.rint(q).astype(int)),12,255,-1)
    pts=cv2.goodFeaturesToTrack(gray,100,.015,5,mask=band,blockSize=5)
    if pts is None or len(pts)<8:return None
    moved,status,error=cv2.calcOpticalFlowPyrLK(gray,next_gray,pts,None,winSize=(25,25),maxLevel=3)
    if moved is None or status is None or error is None:return None
    good=(status.reshape(-1)>0)&(error.reshape(-1)<35)
    if good.sum()<8:return None
    matrix,inliers=cv2.estimateAffinePartial2D(pts[good],moved[good],method=cv2.RANSAC,ransacReprojThreshold=2.5)
    if matrix is None or inliers is None or inliers.sum()<8:return None
    scale=float(np.hypot(matrix[0,0],matrix[1,0]))
    if not .9<scale<1.1:return None
    def warp(points):
        return np.asarray(points)@matrix[:,:2].T+matrix[:,2]
    segments=[];members=[]
    next_mask=mask if mask is not None else red_mask(next_frame)
    supports=[]
    for segment,line in zip(previous['segments'],previous['lines']):
        new=warp(segment)
        ok,a,b=cv2.clipLine((0,0,640,360),tuple(np.rint(new[0]).astype(int)),tuple(np.rint(new[1]).astype(int)))
        if not ok or np.linalg.norm(np.array(a)-b)<35:continue
        a,b=np.array(a,float),np.array(b,float)
        _,z,_=samples(next_mask,a,b,max(4,int(line['width']*.6)))
        if z.any(axis=1).mean()<.58:continue
        d=(b-a)/np.linalg.norm(b-a);n=np.array([-d[1],d[0]])
        item=dict(line,d=d,n=n,b=float(a@n),lo=float(a@d),hi=float(b@d),width=line['width']*scale,
                  observed_segment=warp(line.get('observed_segment', segment)))
        extent=item['observed_segment']@d
        item.update(strong_lo=float(extent.min()),strong_hi=float(extent.max()))
        if previous.get('geometry_validated'):
            support=gate_models.side_evidence(next_mask,item,np.array([a,b]))
            if support is None:continue
            supports.append(support)
        segments.append(np.array([a,b]));members.append(item)
    if not segments:return None
    corners=[]
    for q in previous['corners']:
        q=warp(np.array([q]))[0]
        if not (8<q[0]<632 and 8<q[1]<352):continue
        if sum(np.linalg.norm(segment-q,axis=1).min()<18 for segment in segments)>=2:
            corners.append(q)
    score=max(np.linalg.norm(b-a)*l['width']**1.5 for (a,b),l in zip(segments,members))
    score*=1+.04*(len(members)-1)
    return dict(previous,segments=segments,lines=members,corners=corners,width=float(np.median([l['width'] for l in members])),
                score=float(score),complete=len(corners)==4 and len(segments)==4,tracked=True,
                side_support=supports if previous.get('geometry_validated') else None)


def overlap(a,b):
    mids=np.array([s.mean(axis=0) for s in a['segments']])
    others=np.array([s.mean(axis=0) for s in b['segments']])
    return np.linalg.norm(mids[:,None,:]-others[None,:,:],axis=2).min()<25


def polygon_edges(candidate):
    """矩形管框每个管端只能接一根邻边；部分可见框仍然是同一条链。"""
    if candidate is None:return None
    if candidate.get('geometry_validated'):return candidate
    segments=[np.asarray(s,float) for s in candidate['segments']]
    members=[]
    for segment,line in zip(segments,candidate['lines']):
        a,b=segment;length=np.linalg.norm(b-a)
        if length<20:continue
        d=(b-a)/length;n=np.array([-d[1],d[0]])
        members.append(dict(line,d=d,n=n,b=float(a@n),lo=float(a@d),hi=float(b@d),
                            width=line.get('width',candidate['width']),segment=segment,length=length))
    selected=[]
    for orientation in (True,False):
        same=sorted([m for m in members if m['vertical']==orientation],key=lambda m:m['length'],reverse=True)[:2]
        if len(same)==2:
            a,b=same
            visible=lambda m:np.all((m['segment'][:,0]>8)&(m['segment'][:,0]<632)&(m['segment'][:,1]>8)&(m['segment'][:,1]<352))
            if visible(a) and visible(b) and a['length']/b['length']>2.3:
                same=same[:1]
        selected.extend(same)
    if not selected:return None
    possible=[]
    for i,a in enumerate(selected):
        for j,b in enumerate(selected[:i]):
            if a['vertical']==b['vertical']:continue
            p=intersection(a,b)
            if p is None:continue
            ea,eb=a['segment'],b['segment']
            da,db=np.linalg.norm(ea-p,axis=1),np.linalg.norm(eb-p,axis=1)
            if da.min()>25 or db.min()>25:continue
            possible.append((float(da.min()+db.min()),i,int(da.argmin()),j,int(db.argmin()),p))
    used=set();links=[]
    for _,i,ei,j,ej,p in sorted(possible,key=lambda q:q[0]):
        if (i,ei) in used or (j,ej) in used:continue
        used.add((i,ei));used.add((j,ej));links.append((i,j,p))
    anchor=max(range(len(selected)),key=lambda i:selected[i]['length']*selected[i]['width']**1.5)
    connected={anchor}
    for _ in range(3):
        for i,j,p in links:
            if i in connected or j in connected:connected.update((i,j))
    kept=[selected[i] for i in sorted(connected)]
    corners=[p for i,j,p in links if i in connected and j in connected and 0<=p[0]<640 and 0<=p[1]<360]
    result=dict(candidate,segments=[m['segment'] for m in kept],lines=kept,corners=corners,
                complete=len(kept)==4 and len(corners)==4)
    return result


def draw(frame, candidates, lines, index, fps, debug=False):
    result=frame.copy()
    _,ratio,offset=prepare_detection_frame(frame)
    scale=1/ratio
    edges=len(candidates[0]['segments']) if candidates else 0
    corners=len(candidates[0]['corners']) if candidates else 0
    caption=f'{index/fps:.1f}s  |  {edges} edges, {corners} corners'
    cv2.putText(result,caption,(12,24),0,.55,(0,0,0),3,cv2.LINE_AA)
    cv2.putText(result,caption,(12,24),0,.55,(255,255,255),1,cv2.LINE_AA)
    if candidates:
        c=candidates[0]
        for segment in c['segments']:
            a,b=np.rint((segment-offset)*scale).astype(int)
            cv2.line(result,tuple(a),tuple(b),(0,255,0),3,cv2.LINE_AA)
        points=np.rint((np.array(c['corners'])-offset)*scale).astype(int) if c['corners'] else np.empty((0,2),int)
        for i,p in enumerate(points):
            cv2.circle(result,tuple(p),8,(0,255,255),-1,cv2.LINE_AA)
            q=np.array(c['corners'][i]);direction=[]
            for l,segment in zip(c['lines'],c['segments']):
                ends=np.linalg.norm(segment-q,axis=1)
                if ends.min()<15:direction.append((l['vertical'],segment[np.argmax(ends)]-q))
            dx=next((v[0] for vertical,v in direction if not vertical),1)
            dy=next((v[1] for vertical,v in direction if vertical),1)
            label=(1 if dx>0 else 2) if dy>0 else (4 if dx>0 else 3)
            cv2.putText(result,str(label),tuple(p+np.array([10,-10])),0,.7,(0,255,255),2,cv2.LINE_AA)
    if debug:
        for j,line in enumerate(lines[0]+lines[1]):
            a,b=np.rint((endpoints(line)-offset)*scale).astype(int)
            cv2.line(result,tuple(a),tuple(b),(255,200,0),1,cv2.LINE_AA)
            cv2.putText(result,str(j),tuple((a+b)//2),0,.4,(255,255,255),1)
    return result


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--input',required=True,help='输入视频路径')
    p.add_argument('--output',default=str(Path(__file__).parent/'annotated.mp4'),help='输出 MP4 路径')
    p.add_argument('--samples',action='store_true')
    p.add_argument('--limit',type=int)
    args=p.parse_args()
    c=cv2.VideoCapture(args.input)
    fps=c.get(cv2.CAP_PROP_FPS);n=int(c.get(cv2.CAP_PROP_FRAME_COUNT))
    out=Path(args.output);out.parent.mkdir(exist_ok=True,parents=True)
    if args.samples:
        rows=[]
        for index in np.linspace(0,n-1,30).astype(int):
            c.set(1,int(index));ok,f=c.read()
            if not ok:continue
            candidates,lines,mask=detect(f)
            selected=select_nearest(candidates)
            candidates=[selected] if selected is not None else []
            a=draw(f,candidates,lines,index,fps,debug=True)
            cv2.imencode('.jpg',a)[1].tofile(str(out.parent/f'debug_{index}.jpg'))
            rows.append(cv2.resize(a,(480,270)))
            print(index,len(candidates),len(lines[0]),len(lines[1]),[len(candidates[0]['segments']),len(candidates[0]['corners']),round(candidates[0]['width'],1)] if candidates else '',flush=True)
        cv2.imencode('.jpg',np.vstack([np.hstack(rows[j:j+5]) for j in range(0,len(rows),5)]))[1].tofile(str(out.parent/'detection_contact.jpg'))
    else:
        encoder=subprocess.Popen(['ffmpeg','-hide_banner','-loglevel','error','-y','-f','rawvideo','-pix_fmt','bgr24',
            '-s',f'{int(c.get(3))}x{int(c.get(4))}','-r',str(fps),'-i','pipe:0','-an','-c:v','libx264',
            '-preset','fast','-threads','2','-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',str(out)],stdin=subprocess.PIPE)
        records=[]
        i=0
        tracker=RedGateTracker(fps)
        while True:
            ok,f=c.read()
            if not ok:break
            selected,lines=tracker.update(f)
            annotated=draw_nearest(f,selected,i,fps)
            encoder.stdin.write(annotated.tobytes())
            geometry=original_geometry(f,selected)
            records.append(dict(frame=i,coordinate_space='original',nearest_gate=geometry,
                                segments=geometry['segments'] if geometry else [],
                                corners=geometry['corners'] if geometry else [],
                                width=geometry['pipe_width_px'] if geometry else None))
            i+=1
            if i%150==0:print(i,n,flush=True)
            if args.limit and i>=args.limit:break
        encoder.stdin.close()
        if encoder.wait()!=0:raise RuntimeError('视频编码失败')
        out.with_suffix('.json').write_text(json.dumps(records),encoding='utf-8')
        print('frames',i,'full',sum(len(r['corners'])==4 for r in records),flush=True)
    c.release()


if __name__=='__main__':
    main()
