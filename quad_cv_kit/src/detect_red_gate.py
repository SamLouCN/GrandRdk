"""独立 OpenCV 视频实验；不改原任务工程。检测结果由每帧图像产生。"""
from pathlib import Path
import argparse
import json
import subprocess
import cv2
import numpy as np


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


def samples(mask, a, b, radius=5):
    length = np.linalg.norm(b-a)
    count = max(10, int(length))
    p = np.linspace(a, b, count)
    normal = np.array([-(b-a)[1], (b-a)[0]]) / max(length, 1)
    q = p[:, None, :] + np.arange(-radius, radius+1)[None, :, None]*normal
    x, y = np.rint(q[..., 0]).astype(int), np.rint(q[..., 1]).astype(int)
    valid = (x >= 0) & (x < mask.shape[1]) & (y >= 0) & (y < mask.shape[0])
    z = mask[np.clip(y, 0, mask.shape[0]-1), np.clip(x, 0, mask.shape[1]-1)] > 0
    z &= valid
    return p, z, normal


def line_from_segment(mask, ends):
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
    if good.mean() < .6:
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


def endpoints(line):
    return np.array([line['n']*line['b']+line['d']*line[k] for k in ('lo', 'hi')])


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


def get_lines(frame, mask):
    lsd = cv2.createLineSegmentDetector(cv2.LSD_REFINE_STD)
    segments = []
    for img in (mask, cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)):
        found = lsd.detect(img)[0]
        if found is not None:
            segments.extend(found.reshape(-1, 4))
    hough=cv2.HoughLinesP(mask,1,np.pi/720,35,minLineLength=55,maxLineGap=10)
    if hough is not None:
        segments.extend(hough.reshape(-1,4))
    lines = [q for s in segments if (q := line_from_segment(mask, s)) is not None]
    lines.sort(key=lambda q: (q['hi']-q['lo'])*q['support'], reverse=True)
    merged = []
    for line in lines:
        group = None
        for old in merged:
            if old['vertical'] != line['vertical'] or np.dot(old['d'], line['d']) < .988:
                continue
            mid = endpoints(line).mean(axis=0)
            distance = abs(mid@old['n']-old['b'])
            if distance < max(5, .7*max(old['width'], line['width'])):
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
            contrast,fraction=tube_contrast(frame,item)
            if contrast>.24 and fraction>.7:
                item['contrast']=contrast
                split.append(item)
    merged=sorted(split,key=lambda q:(q['hi']-q['lo'])*q['support'],reverse=True)
    # 长线优先；不让浮尘短线导致组合数失控。
    vertical = [q for q in merged if q['vertical']][:10]
    horizontal = [q for q in merged if not q['vertical']][:10]
    return vertical, horizontal


def pipe_groups(vertical,horizontal,frame):
    """管子端部组成连通图，缺边的近框和完整框使用同一个选择规则。"""
    lines=vertical+horizontal
    parent=list(range(len(lines)))
    links=[]
    white=elbow_mask(frame)
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
            ends=endpoints(l).copy()
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


def intersection(a, b):
    m = np.stack([a['n'], b['n']])
    if abs(np.linalg.det(m)) < .3:
        return None
    return np.linalg.solve(m, [a['b'], b['b']])


def detect(frame):
    small,_,_=prepare_detection_frame(frame)
    mask=red_mask(small)
    vertical,horizontal=get_lines(small,mask)
    candidates=pipe_groups(vertical,horizontal,small)
    return candidates, (vertical,horizontal), mask


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
    good=(status.reshape(-1)>0)&(error.reshape(-1)<35)
    if good.sum()<8:return None
    matrix,inliers=cv2.estimateAffinePartial2D(pts[good],moved[good],method=cv2.RANSAC,ransacReprojThreshold=2.5)
    if matrix is None or inliers.sum()<8:return None
    scale=float(np.hypot(matrix[0,0],matrix[1,0]))
    if not .9<scale<1.1:return None
    def warp(points):
        return np.asarray(points)@matrix[:,:2].T+matrix[:,2]
    segments=[];members=[]
    next_mask=red_mask(next_frame)
    for segment,line in zip(previous['segments'],previous['lines']):
        new=warp(segment)
        ok,a,b=cv2.clipLine((0,0,640,360),tuple(np.rint(new[0]).astype(int)),tuple(np.rint(new[1]).astype(int)))
        if not ok or np.linalg.norm(np.array(a)-b)<35:continue
        a,b=np.array(a,float),np.array(b,float)
        _,z,_=samples(next_mask,a,b,max(4,int(line['width']*.6)))
        if z.any(axis=1).mean()<.58:continue
        d=(b-a)/np.linalg.norm(b-a);n=np.array([-d[1],d[0]])
        item=dict(line,d=d,n=n,b=float(a@n),lo=float(a@d),hi=float(b@d),width=line['width']*scale)
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
                score=float(score),complete=len(corners)==4 and len(segments)==4,tracked=True)


def overlap(a,b):
    mids=np.array([s.mean(axis=0) for s in a['segments']])
    others=np.array([s.mean(axis=0) for s in b['segments']])
    return np.linalg.norm(mids[:,None,:]-others[None,:,:],axis=2).min()<25


def polygon_edges(candidate):
    """矩形管框每个管端只能接一根邻边；部分可见框仍然是同一条链。"""
    if candidate is None:return None
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
        previous=None;last_gray=None
        while True:
            ok,f=c.read()
            if not ok:break
            small,_,_=prepare_detection_frame(f)
            gray=cv2.cvtColor(small,cv2.COLOR_BGR2GRAY)
            moved=move_with_image(previous,last_gray,gray,None,small) if previous is not None else None
            if i%3==0:
                candidates,lines,mask=detect(f)
                if moved is not None:
                    matching=[q for q in candidates if overlap(q,moved)]
                    if matching and len(matching[0]['segments'])>=len(moved['segments']):
                        previous=matching[0]
                    elif not candidates or candidates[0]['score']<moved['score']*1.15:
                        previous=moved
                    else:previous=candidates[0]
                else:previous=candidates[0] if candidates else None
            else:previous=moved;lines=([],[])
            selected=polygon_edges(previous)
            candidates=[selected] if selected is not None else []
            annotated=draw(f,candidates,lines,i,fps)
            encoder.stdin.write(annotated.tobytes())
            records.append(dict(frame=i,segments=[s.tolist() for s in selected['segments']] if selected else [],
                                corners=[p.tolist() for p in selected['corners']] if selected else [],
                                width=selected['width'] if selected else None))
            last_gray=gray
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
