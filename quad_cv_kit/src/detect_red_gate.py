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
    from .cv_profile import CvFrameProfile
else:
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from src.gate_line_geometry import endpoints, intersection, samples
    from src import gate_models
    from src.cv_profile import CvFrameProfile


def red_mask(frame, backend=None, bounds=None):
    if bounds is not None:
        # The input is already black outside the ROI. Keep sigma=9's full
        # neighborhood, the pyramid grid and four morphology steps.
        h, w = frame.shape[:2]
        x0, y0, x1, y1 = map(int, bounds)
        x0, y0 = max(0, (x0-48)//4*4), max(0, (y0-48)//4*4)
        x1, y1 = min(w, (x1+51)//4*4), min(h, (y1+51)//4*4)
        result = np.zeros((h, w), np.uint8)
        if x1 > x0 and y1 > y0:
            crop = np.ascontiguousarray(frame[y0:y1, x0:x1])
            if backend is not None and hasattr(backend, 'register_roi'):
                backend.register_roi(crop, frame, (x0, y0, x1, y1))
            local = red_mask(crop, backend)
            result[y0:y1, x0:x1] = local
            if backend is not None and hasattr(backend, 'register_roi'):
                backend.register_roi(result, local, (0, 0, x1-x0, y1-y0), (x0, y0))
        return result
    if backend is not None:
        return backend.red_mask(frame)
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


def prepare_detection_frame(frame, backend=None):
    if backend is not None and hasattr(backend, 'prepare_canvas'):
        return backend.prepare_canvas(frame)
    height,width=frame.shape[:2]
    ratio=min(640/width,360/height)
    resized=cv2.resize(frame,(round(width*ratio),round(height*ratio)),interpolation=cv2.INTER_AREA)
    offset=np.array([(640-resized.shape[1])//2,(360-resized.shape[0])//2],float)
    canvas=np.zeros((360,640,3),np.uint8)
    x,y=offset.astype(int)
    canvas[y:y+resized.shape[0],x:x+resized.shape[1]]=resized
    return canvas,ratio,offset


def color_bounds(search_bbox, ratio, offset):
    if search_bbox is None:
        return None
    points = np.asarray(search_bbox, float).reshape(2, 2)*ratio+offset
    low = np.clip(np.floor(points[0]).astype(int), [0, 0], [640, 360])
    high = np.clip(np.ceil(points[1]).astype(int), [0, 0], [640, 360])
    return tuple(map(int, (*low, *high)))


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
    width=float(np.median(widths))
    if (t.max()-t.min())/max(1,width)<5:
        return None
    return dict(d=direction, n=normal, b=float(origin@normal), lo=float(t.min()), hi=float(t.max()),
                width=width, support=float(good.mean()), vertical=vertical)


def elbow_mask(frame):
    h,s,v=cv2.split(cv2.cvtColor(frame,cv2.COLOR_BGR2HSV))
    white=np.uint8((((h>18)&(h<102)&(s<110)&(v>70))|((s<18)&(v>130))))*255
    white=cv2.morphologyEx(white,cv2.MORPH_OPEN,np.ones((2,2),np.uint8))
    _,labels,stats,_=cv2.connectedComponentsWithStats(white)
    keep=(stats[:,4]>=18)&(stats[:,2]>=3)&(stats[:,3]>=3);keep[0]=False
    return np.uint8(keep[labels])*255


def contrast_signal(frame, profile=None, backend=None):
    if backend is not None:
        signal = backend.contrast_signal(frame)
    else:
        green = frame[:, :, 1].astype(np.float32)
        red = frame[:, :, 2].astype(np.float32)
        signal = np.log((red+10)/(green+10))
    if profile is not None:
        profile.mark('lines.contrast_full_image')
        profile.count('contrast_full_image_calls')
    return signal


def tube_contrast(frame,line,profile=None, *, signal=None, sample_step=1):
    # A search shares these two maps across all rods, never across frames.
    if signal is None:
        signal = contrast_signal(frame, profile)
    ends=endpoints(line)
    p=np.linspace(ends[0],ends[1],max(20,int((line['hi']-line['lo'])/sample_step)))
    width=line['width']
    offsets=np.array([-width*1.1-3,-width*.2,0,width*.2,width*1.1+3])
    x = np.rint(p[:, 0, None]+offsets*line['n'][0]).astype(int)
    y = np.rint(p[:, 1, None]+offsets*line['n'][1]).astype(int)
    valid=(x>=0)&(x<frame.shape[1])&(y>=0)&(y<frame.shape[0])
    z=signal[np.clip(y,0,frame.shape[0]-1),np.clip(x,0,frame.shape[1]-1)]
    delta=np.mean(z[:,1:4],axis=1)-(z[:,0]+z[:,4])/2
    delta=delta[valid.all(axis=1)]
    if profile is not None:
        profile.mark('lines.contrast_sample')
    if len(delta)<20:return 0,0
    return float(np.median(delta)),float(np.mean(delta>.035))


def eligible_segments(mask, segments):
    """Exact cheap rejections before allocating per-pixel cross sections."""
    segments = np.asarray(segments, dtype=float).reshape(-1, 4)
    delta = segments[:, 2:]-segments[:, :2]
    keep = (np.linalg.norm(delta, axis=1) >= 32) & (
        (np.abs(delta[:, 0]) < np.abs(delta[:, 1])*.65) |
        (np.abs(delta[:, 1]) < np.abs(delta[:, 0])*.7))
    segments = segments[keep]
    # Only reject rectangles with no red pixels at all. The 11px halo covers
    # every rounded +/-10px normal probe; faint and interrupted rods survive.
    low = np.floor(np.minimum(segments[:, :2], segments[:, 2:])-11).astype(int)
    high = np.ceil(np.maximum(segments[:, :2], segments[:, 2:])+11).astype(int)+1
    low = np.clip(low, [0, 0], [mask.shape[1], mask.shape[0]])
    high = np.clip(high, [0, 0], [mask.shape[1], mask.shape[0]])
    integral = cv2.integral(np.uint8(mask > 0))
    hits = (integral[high[:, 1], high[:, 0]] - integral[low[:, 1], high[:, 0]] -
            integral[high[:, 1], low[:, 0]] + integral[low[:, 1], low[:, 0]])
    return segments[hits > 0]


def get_lines(frame, mask, reference_frame=None, profile=None, extraction_bounds=None, backend=None,
              *, extraction_regions=None, lsd_mode='all', seed_cache=None):
    profile = profile or CvFrameProfile()
    fast = backend is not None and backend.quality == 'fast'
    sample_step = 2 if fast else 1
    lsd = cv2.createLineSegmentDetector(cv2.LSD_REFINE_STD)
    segments = []
    reference_frame = frame if reference_frame is None else reference_frame
    seed_cache = {} if seed_cache is None else seed_cache
    names = ['mask']
    if lsd_mode == 'all':
        if not fast:
            names.append('gray')
        names.append('chroma')
    x0, y0, x1, y1 = 0, 0, mask.shape[1], mask.shape[0]
    if extraction_bounds is not None:
        bx0, by0, bx1, by1 = extraction_bounds
        if bx1 <= bx0 or by1 <= by0:
            profile.mark('lines.prepare')
            return [], []
        # Retain the masked boundary's neighborhood for LSD gradients. CLAHE
        # remains on the full canvas so its tile scale and color values agree.
        # LSD's default 0.8 resize must keep the full-canvas sampling grid:
        # multiples of five give integer origins and dimensions at that scale.
        x0, y0 = max(0, (bx0-16)//5*5), max(0, (by0-16)//5*5)
        x1, y1 = min(mask.shape[1], (bx1+20)//5*5), min(mask.shape[0], (by1+20)//5*5)
    regions = [(x0, y0, x1, y1)] if extraction_regions is None else extraction_regions
    if profile.enabled:
        profile.meta['line_extraction_size'] = [int(x1-x0), int(y1-y0)]
        profile.meta['line_extraction_regions'] = [[int(value) for value in region] for region in regions]
    profile.mark('lines.prepare')
    hough_future = None
    if backend is not None and 'hough' not in seed_cache:
        hough_future = backend.submit_hough_regions(mask, regions)
        profile.meta['line_extract_parallel'] = True
    lsd_futures = {}
    parallel_lsd = backend is not None and hasattr(backend, 'submit_lsd_regions') and sum(name not in seed_cache for name in names) > 1
    try:
        for name in names:
            if name not in seed_cache:
                if name == 'mask':
                    img = mask
                elif name == 'gray':
                    img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                else:
                    chroma = cv2.cvtColor(reference_frame, cv2.COLOR_BGR2LAB)[:, :, 1]
                    img = cv2.createCLAHE(2., (8, 8)).apply(chroma)
                profile.mark('lines.prepare')
                if parallel_lsd:
                    lsd_futures[name] = backend.submit_lsd_regions(img, regions)
                    continue
                found_rows = []
                for rx0, ry0, rx1, ry1 in regions:
                    found = lsd.detect(np.ascontiguousarray(img[ry0:ry1, rx0:rx1]))[0]
                    profile.mark('lines.lsd_'+name)
                    profile.count('lsd_'+name+'_segments', 0 if found is None else len(found))
                    if found is not None:
                        found_rows.extend(found.reshape(-1, 4)+[rx0, ry0, rx0, ry0])
                seed_cache[name] = found_rows
            else:
                profile.count('line_seed_cache_reuses')
        for name, future in lsd_futures.items():
            rows, elapsed = future.result()
            seed_cache[name] = rows
            profile.mark('lines.lsd_'+name)
            profile.count('lsd_'+name+'_segments', len(rows))
            profile.meta.setdefault('lsd_worker_ms', {})[name] = round(elapsed, 3)
        for name in names:
            segments.extend(seed_cache[name])
    finally:
        # Wait for all submitted work on errors too. GPU fitting starts only
        # after both CPU channels and the GPU Hough worker have completed.
        for future in lsd_futures.values():
            try:
                future.result()
            except Exception:
                pass
        if hough_future is not None:
            seed_cache['hough'] = hough_future.result()
            profile.mark('lines.hough_join')
            profile.count('hough_segments', len(seed_cache['hough']))
    if 'hough' not in seed_cache:
        found_rows = []
        for rx0, ry0, rx1, ry1 in regions:
            hough_mask = np.ascontiguousarray(mask[ry0:ry1, rx0:rx1])
            hough = (cv2.HoughLinesP(hough_mask,1,np.pi/720,35,minLineLength=55,maxLineGap=10)
                     if backend is None else backend.hough_segments(hough_mask))
            profile.mark('lines.hough')
            profile.count('hough_segments', 0 if hough is None else len(hough))
            if hough is not None:
                found_rows.extend(hough.reshape(-1,4)+[rx0, ry0, rx0, ry0])
        seed_cache['hough'] = found_rows
    else:
        if hough_future is None:
            profile.count('line_seed_cache_reuses')
    segments.extend(seed_cache['hough'])
    raw_count = len(segments)
    segments = eligible_segments(mask, segments)
    profile.mark('lines.prefilter')
    profile.count('prefilter_segments', len(segments))
    lines = ([q for s in segments if (q := line_from_segment(mask, s, .48)) is not None]
             if backend is None else backend.fit_segments(mask, segments, .48))
    lines.sort(key=lambda q: (q['hi']-q['lo'])*q['support'], reverse=True)
    profile.mark('lines.fit_filter')
    profile.count('raw_segments', raw_count)
    profile.count('fitted_lines', len(lines))
    if backend is not None and hasattr(backend, 'merge_split_lines'):
        return backend.merge_split_lines(lines, mask, frame, reference_frame, profile)
    merged = []
    directions = []
    for line in lines:
        group = None
        line_ends = endpoints(line)
        ax, ay, bx, by = map(float, line_ends.reshape(-1))
        mx, my = (ax+bx)*.5, (ay+by)*.5
        dx, dy = map(float, line['d'])
        for old, (ox, oy, nx, ny) in zip(merged, directions):
            profile.count('merge_comparisons')
            if old['vertical'] != line['vertical'] or ox*dx+oy*dy < .988:
                continue
            distance = abs(mx*nx+my*ny-old['b'])
            pa, pb = ax*ox+ay*oy, bx*ox+by*oy
            projected_low, projected_high = min(pa, pb), max(pa, pb)
            gap = max(old['lo']-projected_high, projected_low-old['hi'], 0)
            if (distance < max(2.5, .45*min(old['width'], line['width']))
                    and gap < min(34, max(12, 2.5*old['width']))):
                group = old
                break
        if group is None:
            merged.append(line)
            directions.append((*map(float, line['d']), *map(float, line['n'])))
        else:
            group['lo'] = min(group['lo'], projected_low)
            group['hi'] = max(group['hi'], projected_high)
            group['width'] = (group['width']+line['width'])/2
    profile.mark('lines.merge')
    profile.count('merged_lines', len(merged))
    split=[]
    signals = None
    for line in merged:
        a,b=endpoints(line)
        p,z,_=samples(mask,a,b,max(2,int(line['width']*.4)), step=sample_step)
        occupancy=z.any(axis=1)
        joined=cv2.morphologyEx(np.uint8(occupancy)[None,:],cv2.MORPH_CLOSE,np.ones((1,3 if fast else 7),np.uint8))[0]>0
        ids=np.flatnonzero(joined)
        runs=np.split(ids,np.flatnonzero(np.diff(ids)>1)+1)
        for run in runs:
            if len(run)<(18 if fast else 35):continue
            lo,hi=p[run[[0,-1]]]@line['d']
            if (hi-lo)/max(line['width'],1)<5:continue
            item=dict(line,lo=float(lo),hi=float(hi),support=float(occupancy[run].mean()))
            profile.mark('lines.split')
            if signals is None:
                reference_signal = contrast_signal(reference_frame, profile, backend)
                enhanced_signal = (reference_signal if reference_frame is frame else contrast_signal(frame, profile, backend))
                signals = (reference_signal, enhanced_signal)
            contrast, fraction = max((tube_contrast(source, item, profile if profile.enabled else None, signal=signal, sample_step=sample_step)
                                     for source, signal in zip((reference_frame, frame), signals)),
                                     key=lambda measurement: measurement[0])
            profile.mark('lines.contrast_sample')
            if contrast>.035 and fraction>.48:
                item['contrast']=contrast
                split.append(item)
    merged=sorted(split,key=lambda q:(q['hi']-q['lo'])*q['support'],reverse=True)
    # 长线优先；不让浮尘短线导致组合数失控。
    limit = 8 if fast else 10
    vertical = [q for q in merged if q['vertical']][:limit]
    horizontal = [q for q in merged if not q['vertical']][:limit]
    profile.mark('lines.split')
    profile.count('supported_lines', len(merged))
    return vertical, horizontal


def pipe_groups(vertical,horizontal,frame,reference_frame=None,backend=None):
    """管子端部组成连通图，缺边的近框和完整框使用同一个选择规则。"""
    lines=vertical+horizontal
    ends_cache={id(line):endpoints(line) for line in lines}
    parent=list(range(len(lines)))
    links=[]
    white = None
    hsv_frame = None
    gpu_color, gpu_joints = (backend.partial_measurements(lines, len(vertical), frame)
        if lines and backend is not None and hasattr(backend, 'partial_measurements') else (None, None))
    color_indices = {id(line): index for index, line in enumerate(lines)}
    def root(i):
        while parent[i]!=i:
            parent[i]=parent[parent[i]];i=parent[i]
        return i
    for i,v in enumerate(vertical):
        for j,h in enumerate(horizontal,len(vertical)):
            if gpu_joints is not None:
                row = gpu_joints[i*len(horizontal)+j-len(vertical)]
                if row[2] == 0:continue
                point = row[:2].astype(float)
                needs_white = row[2] == 2
            else:
                point=intersection(v,h)
                if point is None:continue
                if abs(float(v['d']@h['d']))>.707:
                    continue
                ev,eh=ends_cache[id(v)],ends_cache[id(h)]
                dv=np.linalg.norm(ev-point,axis=1).min()
                dh=np.linalg.norm(eh-point,axis=1).min()
                gap=max(18,min(26,1.8*max(v['width'],h['width'])))
                needs_white = dv>gap or dh>gap
                if needs_white and (dv>55 or dh>55):continue
            x,y=np.rint(point).astype(int)
            # 白色弯头给出真正的管端；同一直线后方的管不能把这个端点吃掉。
            if needs_white:
                if white is None:
                    white=elbow_mask(frame if reference_frame is None else reference_frame)
                patch=white[max(0,y-7):min(white.shape[0],y+8),max(0,x-7):min(white.shape[1],x+8)]
                if np.count_nonzero(patch)<25:continue
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
                ends=ends_cache[id(l)]
                return np.all((ends[:,0]>10)&(ends[:,0]<630)&(ends[:,1]>10)&(ends[:,1]<350))
            if fully_visible(a) and fully_visible(b):
                ra=(a['hi']-a['lo'])/a['width'];rb=(b['hi']-b['lo'])/b['width']
                if max(ra,rb)/min(ra,rb)>2.2:
                    consistent=False;break
        if not consistent:continue
        # 蓝紫色池底纹理只有局部色差；框管仍应有至少一根显著偏红。
        chroma=[]
        for l in members:
            if gpu_color is not None:
                chroma.append(float(gpu_color[color_indices[id(l)], 0]))
                continue
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
            if gpu_color is not None:
                evidence = gpu_color[color_indices[id(l)]]
                if evidence[2] < .4 or evidence[1] < .15:continue
            else:
                if hsv_frame is None:
                    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                if not single_red_line(frame, l, hsv_frame):continue
        corners=[]
        for _,_,p in joints:
            if all(np.linalg.norm(p-q)>15 for q in corners):corners.append(p)
        segments=[]
        for i,l in zip(indices,members):
            l['observed_segment'] = ends_cache[id(l)].copy()
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


def single_red_line(frame, l, hsv_frame=None):
    """CPU reference for the standalone partial rod's 80-point red evidence."""
    a,b=endpoints(l);p=np.linspace(a,b,80)
    x,y=np.rint(p).astype(int).T
    valid=(x>=0)&(x<frame.shape[1])&(y>=0)&(y<frame.shape[0])
    if hsv_frame is None:
        hsv_frame=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    hsv=hsv_frame[np.clip(y,0,frame.shape[0]-1),np.clip(x,0,frame.shape[1]-1)]
    red=((hsv[:,0]<30)|(hsv[:,0]>143))&(hsv[:,1]>35)
    if np.mean(red[valid])<.4:return False
    pixels=frame[np.clip(y,0,frame.shape[0]-1),np.clip(x,0,frame.shape[1]-1)].astype(float)
    redness=np.median(np.log((pixels[valid,2]+10)/(pixels[valid,1]+10)))
    return bool(redness>=.15)


def significant_unexplained_evidence(mask):
    """Alarm for rod-sized evidence, INCLUDING fragmented weak rods.

    Counting only connected components can miss a supported rod made of short
    red patches. Prefer extra full searches over hiding that observation.
    """
    ys, xs = np.nonzero(mask)
    if len(xs) < 16:
        return False
    return bool(max(xs.max()-xs.min(), ys.max()-ys.min()) >= 25)


def prediction_bands(prediction, bounds, shape):
    """Side crops share the full canvas sampling grid and include motion halos."""
    band_mask = np.zeros(shape, np.uint8)
    regions = []
    for segment, line in zip(prediction['segments'], prediction['lines']):
        a, b = np.asarray(segment, float)
        d = (b-a)/max(1, np.linalg.norm(b-a))
        n = np.array([-d[1], d[0]])
        radius = max(12., line['width']*1.5)+2*prediction.get('motion_rms', 0)+min(12., .3*prediction.get('motion_px', 0))
        polygon = np.array([a-d*radius-n*radius, b+d*radius-n*radius,
                            b+d*radius+n*radius, a-d*radius+n*radius])
        cv2.fillConvexPoly(band_mask, np.rint(polygon).astype(np.int32), 255)
        low = np.floor(polygon.min(axis=0)-8).astype(int)//5*5
        high = (np.ceil(polygon.max(axis=0)+8).astype(int)+4)//5*5
        x0, y0 = np.maximum(low, [0, 0])
        x1, y1 = np.minimum(high, [shape[1], shape[0]])
        if x1 > x0 and y1 > y0:
            regions.append(tuple(map(int, (x0, y0, x1, y1))))
    area = max(0, bounds[2]-bounds[0])*max(0, bounds[3]-bounds[1])
    if len(regions) != 4 or sum((r[2]-r[0])*(r[3]-r[1]) for r in regions) >= area*.8:
        return [], band_mask
    return regions, band_mask


def certified_local_model(candidates, prediction, mask):
    """Accept only an isolated, dense-validated complete continuation."""
    if len(candidates) != 1:
        return False
    candidate = candidates[0]
    if not candidate.get('geometry_validated') or not candidate.get('complete') or len(candidate['lines']) != 4:
        return False
    if min(candidate['side_support']) < .90:
        return False
    old = np.array(prediction['corners'])
    new = np.array(candidate['corners'])
    if old.shape != (4, 2) or new.shape != (4, 2):
        return False
    distances = np.linalg.norm(new[:, None]-old[None, :], axis=2)
    if np.any(distances.min(axis=1) > 4) or len(set(distances.argmin(axis=1))) != 4:
        return False
    width_ratio = apparent_pipe_width(candidate)/max(1., apparent_pipe_width(prediction))
    if not .8 <= width_ratio <= 1.25:
        return False
    explained = np.zeros_like(mask)
    for segment, line in zip(candidate['segments'], candidate['lines']):
        a, b = np.rint(segment).astype(int)
        cv2.line(explained, tuple(a), tuple(b), 255, max(3, int(np.ceil(line['width']))+6))
    return not significant_unexplained_evidence(cv2.bitwise_and(mask, cv2.bitwise_not(explained)))


def strong_color_on_sides(candidate, score):
    """Weak rods always request chroma LSD, even if mask seeds form a quad."""
    strong = np.uint8(score > .30)
    for segment, line in zip(candidate['segments'], candidate['lines']):
        a, b = np.asarray(segment, float)
        margin = .1*(b-a)
        _, hits, _ = samples(strong, a+margin, b-margin, max(2, min(5, int(line['width']*.4))))
        if hits.any(axis=1).mean() < .8:
            return False
    return True


def refine_local_model(candidates, mask, score, bounds, valid_mask):
    """Restore original-resolution CPU fitting, trim and eight-bin validation.

    Only four surviving rods are refined. No approximate GPU trim output is
    accepted directly as the final local geometry.
    """
    if len(candidates) != 1 or not candidates[0].get('complete'):
        return candidates
    lines = [line_from_segment(mask, endpoints(line).reshape(-1), .48) for line in candidates[0]['lines']]
    if any(line is None for line in lines):
        return []
    lines = gate_models.trim_lines(lines, mask, score, bounds)
    return gate_models.complete_models([line for line in lines if line['vertical']],
                                      [line for line in lines if not line['vertical']], mask, bounds, valid_mask)


def anchored_to(candidate, anchor):
    for segment in candidate['segments']:
        points = np.linspace(segment[0], segment[1], 40)
        if np.count_nonzero(np.all((points >= anchor[0]-3) & (points <= anchor[1]+3), axis=1)) >= 2:
            return True
    return False


def detect(frame, search_bbox=None, anchor_bbox=None, reference_frame=None, valid_mask=None, profile=None,
           *, _prepared=None, _evidence=None, backend=None, _prediction=None,
           _regions=None, _lsd_mode='all', _seed_cache=None):
    if backend is not None and hasattr(backend, 'search_batch'):
        with backend.frame_batch(), backend.search_batch():
            return _detect(frame, search_bbox, anchor_bbox, reference_frame, valid_mask, profile,
                _prepared=_prepared, _evidence=_evidence, backend=backend, _prediction=_prediction,
                _regions=_regions, _lsd_mode=_lsd_mode, _seed_cache=_seed_cache)
    return _detect(frame, search_bbox, anchor_bbox, reference_frame, valid_mask, profile,
        _prepared=_prepared, _evidence=_evidence, backend=backend, _prediction=_prediction,
        _regions=_regions, _lsd_mode=_lsd_mode, _seed_cache=_seed_cache)


def _detect(frame, search_bbox=None, anchor_bbox=None, reference_frame=None, valid_mask=None, profile=None,
            *, _prepared=None, _evidence=None, backend=None, _prediction=None,
            _regions=None, _lsd_mode='all', _seed_cache=None):
    """Extract lines in the ROI, retaining full-frame scale and color context."""
    profile = profile or CvFrameProfile()
    if reference_frame is not None and reference_frame.shape != frame.shape:
        raise ValueError('Reference and enhanced frames must have identical dimensions')
    if _prepared is None:
        small,ratio,offset=prepare_detection_frame(frame, backend)
        reference = small if reference_frame is None else prepare_detection_frame(reference_frame, backend)[0]
    else:
        small, reference, ratio, offset = _prepared
    profile.mark('detect.prepare')
    if _evidence is None:
        mask, color_score = gate_models.combined_evidence(small, reference, profile=profile, prefix='detect.color',
                                                         backend=backend, bounds=color_bounds(search_bbox, ratio, offset))
    else:
        # Tracking and search see the same prepared frame. Copy before masking
        # the ROI, and keep this cache local to update (never reuse old colors).
        mask, color_score = _evidence
        mask = backend.copy_host(mask) if backend is not None and hasattr(backend, 'copy_host') else mask.copy()
        profile.mark('detect.color.reuse')
        profile.count('color_evidence_reuses')
    width, height = round(frame.shape[1]*ratio), round(frame.shape[0]*ratio)
    bounds = (int(offset[0]), int(offset[1]), int(offset[0])+width, int(offset[1])+height)
    small_valid = None
    if valid_mask is not None:
        if backend is not None and hasattr(backend, 'prepare_valid_canvas'):
            small_valid = backend.prepare_valid_canvas(valid_mask, frame.shape[1], frame.shape[0], ratio, offset)
            mask = backend.restrict_mask(mask, small_valid)
        else:
            small_valid = np.zeros_like(mask, bool)
            small_valid[bounds[1]:bounds[3], bounds[0]:bounds[2]] = cv2.resize(
                np.uint8(valid_mask), (width, height), interpolation=cv2.INTER_NEAREST) > 0
            mask[~small_valid] = 0
    if search_bbox is not None:
        points=np.asarray(search_bbox,float).reshape(2,2)*ratio+offset
        low=np.floor(points[0]).astype(int);high=np.ceil(points[1]).astype(int)
        low=np.clip(low,[0,0],[640,360]);high=np.clip(high,[0,0],[640,360])
        # Rectangular copies avoid boolean-indexing three-channel full images
        # (~11ms on S100), while retaining the same black ROI boundary for LSD.
        ys, xs = slice(low[1], high[1]), slice(low[0], high[0])
        roi_mask = np.zeros_like(mask)
        roi_mask[ys, xs] = mask[ys, xs]
        if backend is not None and hasattr(backend, 'register_roi'):
            backend.register_roi(roi_mask, mask, (*low, *high), tuple(low))
        mask = roi_mask
        roi_small = np.zeros_like(small)
        roi_small[ys, xs] = small[ys, xs]
        if backend is not None and hasattr(backend, 'register_roi'):
            backend.register_roi(roi_small, small, (*low, *high), tuple(low))
        if reference is small:
            reference = roi_small
        else:
            roi_reference = np.zeros_like(reference)
            roi_reference[ys, xs] = reference[ys, xs]
            if backend is not None and hasattr(backend, 'register_roi'):
                backend.register_roi(roi_reference, reference, (*low, *high), tuple(low))
            reference = roi_reference
        small = roi_small
        bounds = (max(bounds[0], low[0]), max(bounds[1], low[1]),
                  min(bounds[2], high[0]), min(bounds[3], high[1]))
    profile.mark('detect.roi')
    if profile.enabled:
        profile.meta['roi_fraction'] = round(max(0, bounds[2]-bounds[0])*max(0, bounds[3]-bounds[1])/(640*360), 4)
    if _prediction is not None:
        regions, band_mask = prediction_bands(_prediction, bounds, mask.shape)
        profile.mark('detect.predict_bands')
        outside = cv2.bitwise_and(mask, cv2.bitwise_not(band_mask))
        if regions and not significant_unexplained_evidence(outside):
            cache = {}
            # All model/trim checks below still use the FULL current ROI mask.
            # Only candidate extraction is restricted to predicted side regions.
            audit_mask = mask
            args = dict(_prepared=_prepared, _evidence=(mask, color_score), backend=backend,
                        _regions=regions, _seed_cache=cache)
            for lsd_mode in ('mask', 'all'):
                local = detect(frame, search_bbox, anchor_bbox, reference_frame, valid_mask, profile,
                               _lsd_mode=lsd_mode, **args)
                refined = refine_local_model(local[0], mask, color_score, bounds, small_valid)
                if anchor_bbox is not None:
                    anchor = np.asarray(anchor_bbox, float).reshape(2, 2)*ratio+offset
                    refined = [candidate for candidate in refined if anchored_to(candidate, anchor)]
                profile.mark('detect.band_dense_refine')
                if (certified_local_model(refined, _prediction, audit_mask) and
                        (lsd_mode == 'all' or strong_color_on_sides(refined[0], color_score))):
                    profile.meta.update(search_scope='bands', lsd_policy='mask-only' if lsd_mode == 'mask' else 'supplemented',
                                        band_extraction_pixels=sum((r[2]-r[0])*(r[3]-r[1]) for r in regions))
                    profile.count('band_search_successes')
                    profile.mark('detect.band_accept')
                    return refined, local[1], local[2]
                profile.mark('detect.band_revalidate')
            profile.meta['band_fallback_reason'] = 'local_model_not_certified'
        else:
            profile.meta['band_fallback_reason'] = 'outside_evidence' if regions else 'bands_not_smaller'
        profile.count('band_search_fallbacks')
        # Return to the SAME full ROI and original algorithms in this frame.
        profile.mark('detect.band_fallback')
    if _regions is None:
        profile.meta.update(search_scope='full', lsd_policy='all')
    vertical,horizontal=get_lines(small,mask,reference,profile, extraction_bounds=bounds, backend=backend,
                                 extraction_regions=_regions, lsd_mode=_lsd_mode, seed_cache=_seed_cache)
    recovered = gate_models.trim_lines(vertical+horizontal, mask, color_score, bounds, backend=backend)
    vertical = [line for line in recovered if line['vertical']][:10]
    horizontal = [line for line in recovered if not line['vertical']][:10]
    profile.mark('detect.trim_complete')
    profile.count('vertical_lines', len(vertical))
    profile.count('horizontal_lines', len(horizontal))
    complete = gate_models.complete_models(vertical, horizontal, mask, bounds, small_valid, profile=profile, backend=backend)
    profile.mark('detect.complete_models')
    profile.count('complete_candidates', len(complete))
    # The weak LAB evidence is safe only with four-side geometric validation.
    # Partial chains have no enclosing model to stop a warm white support leg.
    strict_mask = red_mask(reference, backend, bounds)
    partial_mask = (backend.mask_and(mask, strict_mask) if backend is not None and hasattr(backend, 'mask_and')
                    else cv2.bitwise_and(mask, strict_mask))
    profile.mark('detect.partial_mask')
    partial_lines = gate_models.trim_lines(vertical+horizontal, partial_mask, color_score, bounds, backend=backend)
    # Preserve the old partial detector's minimum rod length after re-trimming.
    # Tiny red patches on a white foot must not add an incompatible fourth rod.
    partial_lines = [line for line in partial_lines if line['hi']-line['lo'] >= 35
                     and (line['hi']-line['lo'])/max(1, line['width']) >= 5]
    profile.mark('detect.trim_partial')
    profile.count('partial_lines', len(partial_lines))
    partial = pipe_groups([line for line in partial_lines if line['vertical']],
                          [line for line in partial_lines if not line['vertical']], small, reference, backend)
    if backend is not None and hasattr(backend, 'partial_measurements'):
        profile.meta['partial_execution'] = 'gpu-batched-evidence'
    profile.mark('detect.partial_groups')
    profile.count('partial_candidates', len(partial))
    # A supported complete model wins over competing fragments of those same rods.
    partial = [candidate for candidate in partial if not any(
        sum(any(abs(line['d']@member['d']) > .99 and
                    abs(endpoints(line).mean(axis=0)@member['n']-member['b']) < max(3, member['width']*.55)
                    for member in model['lines']) for line in candidate['lines']) >= 2
        for model in complete)]
    profile.mark('detect.deduplicate')
    candidates = complete+partial
    if anchor_bbox is not None:
        anchor=np.asarray(anchor_bbox,float).reshape(2,2)*ratio+offset
        candidates=[c for c in candidates if anchored_to(c, anchor)]
    profile.mark('detect.anchor_filter')
    profile.count('final_candidates', len(candidates))
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

    def __init__(self, fps=30, detect_every=3, hold_seconds=.2, profile=False, backend=None, adaptive_search=True):
        if not np.isfinite(fps) or fps <= 0 or detect_every < 1 or hold_seconds < 0:
            raise ValueError('Invalid frame rate or tracking settings')
        self.detect_every = int(detect_every)
        self.hold_frames = int(round(fps * hold_seconds))
        self.previous = None
        self.last_gray = None
        self.last_observed = None
        self.index = 0
        self.last_status = {}
        self.profile_enabled = profile
        self.backend = backend
        self.adaptive_search = adaptive_search
        self.full_search_interval = max(self.detect_every, int(round(fps*.5)))
        self.last_full_search = None

    def reset(self):
        """Discard old target motion when switching to a different YOLO door."""
        self.previous = None
        self.last_gray = None
        self.last_observed = None
        self.last_full_search = None

    def update(self, frame, search_bbox=None, anchor_bbox=None, allow_detect=True,
               prefer_previous_width=False, reference_frame=None, valid_mask=None):
        if self.backend is not None and hasattr(self.backend, 'frame_batch'):
            with self.backend.frame_batch():
                return self._update(frame, search_bbox, anchor_bbox, allow_detect,
                                    prefer_previous_width, reference_frame, valid_mask)
        return self._update(frame, search_bbox, anchor_bbox, allow_detect,
                            prefer_previous_width, reference_frame, valid_mask)

    def _update(self, frame, search_bbox=None, anchor_bbox=None, allow_detect=True,
                prefer_previous_width=False, reference_frame=None, valid_mask=None):
        profile = CvFrameProfile(self.profile_enabled)
        small, ratio, offset = prepare_detection_frame(frame, self.backend)
        reference = prepare_detection_frame(reference_frame, self.backend)[0] if reference_frame is not None else small
        if reference_frame is not None and reference_frame.shape != frame.shape:
            raise ValueError('Reference frame must match detection frame dimensions')
        gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
        profile.mark('tracker.prepare')
        moved = None
        evidence = None
        age = self.index - self.last_observed if self.last_observed is not None else 0
        if self.previous is not None and age <= self.hold_frames:
            def prepare_evidence(warped=None):
                nonlocal evidence
                bounds = color_bounds(search_bbox, ratio, offset) if warped is not None else None
                if bounds is not None and warped is not None:
                    # Include EVERY pixel queried by tracking, even outside
                    # YOLO's ROI. Build this after LK, using actual motion.
                    radius = max(16, max(int(line['width']*1.1)+4 for line in self.previous['lines']))
                    points = np.clip(np.concatenate(warped), [0, 0], [639, 359])
                    low = np.floor(points.min(axis=0)-radius).astype(int)
                    high = np.ceil(points.max(axis=0)+radius+1).astype(int)
                    bounds = (min(bounds[0], int(low[0])), min(bounds[1], int(low[1])),
                              max(bounds[2], int(high[0])), max(bounds[3], int(high[1])))
                mask, score = gate_models.combined_evidence(small, reference, profile=profile, prefix='track.color',
                                                           backend=self.backend, bounds=bounds)
                if valid_mask is not None:
                    if self.backend is not None and hasattr(self.backend, 'prepare_valid_canvas'):
                        valid_small = self.backend.prepare_valid_canvas(valid_mask, frame.shape[1], frame.shape[0], ratio, offset)
                        mask = self.backend.restrict_mask(mask, valid_small)
                    else:
                        valid_small = np.zeros_like(mask)
                        h,w = frame.shape[:2]
                        x,y = offset.astype(int)
                        resized = cv2.resize(np.uint8(valid_mask), (round(w*ratio),round(h*ratio)), interpolation=cv2.INTER_NEAREST)
                        valid_small[y:y+resized.shape[0],x:x+resized.shape[1]] = resized
                        mask[valid_small == 0] = 0
                evidence = (mask, score)
                profile.mark('track.valid_mask')
                return mask
            if self.backend is not None:
                moved = move_with_image(self.previous, self.last_gray, gray, None, small,
                                        profile=profile if profile.enabled else None,
                                        evidence_provider=prepare_evidence, backend=self.backend)
            else:
                current_mask = prepare_evidence()
                if profile.enabled:
                    moved = move_with_image(self.previous, self.last_gray, gray, current_mask, small, profile=profile)
                else:
                    moved = move_with_image(self.previous, self.last_gray, gray, current_mask, small)
        profile.mark('track.finish')
        fresh = None
        lines = ([], [])
        candidate_count = None
        search_reason = 'disabled'
        if allow_detect and (self.index % self.detect_every == 0 or moved is None):
            search_reason = ('no_previous' if self.previous is None else 'hold_expired' if age > self.hold_frames else
                             'tracking_failed' if moved is None else 'scheduled')
            profile_kwargs = {'profile': profile} if profile.enabled else {}
            profile_kwargs.update(_prepared=(small, reference, ratio, offset), _evidence=evidence)
            if self.backend is not None:
                profile_kwargs['backend'] = self.backend
            full_due = self.last_full_search is None or self.index-self.last_full_search >= self.full_search_interval
            reliable = (moved is not None and moved.get('complete') and moved.get('geometry_validated') and
                        moved.get('motion_inlier_ratio', 0) >= .8 and moved.get('motion_rms', float('inf')) <= 1.5 and
                        moved.get('motion_px', float('inf')) <= 20)
            if self.adaptive_search and reliable and not full_due:
                profile_kwargs['_prediction'] = moved
            else:
                profile.meta['full_search_reason'] = ('periodic' if full_due else 'prediction_unreliable')
            if search_bbox is None:
                candidates, lines, _ = detect(frame, reference_frame=reference_frame, valid_mask=valid_mask, **profile_kwargs)
            else:
                candidates, lines, _ = detect(frame, search_bbox, anchor_bbox, reference_frame, valid_mask, **profile_kwargs)
            if profile.meta.get('search_scope') != 'bands':
                self.last_full_search = self.index
            candidate_count = len(candidates)
            fresh = select_nearest(candidates, moved)
            if (prefer_previous_width and fresh is not None and moved is not None
                    and apparent_pipe_width(fresh) < apparent_pipe_width(moved)*.7):
                # A thin background gate must not replace a still-supported near pipe.
                fresh = None
        elif allow_detect:
            search_reason = 'interval_tracking'
        selected = fresh if fresh is not None else polygon_edges(moved)
        if fresh is not None:
            selected = dict(selected, tracked=False)
            self.last_observed = self.index
        elif selected is not None:
            selected = dict(selected, tracked=True)
        if selected is not None:
            selected['apparent_width'] = apparent_pipe_width(selected)
        profile.mark('tracker.select')
        self.last_status = dict(frame=self.index, candidate_count=candidate_count,
                                detection_ran=candidate_count is not None,
                                observation=('tracked' if selected.get('tracked') else 'detected')
                                if selected else 'missing',
                                age_frames=self.index-self.last_observed
                                if selected and self.last_observed is not None else None,
                                ratio=ratio, offset=offset.tolist())
        if profile.enabled:
            if self.backend is not None:
                # Include deferred event collection in CV latency; moving it
                # out of the timer would falsely inflate the reported gain.
                self.backend.runtime.finish()
                profile.mark('tracker.gpu_finalize')
            profile.meta.update(mode='search' if candidate_count is not None else 'track' if moved is not None else 'idle',
                                search_reason=search_reason, input_size=[frame.shape[1], frame.shape[0]],
                                canvas_size=[640, 360], opencv_threads=cv2.getNumThreads())
            profile.meta['cv_backend'] = 'cpu' if self.backend is None else self.backend.runtime.info['selected']
            profile.meta['cv_quality'] = 'precise' if self.backend is None else self.backend.quality
            self.last_status['cv_profile'] = profile.finish()
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


def move_with_image(previous,gray,next_gray,mask,next_frame,profile=None, *, evidence_provider=None, backend=None):
    """跨相邻帧用当前图像运动预测杆线，定期搜索重新测量。"""
    if previous is None:return None
    band=np.zeros_like(gray)
    for segment in previous['segments']:
        a,b=np.rint(segment).astype(int)
        cv2.line(band,tuple(a),tuple(b),255,16)
    for q in previous['corners']:
        cv2.circle(band,tuple(np.rint(q).astype(int)),12,255,-1)
    pts=cv2.goodFeaturesToTrack(gray,100,.015,5,mask=band,blockSize=5)
    if profile is not None:
        profile.mark('track.features')
        profile.count('track_features', 0 if pts is None else len(pts))
    if pts is None or len(pts)<8:return None
    moved,status,error=cv2.calcOpticalFlowPyrLK(gray,next_gray,pts,None,winSize=(25,25),maxLevel=3)
    if profile is not None:
        profile.mark('track.optical_flow')
    if moved is None or status is None or error is None:return None
    good=(status.reshape(-1)>0)&(error.reshape(-1)<35)
    if profile is not None:
        profile.count('track_good_features', good.sum())
    if good.sum()<8:return None
    matrix,inliers=cv2.estimateAffinePartial2D(pts[good],moved[good],method=cv2.RANSAC,ransacReprojThreshold=2.5)
    if profile is not None:
        profile.mark('track.ransac')
    if matrix is None or inliers is None or inliers.sum()<8:return None
    scale=float(np.hypot(matrix[0,0],matrix[1,0]))
    if not .9<scale<1.1:return None
    def warp(points):
        return np.asarray(points)@matrix[:,:2].T+matrix[:,2]
    original_points = pts[good].reshape(-1, 2)
    moved_points = moved[good].reshape(-1, 2)
    inlier_mask = inliers.reshape(-1).astype(bool)
    motion_rms = float(np.sqrt(np.mean(np.sum((warp(original_points)[inlier_mask]-moved_points[inlier_mask])**2, axis=1))))
    motion_px = float(np.median(np.linalg.norm(moved_points-original_points, axis=1)))
    segments=[];members=[]
    next_mask = (evidence_provider([warp(segment) for segment in previous['segments']])
                 if evidence_provider is not None else mask if mask is not None else red_mask(next_frame))
    supports=[]
    sample_widths=[]
    batched = backend is not None and hasattr(backend, 'track_support')
    for segment,line in zip(previous['segments'],previous['lines']):
        new=warp(segment)
        ok,a,b=cv2.clipLine((0,0,640,360),tuple(np.rint(new[0]).astype(int)),tuple(np.rint(new[1]).astype(int)))
        if not ok or np.linalg.norm(np.array(a)-b)<35:continue
        a,b=np.array(a,float),np.array(b,float)
        if not batched:
            _,z,_=samples(next_mask,a,b,max(4,int(line['width']*.6)))
            if z.any(axis=1).mean()<.58:continue
        d=(b-a)/np.linalg.norm(b-a);n=np.array([-d[1],d[0]])
        item=dict(line,d=d,n=n,b=float(a@n),lo=float(a@d),hi=float(b@d),width=line['width']*scale,
                  observed_segment=warp(line.get('observed_segment', segment)))
        extent=item['observed_segment']@d
        item.update(strong_lo=float(extent.min()),strong_hi=float(extent.max()))
        if previous.get('geometry_validated') and not batched:
            support=gate_models.side_evidence(next_mask,item,np.array([a,b]))
            if support is None:continue
            supports.append(support)
        segments.append(np.array([a,b]));members.append(item)
        sample_widths.append(line['width'])
    if batched and segments:
        # Sampling radius uses the previous width, before the motion scale.
        occupancy, side_support = backend.track_support(next_mask, members, segments,
            bool(previous.get('geometry_validated')), sample_widths=sample_widths)
        good = occupancy >= .58
        if previous.get('geometry_validated'):
            # Full side validation uses the scaled width, as the CPU path does.
            good &= side_support >= 0
            supports = [float(value) for value, keep in zip(side_support, good) if keep]
        segments = [segment for segment, keep in zip(segments, good) if keep]
        members = [member for member, keep in zip(members, good) if keep]
    if not segments:
        if profile is not None:
            profile.mark('track.revalidate')
        return None
    corners=[]
    for q in previous['corners']:
        q=warp(np.array([q]))[0]
        if not (8<q[0]<632 and 8<q[1]<352):continue
        if sum(np.linalg.norm(segment-q,axis=1).min()<18 for segment in segments)>=2:
            corners.append(q)
    score=max(np.linalg.norm(b-a)*l['width']**1.5 for (a,b),l in zip(segments,members))
    score*=1+.04*(len(members)-1)
    if profile is not None:
        profile.mark('track.revalidate')
    return dict(previous,segments=segments,lines=members,corners=corners,width=float(np.median([l['width'] for l in members])),
                score=float(score),complete=len(corners)==4 and len(segments)==4,tracked=True,
                side_support=supports if previous.get('geometry_validated') else None,
                motion_inlier_ratio=float(inlier_mask.mean()), motion_rms=motion_rms, motion_px=motion_px)


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
