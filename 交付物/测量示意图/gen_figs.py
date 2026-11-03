# -*- coding: utf-8 -*-
"""生成麦轮小车参数测量示意图（SVG → ImageMagick 转 PNG）。
约束：ImageMagick 内置 MSVG 渲染器，只支持 rect/circle/ellipse/line/polygon/text/g+transform，
文本必须显式 font-family。"""
import math, os

INK='#1f2937'; MUT='#6b7280'; BLUE='#2563eb'; BFILL='#dbeafe'
RED='#dc2626';  RFILL='#fee2e2'; GREEN='#16a34a'; GFILL='#dcfce7'
AMBER='#d97706';AFILL='#fef3c7'; SLATE='#334155'; LFILL='#f1f5f9'
FONT='Noto Sans SC'
OUT=os.path.dirname(os.path.abspath(__file__))

def esc(s): return s.replace('&','&amp;').replace('<','&lt;').replace('>','&gt;')

def T(x,y,s,size=15,fill=INK,a='start',bold=False):
    w=' font-weight="bold"' if bold else ''
    return f'<text x="{x}" y="{y}" font-size="{size}" fill="{fill}" text-anchor="{a}"{w} font-family="{FONT}">{esc(s)}</text>'

def L(x1,y1,x2,y2,stroke=INK,w=1.5,dash=None):
    d=f' stroke-dasharray="{dash}"' if dash else ''
    return f'<line x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}" stroke="{stroke}" stroke-width="{w}"{d}/>'

def RC(x,y,wd,ht,fill='none',stroke=INK,sw=1.5,rx=0):
    r=f' rx="{rx}"' if rx else ''
    return f'<rect x="{x:.1f}" y="{y:.1f}" width="{wd:.1f}" height="{ht:.1f}" fill="{fill}" stroke="{stroke}" stroke-width="{sw}"{r}/>'

def C(cx,cy,r,fill='none',stroke=INK,sw=1.5):
    return f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="{r:.1f}" fill="{fill}" stroke="{stroke}" stroke-width="{sw}"/>'

def EL(cx,cy,rx,ry,fill='none',stroke=INK,sw=1.5,rot=None):
    t=f' transform="rotate({rot} {cx:.1f} {cy:.1f})"' if rot is not None else ''
    return f'<ellipse cx="{cx:.1f}" cy="{cy:.1f}" rx="{rx:.1f}" ry="{ry:.1f}" fill="{fill}" stroke="{stroke}" stroke-width="{sw}"{t}/>'

def PG(pts,fill='none',stroke=None,sw=1.2):
    p=' '.join(f'{a:.1f},{b:.1f}' for a,b in pts)
    s=f' stroke="{stroke}" stroke-width="{sw}"' if stroke else ''
    return f'<polygon points="{p}" fill="{fill}"{s}/>'

def G(tx,ty,rot,inner):
    tr=f'translate({tx:.1f},{ty:.1f})'+(f' rotate({rot})' if rot else '')
    return f'<g transform="{tr}">{inner}</g>'

def arrow(x1,y1,x2,y2,color=INK,w=2,hl=9,hw=4):
    ang=math.atan2(y2-y1,x2-x1); bx,by=x2-hl*math.cos(ang),y2-hl*math.sin(ang)
    p2=(bx+hw*math.sin(ang),by-hw*math.cos(ang)); p3=(bx-hw*math.sin(ang),by+hw*math.cos(ang))
    return L(x1,y1,bx,by,color,w)+PG([(x2,y2),p2,p3],fill=color)

def dim_h(x1,x2,y,label,color=BLUE,size=13,dy=-7):
    return arrow(x1,y,x2,y,color,1.6)+arrow(x2,y,x1,y,color,1.6)+T((x1+x2)/2,y+dy,label,size,color,a='middle')

def dim_v(y1,y2,x,label,color=BLUE,size=13,dx=7,anchor='start'):
    return arrow(x,y1,x,y2,color,1.6)+arrow(x,y2,x,y1,color,1.6)+T(x+dx,(y1+y2)/2+4,label,size,color,a=anchor)

def ext(x1,y1,x2,y2): return L(x1,y1,x2,y2,'#9ca3af',1,dash='4 3')

def header(title,ref,w,h):
    return T(24,34,title,20,INK,bold=True)+T(w-24,34,ref,13,MUT,a='end')+L(24,46,w-24,46,'#e5e7eb',1.5)

def note(w,y,lines,size=14,fill='#f8fafc',lh=22,pad=12):
    h=len(lines)*lh+2*pad
    b=[RC(24,y,w-48,h,fill=fill,stroke='#e5e7eb',sw=1.2,rx=8)]
    for i,s in enumerate(lines):
        b.append(T(24+pad,y+pad+16+i*lh,s,size,INK))
    return ''.join(b),h

def svg(w,h,body):
    return (f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}" viewBox="0 0 {w} {h}">'
            f'<rect width="{w}" height="{h}" fill="#ffffff"/>{body}</svg>')

def wheel_tv(wx,wy,t=24,D=80,rdir=1,body='#ffffff'):
    """俯视麦轮：长D沿Y（前后），宽t沿X（左右）。rdir=1 辊子呈 ╲，-1 呈 ╱"""
    p=[RC(wx-t/2,wy-D/2,t,D,fill=body,stroke=SLATE,sw=2,rx=5)]
    for k in range(-3,4):
        yy=wy+k*15
        if abs(yy-wy)>D/2-11: continue
        x1,x2=wx-(t/2-3),wx+(t/2-3)
        if rdir>0: p.append(L(x1,yy-8,x2,yy+8,SLATE,1.3))
        else:      p.append(L(x1,yy+8,x2,yy-8,SLATE,1.3))
        k+=1
    p.append(C(wx,wy,3,SLATE,SLATE,1))
    return ''.join(p)

def wheel_side_face(cx,cy,R=28):
    """侧视：轮=圆+轮毂"""
    return C(cx,cy,R,'#f8fafc',SLATE,2)+C(cx,cy,R*0.35,'#e5e7eb',SLATE,1.5)

def ground(x1,x2,y):
    return L(x1,y,x2,y,SLATE,3)+ ''.join(L(x,y+7,x-8,y, '#9ca3af',1.2) for x in range(int(x1)+14,int(x2)-6,34))

# 每张图 ------------------------------------------------------------
FIGS={}

def f01():  # 坐标系与轮序
    w,h=800,500
    b=[header('A组 · 坐标系与轮序（俯视）','对应清单条目 #5 – #9',w,h)]
    b.append(RC(210,120,260,260,fill='#f8fafc',stroke=SLATE,sw=2,rx=8))
    for (wx,wy,rd) in [(225,170,1),(455,170,-1),(225,330,-1),(455,330,1)]:
        b.append(wheel_tv(wx,wy,rdir=rd))
    # 坐标轴（原点=四轮接地矩形中心）
    ox,oy=340,250
    b.append(arrow(ox,oy,ox,84,BLUE,2.5));  b.append(T(ox+10,100,'X 正方向（前进）',15,BLUE,bold=True))
    b.append(arrow(ox,oy,164,oy,BLUE,2.5)); b.append(T(180,242,'Y 正方向（向左）',15,BLUE,bold=True))
    b.append(C(ox,oy,4,BLUE,BLUE,1))
    b.append(C(ox,oy,40,'none',BLUE,1.4))
    b.append(arrow(350,210,330,210,BLUE,1.8))
    b.append(T(492,246,'ω 正方向（俯视逆时针为正）',13,BLUE,bold=True))
    # 轮序
    for (cx,cy,s,anc) in [(180,166,'1 = 左前','end'),(500,166,'2 = 右前','start'),
                          (180,326,'3 = 左后','end'),(500,326,'4 = 右后','start')]:
        b.append(C(cx,cy,9,RED,RED,1))
        tx = cx-14 if anc=='end' else cx+14
        b.append(T(tx,cy+5,s,14,RED,a=anc,bold=True))
    n,_=note(w,414,['建议：轮序 1=左前、2=右前、3=左后、4=右后，定好后立刻在车身贴标签并拍照，之后全程不许更改。',
                    '车体原点建议取「四个轮轴心连线矩形」的中心；X 沿前进方向，Y 向左，俯视逆时针为 ω 正方向（右手系）。'])
    b.append(n)
    FIGS['fig01_俯视_坐标系与轮序']=svg(w,h,''.join(b))

def f02():  # 整车外形尺寸
    w,h=800,560
    b=[header('A组 · 整车外形尺寸（左：俯视 / 右：侧视）','对应清单条目 #1 – #4',800,h)]
    # 俯视（左）
    b.append(T(60,80,'俯视：量总宽 W',15,INK,bold=True))
    b.append(RC(110,120,180,200,fill='#f8fafc',stroke=SLATE,sw=2,rx=8))
    for (wx,wy,rd) in [(95,165,1),(305,165,-1),(95,275,-1),(305,275,1)]:
        b.append(wheel_tv(wx,wy,t=20,D=70,rdir=rd))
    b.append(ext(85,330,85,352)+ext(315,330,315,352))
    b.append(dim_h(85,315,344,'W 总宽（含轮外缘，量最凸点）'))
    b.append(ext(110,326,110,388)+ext(290,326,290,388))
    b.append(dim_h(110,290,380,'车体本体宽（不含轮）',MUT))
    b.append(T(60,416,'总宽 = 本体宽 + 左悬出量 + 右悬出量',13,MUT))
    # 侧视（右）
    b.append(T(430,80,'侧视：量总长 L / 总高 H / 离地间隙',15,INK,bold=True))
    b.append(ground(410,780,430))
    b.append(wheel_side_face(470,402)); b.append(wheel_side_face(720,402))
    b.append(RC(450,360,290,16,fill=LFILL,stroke=SLATE,sw=2))
    b.append(RC(560,322,100,38,fill=AFILL,stroke=AMBER,sw=2)); b.append(T(610,346,'电池(最高件)',12,INK,a='middle'))
    b.append(ext(442,430,442,462)+ext(748,430,748,462))
    b.append(dim_h(442,748,454,'L 总长（含轮外缘）'))
    b.append(ext(672,322,766,322)+ext(742,430,766,430))
    b.append(dim_v(322,430,752,'H 总高（量到最高点）',dx=6,anchor='start'))
    b.append(ext(430,376,442,376))
    b.append(dim_v(376,430,436,'h_g 离地间隙',dx=4,anchor='end'))
    n,_=note(800,490,['总长：沿车体最长方向，两端取最凸出点（含轮子、保险杠等）；总高：水平尺压在最高点上量到地面。',
                      '离地间隙 h_g：量底盘最低点到地面，直尺或塞尺，精度 ±1 mm。'])
    b.append(n)
    FIGS['fig02_外形尺寸_俯视与侧视']=svg(800,h,''.join(b))

def f03():  # 轴距轮距与轮心偏移
    w,h=800,535
    b=[header('B组 · 轴距、轮距与轮心偏移（俯视）','对应清单条目 #14 – #18',w,h)]
    b.append(RC(210,120,240,240,fill='#f8fafc',stroke=SLATE,sw=2,rx=8))
    for (wx,wy,rd) in [(225,170,1),(455,170,-1),(225,330,-1),(455,330,1)]:
        b.append(wheel_tv(wx,wy,rdir=rd))
    for (wx,wy) in [(225,170),(455,170),(225,330),(455,330)]:
        b.append(C(wx,wy,5,'none',RED,2.5)+L(wx-9,wy,wx+9,wy,RED,1.6)+L(wx,wy-9,wx,wy+9,RED,1.6))
    b.append(T(150,110,'红×=轮轴心',13,RED))
    # 轴距（纵向）
    b.append(ext(225,170,150,170)+ext(225,330,150,330))
    b.append(dim_v(170,330,158,'轴距 L_b',dx=-8,anchor='end'))
    # 轮距（横向）
    b.append(ext(225,330,225,415)+ext(455,330,455,415))
    b.append(dim_h(225,455,408,'轮距 W_b（量轴心↔轴心，不加轮厚）'))
    # 几何中心 vs 四轮中心
    b.append(G(310,222,0,PG([(0,-9),(9,0),(0,9),(-9,0)],fill=BLUE)))
    b.append(T(310,205,'车体几何中心',13,BLUE,a='middle'))
    b.append(arrow(318,228,334,244,RED,2))
    b.append(T(330,108,'偏移已放大示意',12,MUT,a='end'))
    b.append(C(340,250,4,'none',RED,2.5)); b.append(T(352,268,'四轮轴心矩形中心',13,RED))
    n,_=note(w,436,['W_b = 外缘距 − t = 内缘距 + t（t 为轮宽）；前后轮距不等时分别量、分别填。',
                    'dx = 四轮轴心X均值 − 车体几何中心X；dy 同理取Y向。两个偏移直接进运动学模型，别默认为 0。',
                    '轴距 L_b：同一侧前轮轴心到后轮轴心的纵向距离，左右各量一次取平均，不加轮径。'])
    b.append(n)
    FIGS['fig03_俯视_轴距轮距与轮心偏移']=svg(w,h,''.join(b))

def f04():  # 悬出量（重点图）
    w,h=800,625
    b=[header('B组 · 轮子悬出量：到底量哪里（重点）','对应清单条目 #19',w,h)]
    # 上：俯视全图
    b.append(T(30,76,'① 俯视：先找到过轮心的横向断面 A–A',15,INK,bold=True))
    b.append(RC(120,96,240,180,fill='#f8fafc',stroke=SLATE,sw=2,rx=8))
    for (wx,wy,rd) in [(140,140,1),(370,140,-1),(140,236,-1),(370,236,1)]:
        b.append(wheel_tv(wx,wy,t=24,D=70,rdir=rd))
    b.append(L(370,88,370,286,RED,1.6,dash='6 4'))
    b.append(T(378,100,'断面 A–A（过右轮轴心）',13,RED))
    # 下：A–A 断面（从车头方向看）
    b.append(T(30,316,'② 在断面 A–A 上量（从车前方看过去的截面）',15,INK,bold=True))
    b.append(RC(80,346,300,70,fill=LFILL,stroke=SLATE,sw=2))
    b.append(T(90,338,'车体（该断面处的侧边缘）',12,MUT))
    b.append(RC(412,330,26,100,fill='#ffffff',stroke=SLATE,sw=2,rx=4))
    b.append(T(425,452,'轮子（宽=轮厚 t）',12,SLATE,a='middle'))
    b.append(ext(380,346,380,478)+ext(412,430,412,478))
    b.append(dim_h(380,412,470,'悬出量 s',GREEN,dy=-7))
    # 错误示范
    b.append(RC(455,356,115,50,fill='none',stroke=RED,sw=1.5,rx=4))
    b.append(T(512,348,'车体其他更宽的部位',12,RED,a='middle'))
    b.append(L(455,352,500,336,RED,1.4,dash='5 4'))
    b.append(T(512,330,'× 不要量到这里',13,RED,a='middle',bold=True))
    n,_=note(w,502,['答案：量「过轮心的断面 A–A 处」的车体边缘 → 轮子最外点，不是量到全车最宽处！只有当车体是规则矩形、',
                    '轮子恰好在最宽断面上时，两种量法才恰好相等。尺子必须垂直于车体侧边（横向量），不要斜量。',
                    '轮子缩进车体内时记负值。校核式：车体总宽 = 车体本体宽 + 左悬出量 + 右悬出量。'])
    b.append(n)
    FIGS['fig04_俯视与断面_轮子悬出量']=svg(w,h,''.join(b))

def f05():  # 共面与载荷
    w,h=800,540
    b=[header('B/D组 · 四轮共面检查 与 静态载荷称重','对应清单条目 #20 – #22、#56 – #58',800,h)]
    # 左：共面
    b.append(T(40,80,'① 四轮接地点是否共面（侧视，示意放大）',15,INK,bold=True))
    b.append(L(40,360,360,360,SLATE,3)); b.append(T(46,382,'平板（玻璃/大理石台面）',12,MUT))
    b.append(G(210,298,-4,RC(-125,-38,250,40,fill=LFILL,stroke=SLATE,sw=2,rx=6)))
    b.append(C(120,324,26,'#f8fafc',SLATE,2))
    b.append(C(300,334,26,'#f8fafc',SLATE,2))
    b.append(PG([(120,352),(120,360),(111,360)],fill=RED))
    b.append(T(148,380,'红楔 = 塞尺',12,RED))
    b.append(arrow(196,150,128,290,RED,1.8)); b.append(T(202,146,'一角垫起后量',13,RED))
    b.append(T(40,412,'四角轮流塞入，间隙 ≤1 mm 合格；对角垫起量另一角变化 = 扭转刚度。',13,MUT))
    # 右：称重
    b.append(T(430,80,'② 四轮静态载荷（俯视）',15,INK,bold=True))
    b.append(RC(470,110,240,240,fill='#f8fafc',stroke=SLATE,sw=2,rx=8))
    for (wx,wy,rd,tag) in [(485,160,1,'N1'),(695,160,-1,'N2'),(485,300,-1,'N3'),(695,300,1,'N4')]:
        b.append(wheel_tv(wx,wy,rdir=rd))
        b.append(RC(wx-24,wy+34,48,26,fill='#e5e7eb',stroke=MUT,sw=1.2,rx=3))
        b.append(T(wx,wy+51,tag,13,RED,a='middle',bold=True))
    b.append(T(590,250,'每个轮子下垫电子秤',13,MUT,a='middle'))
    n,_=note(800,436,['车上放常态载重、摆正常使用姿态再称。分配比 = N_i / (N1+N2+N3+N4)；不平衡度 = (最大−最小)/平均 ≤ 5%。',
                      '载荷比就是质心位置的「实测验证」：后轮偏重 = 质心靠后，与 D 组计算结果互相校验。'])
    b.append(n)
    FIGS['fig05_共面检查与四轮载荷']=svg(800,h,''.join(b))

def f06():  # 板层
    w,h=800,400
    b=[header('B组 · 底盘板层与层间距（侧视）','对应清单条目 #11 – #13',w,h)]
    b.append(RC(140,150,420,16,fill=LFILL,stroke=SLATE,sw=2))
    b.append(RC(140,250,420,16,fill=LFILL,stroke=SLATE,sw=2))
    for x in (180,330,480):
        b.append(RC(x,166,18,84,fill='#e5e7eb',stroke=MUT,sw=1.2))
    b.append(T(330,320,'支撑铜柱 / 螺柱',12,MUT,a='middle'))
    b.append(ext(566,150,600,150)+ext(566,166,600,166))
    b.append(dim_v(150,166,586,'t_plate 板厚（卡尺量板边）',dx=6))
    b.append(ext(566,166,600,166)+ext(566,250,600,250))
    b.append(dim_v(166,250,586,'d_layer 层间距',dx=6))
    b.append(T(70,180,'第1层',12,MUT,a='end')); b.append(T(70,262,'第2层',12,MUT,a='end'))
    n,_=note(w,340,['记录板层数 n；有几层就量几层的厚度和间距。层间距决定电池、电路板能塞进哪个高度，影响质心高度计算。'])
    b.append(n)
    FIGS['fig06_侧视_底盘板层']=svg(w,h,''.join(b))

def f07():  # 安装精度
    w,h=800,500
    b=[header('B/C组 · 电机安装精度检查','对应清单条目 #23、#38 – #40',800,h)]
    # 左：垂直度
    b.append(T(40,80,'① 轮轴 ⊥ 底盘？（后视）',15,INK,bold=True))
    b.append(RC(70,170,240,14,fill=LFILL,stroke=SLATE,sw=2))
    b.append(L(240,184,240,330,SLATE,2,dash='6 4')); b.append(T(250,338,'轮轴延长线',12,SLATE))
    b.append(RC(224,184,32,90,fill=BFILL,stroke=SLATE,sw=2)); b.append(T(286,240,'电机',13,SLATE))
    b.append(RC(210,274,60,44,fill='#ffffff',stroke=SLATE,sw=2,rx=6)); b.append(T(240,344,'轮',12,SLATE,a='middle'))
    b.append(L(120,170,120,330,'#9ca3af',3))
    b.append(T(60,152,'角尺',12,MUT)); b.append(T(194,166,'底盘板',12,MUT))
    b.append(arrow(150,225,205,265,RED,1.8))
    b.append(T(60,252,'θ ≤ 1°',14,RED,bold=True))
    b.append(T(40,392,'把角尺一边贴底盘面、一边贴轮轴，看缝隙；左右四个轮子都要查。',13,MUT))
    # 右：跳动
    b.append(T(430,80,'② 轮子跳动（端面视图）',15,INK,bold=True))
    b.append(C(590,240,80,'#f8fafc',SLATE,2)); b.append(C(590,240,28,'#e5e7eb',SLATE,1.5))
    b.append(L(590,240,590,120,MUT,1.2,dash='4 3')); b.append(T(590,110,'径向跳动 ≤ 0.5 mm',13,BLUE,a='middle'))
    b.append(RC(690,196,26,44,fill='#ffffff',stroke=SLATE,sw=1.5)); b.append(L(690,218,666,218,SLATE,2))
    b.append(T(726,214,'千分表',12,SLATE)); b.append(T(726,236,'(可用目测)',11,MUT))
    b.append(T(430,392,'手转轮子一整圈看表针摆动（径向/端面都要测）。',13,MUT))
    b.append(T(430,410,'同时确认：轮子与轴固定可靠、不打滑（顶丝/D轴/夹紧）。',13,MUT))
    n,_=note(800,420,['③ 电机安装座与轮心的同轴/偏斜偏差：目视 + 直尺，±0.5 mm。任何一项超标，先修机械再谈 PID。'])
    b.append(n)
    FIGS['fig07_安装精度_垂直度与跳动']=svg(800,h,''.join(b))

def f08():  # 轮径与轮宽
    w,h=800,455
    b=[header('C组 · 轮子外径 D 与轮宽 t','对应清单条目 #25、#27、#28',800,h)]
    # 左：正视
    b.append(T(40,80,'正视：量外径 D',15,INK,bold=True))
    cx,cy,R=220,260,95
    for k in range(12):
        a=k*30
        px,py=cx+R*math.cos(math.radians(a)),cy+R*math.sin(math.radians(a))
        b.append(G(px,py,a,EL(0,0,26,9,fill=AFILL,stroke=AMBER,sw=1.5)))
    b.append(C(cx,cy,R,'none',SLATE,2)); b.append(C(cx,cy,30,'#e5e7eb',SLATE,1.5))
    b.append(ext(cx-R,cy,cx-R-10,cy)+ext(cx+R,cy,cx+R+10,cy))
    b.append(dim_h(cx-R,cx+R,cy-40,'D 外径（含辊子外廓）',dy=-8))
    b.append(T(40,410,'每个轮子转 60° 量 3 处取平均；四轮 ΔD = 最大 − 最小，越小越好。',13,MUT))
    # 右：侧视
    b.append(T(480,80,'侧视：量轮宽 t',15,INK,bold=True))
    b.append(RC(580,150,64,200,fill='#ffffff',stroke=SLATE,sw=2,rx=8))
    b.append(EL(612,146,28,10,fill=AFILL,stroke=AMBER,sw=1.5)); b.append(EL(612,354,28,10,fill=AFILL,stroke=AMBER,sw=1.5))
    b.append(ext(580,360,580,392)+ext(644,360,644,392))
    b.append(dim_h(580,644,384,'t 轮宽（两侧面间距）'))
    b.append(ext(652,150,690,150)+ext(652,350,690,350))
    b.append(dim_v(150,350,676,'D',dx=4))
    n,_=note(800,414,['用游标卡尺，精度 ±0.2 mm。注意：D 是含辊子的外廓直径，不是轮毂直径；t 是轮子两侧面之间的距离。'])
    b.append(n)
    FIGS['fig08_轮径与轮宽']=svg(800,h,''.join(b))

def f09():  # 辊子参数
    w,h=800,600
    b=[header('C组 · 辊子几何参数','对应清单条目 #29 – #35',800,h)]
    cx,cy,R=230,290,125
    for k in range(10):
        a=k*36
        px,py=cx+R*math.cos(math.radians(a)),cy+R*math.sin(math.radians(a))
        b.append(G(px,py,a,EL(0,0,26,10,fill=AFILL,stroke=AMBER,sw=1.5)))
    b.append(C(cx,cy,R,'none',SLATE,2)); b.append(C(cx,cy,34,'#e5e7eb',SLATE,1.5))
    a1,a2=18,54
    p1=(cx+(R+18)*math.cos(math.radians(a1)),cy+(R+18)*math.sin(math.radians(a1)))
    p2=(cx+(R+18)*math.cos(math.radians(a2)),cy+(R+18)*math.sin(math.radians(a2)))
    b.append(L(cx,cy,p1[0],p1[1],MUT,1.2)+L(cx,cy,p2[0],p2[1],MUT,1.2))
    b.append(T(352,398,'p_r 节距（相邻辊子中心，',12,BLUE)+T(352,414,'沿外缘弧长，软尺量）',12,BLUE))
    b.append(T(60,470,'辊子数 n_r：目视清点，四轮对比',13,INK))
    b.append(T(60,492,'磨损：四轮对比拍照记录',13,INK))
    # 右：45°剖面
    b.append(T(500,80,'45° 剖面：β 与 e',15,INK,bold=True))
    b.append(L(600,130,600,430,SLATE,3)); b.append(T(600,120,'轮子中分面（轮平面）',13,SLATE,a='middle'))
    b.append(L(520,410,770,410,SLATE,3)); b.append(T(524,428,'地面',12,MUT))
    b.append(G(600,300,-45,RC(-9,6,18,150,fill=AFILL,stroke=AMBER,sw=2)))
    b.append(T(742,382,'辊子',13,AMBER))
    # 角度弧
    pts=[(600+52*math.sin(math.radians(t)),300+52*math.cos(math.radians(t))) for t in range(0,46,6)]
    for i in range(len(pts)-1): b.append(L(pts[i][0],pts[i][1],pts[i+1][0],pts[i+1][1],RED,1.6))
    b.append(T(630,362,'β = 45°',14,RED,bold=True))
    b.append(ext(600,444,600,478)+ext(700,414,700,478))
    b.append(dim_h(600,700,470,'偏移量 e',GREEN))
    n,_=note(800,502,['β：辊子轴线与轮平面的夹角，标准 45°，用照片 + 量角器量（±1°）。',
                      'e：辊子轴线到轮平面的偏移，决定有效滚动半径 R_eff ≠ D/2，是正运动学误差的大头，务必量（或查图纸）。',
                      '辊面材质（PU / 橡胶 / 塑料）记录下来，影响摩擦与滑移。'])
    b.append(n)
    FIGS['fig09_辊子几何参数']=svg(800,h,''.join(b))

def f10():  # X型 O型
    w,h=800,470
    b=[header('C组 · 麦轮布局判定：X 型 / O 型（俯视）','对应清单条目 #10、#36、#37',800,h)]
    def mini(ox,oy,pat,title):
        s=[RC(ox,oy,220,190,fill='#f8fafc',stroke=SLATE,sw=2,rx=8)]
        pos=[(ox+45,oy+55),(ox+175,oy+55),(ox+45,oy+135),(ox+175,oy+135)]
        for (wx,wy),rd in zip(pos,pat): s.append(wheel_tv(wx,wy,t=20,D=64,rdir=rd))
        s.append(T(ox+110,oy+228,title,17,INK,a='middle',bold=True))
        return ''.join(s)
    b.append(mini(60,80,[1,-1,-1,1],'X 型：延长线交叉成 X'))     # FL╲ FR╱ RL╱ RR╲
    b.append(mini(440,80,[-1,1,1,-1],'O 型：延长线围成 O'))     # FL╱ FR╲ RL╲ RR╱
    b.append(arrow(320,170,390,170,MUT,2)); b.append(T(355,158,'? 判定',12,MUT,a='middle'))
    n,_=note(800,368,['俯视逐个轮子看辊子斜向：两两平行、交叉成 X → X 型；首尾相接围成 O → O 型。',
                      '布局型式直接决定逆运动学方程里每一项的符号，填错全盘皆错。',
                      '同一时刻俯拍四个轮子，比较辊子初始相位是否一致（±2°）；相差过多的轮子做标记。拿不准就把俯视照片发我。'])
    b.append(n)
    FIGS['fig10_俯视_麦轮X型O型判定']=svg(800,h,''.join(b))

def f11():  # 质心计算
    w,h=800,520
    b=[header('D组 · 质量分布与质心计算（俯视）','对应清单条目 #41 – #54、#59 – #61',800,h)]
    b.append(RC(150,110,300,300,fill='#f8fafc',stroke=SLATE,sw=2,rx=8))
    for (wx,wy,rd) in [(165,160,1),(435,160,-1),(165,360,-1),(435,360,1)]:
        b.append(wheel_tv(wx,wy,t=20,D=64,rdir=rd))
        b.append(T(wx,wy-40,'m_m',11,SLATE,a='middle'))
    b.append(RC(230,150,140,50,fill=AFILL,stroke=AMBER,sw=2)); b.append(T(300,180,'电池 m_b',14,INK,a='middle'))
    b.append(RC(240,300,120,44,fill=GFILL,stroke=GREEN,sw=2)); b.append(T(300,327,'主控+驱动 m_c',13,INK,a='middle'))
    b.append(arrow(300,250,300,120,BLUE,1.6)); b.append(T(306,132,'X',13,BLUE,bold=True))
    b.append(arrow(300,250,190,250,BLUE,1.6)); b.append(T(198,242,'Y',13,BLUE,bold=True))
    b.append(C(300,250,4,BLUE,BLUE,1)); b.append(T(308,268,'原点(0,0)',12,BLUE))
    # 公式面板
    b.append(RC(520,110,250,220,fill='#f8fafc',stroke='#e5e7eb',sw=1.2,rx=8))
    b.append(T(536,140,'逐件称重 + 量坐标：',14,INK,bold=True))
    for i,s in enumerate(['x_c = Σ(m_i·x_i) / Σ m_i','y_c = Σ(m_i·y_i) / Σ m_i','z_c = Σ(m_i·z_i) / Σ m_i',
                          'I_zz = Σ m_i·(x_i²+y_i²)','I_xx = Σ m_i·(y_i²+z_i²)','I_yy = Σ m_i·(x_i²+z_i²)']):
        b.append(T(536,168+i*26,s,14,BLUE))
    n,_=note(800,436,['每个部件都要：称质量（电子秤 ±2 g）+ 量几何中心三维坐标（相对原点，±2 mm）。配重块、螺丝包也别漏。',
                      '算完用四轮静态载荷比交叉验证（图5）；悬吊法实测校验质心高度（图12）。'])
    b.append(n)
    FIGS['fig11_俯视_质量分布与质心计算']=svg(800,h,''.join(b))

def f12():  # 悬吊法
    w,h=800,545
    b=[header('D组 · 质心高度：双线悬吊法（侧视）','对应清单条目 #55',800,h)]
    b.append(RC(120,64,560,14,fill=LFILL,stroke=SLATE,sw=2)); b.append(T(120,54,'横梁',12,MUT))
    # 吊点A
    b.append(L(240,78,240,220,SLATE,2)); b.append(C(240,222,5,RED,RED,1))
    b.append(G(240,226,-9,RC(-150,-8,230,64,fill='#f8fafc',stroke=SLATE,sw=2,rx=6)))
    b.append(G(122,296,-9,C(0,0,22,'#ffffff',SLATE,2)))
    b.append(G(58,300,-9,C(0,0,22,'#ffffff',SLATE,2)))
    b.append(L(240,78,240,420,BLUE,1.4,dash='6 4'))
    b.append(T(250,410,'垂线1（沿绳延长画在车侧面）',13,BLUE))
    # 吊点B
    b.append(L(560,78,560,220,SLATE,2)); b.append(C(560,222,5,RED,RED,1))
    b.append(G(560,226,9,RC(-80,-8,230,64,fill='#f8fafc',stroke=SLATE,sw=2,rx=6)))
    b.append(G(678,296,9,C(0,0,22,'#ffffff',SLATE,2)))
    b.append(G(742,300,9,C(0,0,22,'#ffffff',SLATE,2)))
    b.append(L(560,78,560,420,GREEN,1.4,dash='6 4'))
    b.append(T(548,410,'垂线2',13,GREEN,a='end'))
    n,_=note(800,436,['步骤：① 从吊点A吊起小车让其自然下垂，把绳的延长线画/拍在车侧面上；② 换吊点B重复；',
                      '③ 两条垂线的交点就是质心在竖直平面内的投影（★）；④ 换车的另一侧再做一次得到另一平面投影，即可得 z_c。',
                      '拍照后按照片比例换算（画面里放一把钢尺作比例尺），精度 ±3 mm。'])
    b.append(n)
    FIGS['fig12_侧视_质心高度悬吊法']=svg(800,h,''.join(b))

def f13():  # 铭牌与死区
    w,h=800,555
    b=[header('E组 · 电机铭牌 / PWM 死区与线性度','对应清单条目 #62 – #74',800,h)]
    # 左：电机
    b.append(T(40,80,'电机与铭牌',15,INK,bold=True))
    b.append(RC(70,180,150,100,fill=BFILL,stroke=SLATE,sw=2,rx=20))
    b.append(RC(220,214,60,32,fill='#e5e7eb',stroke=SLATE,sw=1.5)); b.append(T(250,206,'减速箱',12,SLATE,a='middle'))
    b.append(RC(280,222,56,16,fill=SLATE,stroke=SLATE,sw=1)); b.append(T(308,260,'输出轴(量直径)',12,SLATE,a='middle'))
    b.append(RC(95,205,100,50,fill='#ffffff',stroke=RED,sw=1.6,rx=4))
    b.append(T(145,226,'铭牌：型号 / 电压',13,RED,a='middle',bold=True))
    b.append(T(145,244,'减速比 i（如1:48）',13,RED,a='middle',bold=True))
    b.append(T(70,320,'拍铭牌照片；空载转速查手册或悬空全速实测；堵转转矩等查手册。',13,MUT))
    # 右：曲线
    ox,oy=470,410
    b.append(T(500,80,'PWM 占空比 → 转速（四轮分别测）',15,INK,bold=True))
    b.append(arrow(ox,oy,ox,140,INK,2)); b.append(T(ox,130,'转速 rpm',13,INK,a='middle'))
    b.append(arrow(ox,oy,770,oy,INK,2)); b.append(T(770,452,'PWM 占空比 %',13,INK,a='end'))
    for p in (20,40,60,80,100):
        x=ox+p*2.6
        b.append(L(x,oy,x,oy-6,MUT,1.2)); b.append(T(x,oy+20,f'{p}%',12,MUT,a='middle'))
    b.append(RC(ox,oy-5,62,5,fill=RFILL,stroke=RED,sw=1.2))
    b.append(T(ox+31,oy+38,'死区 DZ',13,RED,a='middle',bold=True))
    pts=[(0,0),(20,0),(40,60),(60,120),(80,180),(100,240)]
    for i in range(len(pts)-1):
        x1=ox+pts[i][0]*2.6; y1=oy-pts[i][1]; x2=ox+pts[i+1][0]*2.6; y2=oy-pts[i+1][1]
        b.append(L(x1,y1,x2,y2,BLUE,2.5))
    for p,v in pts[2:]:
        b.append(C(ox+p*2.6,oy-v,4,BLUE,BLUE,1))
    b.append(T(600,250,'同一占空比下四轮转速差 Δn',13,BLUE))
    n,_=note(800,470,['死区：占空比从 0 缓慢上调，记下四个轮子各自开始转动的最小值（±1%），这是「各轮死区补偿」的来源。',
                      '在 20/40/60/80/100% 五个点读四轮转速：既得线性度，又得四轮一致性 Δn，还顺便完成 k_i 标定的数据采集。'])
    b.append(n)
    FIGS['fig13_电机铭牌与PWM死区']=svg(800,h,''.join(b))

def f14():  # 编码器标定
    w,h=800,575
    b=[header('F组 · 编码器：单圈计数 与 每米计数标定','对应清单条目 #75 – #86',800,h)]
    # 左：编码器位置
    b.append(T(40,80,'① 编码器装在哪根轴？',15,INK,bold=True))
    b.append(C(78,250,24,'#ffffff',BLUE,2)); b.append(T(78,296,'编码器',12,BLUE,a='middle'))
    b.append(RC(102,210,120,80,fill=BFILL,stroke=SLATE,sw=2,rx=16)); b.append(T(162,196,'电机转子',12,SLATE,a='middle'))
    b.append(RC(222,220,70,60,fill='#e5e7eb',stroke=SLATE,sw=1.5)); b.append(T(257,300,'减速箱',12,SLATE,a='middle'))
    b.append(RC(292,242,50,16,fill=SLATE,stroke=SLATE,sw=1)); b.append(T(317,278,'输出轴→轮',12,SLATE,a='middle'))
    b.append(T(40,330,'PPR（单相线数，常见11/13）查手册；计数倍频 1x/2x/4x 按你代码实际写。',13,MUT))
    # 右上：单圈
    b.append(T(430,80,'② 手转 1 整圈 → C_1rev',15,INK,bold=True))
    b.append(C(520,170,45,'#f8fafc',SLATE,2)); b.append(C(520,170,14,'#e5e7eb',SLATE,1.5))
    pts=[(520+62*math.sin(math.radians(t)),170-62*math.cos(math.radians(t))) for t in range(0,301,10)]
    for i in range(len(pts)-1): b.append(L(pts[i][0],pts[i][1],pts[i+1][0],pts[i+1][1],RED,1.8))
    b.append(PG([(pts[-1][0],pts[-1][1]),(pts[-1][0]-8,pts[-1][1]-8),(pts[-1][0]+9,pts[-1][1]-6)],fill=RED))
    b.append(RC(620,140,150,60,fill=LFILL,stroke=SLATE,sw=1.5,rx=6))
    b.append(T(695,165,'计数器读数差',13,INK,a='middle')); b.append(T(695,187,'= C_1rev',15,BLUE,a='middle',bold=True))
    # 右下：1m 推车
    b.append(T(430,300,'③ 推车 1.00 m → C_m',15,INK,bold=True))
    b.append(ground(430,770,420))
    b.append(L(470,395,470,445,RED,2.5)); b.append(T(470,462,'起点',12,RED,a='middle'))
    b.append(L(740,395,740,445,RED,2.5)); b.append(T(740,462,'终点',12,RED,a='middle'))
    b.append(ext(470,395,470,352)+ext(740,395,740,352))
    b.append(dim_h(470,740,344,'地面画准 1.00 m'))
    b.append(RC(560,368,90,26,fill='#f8fafc',stroke=SLATE,sw=2,rx=5))
    b.append(C(575,407,13,'#ffffff',SLATE,2)); b.append(C(635,407,13,'#ffffff',SLATE,2))
    b.append(arrow(600,330,660,330,BLUE,2)); b.append(T(630,320,'匀速直线推过',12,BLUE,a='middle'))
    n,_=note(800,492,['C_m = ΔN / 1.00 m（counts/m）。四个轮子分别推一次，互差应 ≤1%；同时记录正转时计数是 + 还是 −。',
                      '由 C_m 反推有效滚动半径 R_eff 与 D_eff——这是整车里程精度的根。编码器供电、丢脉冲检查见 H 组。'])
    b.append(n)
    FIGS['fig14_编码器标定']=svg(800,h,''.join(b))

def f15():  # IMU
    w,h=800,590
    b=[header('G组 · IMU（GY-25T）安装与零偏','对应清单条目 #87 – #100',800,h)]
    # 左：安装
    b.append(T(40,80,'① 安装位置与朝向（俯视）',15,INK,bold=True))
    b.append(RC(70,110,280,280,fill='#f8fafc',stroke=SLATE,sw=2,rx=8))
    for (wx,wy,rd) in [(85,160,1),(335,160,-1),(85,340,-1),(335,340,1)]:
        b.append(wheel_tv(wx,wy,t=18,D=56,rdir=rd))
    ox,oy=210,250
    b.append(arrow(ox,oy,ox,140,BLUE,2)); b.append(T(ox+6,152,'车体 X',13,BLUE,bold=True))
    b.append(RC(280,190,60,40,fill=GFILL,stroke=GREEN,sw=2))
    b.append(arrow(310,212,324,168,GREEN,2))
    b.append(T(300,176,'IMU 的X轴',13,GREEN,bold=True,a='end'))
    b.append(T(258,206,'θ ≤ 1°',13,RED,bold=True))
    b.append(T(310,258,'量安装坐标 (x,y)',12,MUT))
    b.append(T(40,416,'手动把车转 90°，检查输出符号与车体坐标系是否一致（±90°/±180° 对得上）。',13,MUT))
    # 右：零偏
    b.append(T(430,80,'② 静态零偏：静置 60 s',15,INK,bold=True))
    gx,gy=470,420
    b.append(arrow(gx,gy,gx,180,INK,2)); b.append(T(gx,170,'角度 °',13,INK,a='middle'))
    b.append(arrow(gx,gy,770,gy,INK,2)); b.append(T(770,464,'时间 s',13,INK,a='end'))
    for t in (0,20,40,60):
        x=gx+t*4.6; b.append(L(x,gy,x,gy-6,MUT,1.2)); b.append(T(x,gy+20,f'{t}',12,MUT,a='middle'))
    prev=None
    for t in range(0,61,5):
        x=gx+t*4.6; y=gy-(90+ t*0.35 + (2.2 if (t//5)%2 else -2.2))
        if prev: b.append(L(prev[0],prev[1],x,y,BLUE,2))
        prev=(x,y)
    b.append(L(gx,gy-90,770,gy-90,MUT,1,dash='4 3'))
    b.append(T(700,gy-76,'初始零点',12,MUT,a='middle'))
    b.append(arrow(620,gy-90,620,gy-111,RED,1.8))
    b.append(T(630,gy-106,'60s 漂移量',13,RED,bold=True))
    n,_=note(800,486,['零偏测试：上电静置 1~2 s 后开始记录，静止 60 s 内角度变化即漂移；角速度输出平均值即零偏（±0.5°/s）。',
                      'GY-25T 无磁力计，航向靠陀螺积分、随时间漂移——长时间任务要有归零/重标定策略。',
                      '接口（TTL 串口或 I2C）、波特率、输出帧格式、数据率：拍丝印 + 查手册一并记录。'])
    b.append(n)
    FIGS['fig15_IMU安装与零偏']=svg(800,h,''.join(b))

def f16():  # 供电
    w,h=800,430
    b=[header('H组 · 供电与电压跌落测量','对应清单条目 #101 – #110',800,h)]
    b.append(RC(90,220,130,80,fill=AFILL,stroke=AMBER,sw=2)); b.append(T(155,266,'电池',15,INK,a='middle',bold=True))
    b.append(L(220,260,280,260,SLATE,2)); b.append(C(280,260,4,SLATE,SLATE,1)); b.append(C(330,260,4,SLATE,SLATE,1))
    b.append(L(280,260,330,260,SLATE,2)); b.append(T(305,244,'开关',12,SLATE,a='middle'))
    b.append(RC(330,220,130,80,fill=LFILL,stroke=SLATE,sw=2)); b.append(T(395,266,'驱动板',15,INK,a='middle'))
    b.append(L(460,260,520,260,SLATE,2))
    b.append(C(550,260,30,'#f8fafc',SLATE,2)); b.append(T(550,266,'M',15,SLATE,a='middle',bold=True))
    # 万用表
    b.append(C(155,130,36,'#ffffff',SLATE,2)); b.append(T(155,136,'V',17,BLUE,a='middle',bold=True))
    b.append(L(155,166,155,220,RED,1.6)); b.append(L(191,130,240,130,RED,1.6)); b.append(L(240,130,240,196,RED,1.6)); b.append(L(240,196,214,218,RED,1.6))
    b.append(T(70,190,'红表笔接电池两端',12,RED))
    # 读数
    b.append(RC(610,140,160,140,fill='#f8fafc',stroke='#e5e7eb',sw=1.2,rx=8))
    b.append(T(626,170,'空载   U_0 = __.__ V',14,INK))
    b.append(T(626,200,'全转   U_1 = __.__ V',14,INK))
    b.append(T(626,230,'ΔU = U_0 − U_1',14,BLUE,bold=True))
    b.append(T(626,258,'(带载电压跌落)',12,MUT))
    n,_=note(800,320,['分别在「满电」和「亏电」时给同一占空比测轮速 → 速度差 %（±3%）。电压一掉，同样的 PWM 转速完全不同。',
                      '记录：电池串数/容量、截止电压；电机与主控是否独立供电；电机运转时编码器有无丢脉冲（长时间对比计数）；',
                      '抖动线束看是否复位或丢数（接触可靠性）。'])
    b.append(n)
    FIGS['fig16_供电与电压跌落']=svg(800,h,''.join(b))

def f17():  # 场地布置
    w,h=800,560
    b=[header('J组 · 运动性能测试场地布置（示意，不按比例）','对应清单条目 #121 – #131',800,h)]
    # 直行
    b.append(T(40,80,'① 直行 2 m',15,INK,bold=True))
    b.append(L(60,220,340,220,MUT,2,dash='8 6')); b.append(L(60,196,60,244,RED,2.5)); b.append(T(60,262,'起点',12,RED,a='middle'))
    b.append(L(340,196,340,244,RED,2.5)); b.append(T(340,262,'终点线',12,RED,a='middle'))
    b.append(RC(60,140,50,22,fill='#f8fafc',stroke=SLATE,sw=2,rx=4)); b.append(arrow(120,151,170,151,BLUE,2))
    b.append(RC(292,196,50,22,fill='#f8fafc',stroke=SLATE,sw=2,rx=4))
    b.append(arrow(352,204,392,190,RED,1.6)); b.append(T(398,186,'横向偏差 ≤20mm',12,RED))
    b.append(L(340,220,410,214,BLUE,1.4,dash='5 4'))
    b.append(T(398,186,'航向偏差 ≤2°',12,BLUE))
    b.append(T(60,300,'地面贴长直标线（美纹纸），IMU 读数 + 卷尺量终点偏离。',12,MUT))
    # 旋转
    b.append(T(460,80,'② 原地旋转 360°',15,INK,bold=True))
    b.append(C(580,210,70,'none',MUT,1.5))
    b.append(RC(560,190,40,20,fill='#f8fafc',stroke=SLATE,sw=2,rx=4))
    b.append(arrow(580,210,580,120,RED,2)); b.append(T(588,128,'起始朝向',12,RED))
    b.append(arrow(580,210,650,160,BLUE,2)); b.append(T(654,152,'实际终朝向',12,BLUE))
    b.append(T(580,312,'应转 360°，实测差 ≤5°（地面做标记）',12,MUT,a='middle'))
    b.append(T(460,344,'反推有效轮距 W_eff / L_eff',13,INK,bold=True))
    # 横移
    b.append(T(40,392,'③ 横移 1 m',15,INK,bold=True))
    b.append(L(60,470,360,470,MUT,2)); b.append(L(60,500,360,500,MUT,2))
    b.append(RC(70,474,44,22,fill='#f8fafc',stroke=SLATE,sw=2,rx=4))
    b.append(arrow(130,485,190,485,BLUE,2)); b.append(T(150,461,'横移',12,BLUE,a='middle'))
    b.append(dim_v(470,500,386,'1 m',dx=6))
    b.append(T(40,534,'终点偏离 ≤30 mm。',12,MUT))
    b.append(T(460,392,'④ 极限：不打滑、不丢步的最大 v / a',13,INK,bold=True))
    b.append(T(460,414,'逐步加大速度直到轮子打滑或丢步，记录上限。',12,MUT))
    FIGS['fig17_运动测试场地布置']=svg(800,h,''.join(b))

for fn in (f01,f02,f03,f04,f05,f06,f07,f08,f09,f10,f11,f12,f13,f14,f15,f16,f17):
    fn()

for name,content in FIGS.items():
    with open(os.path.join(OUT,name+'.svg'),'w') as f: f.write(content)
print('generated',len(FIGS),'svg:',*FIGS.keys(),sep='\n  ')
