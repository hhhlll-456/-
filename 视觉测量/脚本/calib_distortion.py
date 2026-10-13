#!/usr/bin/env python3
"""calib_distortion.py —— 用 4 个边界 ArUco 码标定镜头畸变（前提: 相机正俯视固定）。

原理
----
相机光轴垂直场地时, 场地点 -> 理想(针孔, 无畸变)像平面 的映射只是**相似变换**
(旋转 + 均匀缩放 + 平移, 共 4 个自由度)。4 个边界码中心给出 8 个像素约束,
扣掉相似变换的 4 个自由度, 余下 4 个才用来拟合径向畸变系数 k1, k2。
拟合目标: 去畸变后的 4 点应构成一个与真实矩形(已知长宽比、四角 90°)相似的四边形。

注意: 残余误差**存在且非零**才说明判据有效(不是循环论证)。
     若残余很大 -> 说明相机没真正俯视(有透视), 需先摆正相机。

用法
----
  # 真实照片(可多张, 共享畸变、各图各自相似变换):
  python3 calib_distortion.py --photo ../照片/xxx.jpg --field-w 600 --field-h 400
  python3 calib_distortion.py --photo a.jpg b.jpg --field-w 600 --field-h 400 --with-tangential

  # 合成数据自检(不需要照片/尺寸, 验证算法能否反解畸变):
  python3 calib_distortion.py --selftest

输出(默认写到 视觉测量/结果/):
  calib_distortion.json   标定参数(K, dist, 场地->mm 相似变换), 供后续测量脚本复用
  calib_report.txt        人类可读报告
  calib_overlay.jpg       可视化(原检测点 / 去畸变点 / 拟合矩形)
"""
import argparse
import json
import os
import sys

import cv2
import numpy as np
from scipy.optimize import least_squares

_HERE = os.path.dirname(os.path.abspath(__file__))
_RESULT = os.path.normpath(os.path.join(_HERE, "..", "结果"))
_DEFAULT_BOUNDARY_IDS = (20, 21, 22, 23)  # TL / TR / BR / BL, 见 码表/aruco_markers.json
_F_NOMINAL = 1400.0  # 名义焦距(按 ~70° HFOV 估: 1920/(2 tan35°)≈1371); 见报告说明

# 多参数融合检测(与 detect5.py 同源; 小码默认参数会漏检)
_PASSES = [
    ("原图/默认", 1.0, {}),
    ("原图/中窗", 1.0, dict(adaptiveThreshWinSizeMin=7, adaptiveThreshWinSizeMax=23,
                          adaptiveThreshWinSizeStep=8, adaptiveThreshConstant=5)),
    ("2x/默认", 2.0, {}),
    ("2x/小窗", 2.0, dict(adaptiveThreshWinSizeMin=3, adaptiveThreshWinSizeMax=11,
                        adaptiveThreshWinSizeStep=4, adaptiveThreshConstant=5)),
    ("2x/大窗", 2.0, dict(adaptiveThreshWinSizeMin=15, adaptiveThreshWinSizeMax=85,
                        adaptiveThreshWinSizeStep=10, adaptiveThreshConstant=12)),
]


# ----------------------------------------------------------------------------- 检测
def detect_markers(gray):
    """多参数融合检测, 返回 {id: corners(4x2, 原图像素)}。"""
    adict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    best = {}
    for tag, scale, kw in _PASSES:
        g = (cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
             if scale != 1.0 else gray)
        p = cv2.aruco.DetectorParameters_create()
        p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        for k, v in kw.items():
            setattr(p, k, v)
        c, i, _ = cv2.aruco.detectMarkers(g, adict, parameters=p)
        if i is not None:
            for cc, ii in zip(c, i.flatten()):
                ii = int(ii)
                if ii not in best:
                    best[ii] = (tag, np.asarray(cc).reshape(-1, 2) / scale)
        if len(best) >= 5:
            break
    return {k: v[1] for k, v in best.items()}


def order_quad(corners_by_id):
    """把 4 个码按画面方位排成 [TL, TR, BR, BL], 返回 (ids, centers4x2)。"""
    ids = list(corners_by_id)
    cent = {i: corners_by_id[i].mean(axis=0) for i in ids}
    pts = np.array([cent[i] for i in ids])
    s = pts[:, 0] + pts[:, 1]
    d = pts[:, 0] - pts[:, 1]
    TL = ids[int(np.argmin(s))]
    BR = ids[int(np.argmax(s))]
    TR = ids[int(np.argmax(d))]
    BL = ids[int(np.argmin(d))]
    order = [TL, TR, BR, BL]
    return order, np.array([cent[i] for i in order], float)


# ----------------------------------------------------------------------------- 模型
def rect_corners(field_w, field_h):
    """真实矩形的 4 角(任意长度单位), 顺序同 [TL, TR, BR, BL], y 轴朝下。"""
    a, b = field_w / 2.0, field_h / 2.0
    return np.array([[-a, -b], [a, -b], [a, b], [-a, b]], float)


def _load_measured_K():
    """若存在棋盘格内参标定结果(calib_intrinsics_chess.json), 用实测 K; 否则返回 None。

    实测内参由棋盘格多姿态标定得到 (见该 JSON)。K 仅参与"去畸变"步骤;
    当畸变≈0 时去畸变近似恒等, 故 K 对场地测量结果影响极小, 主要用于 FOV/几何报告。
    """
    p = os.path.join(_RESULT, "calib_intrinsics_chess.json")
    try:
        with open(p) as f:
            K = np.array(json.load(f)["K"], float)
        if K.shape == (3, 3):
            return K
    except Exception:
        pass
    return None


def make_K(w, h):
    K = _load_measured_K()
    if K is not None:
        return K.copy()
    return np.array([[_F_NOMINAL, 0, w / 2.0], [0, _F_NOMINAL, h / 2.0], [0, 0, 1]], float)


def undistort_px(pts_px, k1, k2, p1, p2, K):
    """去畸变, 返回理想(针孔)像素坐标。"""
    dist = np.array([k1, k2, p1, p2, 0.0], float)
    q = cv2.undistortPoints(pts_px.reshape(-1, 1, 2).astype(np.float64), K, dist).reshape(-1, 2)
    return q * K[0, 0] + np.array([K[0, 2], K[1, 2]])


def distort_px(ideal_px, k1, k2, p1, p2, K):
    """正向畸变(仅用于合成自检)。"""
    f, cx, cy = K[0, 0], K[0, 2], K[1, 2]
    x = (ideal_px[:, 0] - cx) / f
    y = (ideal_px[:, 1] - cy) / f
    r2 = x * x + y * y
    radial = 1 + k1 * r2 + k2 * r2 * r2
    xd = x * radial + 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
    yd = y * radial + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y
    return np.stack([xd * f + cx, yd * f + cy], axis=1)


def similarity_fit(src, dst):
    """求 s,R,t 使 dst ≈ s·R·src + t (仅旋转, 不镜像)。返回 (s,R,t,pred)。"""
    n = len(src)
    mu_s, mu_d = src.mean(0), dst.mean(0)
    X, Y = src - mu_s, dst - mu_d
    U, S, Vt = np.linalg.svd((Y.T @ X) / n)
    D = np.ones(2)
    if np.linalg.det(U @ Vt) < 0:
        D[1] = -1
    R = U @ np.diag(D) @ Vt
    var_s = (X ** 2).sum() / n
    s = float((S * D).sum() / var_s)
    t = mu_d - s * R @ mu_s
    pred = (s * R @ src.T).T + t
    return s, R, t, pred


def _residuals(params, obs_list, truth, K, use_tan):
    k1, k2 = params[0], params[1]
    p1, p2 = (params[2], params[3]) if use_tan else (0.0, 0.0)
    out = []
    for obs in obs_list:
        ideal = undistort_px(obs, k1, k2, p1, p2, K)
        _, _, _, pred = similarity_fit(truth, ideal)
        out.append((ideal - pred).ravel())
    return np.concatenate(out)


def calibrate(obs_list, field_w, field_h, img_wh, use_tan=False, k_init=(-0.1, 0.02)):
    """obs_list: 每张图的 4x2 边界码中心(px, 顺序 TL/TR/BR/BL)。返回结果 dict。"""
    K = make_K(*img_wh)
    truth = rect_corners(field_w, field_h)

    # 标定前(dist=0)的残余, 作为对比基线
    r0 = _residuals([0.0, 0.0], obs_list, truth, K, use_tan)

    lo = [-0.5, -0.5] + ([-0.1, -0.1] if use_tan else [])
    hi = [0.5, 0.5] + ([0.1, 0.1] if use_tan else [])
    sol = least_squares(_residuals, list(k_init) + ([0.0, 0.0] if use_tan else []),
                        args=(obs_list, truth, K, use_tan), bounds=(lo, hi), method="trf")
    params = sol.x
    k1, k2 = params[0], params[1]
    p1, p2 = (params[2], params[3]) if use_tan else (0.0, 0.0)
    r1 = _residuals(params, obs_list, truth, K, use_tan)

    # 标定后, 用第一张图算几何指标
    ideal = undistort_px(obs_list[0], k1, k2, p1, p2, K)
    s, R, t, pred = similarity_fit(truth, ideal)
    quad = ideal  # TL,TR,BR,BL
    ang = []
    for i in range(4):
        a, b, c = quad[(i - 1) % 4], quad[i], quad[(i + 1) % 4]
        v1, v2 = a - b, c - b
        ang.append(np.degrees(np.arccos(np.clip(np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2)), -1, 1))))
    sides = [np.linalg.norm(quad[(i + 1) % 4] - quad[i]) for i in range(4)]  # TL-TR,TR-BR,BR-BL,BL-TL
    px_per_mm = float(s)  # similarity_fit 把真实 mm 映到理想 px, 故 s 即 px/mm
    resid_after_px = float(np.sqrt(np.mean(r1 ** 2)))
    usable = (px_per_mm > 0) and (resid_after_px / px_per_mm <= 2.0)

    return dict(usable=usable,
        K=K.tolist(), dist=[k1, k2, p1, p2, 0.0], use_tan=bool(use_tan),
        field_wh_mm=[field_w, field_h],
        px_per_mm=px_per_mm, similarity=dict(s=float(s), R=R.tolist(), t=t.tolist()),
        resid_before_px=float(np.sqrt(np.mean(r0 ** 2))),
        resid_after_px=float(np.sqrt(np.mean(r1 ** 2))),
        field_quad_px=quad.tolist(), rectangle_fit_px=pred.tolist(),
        corner_ang_deg=ang, side_px=sides,
        obs_list=[o.tolist() for o in obs_list],
    )


def calibrate_auto(obs_list, corners_list, img_wh, marker_mm=25.0, use_tan=False):
    """更稳健的标定(推荐):
      1) 畸变 k1,k2 —— 由"四个边界码应为全等正方形"拟合(用 16 个角点, 良态, 不依赖场地尺寸)
      2) 场地长宽比 —— 去畸变后中心四边形 与 真值矩形 的相似残差最小
      3) 绝对比例尺 —— 由已知码物理边长

    obs_list     每张图的 4x2 边界码中心(TL/TR/BR/BL)
    corners_list 每张图的 {id: 4x2 角点}
    marker_mm    码的物理边长(mm)
    """
    from scipy.optimize import minimize_scalar, least_squares

    K = make_K(*img_wh)
    unit = np.array([[0.0, 0], [1, 0], [1, 1], [0, 1]])
    p1 = p2 = 0.0

    def fit_dist():
        def resid(p):
            k1, k2 = p
            shp, scl = [], []
            for cb in corners_list:
                for c in cb.values():
                    ideal = undistort_px(np.asarray(c, float), k1, k2, 0, 0, K)
                    s, _, _, pred = similarity_fit(unit, ideal)
                    shp.append((ideal - pred).ravel())
                    scl.append(s)
            scl = np.array(scl)
            return np.concatenate([np.concatenate(shp) * 3.0, scl / scl.mean() - 1.0])
        sol = least_squares(resid, [-0.05, 0.02], bounds=([-0.4, -0.3], [0.4, 0.3]))
        return float(sol.x[0]), float(sol.x[1])

    k1, k2 = fit_dist()

    def rms_center(ar, a, b):
        truth = rect_corners(ar, 1.0)
        out = []
        for obs in obs_list:
            ideal = undistort_px(obs, a, b, p1, p2, K)
            _, _, _, pred = similarity_fit(truth, ideal)
            out.append((ideal - pred).ravel())
        return np.concatenate(out)

    opt = minimize_scalar(lambda ar: float(np.sqrt((rms_center(ar, k1, k2) ** 2).mean())),
                          bounds=(0.4, 4.0), method="bounded", options=dict(xatol=1e-3))
    ar = float(opt.x)

    r0 = rms_center(ar, 0.0, 0.0)
    r1 = rms_center(ar, k1, k2)
    ideal = undistort_px(obs_list[0], k1, k2, p1, p2, K)
    s, R, t, pred = similarity_fit(rect_corners(ar, 1.0), ideal)
    quad = ideal
    ang = []
    for i in range(4):
        a0, b0, c0 = quad[(i - 1) % 4], quad[i], quad[(i + 1) % 4]
        v1, v2 = a0 - b0, c0 - b0
        ang.append(np.degrees(np.arccos(np.clip(np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2)), -1, 1))))
    sides = [np.linalg.norm(quad[(i + 1) % 4] - quad[i]) for i in range(4)]

    per_side = []
    for cb in corners_list:
        for c in cb.values():
            ii = undistort_px(np.asarray(c, float), k1, k2, p1, p2, K)
            per_side.append(float(np.mean([np.linalg.norm(ii[(j + 1) % 4] - ii[j]) for j in range(4)])))
    px_per_mm = float(np.mean(per_side)) / marker_mm
    spread = 100.0 * (max(per_side) - min(per_side)) / float(np.mean(per_side))

    w_px = (np.linalg.norm(quad[1] - quad[0]) + np.linalg.norm(quad[2] - quad[3])) / 2
    h_px = (np.linalg.norm(quad[3] - quad[0]) + np.linalg.norm(quad[2] - quad[1])) / 2
    field_wh_mm = [round(w_px / px_per_mm, 1), round(h_px / px_per_mm, 1)]
    resid_after_px = float(np.sqrt(np.mean(r1 ** 2)))

    # 场地平面单应(后续测量的核心变换): 4 个边界码中心(去畸变后) -> 场地矩形
    #   H_px2unit : 图像px -> 归一化场地坐标[0,1]²  (约定同 码表/aruco_markers.json: (0,0)=TL)
    #   H_px2mm   : 图像px -> 场地mm           (原点TL, x右, y下)
    # 用法: 先去畸变(undistort_px) 再乘 H; 或因畸变极小(见报告)近似直接乘 H。
    src4 = quad.astype(np.float32)
    unit_rect = np.array([[0, 0], [1, 0], [1, 1], [0, 1]], np.float32)
    fw, fh = field_wh_mm
    mm_rect = np.array([[0, 0], [fw, 0], [fw, fh], [0, fh]], np.float32)
    H_px2unit = cv2.getPerspectiveTransform(src4, unit_rect)
    H_px2mm = cv2.getPerspectiveTransform(src4, mm_rect)

    return dict(
        usable=resid_after_px / px_per_mm <= 2.0,
        K=K.tolist(), dist=[k1, k2, p1, p2, 0.0], use_tan=False,
        field_aspect=ar, field_wh_mm=field_wh_mm, marker_mm=marker_mm,
        px_per_mm=px_per_mm, marker_side_px=[round(x, 2) for x in per_side],
        marker_size_spread_pct=round(spread, 2),
        similarity=dict(s=float(s), R=R.tolist(), t=t.tolist()),
        H_px2unit=H_px2unit.tolist(), H_px2mm=H_px2mm.tolist(),
        resid_before_px=float(np.sqrt(np.mean(r0 ** 2))), resid_after_px=resid_after_px,
        field_quad_px=quad.tolist(), rectangle_fit_px=pred.tolist(),
        corner_ang_deg=ang, side_px=sides, obs_list=[o.tolist() for o in obs_list],
    )


# ----------------------------------------------------------------------------- 自检
def selftest():
    print("== 合成自检 ==")
    W, H = 600.0, 400.0
    img_wh = (1920, 1080)
    K = make_K(*img_wh)
    truth = rect_corners(W, H)
    true_k = (-0.18, 0.05)
    theta = np.radians(5.0)
    R = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
    s = 1.7                                    # px/mm
    t = np.array([1050.0, 620.0])              # 偏心, 使四角半径有足够张度(贴近真实场地)
    ideal = (s * R @ truth.T).T + t

    def report(tag, obs, expect_fix):
        res = calibrate([obs], W, H, img_wh)
        k1, k2 = res["dist"][0], res["dist"][1]
        print(f"\n[{tag}]")
        print(f"  真值 k1,k2 = {true_k}")
        print(f"  反解 k1,k2 = ({k1:.4f}, {k2:.4f})")
        print(f"  残余: 标定前 {res['resid_before_px']:.3f}px -> 标定后 {res['resid_after_px']:.4f}px")
        print(f"  去畸变四角角度 = {[round(a,2) for a in res['corner_ang_deg']]} (应≈90)")
        return res

    # 测试1: 纯畸变 (真·正俯视) -> 必须能反解并归零
    obs1 = distort_px(ideal, true_k[0], true_k[1], 0.0, 0.0, K)
    r1 = report("测试1 纯畸变(正俯视)", obs1, True)
    k1, k2 = r1["dist"][0], r1["dist"][1]
    ok1 = (abs(k1 - true_k[0]) < 0.01 and abs(k2 - true_k[1]) < 0.01
           and r1["resid_after_px"] < 0.05 and r1["resid_before_px"] > 0.3)
    print("  -> ", "PASS" if ok1 else "FAIL")

    # 测试2: 相机倾斜(引入透视) -> 相似+径向模型拟合不掉, 残余应显著残留(工具应报警)
    tilted = ideal.copy()
    tilted[0] = ideal[0] + np.array([60.0, 0.0])   # TL 外扩
    tilted[1] = ideal[1] + np.array([-60.0, 0.0])  # TR 内收 -> 上边呈梯形
    obs2 = distort_px(tilted, true_k[0], true_k[1], 0.0, 0.0, K)
    r2 = report("测试2 相机倾斜(含透视)", obs2, False)
    ok2 = (not r2["usable"]) and (r2["resid_after_px"] > 2.0)   # 应判定不可用且残余显著
    print(f"  -> 残余 {r2['resid_after_px']:.1f}px, usable={r2['usable']}, "
          f"{'PASS(能识别出未正俯视)' if ok2 else 'FAIL'}")

    ok = ok1 and ok2
    print("\n自检总结果:", "PASS" if ok else "FAIL")
    return ok


# ----------------------------------------------------------------------------- 可视化
def save_overlay(photo_paths, res, out_path):
    img = cv2.imread(photo_paths[0])
    if img is None:
        return
    obs = np.array(res["obs_list"][0], float)
    quad = np.array(res["field_quad_px"], float)
    pred = np.array(res["rectangle_fit_px"], float)
    for i in range(4):
        p0 = tuple(np.round(obs[i]).astype(int))
        p1 = tuple(np.round(quad[i]).astype(int))
        cv2.circle(img, p0, 6, (0, 0, 255), -1)       # 红: 原始检测
        cv2.circle(img, p1, 6, (0, 255, 0), -1)       # 绿: 去畸变
        cv2.arrowedLine(img, p0, p1, (255, 0, 0), 2, tipLength=0.25)  # 蓝: 畸变位移
    cv2.polylines(img, [np.round(pred).astype(int)], True, (0, 255, 0), 2)  # 拟合矩形
    cv2.putText(img, f"k1={res['dist'][0]:.4f} k2={res['dist'][1]:.4f} "
                     f"resid {res['resid_before_px']:.1f}->{res['resid_after_px']:.2f}px",
                (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 200, 255), 2)
    cv2.imwrite(out_path, img, [cv2.IMWRITE_JPEG_QUALITY, 92])


# ----------------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--photo", nargs="+", help="标定用照片(可多张)")
    ap.add_argument("--auto-aspect", action="store_true",
                    help="自动推断场地长宽比 + 用已知码边长定比例尺(只需 --marker-mm)")
    ap.add_argument("--marker-mm", type=float, default=25.0, help="码的物理边长 mm (默认 25 = 2.5cm)")
    ap.add_argument("--field-w", type=float, help="真实矩形宽(mm), 与 --field-h 同单位即可")
    ap.add_argument("--field-h", type=float, help="真实矩形高(mm)")
    ap.add_argument("--with-tangential", action="store_true", help="同时拟合切向畸变 p1,p2(4参->残余0,会过拟合)")
    ap.add_argument("--selftest", action="store_true", help="合成数据自检")
    ap.add_argument("--outdir", default=_RESULT)
    args = ap.parse_args()

    if args.selftest:
        sys.exit(0 if selftest() else 1)

    if not args.photo:
        ap.error("需要 --photo (或用 --selftest)")
    if not args.auto_aspect and (args.field_w is None or args.field_h is None):
        ap.error("需要 --field-w/--field-h, 或使用 --auto-aspect")

    obs_list, corners_list, used, img_wh = [], [], [], None
    for path in args.photo:
        img = cv2.imread(path)
        if img is None:
            sys.exit(f"无法读取 {path}")
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        found = detect_markers(gray)
        want = [i for i in _DEFAULT_BOUNDARY_IDS if i in found]
        if len(want) != 4:
            sys.exit(f"{os.path.basename(path)}: 只检出边界码 {want} (需要 {list(_DEFAULT_BOUNDARY_IDS)}); "
                     f"全部检出={sorted(found)}")
        ids, cent = order_quad({i: found[i] for i in want})
        obs_list.append(cent)
        corners_list.append({i: found[i] for i in want})
        used.append(os.path.basename(path))
        img_wh = (img.shape[1], img.shape[0])
        print(f"{os.path.basename(path)}: 边界码 {ids} 中心 {cent.round(1).tolist()}")

    if args.auto_aspect:
        res = calibrate_auto(obs_list, corners_list, img_wh, marker_mm=args.marker_mm,
                             use_tan=args.with_tangential)
        rect_desc = (f"自动推断 {res['field_aspect']:.3f}:1  =>  约 "
                     f"{res['field_wh_mm'][0]:.0f} x {res['field_wh_mm'][1]:.0f} mm")
    else:
        res = calibrate(obs_list, args.field_w, args.field_h, img_wh, use_tan=args.with_tangential)
        rect_desc = f"指定 {args.field_w} x {args.field_h}"
    res["photos"] = used

    os.makedirs(args.outdir, exist_ok=True)
    with open(os.path.join(args.outdir, "calib_distortion.json"), "w") as f:
        json.dump(res, f, indent=2, ensure_ascii=False)
    save_overlay(args.photo, res, os.path.join(args.outdir, "calib_overlay.jpg"))

    lines = [
        "== 镜头畸变标定报告 (4 边界码, 正俯视假设) ==",
        f"照片: {', '.join(used)}",
        f"场地矩形: {rect_desc}",
        f"畸变系数: k1={res['dist'][0]:.5f}  k2={res['dist'][1]:.5f}"
        + (f"  p1={res['dist'][2]:.5f}  p2={res['dist'][3]:.5f}" if args.with_tangential else ""),
        f"残余误差: 标定前 {res['resid_before_px']:.2f}px -> 标定后 {res['resid_after_px']:.3f}px",
        f"比例尺: {res['px_per_mm']:.4f} px/mm  (1px = {1.0/res['px_per_mm']:.4f} mm)",
        f"去畸变后四角角度(deg): {[round(a,2) for a in res['corner_ang_deg']]}  (应≈90)",
        f"去畸变后四边形边长(px): TL-TR={res['side_px'][0]:.1f} TR-BR={res['side_px'][1]:.1f} "
        f"BR-BL={res['side_px'][2]:.1f} BL-TL={res['side_px'][3]:.1f}",
    ]
    if "marker_side_px" in res:
        lines.append(f"码边长(去畸变后px, 应四码一致): {res['marker_side_px']}"
                     f"  离散 {res['marker_size_spread_pct']:.2f}%")
    mm = 1.0 / res["px_per_mm"]
    lines.append(f"残余换算: 标定后 {res['resid_after_px']*mm:.2f} mm   (usable={res['usable']})")
    # 畸变量级评估: 视场边缘处的径向缩放偏差 (r = 半对角线归一化)
    r_edge = np.hypot(res["K"][0][2], res["K"][1][2]) / res["K"][0][0]
    radial = res["dist"][0] * r_edge ** 2 + res["dist"][1] * r_edge ** 4
    lines.append(f"畸变量级: 视场边缘径向偏差 ≈ {abs(radial)*100:.2f}%  "
                 f"(≈ {abs(radial)*np.hypot(res['K'][0][2], res['K'][1][2]):.1f}px)")
    if "H_px2mm" in res:
        lines.append("已输出场地平面单应 H_px2mm / H_px2unit (图像px -> 场地mm / 归一化坐标)。")
    if not res["usable"]:
        lines.append("说明: 残余主要来自**透视(相机未严格正俯视)/码摆放**, 调畸变系数无法再降低; "
                     "测量请用 H_px2mm(平面单应), 它已把 4 码中心精确映成矩形。")
    else:
        lines.append("结论: 残余很小, 正俯视假设成立, 畸变标定可用。")
    report = "\n".join(lines)
    with open(os.path.join(args.outdir, "calib_report.txt"), "w") as f:
        f.write(report + "\n")
    print("\n" + report)
    print(f"\n已写: {args.outdir}/calib_distortion.json, calib_report.txt, calib_overlay.jpg")


if __name__ == "__main__":
    main()
