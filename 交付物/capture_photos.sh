#!/usr/bin/env bash
# 麦轮小车拍照采集脚本
# 用法:
#   bash capture_photos.sh              # 默认保存到 ~/麦轮小车pid/视觉测量/照片
#   bash capture_photos.sh /path/to/dir # 指定保存目录
#
# 说明: 在普通终端里运行（不要在本助手的沙箱里跑，沙箱访问不到 /dev/video*）。
set -euo pipefail

OUTDIR="${1:-$HOME/麦轮小车pid/视觉测量/照片}"
mkdir -p "$OUTDIR"

echo "== 探测摄像头 =="
DEV=""
for d in /dev/video*; do
  [ -e "$d" ] || continue
  if ffmpeg -hide_banner -f v4l2 -list_formats all -i "$d" >/dev/null 2>&1; then
    echo "  可用设备: $d"
    DEV="${DEV:-$d}"
  fi
done

if [ -z "$DEV" ]; then
  echo "!! 没找到可用的 /dev/video* 设备。" >&2
  echo "   请确认摄像头已插好，并用 'ls /dev/video*' 检查。" >&2
  exit 1
fi
echo "== 使用设备: $DEV =="

echo
echo "== 支持的格式 =="
ffmpeg -hide_banner -f v4l2 -list_formats all -i "$DEV" 2>&1 | sed -n '1,20p'
echo

RES="${RES:-1920x1080}"
echo "采集分辨率: $RES （如需修改: RES=1280x720 bash capture_photos.sh）"
echo "照片保存在: $OUTDIR"
echo

shoot() {
  local name="$1" hint="$2" wait_s="${3:-5}"
  echo "----------------------------------------------------------------"
  echo "【$name】 $hint"
  for ((i=wait_s; i>0; i--)); do printf "\r  %d 秒后拍摄… " "$i"; sleep 1; done
  printf "\r  拍摄中…            \n"
  ffmpeg -hide_banner -loglevel error -f v4l2 -input_format mjpeg \
         -video_size "$RES" -i "$DEV" -frames:v 1 -q:v 2 \
         -y "$OUTDIR/$name" < /dev/null
  echo "  已保存 $OUTDIR/$name"
  echo "  按回车继续（输入 s 跳过，输入 q 结束）"
  read -r ans </dev/tty || ans=""
  [ "$ans" = "q" ] && return 1
  return 0
}

shoot "01_top.png"     "整车俯视：相机置于车体正上方、镜头垂直向下，画面内放一把钢尺" 5 || exit 0
shoot "02_front.png"   "整车正视：镜头与车体等高、正对车头，画面保持水平" 5 || exit 0
shoot "03_side.png"    "整车侧视：镜头与车体等高、正对左侧面，画面保持水平" 5 || exit 0
shoot "04_bottom.png"  "整车底视：把小车举高或架起，从正下方拍摄，注意补光" 5 || exit 0
for n in 1 2 3 4; do
  shoot "05_roller_$n.png" "第 $n 只轮子俯视特写：从正上方约 30cm 垂直拍摄，画面含整轮" 4 || exit 0
done
for n in 1 2 3 4; do
  shoot "06_wheel_$n.png"  "第 $n 只轮子侧视/45°斜视特写：近距，含轮子全高" 4 || exit 0
done
shoot "07_motor_label.png" "电机/减速箱铭牌特写：近距对焦，字迹清晰" 5 || exit 0
shoot "08_encoder.png"     "编码器与线束特写：看清编码器盘/磁环与引出线" 5 || exit 0
shoot "09_board.png"       "主控板与驱动板特写：正面垂直拍摄，丝印清晰" 5 || exit 0
shoot "10_imu.png"         "IMU 安装位置与朝向特写：画面含车体前后方向标记" 5 || exit 0
shoot "11_layout.png"      "电池与配重布局俯视特写" 5 || exit 0
shoot "12_scale.png"       "整车+钢尺同框（俯视）：钢尺紧贴车体长边、与车体共面" 5 || exit 0
for n in 1 2 3 4; do
  shoot "13_load_$n.png"   "第 $n 只轮子压在电子秤上：照片能读到秤的读数" 5 || exit 0
done
echo
echo "== 剩余: 相机标定（去畸变用） =="
echo "请手持打印好的棋盘格（如 8x6 格 / 格边 25mm），在画面中变换角度与位置。"
echo "下面连续拍摄 15 张，每 2 秒一张；不用一直停着不动，慢慢变换角度即可。"
for i in $(seq 1 15); do
  printf "  calib %02d/15\n" "$i"
  ffmpeg -hide_banner -loglevel error -f v4l2 -input_format mjpeg \
         -video_size "$RES" -i "$DEV" -frames:v 1 -q:v 2 \
         -y "$OUTDIR/15_calib_$(printf '%02d' "$i").png" < /dev/null
  sleep 1
done

shoot "16_floor.png" "测试场地全景：拍实际测试用的地面与标线" 5 || exit 0
shoot "14_com_suspend.png" "质心悬吊试验：两根绳吊起小车，拍下悬垂线位置" 5 || exit 0

echo
echo "== 完成 =="
ls -1 "$OUTDIR"
echo
echo "把这些照片放在一起（或直接告诉助手目录），助手会做去畸变、量取尺寸并生成图片描述文档。"
