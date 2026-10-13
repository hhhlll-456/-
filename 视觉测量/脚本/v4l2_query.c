/* 查询 V4L2 摄像头对焦/曝光相关控件的硬件能力 */
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>

static const char *type_name(__u32 t) {
    switch (t) {
        case V4L2_CTRL_TYPE_INTEGER: return "INTEGER";
        case V4L2_CTRL_TYPE_BOOLEAN: return "BOOLEAN";
        case V4L2_CTRL_TYPE_MENU: return "MENU";
        case V4L2_CTRL_TYPE_INTEGER_MENU: return "INTEGER_MENU";
        case V4L2_CTRL_TYPE_CTRL_CLASS: return "CLASS";
        case V4L2_CTRL_TYPE_STRING: return "STRING";
        case V4L2_CTRL_TYPE_BITMASK: return "BITMASK";
        case V4L2_CTRL_TYPE_INTEGER64: return "INTEGER64";
        default: return "?";
    }
}

static void query(int fd, __u32 id) {
    struct v4l2_queryctrl q;
    memset(&q, 0, sizeof(q));
    q.id = id;
    if (ioctl(fd, VIDIOC_QUERYCTRL, &q) < 0) {
        printf("  id=0x%08x  -> 不存在/不支持 (%s)\n", id, strerror(errno));
        return;
    }
    if (q.flags & V4L2_CTRL_FLAG_DISABLED) {
        printf("  [%s] id=0x%08x -> 存在但被禁用\n", q.name, id);
        return;
    }
    printf("  [%s] id=0x%08x type=%s range=[%d..%d] step=%d default=%d flags=0x%x%s%s%s\n",
           q.name, id, type_name(q.type), q.minimum, q.maximum, q.step, q.default_value, q.flags,
           (q.flags & V4L2_CTRL_FLAG_UPDATE) ? " UPDATE" : "",
           (q.flags & V4L2_CTRL_FLAG_EXECUTE_ON_WRITE) ? " EXEC_ON_WRITE" : "",
           (q.flags & V4L2_CTRL_FLAG_READ_ONLY) ? " READ_ONLY" : "");
    /* 读当前值 */
    struct v4l2_control c;
    memset(&c, 0, sizeof(c));
    c.id = id;
    if (ioctl(fd, VIDIOC_G_CTRL, &c) == 0)
        printf("      当前值 = %d\n", c.value);
}

int main(int argc, char **argv) {
    const char *dev = argc > 1 ? argv[1] : "/dev/video0";
    int fd = open(dev, O_RDWR | O_NONBLOCK);
    if (fd < 0) { perror("open"); return 1; }

    struct v4l2_capability cap;
    memset(&cap, 0, sizeof(cap));
    if (ioctl(fd, VIDIOC_QUERYCAP, &cap) == 0) {
        printf("设备: %s\n", dev);
        printf("  driver  : %s\n", cap.driver);
        printf("  card    : %s\n", cap.card);
        printf("  bus_info: %s\n", cap.bus_info);
        printf("  version : %u.%u.%u\n", (cap.version >> 16) & 0xff, (cap.version >> 8) & 0xff, cap.version & 0xff);
        printf("  caps    : 0x%08x  device_caps: 0x%08x\n", cap.capabilities, cap.device_caps);
    }

    printf("\n== 对焦相关控件 ==\n");
    query(fd, V4L2_CID_FOCUS_ABSOLUTE);
    query(fd, V4L2_CID_FOCUS_RELATIVE);
    query(fd, V4L2_CID_FOCUS_AUTO);
    query(fd, V4L2_CID_AUTO_FOCUS_START);
    query(fd, V4L2_CID_AUTO_FOCUS_STATUS);

    printf("\n== 曝光/白平衡等其他控件 ==\n");
    query(fd, V4L2_CID_EXPOSURE_AUTO);
    query(fd, V4L2_CID_EXPOSURE_ABSOLUTE);
    query(fd, V4L2_CID_AUTO_FOCUS_RANGE);

    /* 枚举 class: Camera class 0x009a0900, User class 0x00980900 */
    printf("\n== 遍历 Camera class 控件 ==\n");
    for (__u32 id = 0x009a0900; id < 0x009a0a00; id++) {
        struct v4l2_queryctrl q;
        memset(&q, 0, sizeof(q));
        q.id = id | V4L2_CTRL_FLAG_NEXT_CTRL;
        if (ioctl(fd, VIDIOC_QUERYCTRL, &q) == 0 && (q.id & 0x0fff0000) == 0x009a0000) {
            printf("  [%s] id=0x%08x type=%s range=[%d..%d] def=%d\n",
                   q.name, q.id, type_name(q.type), q.minimum, q.maximum, q.default_value);
        } else break;
    }
    printf("\n== 遍历 User class 控件 ==\n");
    for (__u32 id = 0x00980900; id < 0x00980a00; id++) {
        struct v4l2_queryctrl q;
        memset(&q, 0, sizeof(q));
        q.id = id | V4L2_CTRL_FLAG_NEXT_CTRL;
        if (ioctl(fd, VIDIOC_QUERYCTRL, &q) == 0 && (q.id & 0x0fff0000) == 0x00980000) {
            printf("  [%s] id=0x%08x type=%s range=[%d..%d] def=%d\n",
                   q.name, q.id, type_name(q.type), q.minimum, q.maximum, q.default_value);
        } else break;
    }

    close(fd);
    return 0;
}
