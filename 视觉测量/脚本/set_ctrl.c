/* V4L2 控件读写工具: ./set_ctrl <dev> get <id> | set <id> <value>  (id 可用 0x.. 或 focus/af/zoom/pan/tilt) */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>

static __u32 parse_id(const char *s) {
    if (!strcmp(s, "focus")) return V4L2_CID_FOCUS_ABSOLUTE;
    if (!strcmp(s, "af")) return V4L2_CID_FOCUS_AUTO;
    if (!strcmp(s, "zoom")) return V4L2_CID_ZOOM_ABSOLUTE;
    if (!strcmp(s, "pan")) return V4L2_CID_PAN_ABSOLUTE;
    if (!strcmp(s, "tilt")) return V4L2_CID_TILT_ABSOLUTE;
    if (!strcmp(s, "exposure")) return V4L2_CID_EXPOSURE_ABSOLUTE;
    if (!strcmp(s, "exposure_auto")) return V4L2_CID_EXPOSURE_AUTO;
    return (__u32)strtoul(s, NULL, 0);
}

int main(int argc, char **argv) {
    if (argc < 4) {
        fprintf(stderr, "usage: %s <dev> get|set <id> [value]\n", argv[0]);
        return 1;
    }
    int fd = open(argv[1], O_RDWR);
    if (fd < 0) { perror("open"); return 1; }
    __u32 id = parse_id(argv[3]);

    if (!strcmp(argv[2], "get")) {
        struct v4l2_control c = { .id = id };
        if (ioctl(fd, VIDIOC_G_CTRL, &c) < 0) { perror("G_CTRL"); return 1; }
        printf("%d\n", c.value);
    } else {
        struct v4l2_queryctrl q = { .id = id };
        if (ioctl(fd, VIDIOC_QUERYCTRL, &q) < 0) { perror("QUERYCTRL"); return 1; }
        struct v4l2_control c = { .id = id, .value = atoi(argv[4]) };
        if (ioctl(fd, VIDIOC_S_CTRL, &c) < 0) { perror("S_CTRL"); return 1; }
        struct v4l2_control r = { .id = id };
        ioctl(fd, VIDIOC_G_CTRL, &r);
        printf("set %s [%d..%d] -> %d\n", q.name, q.minimum, q.maximum, r.value);
    }
    close(fd);
    return 0;
}
