#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>

#define SYS_GPIO_CLASS "/sys/class/heartbeat_gpio"
#define BUF_SIZE 64

// 해당 GPIO를 sysfs를 통해 export
int GPIOExport(int gpio) {
    char path[BUF_SIZE];
    int fd;
    snprintf(path, sizeof(path), SYS_GPIO_CLASS "/export");

    fd = open(path, O_WRONLY);
    if (fd < 0) {
        perror("GPIOExport: open");
        return -1;
    }

    dprintf(fd, "%d", gpio);
    close(fd);
    return 0;
}


// 해당 GPIO를 sysfs를 통해 unexport
int GPIOUnexport(int gpio) {
    char path[BUF_SIZE];
    int fd;
    snprintf(path, sizeof(path), SYS_GPIO_CLASS "/unexport");

    fd = open(path, O_WRONLY);
    if (fd < 0) {
        perror("GPIOUnexport: open");
        return -1;
    }

    dprintf(fd, "%d", gpio);
    close(fd);
    return 0;
}

// 해당 GPIO의 direction 설정
int GPIODirection(int gpio, const char *dir) {
    char path[BUF_SIZE];
    int fd;
    snprintf(path, sizeof(path), SYS_GPIO_CLASS "/gpio%d/direction", gpio);

    fd = open(path, O_WRONLY);
    if (fd < 0) {
        perror("GPIODirection: open");
        return -1;
    }

    write(fd, dir, strlen(dir));
    close(fd);
    return 0;
}

// 해당 GPIO에 값을 씀
int GPIOWrite(int gpio, int value) {
    char path[BUF_SIZE];
    int fd;
    snprintf(path, sizeof(path), SYS_GPIO_CLASS "/gpio%d/value", gpio);

    fd = open(path, O_WRONLY);
    if (fd < 0) {
        perror("GPIOWrite: open");
        return -1;
    }

    dprintf(fd, "%d", value);
    close(fd);
    return 0;
}


// 해당 GPIO의 값을 읽음
int GPIORead(int gpio) {
    char path[BUF_SIZE];
    char val;
    int fd;
    snprintf(path, sizeof(path), SYS_GPIO_CLASS "/gpio%d/value", gpio);

    fd = open(path, O_RDONLY);
    if (fd < 0) {
        perror("GPIORead: open");
        return -1;
    }

    if (read(fd, &val, 1) != 1) {
        perror("GPIORead: read");
        close(fd);
        return -1;
    }

    close(fd);
    return (val == '1') ? 1 : 0;
}