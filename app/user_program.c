#include "api.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <signal.h>
#include <pthread.h>
#include <poll.h>
#include <stdbool.h>


//IOCTL 매직 넘버 및 명령어 정의
#define GPIO_IOCTL_MAGIC      'G'
#define GPIO_IOCTL_FD_CONFIG  _IOW(GPIO_IOCTL_MAGIC, 3, struct fd_config)
#define GPIO_IOCTL_FD_STOP    _IO(GPIO_IOCTL_MAGIC, 4)


//GPIO 핀 정의
#define TX_PULSE        17
#define CLK_STATUS_OUT  27
#define CLK_STATUS_IN   19
#define RX_PULSE        26

#define DURATION_SEC    40


//IOCTL 설정 구조체
struct fd_config {
    int tx_pulse;
    int clk_out;
    int clk_in;
    int rx_pulse;
};

// 종료 감지 플래그
static volatile sig_atomic_t keep_running = 1;

static void sigint_handler(int signo) {
    keep_running = 0;
}

static void *kmsg_thread(void *arg) {
    int kfd = open("/dev/kmsg", O_RDONLY | O_NONBLOCK);
    if (kfd < 0) {
        perror("open /dev/kmsg");
        return NULL;
    }

    char drain[512];
    while (read(kfd, drain, sizeof(drain)) > 0) {}

    struct pollfd pfd = { .fd = kfd, .events = POLLIN };
    char buf[512];

    // 메시지 프레임 수집
    bool frame_collecting = false;
    bool seen_status = false;
    int tx_count = 0, rx_count = 0;
    char tx_bits[1024] = {0}, rx_bits[1024] = {0};
    char message[128] = {0};
    char checksum[64] = {0};

    while (keep_running) {
        if (poll(&pfd, 1, 500) <= 0)
            continue;
        if (!(pfd.revents & POLLIN))
            continue;

        ssize_t n = read(kfd, buf, sizeof(buf) - 1);
        if (n <= 0)
            continue;
        buf[n] = '\0';

        char *msg = strchr(buf, ';');
        if (!msg)
            continue;
        msg++;

        // 콘솔에 BPM 출력
        if (strstr(msg, "Time=") && strstr(msg, "BPM=")) {
            char *bp = strstr(msg, "BPM=");
            int bpm;
            if (bp && sscanf(bp + 4, "%d", &bpm) == 1) {
                printf("BPM value: %d\n", bpm);
            }
        }

        // 콘솔에 평균 BPM 계산 결과 출력
        else if (strstr(msg, "RX: avg BPM over")) {
            while (*msg == ' ') msg++;
            printf("%s", msg);
            frame_collecting = false;
            seen_status = false;
        }

        // 상태 메시지 전송 시작
        else if (strstr(msg, "TX: sending status")) {
            if (!seen_status) {
                seen_status = true;
            } else {
                // 실제 메시지 프레임 수집 시작
                frame_collecting = true;
                tx_count = rx_count = 0;
                tx_bits[0] = rx_bits[0] = '\0';
                message[0] = checksum[0] = '\0';
                while (*msg == ' ') msg++;
                printf("\n%s\n", msg);
            }
        }

        // 송신된 비트 수집
        else if (frame_collecting && strstr(msg, "TX-BIT[")) {
            char *p = strstr(msg, "]=");
            if (p && (p[2] == '0' || p[2] == '1') && tx_count < (int)(sizeof(tx_bits) - 1)) {
                tx_bits[tx_count++] = p[2];
                tx_bits[tx_count] = '\0';
            }
        }
        else if (frame_collecting && strstr(msg, "RX-BIT=")) {
            char *p = strstr(msg, "RX-BIT=");
            if (p && (p[7] == '0' || p[7] == '1') && rx_count < (int)(sizeof(rx_bits) - 1)) {
                rx_bits[rx_count++] = p[7];
                rx_bits[rx_count] = '\0';
            }
        }

        // 수신된 메시지를 확인하고, 중요한 부분을 요약해 출력
        else if (frame_collecting && strstr(msg, "RX: received message=\"")) {
            char *start = strstr(msg, "RX: received message=\"");
            if (start) {
                start += strlen("RX: received message=\"");
                char *end = strchr(start, '"');
                if (end) {
                    size_t len = end - start;
                    if (len < sizeof(message)) {
                        strncpy(message, start, len);
                        message[len] = '\0';
                    }
                }

                char *ok = strstr(end + 1, "(OK)");
                if (ok) {
                    snprintf(checksum, sizeof(checksum), "Checksum status  : OK");
                } else {
                    snprintf(checksum, sizeof(checksum), "Checksum status  : FAIL");
                }

                printf("\n=== Frame Summary ===\n");
                printf("Sent by TX       : %s\n", tx_bits);
                printf("Received by RX   : %s\n", rx_bits);
                printf("Assembled message: %s\n", message);
                printf("%s\n", checksum);

                frame_collecting = false;
                seen_status = false;
            }
        }

        fflush(stdout);
    }

    close(kfd);
    return NULL;
}


int main(void) {
    int pins[] = { TX_PULSE, CLK_STATUS_OUT, CLK_STATUS_IN, RX_PULSE, -1 };
    struct fd_config cfg = {
        .tx_pulse = TX_PULSE,
        .clk_out  = CLK_STATUS_OUT,
        .clk_in   = CLK_STATUS_IN,
        .rx_pulse = RX_PULSE
    };
    pthread_t logger;
    int fd;

    signal(SIGINT, sigint_handler);

    // 로그 모니터링용 스레드 시작
    pthread_create(&logger, NULL, kmsg_thread, NULL);

    //GPIO 핀 export 및 direction 설정
    for (int i = 0; pins[i] >= 0; ++i)
        GPIOExport(pins[i]);
    GPIODirection(TX_PULSE,       "out");
    GPIODirection(CLK_STATUS_OUT, "out");
    GPIODirection(CLK_STATUS_IN,  "in");
    GPIODirection(RX_PULSE,       "in");
    GPIOWrite(TX_PULSE, 0);
    GPIOWrite(CLK_STATUS_OUT, 0);

    // 디바이스 파일을 열고, 드라이버 설정 전달
    char dev_path[32];
    snprintf(dev_path, sizeof(dev_path), "/dev/gpio%d", TX_PULSE);
    fd = open(dev_path, O_RDWR);
    if (fd < 0) {
        perror("open gpio device");
        goto cleanup;
    }
    if (ioctl(fd, GPIO_IOCTL_FD_CONFIG, &cfg) < 0) {
        perror("ioctl FD_CONFIG");
        close(fd);
        goto cleanup;
    }

    time_t start = time(NULL);
    while (keep_running && difftime(time(NULL), start) < DURATION_SEC)
        sleep(1);

    // 종료 요청 이후 드라이버 정지
    keep_running = 0;
    ioctl(fd, GPIO_IOCTL_FD_STOP);
    close(fd);

cleanup:
    // GPIO Unexport
    for (int i = 0; pins[i] >= 0; ++i)
        GPIOUnexport(pins[i]);
    // 로깅용 스레드 종료
    pthread_join(logger, NULL);
    return 0;
}