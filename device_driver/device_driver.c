#include <linux/module.h>
#include <linux/init.h>
#include <linux/ctype.h>
#include <linux/gpio/consumer.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/fcntl.h>
#include <linux/signal.h>
#include <linux/poll.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/timekeeping.h>
#include <linux/string.h>
#include <linux/jiffies.h>
#include <linux/wait.h>
#include <linux/atomic.h>


// 매크로 및 상수 정의
#define CLASS_NAME           "heartbeat_gpio"
#define MAX_GPIO             10
#define GPIOCHIP_BASE        512

#define GPIO_IOCTL_MAGIC     'G'
#define GPIO_IOCTL_ENABLE_IRQ _IOW(GPIO_IOCTL_MAGIC, 1, int)
#define GPIO_IOCTL_DISABLE_IRQ _IOW(GPIO_IOCTL_MAGIC, 2, int)
#define GPIO_IOCTL_FD_CONFIG  _IOW(GPIO_IOCTL_MAGIC, 3, struct fd_config)
#define GPIO_IOCTL_FD_STOP    _IO(GPIO_IOCTL_MAGIC, 4)

#define START_BYTE           0x7E
#define MAX_DATA_LEN         32
#define MEASURE_SEC          10
#define MIN_BPM              20
#define MAX_SLEEP_MS         2000
#define WAIT_TIMEOUT_MS      5000


// GPIO 엔트리 구조체 정의
struct gpio_entry {
    int bcm_num;
    struct gpio_desc *desc;
    struct device *dev;
    int irq_num;
    bool irq_enabled;
    struct fasync_struct *async_queue;
};

// Full-Duplex 설정용 구조체 정의
struct fd_config {
    int tx_pulse;
    int clk_out;
    int clk_in;
    int rx_pulse;
};

// Character device, GPIO 테이블 전역 변수 정의
static dev_t dev_num_base;
static struct cdev gpio_cdev;
static int major_num;
static struct class *gpiod_class;
static struct gpio_entry *gpio_table[MAX_GPIO];

// Full-Duplex GPIO 및 스레드 관련 전역 변수 정의
static struct gpio_desc *fd_tx_desc;
static struct gpio_desc *fd_clk_out_desc;
static struct gpio_desc *fd_clk_in_desc;
static struct gpio_desc *fd_rx_desc;
static struct task_struct *fd_tx_thread;
static struct task_struct *fd_rx_thread;
static DEFINE_MUTEX(fd_lock);
static char fd_tx_msg[MAX_DATA_LEN + 1];

// IRQ 인터럽트 기반 수신 이벤트 큐 정의
static wait_queue_head_t rx_wq;
static atomic_t clk_event;
static int clk_in_irq = -1;

static int find_gpio_index(int bcm)
{
    int i;
    for (i = 0; i < MAX_GPIO; i++)
        if (gpio_table[i] && gpio_table[i]->bcm_num == bcm)
            return i;
    return -1;
}

// GPIO IRQ 인터럽트 핸들러
static irqreturn_t gpio_irq_handler(int irq, void *dev_id)
{
    struct gpio_entry *entry = dev_id;
    pr_info_ratelimited("[heartbeat_gpio] IRQ on GPIO %d\n", entry->bcm_num);
    if (entry->async_queue)
        kill_fasync(&entry->async_queue, SIGIO, POLL_IN);
    return IRQ_HANDLED;
}

//clk_in 인터럽트 핸들러
static irqreturn_t clk_in_irq_handler(int irq, void *dev_id)
{
    atomic_inc(&clk_event);
    wake_up_interruptible(&rx_wq);
    return IRQ_HANDLED;
}


// LSB-first 1바이트 문자 전송
static void send_char_desc(char c)
{
    int i;
    for (i = 0; i < 8; i++) {
        int bit = ((unsigned char)c >> i) & 1;
        pr_info("TX-BIT[%d]=%d\n", i, bit);
        gpiod_set_value(fd_tx_desc, bit);
        gpiod_set_value(fd_clk_out_desc, 1);
        udelay(500);
        gpiod_set_value(fd_clk_out_desc, 0);
        udelay(500);
        if (kthread_should_stop())
            return;
    }
    gpiod_set_value(fd_tx_desc, 0);
}

// 프레임 포함 메시지 전체 전송
static void send_frame_desc(const char *msg)
{
    unsigned char frame[2 + MAX_DATA_LEN + 1];
    size_t len = strnlen(msg, MAX_DATA_LEN);
    size_t i;

    frame[0] = START_BYTE;
    frame[1] = (unsigned char)len;
    memcpy(&frame[2], msg, len);
    frame[2 + len] = 0;
    for (i = 0; i < len; i++)
        frame[2 + len] ^= (unsigned char)msg[i];

    for (i = 0; i < 3 + len; i++) {
        send_char_desc(frame[i]);
        udelay(2000);
        if (kthread_should_stop())
            break;
    }
}

// 비트를 수신하고 받은 비트에 대해 로그 출력
static int recv_bit(void)
{
    int ret;
    ret = wait_event_interruptible_timeout(
        rx_wq, atomic_read(&clk_event) > 0, msecs_to_jiffies(WAIT_TIMEOUT_MS));
    if (ret <= 0)
        return -1;
    atomic_dec(&clk_event);
    int bit = gpiod_get_value(fd_rx_desc);
    pr_info("RX-BIT=%d\n", bit);
    return bit;
}

/*  1바이트 수신
    클럭 이벤트가 발생할 때까지 대기한 뒤 각 비트를 recv_bit()로 받아 1바이트 조립
*/
static char recv_char_desc_timeout(void)
{
    char result = 0;
    int i, bit;
    for (i = 0; i < 8; i++) {
        bit = recv_bit();
        if (bit < 0)
            return -1;
        result |= (bit << i);
        udelay(500);
        if (kthread_should_stop())
            break;
    }
    return result;
}


/*  송신용 커널 스레드
    공유 메시지 버퍼에 메시지가 대기 중이면 send_frame_desc()를 이용하여 전송
    메시지가 없으면 심박 펄스 송신
*/
static int fd_tx_thread_fn(void *data)
{
    struct timespec64 ts_real;
    pr_info("heartbeat_gpio: FD TX thread started\n");
    while (!kthread_should_stop()) {
        int bpm_int;
        char status[16];

        ktime_get_real_ts64(&ts_real);
        bpm_int = ts_real.tv_nsec % 200;
        if (bpm_int < 10)
            strncpy(status, "CRITICAL", sizeof(status));
        else if (bpm_int < 60)
            strncpy(status, "DANGER", sizeof(status));
        else if (bpm_int < 130)
            strncpy(status, "NORMAL", sizeof(status));
        else
            strncpy(status, "CRITICAL", sizeof(status));
        status[sizeof(status)-1] = '\0';

        pr_info_ratelimited(
            "heartbeat_gpio: Time=%lld.%09u, BPM=%d, status=\"%s\"\n",
            (long long)ts_real.tv_sec, (unsigned)ts_real.tv_nsec,
            bpm_int, status);

        mutex_lock(&fd_lock);
        if (fd_tx_msg[0]) {
            pr_info_ratelimited("TX: sending status \"%s\"\n", fd_tx_msg);
            send_frame_desc(fd_tx_msg);
            fd_tx_msg[0] = '\0';
        } else {
            gpiod_set_value(fd_tx_desc, 1);
            gpiod_set_value(fd_clk_out_desc, 1);
            udelay(500);
            gpiod_set_value(fd_clk_out_desc, 0);
            gpiod_set_value(fd_tx_desc, 0);
        }
        mutex_unlock(&fd_lock);

        if (bpm_int <= 0)
            msleep(1000);
        else {
            unsigned long sleep_ms = 60000UL /
                (bpm_int < MIN_BPM ? MIN_BPM : bpm_int);
            if (sleep_ms > MAX_SLEEP_MS)
                sleep_ms = MAX_SLEEP_MS;
            msleep(sleep_ms);
        }
    }
    pr_info("heartbeat_gpio: FD TX thread stopping\n");
    return 0;
}


/*  수신용 커널 스레드
    1. 10초 동안 펄스 간격을 측정
    2. 평균 BPM 계산 및 상태 메시지 준비 후 전송 요청
    3. 메시지 프레임이 들어오면 수신하여 Checksum으로 무결성을 검사하고, 로그 출력력
*/
static int fd_rx_thread_fn(void *data)
{
    pr_info("heartbeat_gpio: FD RX thread started\n");
    while (!kthread_should_stop()) {
        ktime_t start = ktime_get();
        ktime_t end = ktime_add_ns(start, (u64)MEASURE_SEC * 1000000000ULL);
        u64 timestamps[128];
        int count = 0;
        u64 start_ns = ktime_to_ns(start);

        // 심박 측정
        while (!ktime_after(ktime_get(), end) && count < ARRAY_SIZE(timestamps)) {
            if (recv_bit() < 0) {
                pr_info("RX: pulse wait timeout, count=%d\n", count);
                break;
            }
            timestamps[count] = ktime_to_ns(ktime_get()) - start_ns;
            pr_info_ratelimited("RX: pulse #%d at %llu.%02llus\n",
                count+1,
                (unsigned long long)(timestamps[count]/1000000000ULL),
                (unsigned long long)((timestamps[count]%1000000000ULL)/10000000ULL));
            count++;
            if (kthread_should_stop())
                break;
        }
        
    
        if (count >= 2) {
            u64 total_ns = timestamps[count-1] - timestamps[0];
            u64 num = (u64)(count-1) * 60ULL * 1000000000ULL;
            u64 bpm_int = total_ns ? num / total_ns : 0;
            const char *st;
            if (bpm_int < 10)       st = "CRITICAL";
            else if (bpm_int < 60)  st = "DANGER";
            else if (bpm_int < 130) st = "NORMAL";
            else                     st = "CRITICAL";
            pr_info("RX: avg BPM over %d sec = %llu\n", MEASURE_SEC, bpm_int);
            pr_info("TX: sending status \"%s\"\n", st);
            mutex_lock(&fd_lock);
            strncpy(fd_tx_msg, st, sizeof(fd_tx_msg)-1);
            fd_tx_msg[sizeof(fd_tx_msg)-1] = '\0';
            mutex_unlock(&fd_lock);
        } else {
            pr_info("RX: Not enough pulses (%d) for BPM calculation\n", count);
        }
     
        while (!kthread_should_stop()) {
            char byte = recv_char_desc_timeout();
            if (byte < 0) {
                pr_info("RX: waiting for START_BYTE timed out, retry cycle\n");
                break;
            }
            if (byte != START_BYTE)
                continue;

            /* length */
            int len_b = recv_char_desc_timeout();
            if (len_b < 0) {
                pr_info("RX: len byte timeout, abort\n");
                break;
            }
            unsigned char len = (unsigned char)len_b;
            if (len > MAX_DATA_LEN) {
                pr_info("RX: invalid len %u, abort\n", len);
                break;
            }
            char buf[MAX_DATA_LEN+1] = {0};
            int i;
            for (i = 0; i < len; i++) {
                int c = recv_char_desc_timeout();
                if (c < 0) { pr_info("RX: data byte timeout\n"); break; }
                buf[i] = (char)c;
            }
            if (i < len) break;

            // Checksum
            int crc_b = recv_char_desc_timeout();
            if (crc_b < 0) { pr_info("RX: Checksum timeout\n"); break; }
            {
                unsigned char crc = (unsigned char)crc_b;
                unsigned char calc = 0;
                for (i = 0; i < len; i++) calc ^= (unsigned char)buf[i];
                if (crc == calc)
                    pr_info("RX: received message=\"%s\" (OK)\n", buf);
                else
                    pr_info("RX: checksum mismatch recv=0x%02X calc=0x%02X\n",
                        crc, calc);
            }
            break;
        }
    }
    pr_info("heartbeat_gpio: FD RX thread stopping\n");
    return 0;
}

/*  파일 열기 함수
    inode의 minor 번호로 gpio talbe에서 entry 구초체를 얻어와 private date로 저장
*/
static int gpio_fops_open(struct inode *inode, struct file *filp) {
    int minor = iminor(inode);
    if (minor >= MAX_GPIO || !gpio_table[minor]) return -ENODEV;
    filp->private_data = gpio_table[minor];
    return 0;
}

/*  파일 닫기기 함수
    IRQ가 설정되었을 경우 해제하고, 큐 정리
*/
static int gpio_fops_release(struct inode *inode, struct file *filp) {
    struct gpio_entry *entry = filp->private_data;
    if (entry && entry->irq_enabled) {
        free_irq(entry->irq_num, entry);
        entry->irq_enabled = false;
    }
    fasync_helper(-1, filp, 0, &entry->async_queue);
    return 0;
}

// 비동기 신호 설정용 fasync 파일 연산자
static int gpio_fops_fasync(int fd, struct file *filp, int mode) {
    struct gpio_entry *entry = filp->private_data;
    return fasync_helper(fd, filp, mode, &entry->async_queue);
}

// GPIO 값을 읽어 사용자 공간에 전달
static ssize_t gpio_fops_read(struct file *filp, char __user *buf, size_t len, loff_t *off) {
    struct gpio_entry *entry = filp->private_data;
    char val = gpiod_get_value(entry->desc) ? '1' : '0';
    if (copy_to_user(buf, &val, 1)) return -EFAULT;
    return 1;
}

// GPIO에 값과 direction을 작성 (0, 1, in, out)
static ssize_t gpio_fops_write(struct file *filp, const char __user *buf, size_t len, loff_t *off) {
    struct gpio_entry *entry = filp->private_data;
    char kbuf[8] = {0};
    if (len >= sizeof(kbuf)) return -EINVAL;
    if (copy_from_user(kbuf, buf, len)) return -EFAULT;
    kbuf[len] = '\0';
    if (sysfs_streq(kbuf, "1")) {
        if (gpiod_get_direction(entry->desc)) return -EPERM;
        gpiod_set_value(entry->desc, 1);
    } else if (sysfs_streq(kbuf, "0")) {
        if (gpiod_get_direction(entry->desc)) return -EPERM;
        gpiod_set_value(entry->desc, 0);
    } else if (sysfs_streq(kbuf, "in")) {
        gpiod_direction_input(entry->desc);
    } else if (sysfs_streq(kbuf, "out")) {
        gpiod_direction_output(entry->desc, 0);
    } else {
        return -EINVAL;
    }
    return len;
}

// ioctl 시스템 콜 핸들러
// IRQ 설정, Full-Duplex 통신 설정 및 중단 등 다양한 제어 명령 처리
static long gpio_fops_ioctl(struct file *filp, unsigned int cmd, unsigned long arg) {
    struct gpio_entry *entry = filp->private_data;
    int irq;

    // IRQ 활성화
    if (cmd == GPIO_IOCTL_ENABLE_IRQ) {
        if (entry->irq_enabled) return -EBUSY;
        irq = gpiod_to_irq(entry->desc);
        if (irq < 0) return -EINVAL;
        if (request_irq(irq, gpio_irq_handler,
                        IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
                        "gpio_irq", entry)) {
            pr_err("[heartbeat_gpio] IRQ request failed\n");
            return -EIO;
        }
        entry->irq_num = irq;
        entry->irq_enabled = true;
        return 0;

    // IRQ 비활성화
    } else if (cmd == GPIO_IOCTL_DISABLE_IRQ) {
        if (!entry->irq_enabled) return -EINVAL;
        free_irq(entry->irq_num, entry);
        entry->irq_enabled = false;
        return 0;
    
    // 통신 설정
    } else if (cmd == GPIO_IOCTL_FD_CONFIG) {
        struct fd_config cfg;

        // 사용자 공간에서 설정 구조체 복사
        if (copy_from_user(&cfg, (void __user *)arg, sizeof(cfg))) return -EFAULT;

        // 기존 스레드 종료
        if (fd_tx_thread) kthread_stop(fd_tx_thread);
        if (fd_rx_thread) kthread_stop(fd_rx_thread);

        fd_tx_desc = gpio_to_desc(GPIOCHIP_BASE + cfg.tx_pulse);
        fd_clk_out_desc = gpio_to_desc(GPIOCHIP_BASE + cfg.clk_out);
        fd_clk_in_desc = gpio_to_desc(GPIOCHIP_BASE + cfg.clk_in);
        fd_rx_desc = gpio_to_desc(GPIOCHIP_BASE + cfg.rx_pulse);
        if (!fd_tx_desc || !fd_clk_out_desc || !fd_clk_in_desc || !fd_rx_desc) {
            pr_err("heartbeat_gpio: FD config invalid GPIOs\n");
            return -EINVAL;
        }
        
        gpiod_direction_output(fd_tx_desc, 0);
        gpiod_direction_output(fd_clk_out_desc, 0);
        gpiod_direction_input(fd_clk_in_desc);
        gpiod_direction_input(fd_rx_desc);

        // 수신 wait queue 초기화 및 이벤트 카운터 설정
        init_waitqueue_head(&rx_wq);
        atomic_set(&clk_event, 0);
        if (clk_in_irq >= 0) {
            free_irq(clk_in_irq, NULL);
            clk_in_irq = -1;
        }

        // clk_in에 대한 인터럽트 요청
        irq = gpiod_to_irq(fd_clk_in_desc);
        if (irq < 0) {
            pr_err("heartbeat_gpio: failed to get IRQ for CLK_IN\n");
            return -EINVAL;
        }
        if (request_irq(irq, clk_in_irq_handler,
                        IRQF_TRIGGER_RISING, "sysprog_gpio_clk_in", NULL)) {
            pr_err("heartbeat_gpio: request_irq for CLK_IN failed\n");
            return -EIO;
        }
        clk_in_irq = irq;

        mutex_lock(&fd_lock);
        fd_tx_msg[0] = '\0';
        mutex_unlock(&fd_lock);

        // 송신 및 수신 스레드 생성
        fd_tx_thread = kthread_run(fd_tx_thread_fn, NULL, "fd_tx");
        if (IS_ERR(fd_tx_thread)) {
            pr_err("heartbeat_gpio: failed to start FD TX thread\n");
            fd_tx_thread = NULL;
            free_irq(clk_in_irq, NULL);
            clk_in_irq = -1;
            return PTR_ERR(fd_tx_thread);
        }
        fd_rx_thread = kthread_run(fd_rx_thread_fn, NULL, "fd_rx");
        if (IS_ERR(fd_rx_thread)) {
            pr_err("heartbeat_gpio: failed to start FD RX thread\n");
            kthread_stop(fd_tx_thread);
            fd_tx_thread = NULL;
            free_irq(clk_in_irq, NULL);
            clk_in_irq = -1;
            return PTR_ERR(fd_rx_thread);
        }
        pr_info("heartbeat_gpio: Full-Duplex started on TX=%d CLK_OUT=%d CLK_IN=%d RX=%d\n",
                cfg.tx_pulse, cfg.clk_out, cfg.clk_in, cfg.rx_pulse);
        return 0;

    // 통신 중단
    } else if (cmd == GPIO_IOCTL_FD_STOP) {
        if (fd_tx_thread) kthread_stop(fd_tx_thread);
        if (fd_rx_thread) kthread_stop(fd_rx_thread);
        fd_tx_thread = fd_rx_thread = NULL;
        if (clk_in_irq >= 0) {
            free_irq(clk_in_irq, NULL);
            clk_in_irq = -1;
        }
        pr_info("heartbeat_gpio: Full-Duplex stopped\n");
        return 0;
    }
    return -ENOTTY;
}

// character device에서 사용할 파일 연산자 구조체 정의
static const struct file_operations gpio_fops = {
    .owner = THIS_MODULE,
    .open = gpio_fops_open,
    .read = gpio_fops_read,
    .write = gpio_fops_write,
    .release = gpio_fops_release,
    .fasync = gpio_fops_fasync,
    .unlocked_ioctl = gpio_fops_ioctl,
};

// sysfs를 통해 GPIO의 value 읽기
static ssize_t value_show(struct device *dev, struct device_attribute *attr, char *buf) {
    struct gpio_entry *entry = dev_get_drvdata(dev);
    int val = gpiod_get_value(entry->desc);
    return scnprintf(buf, PAGE_SIZE, "%d\n", val);
}

// sysfs를 통해 GPIO의 value 쓰기
static ssize_t value_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
    struct gpio_entry *entry = dev_get_drvdata(dev);
    if (gpiod_get_direction(entry->desc)) return -EPERM;
    if (buf[0] == '1') gpiod_set_value(entry->desc, 1);
    else if (buf[0] == '0') gpiod_set_value(entry->desc, 0);
    else return -EINVAL;
    return count;
}

// sysfs를 통해 GPIO의 direction 읽기
static ssize_t direction_show(struct device *dev, struct device_attribute *attr, char *buf) {
    struct gpio_entry *entry = dev_get_drvdata(dev);
    int dir = gpiod_get_direction(entry->desc);
    return scnprintf(buf, PAGE_SIZE, "%s\n", dir ? "in" : "out");
}

// // sysfs를 통해 GPIO의 direction 설정
static ssize_t direction_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
    struct gpio_entry *entry = dev_get_drvdata(dev);
    if (sysfs_streq(buf, "in")) gpiod_direction_input(entry->desc);
    else if (sysfs_streq(buf, "out")) gpiod_direction_output(entry->desc, 0);
    else return -EINVAL;
    return count;
}

static DEVICE_ATTR_RW(value);
static DEVICE_ATTR_RW(direction);

// 지정한 GPIO를 커널에 등록하고 sysfs entry 생성
static ssize_t export_store(const struct class *class, const struct class_attribute *attr, const char *buf, size_t count) {
    int bcm, idx, kernel_gpio;
    struct gpio_entry *entry;
    char name[16];
    if (kstrtoint(buf, 10, &bcm) < 0) return -EINVAL;
    if (find_gpio_index(bcm) >= 0) return -EEXIST;
    for (idx = 0; idx < MAX_GPIO; idx++) if (!gpio_table[idx]) break;
    if (idx == MAX_GPIO) return -ENOMEM;
    entry = kzalloc(sizeof(*entry), GFP_KERNEL);
    if (!entry) return -ENOMEM;
    kernel_gpio = GPIOCHIP_BASE + bcm;
    entry->bcm_num = bcm;
    entry->desc = gpio_to_desc(kernel_gpio);
    if (!entry->desc) { kfree(entry); return -EINVAL; }
    gpiod_direction_input(entry->desc);
    snprintf(name, sizeof(name), "gpio%d", bcm);
    entry->dev = device_create(gpiod_class, NULL, MKDEV(major_num, idx), NULL, name);
    if (IS_ERR(entry->dev)) { kfree(entry); return PTR_ERR(entry->dev); }
    dev_set_drvdata(entry->dev, entry);
    device_create_file(entry->dev, &dev_attr_value);
    device_create_file(entry->dev, &dev_attr_direction);
    gpio_table[idx] = entry;
    pr_info("[heartbeat_gpio] Exported GPIO %d\n", bcm);
    return count;
}

// 지정한 GPIO를 커널에서 해제하고 메모리 반환
static ssize_t unexport_store(const struct class *class, const struct class_attribute *attr, const char *buf, size_t count) {
    int bcm, idx;
    if (kstrtoint(buf, 10, &bcm) < 0) return -EINVAL;
    idx = find_gpio_index(bcm);
    if (idx < 0) return -ENOENT;
    device_remove_file(gpio_table[idx]->dev, &dev_attr_value);
    device_remove_file(gpio_table[idx]->dev, &dev_attr_direction);
    device_destroy(gpiod_class, MKDEV(major_num, idx));
    kfree(gpio_table[idx]);
    gpio_table[idx] = NULL;
    pr_info("[heartbeat_gpio] Unexported GPIO %d\n", bcm);
    return count;
}

static CLASS_ATTR_WO(export);
static CLASS_ATTR_WO(unexport);

/*  모듈 초기화 함수
    클래스 생성, export/unexport 파일 생성
    캐릭터 디바이스 번호 할당 및 cdev 등록
    전역 변수 초기화
*/
static int __init gpio_driver_init(void) {
    int ret;
    pr_info("[heartbeat_gpio] module loading\n");
    gpiod_class = class_create(CLASS_NAME);
    if (IS_ERR(gpiod_class)) return PTR_ERR(gpiod_class);
    ret = class_create_file(gpiod_class, &class_attr_export);
    if (ret) return ret;
    ret = class_create_file(gpiod_class, &class_attr_unexport);
    if (ret) return ret;
    ret = alloc_chrdev_region(&dev_num_base, 0, MAX_GPIO, "gpio");
    if (ret) return ret;
    major_num = MAJOR(dev_num_base);
    cdev_init(&gpio_cdev, &gpio_fops);
    gpio_cdev.owner = THIS_MODULE;
    ret = cdev_add(&gpio_cdev, dev_num_base, MAX_GPIO);
    if (ret) return ret;
    clk_in_irq = -1;
    init_waitqueue_head(&rx_wq);
    atomic_set(&clk_event, 0);
    return 0;
}

/*  모듈 종료 함수
    생성된 스레드 종료
    IRQ 해제
    등록된 GPIO 및 sysfs 속성 제거
    캐릭터 디바이스 삭제 및 클래스 제거
*/
static void __exit gpio_driver_exit(void) {
    int i;
    if (fd_tx_thread) kthread_stop(fd_tx_thread);
    if (fd_rx_thread) kthread_stop(fd_rx_thread);
    if (clk_in_irq >= 0) {
        free_irq(clk_in_irq, NULL);
        clk_in_irq = -1;
    }
    for (i = 0; i < MAX_GPIO; i++) {
        if (gpio_table[i]) {
            device_remove_file(gpio_table[i]->dev, &dev_attr_value);
            device_remove_file(gpio_table[i]->dev, &dev_attr_direction);
            device_destroy(gpiod_class, MKDEV(major_num, i));
            kfree(gpio_table[i]);
            gpio_table[i] = NULL;
        }
    }
    cdev_del(&gpio_cdev);
    unregister_chrdev_region(dev_num_base, MAX_GPIO);
    class_remove_file(gpiod_class, &class_attr_export);
    class_remove_file(gpiod_class, &class_attr_unexport);
    class_destroy(gpiod_class);
    pr_info("[heartbeat_gpio] module unloaded\n");
}

module_init(gpio_driver_init);
module_exit(gpio_driver_exit);

MODULE_LICENSE("GPL");