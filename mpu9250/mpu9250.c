#include <asm-generic/siginfo.h>
#include <asm-generic/errno-base.h>
#include <asm-generic/signal-defs.h>

#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/kern_levels.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/of_irq.h>
#include <linux/moduleparam.h>

#include <linux/time.h>
#include <linux/types.h>
#include <linux/irqreturn.h>
#include <linux/device.h>
#include <linux/gfp.h>
#include <linux/signal.h>
#include <asm-generic/errno.h>
#include <asm/thread_info.h>
#include <linux/sched.h>
#include <linux/signalfd.h>
#include <asm/siginfo.h>
#include <linux/pid.h>
#include "mpu9250.h"

enum mpu9250_register {
    MPU9250_SELF_TEST_X_GYRO =  0x00,
    MPU9250_SELF_TEST_Y_GYRO =  0x01,
    MPU9250_SELF_TEST_Z_GYRO =  0x02,
    MPU9250_SELF_TEST_X_ACCEL = 0x0D,
    MPU9250_SELF_TEST_Y_ACCEL = 0x0E,
    MPU9250_SELF_TEST_Z_ACCEL = 0x0F,
    MPU9250_XG_OFFSET_H =       0x13,
    MPU9250_XG_OFFSET_L =       0x14,
    MPU9250_YG_OFFSET_H =       0x15,
    MPU9250_YG_OFFSET_L =       0x16,
    MPU9250_ZG_OFFSET_H =       0x17,
    MPU9250_ZG_OFFSET_L =       0x18,
    MPU9250_SMPLRT_DIV =        0x19,
    MPU9250_CONFIG =            0x1A,
    MPU9250_GYRO_CONFIG =       0x1B,
    MPU9250_ACCEL_CONFIG =      0x1C,
    MPU9250_ACCEL_CONFIG_2 =    0x1D,
    MPU9250_LP_ACCEL_ODR =      0x1E,
    MPU9250_WOM_THR =           0x1F,
    MPU9250_FIFO_EN =           0x23,
    MPU9250_I2C_MST_CTRL =      0x24,
    MPU9250_I2C_SLV0_ADDR =     0x25,
    MPU9250_I2C_SLV0_REG =      0x26,
    MPU9250_I2C_SLV0_CTRL =     0x27,
    MPU9250_I2C_SLV1_ADDR =     0x28,
    MPU9250_I2C_SLV1_REG =      0x29,
    MPU9250_I2C_SLV1_CTRL =     0x2A,
    MPU9250_I2C_SLV2_ADDR =     0x2B,
    MPU9250_I2C_SLV2_REG =      0x2C,
    MPU9250_I2C_SLV2_CTRL =     0x2D,
    MPU9250_I2C_SLV3_ADDR =     0x2E,
    MPU9250_I2C_SLV3_REG =      0x2F,
    MPU9250_I2C_SLV3_CTRL =     0x30,
    MPU9250_I2C_SLV4_ADDR =     0x31,
    MPU9250_I2C_SLV4_REG =      0x32,
    MPU9250_I2C_SLV4_DO =       0x33,
    MPU9250_I2C_SLV4_CTRL =     0x34,
    MPU9250_I2C_SLV4_DI =       0x35,
    MPU9250_I2C_MST_STATUS =    0x36,
    MPU9250_INT_PIN_CFG =       0x37,
    MPU9250_INT_ENABLE =        0x38,
    MPU9250_INT_STATUS =        0x3A,
    MPU9250_ACCEL_XOUT_H =      0x3B,
    MPU9250_ACCEL_XOUT_L =      0x3C,
    MPU9250_ACCEL_YOUT_H =      0x3D,
    MPU9250_ACCEL_YOUT_L =      0x3E,
    MPU9250_ACCEL_ZOUT_H =      0x3F,
    MPU9250_ACCEL_ZOUT_L =      0x40,
    MPU9250_TEMP_OUT_H =        0x41,
    MPU9250_TEMP_OUT_L =        0x42,
    MPU9250_GYRO_XOUT_H =       0x43,
    MPU9250_GYRO_XOUT_L =       0x44,
    MPU9250_GYRO_YOUT_H =       0x45,
    MPU9250_GYRO_YOUT_L =       0x46,
    MPU9250_GYRO_ZOUT_H =       0x47,
    MPU9250_GYRO_ZOUT_L =       0x48,
    MPU9250_EXT_SENS_DATA_00 =  0x49,
    MPU9250_EXT_SENS_DATA_01 =  0x4A,
    MPU9250_EXT_SENS_DATA_02 =  0x4B,
    MPU9250_EXT_SENS_DATA_03 =  0x4C,
    MPU9250_EXT_SENS_DATA_04 =  0x4D,
    MPU9250_EXT_SENS_DATA_05 =  0x4E,
    MPU9250_EXT_SENS_DATA_06 =  0x4F,
    MPU9250_EXT_SENS_DATA_07 =  0x50,
    MPU9250_EXT_SENS_DATA_08 =  0x51,
    MPU9250_EXT_SENS_DATA_09 =  0x52,
    MPU9250_EXT_SENS_DATA_10 =  0x53,
    MPU9250_EXT_SENS_DATA_11 =  0x54,
    MPU9250_EXT_SENS_DATA_12 =  0x55,
    MPU9250_EXT_SENS_DATA_13 =  0x56,
    MPU9250_EXT_SENS_DATA_14 =  0x57,
    MPU9250_EXT_SENS_DATA_15 =  0x58,
    MPU9250_EXT_SENS_DATA_16 =  0x59,
    MPU9250_EXT_SENS_DATA_17 =  0x5A,
    MPU9250_EXT_SENS_DATA_18 =  0x5B,
    MPU9250_EXT_SENS_DATA_19 =  0x5C,
    MPU9250_EXT_SENS_DATA_20 =  0x5D,
    MPU9250_EXT_SENS_DATA_21 =  0x5E,
    MPU9250_EXT_SENS_DATA_22 =  0x5F,
    MPU9250_EXT_SENS_DATA_23 =  0x60,
    MPU9250_I2C_SLV0_DO =       0x63,
    MPU9250_I2C_SLV1_DO =       0x64,
    MPU9250_I2C_SLV2_DO =       0x65,
    MPU9250_I2C_SLV3_DO =       0x66,
    MPU9250_I2C_MST_DELAY_CTRL =0x67,
    MPU9250_SIGNAL_PATH_RESET = 0x68,
    MPU9250_MOT_DETECT_CTRL =   0x69,
    MPU9250_USER_CTRL =         0x6A,
    MPU9250_PWR_MGMT_1 =        0x6B,
    MPU9250_PWR_MGMT_2 =        0x6C,
    MPU9250_FIFO_COUNTH =       0x72,
    MPU9250_FIFO_COUNTL =       0x73,
    MPU9250_FIFO_R_W =          0x74,
    MPU9250_WHO_AM_I =          0x75,
    MPU9250_XA_OFFSET_H =       0x77,
    MPU9250_XA_OFFSET_L =       0x78,
    MPU9250_YA_OFFSET_H =       0x7A,
    MPU9250_YA_OFFSET_L =       0x7B,
    MPU9250_ZA_OFFSET_H =       0x7D,
    MPU9250_ZA_OFFSET_L =       0x7E
};

#define DRIVER_NAME "mpu9250"

#define REGISTER_COUNT 5
#define BUFFER_SIZE_CHAR (REGISTER_COUNT * sizeof(uint32_t))

int const CHAR_AMOUNT_BUFFER = BUFFER_SIZE_CHAR;

// hex values for floating point form
// from https://godbolt.org/#g:!((g:!((g:!((h:codeEditor,i:(filename:'1',fontScale:14,fontUsePx:'0',j:1,lang:c%2B%2B,selection:(endColumn:33,endLineNumber:12,positionColumn:33,positionLineNumber:12,selectionStartColumn:33,selectionStartLineNumber:12,startColumn:33,startLineNumber:12),source:'%23include+%3Ciostream%3E%0A%0Anamespace+%7B%0A++++float+const+divVal+%3D+16.4f%3B%0A++++float+const+mpuMul+%3D+0.6f%3B%0A%7D%0A%0Aint+main()+%7B%0A++++auto+ptr+%3D+reinterpret_cast%3Cint32_t+const*%3E(%26divVal)%3B%0A++++std::printf(%220x%25x%5Cn%22,+*ptr)%3B%0A++++ptr+%3D+reinterpret_cast%3Cint32_t+const*%3E(%26mpuMul)%3B%0A++++std::printf(%220x%25x%5Cn%22,+*ptr)%3B%0A++++return+0%3B%0A%7D%0A'),l:'5',n:'0',o:'C%2B%2B+source+%231',t:'0')),k:33.333333333333336,l:'4',n:'0',o:'',s:0,t:'0'),(g:!((h:compiler,i:(compiler:g112,filters:(b:'0',binary:'1',commentOnly:'0',demangle:'0',directives:'0',execute:'0',intel:'0',libraryCode:'0',trim:'1'),flagsViewOpen:'1',fontScale:14,fontUsePx:'0',j:1,lang:c%2B%2B,libs:!(),options:'-O3',selection:(endColumn:1,endLineNumber:1,positionColumn:1,positionLineNumber:1,selectionStartColumn:1,selectionStartLineNumber:1,startColumn:1,startLineNumber:1),source:1,tree:'1'),l:'5',n:'0',o:'x86-64+gcc+11.2+(C%2B%2B,+Editor+%231,+Compiler+%231)',t:'0')),k:33.333333333333336,l:'4',n:'0',o:'',s:0,t:'0'),(g:!((h:output,i:(compiler:1,editor:1,fontScale:14,fontUsePx:'0',tree:'1',wrap:'1'),l:'5',n:'0',o:'Output+of+x86-64+gcc+11.2+(Compiler+%231)',t:'0')),k:33.33333333333333,l:'4',n:'0',o:'',s:0,t:'0')),l:'2',n:'0',o:'',t:'0')),version:4
static int GYRO_DIV = 0x42033333; // 32.8f in hex
module_param(GYRO_DIV, int, 0444);
static int ACCL_DIV = 4096;
module_param(ACCL_DIV, int, 0444);
static int MAGN_MUL = 0x3f19999a; // 0.6f in hex
module_param(MAGN_MUL, int, 0444);

static short BUMP_HI = 26000;
module_param(BUMP_HI, short, 0444);
static short BUMP_LO = -26000;
module_param(BUMP_LO, short, 0444);

struct mpu9250_device_t {
	void * regs;
	// "Dishonour on you! Dishonour on your family! Dishonour on your cow!" - Mushu, 1996
	union {
		char buffer[BUFFER_SIZE_CHAR];
		mpu9250_data_t data;
	};
	int size;
	struct miscdevice misc;
	int irq;

	// for bump detection
	/** lock for pid, irq & data */
	spinlock_t lock;
	/** the pid of the task we send the signal to */
	pid_t pid;
	/** the signal we send to the pid */
	int signal;
	bumpData_t * bumpData;
};

#undef BUFFER_SIZE_CHAR
#undef REGISTER_COUNT

// ============================================================================

void writeToAddress8(uint8_t registerAddress, uint8_t data0, struct mpu9250_device_t * dev) {
	static uint32_t const DATA1_PRESENT_MASK = 1 << 24;
	// make sure we do not set DATA1 present.
	uint32_t packet = (~DATA1_PRESENT_MASK) & (data0 << 8 | (registerAddress & 0b01111111));
	memcpy_toio(dev->regs, &packet, sizeof(packet));
}

void writeToAddress16(uint8_t registerAddress, uint16_t value, struct mpu9250_device_t * dev) {
	static uint32_t const DATA1_PRESENT_MASK = 1 << 24;
	uint32_t packet = DATA1_PRESENT_MASK | value << 8 | (registerAddress & 0b01111111);
	memcpy_toio(dev->regs, &packet, sizeof(packet));
}

uint32_t readFromAddress(uint8_t registerAddress, struct mpu9250_device_t * dev) {
	uint32_t reti = 0;
	uint32_t addr = registerAddress & 0b01111111;
	memcpy_toio(dev->regs + 24, &addr, sizeof(addr));
	while(ioread32(dev->regs + 40) == 0); // wait until data ready
	memcpy_fromio(&reti, dev->regs + 44, sizeof(reti));
	return reti;
}

// ============================================================================

static irqreturn_t bumpDetected_threadedHandler(int irq_line, void * arg) {
	struct platform_device * const pdev = arg;
	struct mpu9250_device_t * const device = platform_get_drvdata(pdev);
	siginfo_t sig;
	pid_t pid;
	int signal;
	struct task_struct * task;
	BUG_ON(device == NULL);
	dev_info(&pdev->dev, "threadedHandler | irq_line=%d, irq=%d", irq_line, device->irq);
	memset(&sig, 0, sizeof(sig));

	if (irq_line != device->irq) {
		return IRQ_NONE;
	}
	spin_lock(&device->lock);
	sig.si_signo = device->signal;
	signal = device->signal;
	sig.si_code = SI_QUEUE;
	pid = device->pid;
	spin_unlock(&device->lock);

	task = pid_task(find_vpid(pid), PIDTYPE_PID);
	if(task == NULL)
	{
		dev_info(&pdev->dev, "PID=%d not found", pid);
		return IRQ_HANDLED;
	}

	if (send_sig_info(signal, &sig, task)) {
		dev_info(&pdev->dev, "failed to signal task");
	}

	return IRQ_HANDLED;
}

static irqreturn_t bumpDetected_hardirq(int irq_line, void * arg) {
	static int unsigned currentDataIndex = 0u;
	static DEFINE_SPINLOCK(hardirq_lock);

	struct platform_device * const pdev = arg;
	struct mpu9250_device_t * device = platform_get_drvdata(pdev);
	int unsigned ii = 0;
	int * data = NULL;

	spin_lock_irq(&hardirq_lock);
	BUG_ON(currentDataIndex > 3u);

	dev_info(&pdev->dev, "Bump detected (%d) %d | %d", currentDataIndex, device->irq, irq_line);

	if (irq_line != device->irq) {
		return IRQ_NONE;
	}

	if (currentDataIndex == 0) {
		device->bumpData->timestamp_ns = ktime_get_real() - (265 * 1000000) /* number of ns in a ms */;
	}

	// get data from FPGA
	for (ii = 0; ii < 256; ii++) {
		data = (int *) &device->bumpData->data[currentDataIndex][ii].data.accel;
		*(data + 0) = ioread32(device->regs + 5 * sizeof(int));
		*(data + 1) = ioread32(device->regs + 6 * sizeof(int));
		*(data + 2) = ioread32(device->regs + 7 * sizeof(int));
		*(data + 3) = ioread32(device->regs + 8 * sizeof(int));
		*(data + 4) = ioread32(device->regs + 9 * sizeof(int));
	}
	currentDataIndex++;

	// ack interrupt, this will let the FPGA pull the line low again
	// unless another interrupt occured in the meantime, which makes
	// it so we are immediately run again.
	iowrite32(0, device->regs + 3 * sizeof(int));

	if (currentDataIndex == 4) {
		currentDataIndex = 0;
		spin_unlock_irq(&hardirq_lock);
		return IRQ_WAKE_THREAD;
	}

	spin_unlock_irq(&hardirq_lock);

	return IRQ_HANDLED;
}

// ============================================================================

// file operations
static ssize_t device_read(struct file *, char *, size_t, loff_t *);
static ssize_t device_write(struct file *, const char *, size_t, loff_t *);
static int device_open(struct inode *, struct file *);
static int device_release(struct inode *, struct file *);
static loff_t device_llseek(struct file *, loff_t, int);
static long device_ioctl(struct file *, int unsigned, long unsigned);

static const struct file_operations fops = {
	.owner = THIS_MODULE,
	.read = device_read,
	.write = device_write,
	.open = device_open,
	.release = device_release,
	.llseek = device_llseek,
	.unlocked_ioctl = device_ioctl,
};

// basic layout taken from skull device driver
static loff_t device_llseek(struct file * file, loff_t offset, int origin) {
	struct inode * inode = NULL;
	inode = file->f_mapping->host;
	switch (origin) {
		case SEEK_END:
			offset += inode->i_size;
			break;
		case SEEK_CUR:
			if (offset == 0) {
				return file->f_pos;
			}
			break;
	}

	if (offset < 0 || offset > inode->i_sb->s_maxbytes) {
		return -EINVAL;
	}
	mutex_lock(&file->f_pos_lock);
	if (offset != file->f_pos) {
		file->f_pos = offset;
		file->f_version = 0;
	}
	mutex_unlock(&file->f_pos_lock);

	return offset;
}

static int device_probe(struct platform_device *pdev) {
	static const uint8_t AK8963_I2C_ADDRESS = 0x0C;
	struct mpu9250_device_t * device;
	struct resource * io;
	int bump_interrupt;
	int retVal;
	short bump_upper = max(BUMP_HI, BUMP_LO);
	short bump_lower = min(BUMP_HI, BUMP_LO);

	pr_info("mpu9250_probe hit\n");

	device = devm_kzalloc(&pdev->dev, sizeof(*device), GFP_KERNEL);
	if (device == NULL) {
		return -ENOMEM;
	}
	memset(device, 0, sizeof(*device));

	device->bumpData = devm_kzalloc(&pdev->dev, sizeof(*(device->bumpData)), GFP_KERNEL);
	if (device == NULL) {
		return -ENOMEM;
	}

	spin_lock_init(&device->lock);

	platform_set_drvdata(pdev, device);

	io = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	device->regs = devm_ioremap_resource(&pdev->dev, io);
	if (IS_ERR(device->regs)) {
		return PTR_ERR(device->regs);
	}
	device->size = io->end - io->start + 1; // [start, end[ <- exclusive end!

	device->misc.name = DRIVER_NAME;
	device->misc.parent = &pdev->dev;
	device->misc.fops = &fops;
	device->misc.minor = MISC_DYNAMIC_MINOR;

	retVal = misc_register(&device->misc);
	if (retVal != 0) {
		dev_err(&pdev->dev, "misc_register failed\n");
		return retVal;
	}

	dev_info(&pdev->dev, "mpu9250 driver registered with size=%d\n", device->size);

	// initialize the device - honestly: I hope whoever made the MPU datasheet conciously
	// made it so bad so they never would be asked to make a datasheet again.
	writeToAddress8(MPU9250_I2C_SLV0_CTRL, 0x00, device); msleep(2); // disable i2c peripheral

	writeToAddress8(MPU9250_PWR_MGMT_1, 0b10000000, device); msleep(1); // reset device H_RESET=1
	writeToAddress8(MPU9250_PWR_MGMT_1, 0x01, device); msleep(1); // best select clock source, PLL else internal oscillator
	writeToAddress8(MPU9250_USER_CTRL, 0b00110000, device); msleep(1); // I2C_MST_EN | I2C_IF_DIS
	writeToAddress8(MPU9250_CONFIG, 0x01, device); msleep(1); // gyro 184Hz, temp 188Hz
	writeToAddress8(MPU9250_SMPLRT_DIV, 0, device); msleep(1); // do not use internal sample rate division
	writeToAddress8(MPU9250_GYRO_CONFIG, 0x0, device); msleep(1); // ±250º/s -> val /= GYRO_DIV (32.8f for ±250º/s)
	writeToAddress8(MPU9250_ACCEL_CONFIG, 0b00010000, device); msleep(1); // ±8g -> val /= ACC_DIV (4096f for ±8g)
	writeToAddress8(MPU9250_I2C_MST_CTRL, 0x0D, device); msleep(1); // 400 kHz I2C master clock speed

	// setup writing to magnetometers control registers
	writeToAddress8(MPU9250_I2C_SLV0_ADDR, AK8963_I2C_ADDRESS, device); msleep(1); // write to AK8963
	writeToAddress8(MPU9250_I2C_SLV0_REG, 0x0B, device); msleep(1); // write to AK8964 CNTL2
	writeToAddress8(MPU9250_I2C_SLV0_DO, 0x01, device); msleep(1); // reset AK8964 SRST=1
	writeToAddress8(MPU9250_I2C_SLV0_CTRL, 0x81, device); msleep(2); // I2C_SLV0_EN | read 1 byte

	writeToAddress8(MPU9250_I2C_SLV0_ADDR, AK8963_I2C_ADDRESS, device); msleep(1); // write to AK8963
	writeToAddress8(MPU9250_I2C_SLV0_REG, 0x0A, device); msleep(1); // write to AK8964 CNTL1
	writeToAddress8(MPU9250_I2C_SLV0_DO, 1 << 4 | 0b0110, device); msleep(1); // 16 bit mode | continuous measurement mode 2
	writeToAddress8(MPU9250_I2C_SLV0_CTRL, 0b10000001, device); msleep(2); // I2C_SLV0_EN | read 1 byte

	// do a first read of all the data
	writeToAddress8(MPU9250_I2C_SLV0_ADDR, 0b10000000 | AK8963_I2C_ADDRESS, device); msleep(1); // read from AK8963
	writeToAddress8(MPU9250_I2C_SLV0_REG, 0x03, device); msleep(1); // read from [HXL]
	writeToAddress8(MPU9250_I2C_SLV0_CTRL, 0x87, device); msleep(100); // I2C_SLV0_EN | read 6 + 1 bytes

	// register bump detection
	bump_interrupt = platform_get_irq(pdev, 0);
	if (bump_interrupt < 0) {
		dev_err(&pdev->dev, "recieved negative interrupt number");
		return bump_interrupt;
	}
	device->irq = bump_interrupt;

	// set thresholds before registering the interrupt
	iowrite32(bump_lower << 16 | bump_upper, device->regs + 16);
	iowrite32(bump_lower << 16 | bump_upper, device->regs + 20);
	iowrite32(bump_lower << 16 | bump_upper, device->regs + 28);

	if (devm_request_threaded_irq(device->misc.this_device,
			bump_interrupt,
			bumpDetected_hardirq,
			bumpDetected_threadedHandler,
			0, // device tree tells us how it is triggered
			"MPU9250_bump_detection",
			pdev)) {
		dev_err(&pdev->dev, "Failed to register bump detection.");
	}

	dev_info(&pdev->dev, "Registered bump detection on irq line %d. LO=%d HI=%d", bump_interrupt, bump_lower, bump_upper);

	return 0;
}

static int device_remove(struct platform_device *pdev) {
	struct mpu9250_device_t * device = platform_get_drvdata(pdev);
	dev_info(&pdev->dev, "mpu9250_remove hit\n");

	misc_deregister(&device->misc);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static atomic_t activeFileOpen = ATOMIC_INIT(0); // prevent data races

// fileoperations
static int device_open(struct inode * inode,
						struct file * filep) {
	pr_info("device_open");

	if (!atomic_add_unless(&activeFileOpen, 1, 1)) {
		pr_info("file is already opened by someone else");
		return -EBUSY;
	}

	return 0;
}

static int device_release(struct inode * inode,
						struct file * filep) {
	pr_info("device_release");

	atomic_dec(&activeFileOpen);

	return 0;
}

static ssize_t device_read(struct file *filep,
							char * buffer,
							size_t length,
							loff_t * offset) {
	ktime_t ts;
	struct mpu9250_device_t * device = container_of(filep->private_data,
											struct mpu9250_device_t,
											misc);
	size_t const axisTotalBytes = sizeof(mpu9250_data_t);
	size_t const totalBytesToUser = axisTotalBytes + sizeof(ts);
	size_t notCopiedBytes = 0;
	size_t returnedBytesToUser = 0;

	if (offset == NULL) {
		pr_debug("read| offset is null\n");
		return 0;
	}

	// do not write beyond buffer
	if ((*(offset) + length) > totalBytesToUser) {
		length = totalBytesToUser - *offset;
	}

	/*
	pr_info("read|device->regs=%p,length=%d,offset=%llu,seek=%llu\n",
									device->regs, length, *offset, filep->f_pos);
	//*/

	if (length >= sizeof(ts)) {
		ts = ktime_get_real();
		notCopiedBytes = copy_to_user(buffer + *offset,
									&ts,
									sizeof(ts));
		returnedBytesToUser += (sizeof(ts) - notCopiedBytes);
		*offset += (sizeof(ts) - notCopiedBytes);
		//pr_info("read|(time)copied bytes=%d\n", sizeof(ts) - notCopiedBytes);
	}

	// if (dont write beyond buffer)
	if (length >= axisTotalBytes) {
		memcpy_fromio(device->buffer, device->regs, CHAR_AMOUNT_BUFFER);
		notCopiedBytes = copy_to_user(buffer + *offset,
									device->buffer,
									axisTotalBytes);
		returnedBytesToUser += (axisTotalBytes - notCopiedBytes);
		*offset += axisTotalBytes - notCopiedBytes;
		/*
		pr_info("read|(axis)copied bytes=%d\n", axisTotalBytes - notCopiedBytes);
		pr_info("data|   X   |   Y   |   Z\n"
							"acc |%6hd |%6hd | %6hd\n"
							"gyro|%6hd |%6hd | %6hd\n"
							"magn|%6hd |%6hd | %6hd\n",
							device->data.accel.x, device->data.accel.y, device->data.accel.z,
							device->data.gyro.x, device->data.gyro.y, device->data.gyro.z,
							device->data.mag.x, device->data.mag.y, device->data.mag.z);
		//*/
	}

	//pr_info("read|returned length=%lld/%u", *offset, totalBytesToUser);

	return returnedBytesToUser;
}

static ssize_t device_write(struct file * filep,
							char const * buffer,
							size_t length,
							loff_t * offset) {
	return -ENOSYS;
}

static long device_ioctl(struct file * filep, int unsigned cmd, long unsigned arg_raw) {
	int const arg = arg_raw;
	void __user * const data = (void *) arg_raw;
	struct mpu9250_device_t * device = container_of(filep->private_data, struct mpu9250_device_t, misc);

	pr_info("ioctl cmd=%d", _IOC_NR(cmd));

	if(_IOC_TYPE(cmd) != IOCTL_MPU9250_MAGIC) {
		pr_info("ioctl magic check failed");
		return -ENOTTY;
	}

	switch(cmd) {
		case IOCTL_MPU9250_REGISTER_PID:
			if (!capable(CAP_SYS_ADMIN)) {
				return -EPERM;
			}
			if (!(0 <= arg && arg < 256)) {
				pr_info("requested signal is out of range (%d)", arg);
				return -EINVAL;
			}
			spin_lock(&device->lock);
			device->signal = arg;
			device->pid = current_thread_info()->task->pid;
			spin_unlock(&device->lock);
			break;
		case IOCTL_MPU9250_GIB_DATA:
			if (!capable(CAP_SYS_ADMIN)) {
				return -EPERM;
			}
			if (copy_to_user(data, device->bumpData, sizeof(*device->bumpData))) {
				pr_err("Could not copy all data to user");
				return -ENOMEM;
			}
			break;
		default:
			return -ENOTTY;
	}

	return 0;
}

// ============================================================================
// Define for which hardware version this driver is useful for

struct of_device_id mpu9250_dt_ids[] = {
	{
		.compatible = "breiti,mpu9250-1.0",
	},
	{
		.compatible = "hof,mpu9250-1.0",
	},
	{/* end of table */ }
};
MODULE_DEVICE_TABLE(of, mpu9250_dt_ids);

// ============================================================================
// Define for which hardware platfrom this driver is useful for

static struct platform_driver mpu9250_platform_driver = {
	.probe = device_probe,
	.remove = device_remove,
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(mpu9250_dt_ids)
	}
};

module_platform_driver(mpu9250_platform_driver);

// ============================================================================

MODULE_AUTHOR("esd2021 Grp1");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("milestone 4");

// ============================================================================
