#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/uaccess.h>

#define DRIVER_NAME "sevensegment"

#define SEGMENT_COUNT 6
#define REGISTER_COUNT (SEGMENT_COUNT + 2)

#define PWM_OFFSET 4
#define ENABLE_OFFSET 8

// buffer[7:8] are configuration register
struct altera_sevenseg_t {
	void * regs;
	char buffer[REGISTER_COUNT];
	int size;
	struct miscdevice misc;
};

// file operations
static ssize_t device_read(struct file *, char *, size_t, loff_t *);
static ssize_t device_write(struct file *, const char *, size_t, loff_t *);
static int device_open(struct inode *, struct file *);
static int device_release(struct inode *, struct file *);

static const struct file_operations fops = {
	.owner = THIS_MODULE,
	.read = device_read,
	.write = device_write,
	.open = device_open,
	.release = device_release
};

static int sevenseg_probe(struct platform_device *pdev) {
	struct altera_sevenseg_t * sevenseg;
	struct resource * io;
	int retVal;

	pr_info("sevenseg_probe hit\n");

	sevenseg = devm_kzalloc(&pdev->dev, sizeof(*sevenseg), GFP_KERNEL);
	if (sevenseg == NULL) {
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, sevenseg);

	io = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	sevenseg->regs = devm_ioremap_resource(&pdev->dev, io);
	if (IS_ERR(sevenseg->regs)) {
		return PTR_ERR(sevenseg->regs);
	}
	sevenseg->size = io->end - io->start + 1; // [start, end[ <- exclusive end!

	sevenseg->misc.name = DRIVER_NAME;
	sevenseg->misc.parent = &pdev->dev;
	sevenseg->misc.fops = &fops;
	sevenseg->misc.minor = MISC_DYNAMIC_MINOR;

	retVal = misc_register(&sevenseg->misc);
	if (retVal != 0) {
		dev_err(&pdev->dev, "misc_register failed\n");
		return retVal;
	}

	dev_info(&pdev->dev, "sevensegment driver should work now...pretty please?\n");

	return 0;
}

static int sevenseg_remove(struct platform_device *pdev) {
	struct altera_sevenseg_t * sevenseg = platform_get_drvdata(pdev);
	pr_info("sevenseg_remove hit\n");
	misc_deregister(&sevenseg->misc);
	platform_set_drvdata(pdev, NULL);
	
	return 0;
}

static atomic_t activeFileOpen = ATOMIC_INIT(0); // prevent data races

// fileoperations
static int device_open(struct inode * inode, struct file * file) {
	if (!atomic_add_unless(&activeFileOpen, 1, 1)) {
		return -EBUSY;
	}

	try_module_get(THIS_MODULE);

	return 0;
}

static int device_release(struct inode * inode, struct file * file) {
	atomic_dec(&activeFileOpen);

	module_put(THIS_MODULE);
	return 0;
}

static ssize_t device_read(struct file *filep,
		char * buffer,
		size_t length,
		loff_t * offset) {

		struct altera_sevenseg_t * sevenSeg = container_of(filep->private_data,
														struct altera_sevenseg_t,
														misc);

		if ((*offset) < 0 || (*offset) >= REGISTER_COUNT) {
			return 0;
		}

		// make length % REGISTER_COUNT to not read more memory
		while ((*offset + length) > REGISTER_COUNT) {
			length -= REGISTER_COUNT;
		}
		// length is now [0, REGISTER_COUNT[
		
		length = length - copy_to_user(buffer, sevenSeg->buffer + *offset, length);
		*offset += length;
		return length;
}

static ssize_t device_write(struct file * filep,
		char const * buffer,
		size_t length,
		loff_t * offset) {
	uint32_t segmentValue = 0;

	struct altera_sevenseg_t *sevenseg = container_of(filep->private_data,
				struct altera_sevenseg_t, misc);

	// sanity checks for offset and length
	if (((*offset) < 0) || ((*offset) >= REGISTER_COUNT)) {
		return -EINVAL;
	}

	// make length % REGISTER_COUNT to not read more memory
	while ((*offset + length) > REGISTER_COUNT) {
		length -= REGISTER_COUNT;
	}
	// length is now [0, REGISTER_COUNT[
	length -= copy_from_user(sevenseg->buffer + *offset, buffer, length);

	// segment values
	segmentValue = 0
						| sevenseg->buffer[0] << 16
						| sevenseg->buffer[1] << 8
						| sevenseg->buffer[2] << 0;
	iowrite32(segmentValue, sevenseg->regs);

	// always all enabled
	iowrite32(0xFFFF, sevenseg->regs + ENABLE_OFFSET);
	iowrite32(0xFFFF, sevenseg->regs + PWM_OFFSET);

	*offset += length;
	return length;
}

// ============================================================================
// Define for which hardware version this driver is useful for

struct of_device_id sevensegment_dt_ids[] = {
	{
		.compatible = "hof,sevensegment-1.0",
	},
	{
		.compatible = "breiti,sevensegment-1.0",
	},
	{/* end of table */ }
};
MODULE_DEVICE_TABLE(of, sevensegment_dt_ids);

// ============================================================================
// Define for which hardware platfrom this driver is useful for

static struct platform_driver sevensegment_platform_driver = {
	.probe = sevenseg_probe,
	.remove = sevenseg_remove,
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(sevensegment_dt_ids)
	}
};

module_platform_driver(sevensegment_platform_driver);

// ============================================================================

MODULE_AUTHOR("esd2021 Grp1");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("milestone 1");
