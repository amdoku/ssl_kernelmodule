#include "asm/atomic.h"
#include "asm/io.h"
#include "linux/atomic.h"
#include <linux/delay.h>
#include "linux/kern_levels.h"
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/uaccess.h>

#define DRIVER_NAME "hdc1000"

#define REGISTER_COUNT 1
#define BUFFER_SIZE_CHAR (REGISTER_COUNT * sizeof(int))

int const CHAR_AMOUNT_BUFFER = BUFFER_SIZE_CHAR;

struct hdc1000_t {
	void * regs;
	char buffer[BUFFER_SIZE_CHAR];
	int size;
	struct miscdevice misc;
};

#undef BUFFER_SIZE_CHAR
#undef REGISTER_COUNT

// file operations
static ssize_t device_read(struct file *, char *, size_t, loff_t *);
static ssize_t device_write(struct file *, const char *, size_t, loff_t *);
static int device_open(struct inode *, struct file *);
static int device_release(struct inode *, struct file *);
static loff_t device_llseek(struct file *, loff_t, int);

static const struct file_operations fops = {
	.owner = THIS_MODULE,
	.read = device_read,
	.write = device_write,
	.open = device_open,
	.release = device_release,
	.llseek = device_llseek,
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

static int hdc1000_probe(struct platform_device *pdev) {
	struct hdc1000_t * device;
	struct resource * io;
	int retVal;

	pr_info("hdc1000_probe hit\n");

	device = devm_kzalloc(&pdev->dev, sizeof(*device), GFP_KERNEL);
	if (device == NULL) {
		return -ENOMEM;
	}

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

	// enable sampling
	iowrite8(0xAA, device->regs + 0x10);

	dev_info(&pdev->dev, "hdc1000 driver should work now...pretty please?\n");

	return 0;
}

static int hdc1000_remove(struct platform_device *pdev) {
	struct hdc1000_t * device = platform_get_drvdata(pdev);
	pr_info("hdc1000_remove hit\n");

	// enable sampling
	iowrite8(0xAA, device->regs);

	misc_deregister(&device->misc);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static atomic_t activeFileOpen = ATOMIC_INIT(0); // prevent data races

// fileoperations
static int device_open(struct inode * inode,
						struct file * filep) {
	printk(KERN_ALERT "device_open");

	if (!atomic_add_unless(&activeFileOpen, 1, 1)) {
		printk(KERN_ALERT "file is already opened by someone else");
		return -EBUSY;
	}

	printk(KERN_ALERT "open|inode->i_sb->s_maxbytes=%lld", inode->i_sb->s_maxbytes);
	try_module_get(THIS_MODULE);

	return 0;
}

static int device_release(struct inode * inode,
						struct file * filep) {
	printk(KERN_ALERT "device_release");

	atomic_dec(&activeFileOpen);

	module_put(THIS_MODULE);
	return 0;
}

static ssize_t device_read(struct file *filep,
							char * buffer,
							size_t length,
							loff_t * offset) {

	struct hdc1000_t * device = container_of(filep->private_data,
											struct hdc1000_t,
											misc);

	if (offset == NULL) {
		printk(KERN_ALERT "read| offset is null\n");
		return 0;
	}

	printk(KERN_ALERT "read| offset=%llu", *offset);
	printk(KERN_ALERT "read|inode->i_sb->s_maxbytes=%lld", filep->f_mapping->host->i_sb->s_maxbytes);

	// do not write beyond buffer
	if ((*(offset) + length) > CHAR_AMOUNT_BUFFER) {
		length = CHAR_AMOUNT_BUFFER - *offset;
	}

	printk(KERN_ALERT "read|device->regs=%p,length=%d\n", device->regs, length);
	if (length > 0) {
		memcpy_fromio(device->buffer, device->regs, CHAR_AMOUNT_BUFFER);

		length = length - copy_to_user(buffer, device->buffer + *offset, length);
		*offset += length;
		printk(KERN_ALERT "read|read bytes=%d\n", length);
	}

	return length;
}

static ssize_t device_write(struct file * filep,
							char const * buffer,
							size_t length,
							loff_t * offset) {
	return -EINVAL;
}

// ============================================================================
// Define for which hardware version this driver is useful for

struct of_device_id hdc1000_dt_ids[] = {
	{
		.compatible = "breiti,hdc1000-1.0",
	},
	{
		.compatible = "hof,hdc1000-1.0",
	},
	{/* end of table */ }
};
MODULE_DEVICE_TABLE(of, hdc1000_dt_ids);

// ============================================================================
// Define for which hardware platfrom this driver is useful for

static struct platform_driver hdc1000_platform_driver = {
	.probe = hdc1000_probe,
	.remove = hdc1000_remove,
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(hdc1000_dt_ids)
	}
};

module_platform_driver(hdc1000_platform_driver);

// ============================================================================

MODULE_AUTHOR("esd2021 Grp1");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("milestone 2");
