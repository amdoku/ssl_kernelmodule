#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/fs.h>

#include <asm/uaccess.h> // for put_user

static char const * const DEVICE_NAME ="hof,sevensegment-1.0";

// file operations
static ssize_t device_read(struct file *, char *, size_t, loff_t *);
static ssize_t device_write(struct file *, const char *, size_t, loff_t *);
static int device_open(struct inode *, struct file *);
static int device_release(struct inode *, struct file *);

static struct file_operations fops = {
	.read = device_read,
	.write = device_write,
	.open = device_open,
	.release = device_release
};

// Global vars
static int Major; // Major number of our device driver
static int Device_Open = 0; // prevent data races

static int sevenseg_enter(void) {
	printk(KERN_ALERT "Hi!\n");

	Major = register_chrdev(0, DEVICE_NAME, &fops);

	if (Major < 0) {
		printk(KERN_ALERT "Register char device failed with %d\n", Major);
		return Major;
	}

	printk(KERN_INFO "Major number %d for file /dev/%s\n", Major, DEVICE_NAME);
	printk(KERN_INFO "Create file with 'mknod /dev/%s c %d 0'\n", DEVICE_NAME, Major);
	return 0;
}

static void sevenseg_exit(void) {
	printk(KERN_ALERT "Bye!\n");
	unregister_chrdev(Major, DEVICE_NAME);
	printk(KERN_ALERT "unregister_chrdev Major:%d\n", Major);
}

module_init(sevenseg_enter);
module_exit(sevenseg_exit);

// fileoperations
static int device_open(struct inode * inode, struct file * file) {
	if (Device_Open) {
		return -EBUSY;
	}

	Device_Open++;
	try_module_get(THIS_MODULE);

	return 0;
}

static int device_release(struct inode * inode, struct file * file) {
	Device_Open--;

	module_put(THIS_MODULE);
	return 0;
}

static ssize_t device_read(struct file *filp,
		char * buffer,
		size_t length,
		loff_t * offset) {
	static char DUMMY[3] = {0xFF, 0xAB, 0xCD};
	if (length < 3) return 0;

	char * msg_ptr = DUMMY;

	put_user(msg_ptr++, buffer++);
	put_user(msg_ptr++, buffer++);
	put_user(msg_ptr++, buffer++);

	return 3;
}

static ssize_t device_write(struct file * filp,
		char const * buffer,
		size_t len,
		loff_t * off) {
	printk(KERN_ALERT "Nope. No write.");
	return -EINVAL;
}

MODULE_AUTHOR("esd2021 Grp1");
MODULE_LICENSE("closed");
MODULE_DESCRIPTION("milestone 1");

