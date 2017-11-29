/**
 *
set * Basic kernel driver for GPIO
 */

//#include <stdio.h>
//#include <stdlib.h>
//#include <fcntl.h>
//#include <sys/mman.h>
//#include <unistd.h>

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/gpio.h>       
#include <linux/kobject.h>    
#include <linux/kthread.h>    
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/device.h>
#include <linux/fs.h>             // Header for the Linux file system support
#include <asm/uaccess.h>          // Required for the copy to user function
#include <asm/io.h>
#include <linux/platform_data/bcm2708.h>



#define  DEVICE_NAME "paschar"    ///< The device will appear at /dev/paschar using this value
#define  CLASS_NAME  "pas"

#define BCM2708_PERI_BASE        0x20000000
#define GPIO_BASE                (BCM2708_PERI_BASE + 0x200000) /* GPIO controller */

static unsigned PORT = 0x20200000;

static unsigned RANGE =  0x40;


// GPIO setup macros. Always use INP_GPIO(x) before using OUT_GPIO(x) or SET_GPIO_ALT(x,y)
#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g) *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))
#define SET_GPIO_ALT(g,a) *(gpio+(((g)/10))) |= (((a)<=3?(a)+4:(a)==4?3:2)<<(((g)%10)*3))

#define GPIO_SET *(gpio+7)  // sets   bits which are 1 ignores bits which are 0
#define GPIO_CLR *(gpio+10) // clears bits which are 1 ignores bits which are 0

#define GET_GPIO(g) (*(gpio+13)&(1<<g)) // 0 if LOW, (1<<g) if HIGH

#define GPIO_PULL *(gpio+37) // Pull up/pull down
#define GPIO_PULLCLK0 *(gpio+38) // Pull up/pull down clock



MODULE_LICENSE("GPL");
MODULE_AUTHOR("Piskyscan");
MODULE_DESCRIPTION("Simple Pi Driver for GPIO");
MODULE_VERSION("0.1");

struct GpioRegisters
{
    uint32_t GPFSEL[6];
    uint32_t Reserved1;
    uint32_t GPSET[2];
    uint32_t Reserved2;
    uint32_t GPCLR[2];
};

struct GpioRegisters *s_pGpioRegisters;


int  mem_fd;
void *gpio_map;
volatile unsigned *gpio;

static unsigned int gpioLEDS[] = {17,27,22,5,6,26,23,25};
static unsigned int numLeds = 8;

static int    numberOpens = 0;              ///< Counts the number of times the device is opened

static int    majorNumber;                  ///< Stores the device number -- determined automatically
static char   message[256] = {0};           ///< Memory for the string that is passed from userspace
static short  size_of_message;              ///< Used to remember the size of the string stored
static struct class*  pascharClass  = NULL; ///< The device-driver class struct pointer
static struct device* pascharDevice = NULL; ///< The device-driver device struct pointer

/** size of input buffer is 1 megabyte*/
#define buffersize 1048576

/** @brief structure of our input buffer*/
struct circbuf{
	size_t tail, head; /**< dequeue/read index and enqueue/write index*/
	unsigned char buffer[buffersize];/**< circular buffer backing store*/
};

/** @brief The circular input buffer in use by our program*/
static struct circbuf mybuffer;

/** @brief the mutex used to ensure only one write executes at a time*/
static DEFINE_MUTEX(inputlock);



static unsigned int hertz = 8000;           ///Hertz to drive at.
module_param(hertz, uint, S_IWUSR | S_IWGRP);
MODULE_PARM_DESC(hertz, "Herz to output at"); 

static unsigned int blinkPeriod = 1000;     ///< The blink period in ms
module_param(blinkPeriod, uint, S_IWUSR | S_IWGRP);   ///< Param desc. S_IRUGO can be read/not changed
MODULE_PARM_DESC(blinkPeriod, " Pins to use, least significant first");

static char ledName[20] = "GPIOGroup";          ///< Null terminated default string -- just in case
// static int ledOn = 0;                      ///< Is the LED on or off? Used for flashing
// enum modes { OFF, ON, FLASH };              ///< The available LED modes -- static not useful here
// static enum modes mode = FLASH;             ///< Default mode is flashing

/** @brief A callback function to show the frequency */

// external functions called.

static int setup_io(void);

void writeVals(unsigned int *pins, int val, int count);

static ssize_t hertz_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	printk(KERN_INFO "PAS: Hertz requested %d\n", hertz);
	return sprintf(buf, "%d\n",hertz);
}

/** @brief A callback function to store the frequency */
static ssize_t hertz_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	unsigned long res;
	int thertz =  kstrtoul(buf, 	10,&res);

	printk(KERN_INFO "PAS: Hertz set %d\n", thertz);

	hertz = thertz;
	return count;
}

/** @brief A callback function to display the pins used */
static ssize_t pins_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	printk(KERN_INFO "PAS: pins requested \n");
	return sprintf(buf, "%d\n", blinkPeriod);
}

/** @brief A callback function to store the LED period value */
static ssize_t pins_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	unsigned int period;                     // Using a variable to validate the data sent

	printk(KERN_INFO "PAS: pins set \n");

	sscanf(buf, "%du", &period);             // Read in the period as an unsigned int
	if ((period>1)&&(period<=10000)){        // Must be 2ms or greater, 10secs or less
		blinkPeriod = period;                 // Within range, assign to blinkPeriod variable
	}
	return count;
}

/** Use these helper macros to define the name and access levels of the kobj_attributes
 *  The kobj_attribute has an attribute attr (name and mode), show and store function pointers
 *  The period variable is associated with the blinkPeriod variable and it is to be exposed
 *  with mode 0666 using the period_show and period_store functions above
 */
static struct kobj_attribute hertz_attr = __ATTR(hertz, S_IWUSR | S_IWGRP, hertz_show, hertz_store);
static struct kobj_attribute mode_attr = __ATTR(mode, S_IWUSR | S_IWGRP, pins_show, pins_store);

static struct attribute *pas_attrs[] =
{
		&hertz_attr.attr,                       // The period at which the LED flashes
		&mode_attr.attr,                         // Is the LED on or off?
		NULL,
};

static struct attribute_group attr_group =
{
		.name  = ledName,
		.attrs = pas_attrs,                      // The attributes array defined just above
};

static struct kobject *pas_kobj;            /// The pointer to the kobject
// static struct task_struct *task;            /// The pointer to the thread task


// The prototype functions for the character driver -- must come before the struct definition
static int     dev_open(struct inode *, struct file *);
static int     dev_release(struct inode *, struct file *);
static ssize_t dev_read(struct file *, char *, size_t, loff_t *);
static ssize_t dev_write(struct file *, const char *, size_t, loff_t *);

/** @brief high-resolution timer used for the write-signal to the DAC*/
static struct hrtimer writeclock;


static struct file_operations fops =
{
		.open = dev_open,
		.read = dev_read,
		.write = dev_write,
		.release = dev_release,
};

static int dev_open(struct inode *inodep, struct file *filep)
{
	numberOpens++;

	printk(KERN_INFO "PAS: Device has been opened %d time(s)\n", numberOpens);

	return 0;
}


/* not used at the minute */

static ssize_t dev_read(struct file *filep, char *buffer, size_t len, loff_t *offset)
{
	int error_count = 0;
	// copy_to_user has the format ( * to, *from, size) and returns 0 on success
	error_count = copy_to_user(buffer, message, size_of_message);

	if (error_count==0)
	{            // if true then have success
		printk(KERN_INFO "PAS: Sent %d characters to the user\n", size_of_message);
		return (size_of_message=0);  // clear the position to the start and return 0
	}
	else
	{
		printk(KERN_INFO "PAS: Failed to send %d characters to the user\n", error_count);
		return -EFAULT;              // Failed -- return a bad address message (i.e. -14)
	}
}

static void writeVal(char c)
{
	int j;
	int val;

	for (j = 0;j < numLeds;j++)
	{

		val = c & 1;
		c = c / 2;

		gpio_set_value(gpioLEDS[j],val);       // Use the LED state to light/turn off the LED

	}
}

static ssize_t dev_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
	size_t pulltemp;/**< stores temporary conservative copy of dequeue index of the circular buffer*/
	size_t pushtemp;/**< stores temporary conservative copy of enqueue index of the circular buffer*/
	size_t space;/**< space available in buffer*/
	size_t firstspace;/**< space available in the buffer before wrapping*/
	size_t secondspace;/**< space available in the buffer after writing */
	size_t firstwrite;/**< bytes written to the buffer before the wrap*/
	size_t secondwrite;/**< bytes written to the buffer after the wrap*/
	size_t notwritten = 0;/**< bytes not written if a copy_from_user fails*/

	/** block any other write methods that try to start while this one is in progress fail if impossible*/
	if(mutex_lock_interruptible(&inputlock))
		return 0;
	/** set the variables to their values*/
	pulltemp = mybuffer.tail;
	pushtemp = mybuffer.head;
	space = CIRC_SPACE(pushtemp, pulltemp, buffersize);
	firstspace = CIRC_SPACE_TO_END(pushtemp, pulltemp, buffersize);
	secondspace = space - firstspace;
	firstwrite = (count < firstspace) ? count : firstspace;
	secondwrite = (count < space) ? count - firstwrite : secondspace;

	/** Write any segments of data into the buffer before or after the wrap at the end of the backing store*/
	if(firstwrite)
		notwritten = copy_from_user((void *)(mybuffer.buffer + pushtemp), (const void __user*)buf, firstwrite);
	/* if there was an error in copy_from_user return only the number of bytes copied*/
	if(notwritten){
		/* increment the enqueue index by the number of bytes actually written*/
		mybuffer.head = (pushtemp + firstwrite - notwritten) % buffersize;
		mutex_unlock(&inputlock);
		return firstwrite-notwritten;
	}
	if(secondwrite)/**< if there's some data to write to the other side of the loop, do so*/
		notwritten = copy_from_user((void *)mybuffer.buffer, (const void __user*)(buf+firstwrite), secondwrite);
	if(notwritten){
		mybuffer.head = (pushtemp + firstwrite + secondwrite - notwritten) % buffersize;
		mutex_unlock(&inputlock);
		return firstwrite+secondwrite-notwritten;
	}

	/** increment the enqueue index by the number of bytes actually written*/
	mybuffer.head = (pushtemp + firstwrite + secondwrite) % buffersize;
	/** let other writes commence*/
	mutex_unlock(&inputlock);
	/** return the number of bytes written*/
	return firstwrite+secondwrite;
}


static ssize_t dev_write_old(struct file *filep, const char *buffer, size_t len, loff_t *offset)

{
	int i;
	int j;
	char c;
	int val;

	printk(KERN_INFO "PAS: Received %zu characters from the user\n", len);

	for (i =0;i< len;i++)
	{
		c = buffer[i];

		writeVal(c);

//		for (j = 0;j < numLeds;j++)
//		{
//
//			val = c & 1;
//			c = c / 2;
//
//			gpio_set_value(gpioLEDS[j],val);       // Use the LED state to light/turn off the LED
//
//		}

		usleep_range(1000000/hertz, 1000000/hertz + 1);

	}
	return len;
}

/** @brief The device release function that is called whenever the device is closed/released by
 *  the userspace program
 *  @param inodep A pointer to an inode object (defined in linux/fs.h)
 *  @param filep A pointer to a file object (defined in linux/fs.h)
 */
static int dev_release(struct inode *inodep, struct file *filep)
{
	printk(KERN_INFO "PAS: Device successfully closed\n");
	return 0;
}


/** @brief The LKM initialization function
 *  The static keyword restricts the visibility of the function to within this C file. The __init
 *  macro means that for a built-in driver (not a LKM) the function is only used at initialization
 *  time and that it can be discarded and its memory freed up after that point. In this example this
 *  function sets up the GPIOs and the IRQ
 *  @return returns 0 if successful
 */

static int __init paschar_init(void)
{
	int result = 0;
	int i;

	printk(KERN_INFO "PAS : Initializing the Piadcsnd\n");


	// Try to dynamically allocate a major number for the device -- more difficult but worth it
	majorNumber = register_chrdev(0, DEVICE_NAME, &fops);
	if (majorNumber<0){
		printk(KERN_ALERT "PAS failed to register a major number\n");
		return majorNumber;
	}
	printk(KERN_INFO "paschar: registered correctly with major number %d\n", majorNumber);

	// Register the device class
	pascharClass = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(pascharClass)){                // Check for error and clean up if there is
		unregister_chrdev(majorNumber, DEVICE_NAME);
		printk(KERN_ALERT "Failed to register device class\n");
		return PTR_ERR(pascharClass);          // Correct way to return an error on a pointer
	}
	printk(KERN_INFO "PASChar: device class registered correctly\n");

	// Register the device driver
	pascharDevice = device_create(pascharClass, NULL, MKDEV(majorNumber, 0), NULL, DEVICE_NAME);
	if (IS_ERR(pascharDevice)){               // Clean up if there is an error
		class_destroy(pascharClass);           // Repeated code but the alternative is goto statements
		unregister_chrdev(majorNumber, DEVICE_NAME);
		printk(KERN_ALERT "Failed to create the device\n");
		return PTR_ERR(pascharDevice);
	}

	// setup_io();

	sprintf(ledName, "GpioGroup");

	pas_kobj = kobject_create_and_add("pas", kernel_kobj->parent); // kernel_kobj points to /sys/kernel


	if(!pas_kobj){
		printk(KERN_ALERT "PAS SND: failed to create kobject\n");
		return -ENOMEM;
	}

	result = sysfs_create_group(pas_kobj, &attr_group);
	if(result) {
		printk(KERN_ALERT "PAS SND: failed to create sysfs group\n");
		kobject_put(pas_kobj);                // clean up -- remove the kobject sysfs entry
		return result;
	}

	for (i = 0; i < numLeds;i++)
	{
		gpio_request(gpioLEDS[i], "sysfs");
		gpio_direction_output(gpioLEDS[i], 0);   // Set the gpio to be in output mode and turn on
		gpio_export(gpioLEDS[i], false);  // causes gpio to appear in /sys/class/gpio
		// the second argument prevents the direction from being changed
	}

	// setup buffer
	mybuffer.head = 1;
	mybuffer.tail = 0;
	mybuffer.buffer[0] = 0;

	/** initialize, set callback, and start our DAC loader timer*/
	hrtimer_init(&refreshclock, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	refreshclock.function = &timedRefresh;

	hrtimer_start(&refreshclock, (1000000/hertz), HRTIMER_MODE_REL);



	//    task = kthread_run(flash, NULL, "LED_flash_thread");  // Start the LED flashing thread
	//   if(IS_ERR(task)){                                     // Kthread name is LED_flash_thread
	//      printk(KERN_ALERT "PAS SND: failed to create the task\n");
	//      return PTR_ERR(task);
	//
	//   }
	return result;
}

/** @brief The LKM cleanup function
 *  Similar to the initialization function, it is static. The __exit macro notifies that if this
 *  code is used for a built-in driver (not a LKM) that this function is not required.
 */
static void __exit paschar_exit(void)
{
	int i;
	//   kthread_stop(task);                      // Stop the LED flashing thread
	kobject_put(pas_kobj);                   // clean up -- remove the kobject sysfs entry

	for (i = 0; i < numLeds;i++)
	{
		gpio_set_value(gpioLEDS[i], 0);              // Turn the LED off, indicates device was unloaded
		gpio_unexport(gpioLEDS[i]);                  // Unexport the Button GPIO
		gpio_free(gpioLEDS[i]);
	}

	device_destroy(pascharClass, MKDEV(majorNumber, 0));     // remove the device
	class_unregister(pascharClass);                          // unregister the device class
	class_destroy(pascharClass);                             // remove the device class
	unregister_chrdev(majorNumber, DEVICE_NAME);             // unregister the major number

	printk(KERN_INFO "PAS Card: Goodbye\n");
}

/// This next calls are  mandatory -- they identify the initialization function
/// and the cleanup function (as above).
module_init(paschar_init);
module_exit(paschar_exit);


static enum hrtimer_restart DACrefresh(struct hrtimer* mytimer)
{
  return HRTIMER_NORESTART;/* wait to be started */
}


//
// timer to write value
//

static enum hrtimer_restart timedRefresh(struct hrtimer* mytimer)
{

	writeVal(mybuffer.buffer[mybuffer.tail]);

	/** if there's more than one sample in the buffer, increment the dequeue index*/
	if(CIRC_CNT(mybuffer.head,mybuffer.tail,buffersize) > 1)
	{
		mybuffer.tail = (mybuffer.tail+1) % buffersize;
	}

	/** add one sample interval to our clock*/
	hrtimer_forward_now(mytimer, ((1000000/hertz)));

	/** reset the timer to trigger again later*/
	return HRTIMER_RESTART;
}

