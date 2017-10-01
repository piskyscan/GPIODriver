/**
 *
 * Basic kernel driver for GPIO
 */

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


#define  DEVICE_NAME "paschar"    ///< The device will appear at /dev/paschar using this value
#define  CLASS_NAME  "pas"        ///< The device class -- this is a character device driver

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Piskyscan");
MODULE_DESCRIPTION("Simple Pi Driver for GPIO");
MODULE_VERSION("0.1");

static unsigned int gpioLEDS[] = {17,27,22,5,6,26,23,25};
static unsigned int numLeds = 8;

static int    numberOpens = 0;              ///< Counts the number of times the device is opened

static int    majorNumber;                  ///< Stores the device number -- determined automatically
static char   message[256] = {0};           ///< Memory for the string that is passed from userspace
static short  size_of_message;              ///< Used to remember the size of the string stored
static struct class*  pascharClass  = NULL; ///< The device-driver class struct pointer
static struct device* pascharDevice = NULL; ///< The device-driver device struct pointer


static unsigned int hertz = 1000;           ///Hertz to drive at.
module_param(hertz, uint, S_IWUSR | S_IWGRP);
MODULE_PARM_DESC(hertz, "Herz to output at"); 

static unsigned int blinkPeriod = 1000;     ///< The blink period in ms
module_param(blinkPeriod, uint, S_IWUSR | S_IWGRP);   ///< Param desc. S_IRUGO can be read/not changed
MODULE_PARM_DESC(blinkPeriod, " LED blink period in ms (min=1, default=1000, max=10000)");

static char ledName[20] = "GPIOGroup";          ///< Null terminated default string -- just in case
// static int ledOn = 0;                      ///< Is the LED on or off? Used for flashing
// enum modes { OFF, ON, FLASH };              ///< The available LED modes -- static not useful here
// static enum modes mode = FLASH;             ///< Default mode is flashing

/** @brief A callback function to display the LED mode
 *  @param kobj represents a kernel object device that appears in the sysfs filesystem
 *  @param attr the pointer to the kobj_attribute struct
 *  @param buf the buffer to which to write the number of presses
 *  @return return the number of characters of the mode string successfully displayed
 */
static ssize_t hertz_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n",hertz);
}

/** @brief A callback function to store the LED mode using the enum above */
static ssize_t hertz_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	// the count-1 is important as otherwise the \n is used in the comparison

	unsigned long res;
	int thertz =  kstrtoul(buf, 	10,&res);

	return thertz;
}

/** @brief A callback function to display the LED period */
static ssize_t period_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", blinkPeriod);
}

/** @brief A callback function to store the LED period value */
static ssize_t period_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	unsigned int period;                     // Using a variable to validate the data sent
	sscanf(buf, "%du", &period);             // Read in the period as an unsigned int
	if ((period>1)&&(period<=10000)){        // Must be 2ms or greater, 10secs or less
		blinkPeriod = period;                 // Within range, assign to blinkPeriod variable
	}
	return period;
}

/** Use these helper macros to define the name and access levels of the kobj_attributes
 *  The kobj_attribute has an attribute attr (name and mode), show and store function pointers
 *  The period variable is associated with the blinkPeriod variable and it is to be exposed
 *  with mode 0666 using the period_show and period_store functions above
 */
static struct kobj_attribute hertz_attr = __ATTR(hertz, S_IWUSR | S_IWGRP, hertz_show, hertz_store);
static struct kobj_attribute mode_attr = __ATTR(mode, S_IWUSR | S_IWGRP, period_show, period_store);

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

/** @brief The LED Flasher main kthread loop
 *
 *  @param arg A void pointer used in order to pass data to the thread
 *  @return returns 0 if successful
 */

/**********

static int flash(void *arg){
   printk(KERN_INFO "EBB LED: Thread has started running \n");
   while(!kthread_should_stop()){           // Returns true when kthread_stop() is called
      set_current_state(TASK_RUNNING);
      if (mode==FLASH) ledOn = !ledOn;      // Invert the LED state
      else if (mode==ON) ledOn = true;
      else ledOn = false;
      gpio_set_value(gpioLED, ledOn);       // Use the LED state to light/turn off the LED
      set_current_state(TASK_INTERRUPTIBLE);
      msleep(blinkPeriod/2);                // millisecond sleep for half of the period
   }
   printk(KERN_INFO "EBB LED: Thread has run to completion \n");
   return 0;
}
 */

// The prototype functions for the character driver -- must come before the struct definition
static int     dev_open(struct inode *, struct file *);
static int     dev_release(struct inode *, struct file *);
static ssize_t dev_read(struct file *, char *, size_t, loff_t *);
static ssize_t dev_write(struct file *, const char *, size_t, loff_t *);


static struct file_operations fops =
{
		.open = dev_open,
		.read = dev_read,
		.write = dev_write,
		.release = dev_release,
};

static int dev_open(struct inode *inodep, struct file *filep){
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
		printk(KERN_INFO "EBBChar: Failed to send %d characters to the user\n", error_count);
		return -EFAULT;              // Failed -- return a bad address message (i.e. -14)
	}
}

/** @brief This function is called whenever the device is being written to from user space i.e.
 *  data is sent to the device from the user. The data is copied to the message[] array in this
 *  LKM using the sprintf() function along with the length of the string.
 *  @param filep A pointer to a file object
 *  @param buffer The buffer to that contains the string to write to the device
 *  @param len The length of the array of data that is being passed in the const char buffer
 *  @param offset The offset if required
 */
static ssize_t dev_write(struct file *filep, const char *buffer, size_t len, loff_t *offset)

{
	int i;
	int j;
	char c;
	int val;

	printk(KERN_INFO "PAS: Received %zu characters from the user\n", len);

	for (i =0;i< len;i++)
	{
		c = buffer[i];

//		set_current_state(TASK_RUNNING);

		for (j = 0;j < numLeds;j++)
		{

			val = c & 1;
			c = c / 2;
			printk(KERN_INFO "PAS: Setting port %d to %d\n", gpioLEDS[j],val);

			gpio_set_value(gpioLEDS[j],val);       // Use the LED state to light/turn off the LED

		}

//		set_current_state(TASK_INTERRUPTIBLE);

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
static int __init ebbLED_init(void)
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
	printk(KERN_INFO "EBBChar: device class registered correctly\n");

	// Register the device driver
	pascharDevice = device_create(pascharClass, NULL, MKDEV(majorNumber, 0), NULL, DEVICE_NAME);
	if (IS_ERR(pascharDevice)){               // Clean up if there is an error
		class_destroy(pascharClass);           // Repeated code but the alternative is goto statements
		unregister_chrdev(majorNumber, DEVICE_NAME);
		printk(KERN_ALERT "Failed to create the device\n");
		return PTR_ERR(pascharDevice);
	}

	sprintf(ledName, "GpioGroup");

	pas_kobj = kobject_create_and_add("pas", kernel_kobj->parent); // kernel_kobj points to /sys/kernel

	if(!pas_kobj){
		printk(KERN_ALERT "PAS SND: failed to create kobject\n");
		return -ENOMEM;
	}
	// add the attributes to /sys/ebb/ -- for example, /sys/ebb/led49/ledOn
	result = sysfs_create_group(pas_kobj, &attr_group);
	if(result) {
		printk(KERN_ALERT "PAS SND: failed to create sysfs group\n");
		kobject_put(pas_kobj);                // clean up -- remove the kobject sysfs entry
		return result;
	}

	for (i = 0; i < numLeds;i++)
	{
		gpio_request(gpioLEDS[i], "sysfs");          // gpioLED is 49 by default, request it
		gpio_direction_output(gpioLEDS[i], 0);   // Set the gpio to be in output mode and turn on
		gpio_export(gpioLEDS[i], false);  // causes gpio49 to appear in /sys/class/gpio
		// the second argument prevents the direction from being changed
	}

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
static void __exit ebbLED_exit(void)
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
module_init(ebbLED_init);
module_exit(ebbLED_exit);
