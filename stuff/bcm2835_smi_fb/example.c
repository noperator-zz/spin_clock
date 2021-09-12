#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/fs.h>
static int my_init(void)
{
	return 0;
}

static void my_exit(void)
{
	return;
}

static struct file_operations simple_driver_fops = 
{
    .owner   = THIS_MODULE,
    .write   = device_file_write,
};

static int device_file_major_number = 29;
static const char device_name[] = "smi-fb";
static int register_device(void)
{
        int result = 0;
        printk( KERN_NOTICE "smi-fb: register_device() is called." );
        result = register_chrdev( 0, device_name, &simple_driver_fops );
        if( result < 0 )
        {
            printk( KERN_WARNING "smi_fb:  can\'t register character device with errorcode = %i", result );
            return result;
        }
        device_file_major_number = result;
        printk( KERN_NOTICE "smi-fb: registered character device with major number = %i and minor numbers 0...255"
             , device_file_major_number );
        return 0;
}

void unregister_device(void)
{
    printk( KERN_NOTICE "smi-fb: unregister_device() is called" );
    if(device_file_major_number != 0)
    {
        unregister_chrdev(device_file_major_number, device_name);
    }
}

static ssize_t device_file_write(
	struct file *file_ptr,
	char __user *user_buffer,
	size_t count,
	loff_t *position)


module_init(my_init);
module_exit(my_exit);
