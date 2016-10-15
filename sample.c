#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <asm/uaccess.h>
#include <linux/ktime.h>


/*  
 *  Prototypes 
 */
static int __init sample_init_module(void);
static void __exit sample_cleanup_module(void);
static int sample_open(struct inode *inode, struct file *file);
static int sample_release(struct inode *inode, struct file *file);
static ssize_t sample_ioctl(struct inode *inode, struct file *filep, const unsigned int cmd, const unsigned long arg);
static ssize_t sample_read(struct file *filp, char *buffer,size_t length, loff_t * offset);
static ssize_t sample_write(struct file *filp, const char *buffer,size_t length, loff_t * offset);

/*
 * Other definition
*/
#define SUCCESS 0

/*
 * Definition of the GPIOs we want to use
*/
#define LED	129		// LED
#define TRIG 	131		// TRIGGER, pin D7
#define ECHO	130		// ECHO, pin D8
/*
 * Definition of ioctl commands
*/
#define	INVALID_CMD	-1
#define TRIGGER_GO 	0
#define LED_GO		1
#define WRITE_PERIOD	2

/*
 * Device major number
 */
static uint module_major = 166;

/*
 * Device name
 */
static char * module_name = "sample";

/*
 * Device access lock. Only one process can access the driver at a time
 */
static int sample_lock = 0;

/*
 * Device variables
*/
int mode = INVALID_CMD;	

//static int blk_period;		Blinking period in ms		
//static int trigger_period;	//Trigger period in ms


/*
 * Declare the workqueue
 */
static struct workqueue_struct *my_wq;
static struct workqueue_struct *led_wq;

typedef struct {
	struct work_struct led_work;
	int	blk_p;
} led_work_t;

typedef struct {
	struct work_struct my_work;
	int	delay;
} my_work_t;

static my_work_t work;
static led_work_t l_work;

static int blk_period=0;			//i use this approach with global variable, cause in this way the program is reactive to 							//half period

/*
 * Work function
 */
static void my_wq_function( struct work_struct *work )
{
	int	delay;
	my_work_t *my_work;

	my_work = (my_work_t *)work;
	delay = my_work->delay;				//write this value once
	
	while(1){
		gpio_set_value(TRIG, 1);
		msleep(delay);
		gpio_set_value(TRIG, 0);
		msleep(60);
		
	}
	
} 

static void led_wq_function( struct work_struct *l_work )
{
	
	led_work_t *led_work;
	led_work = (led_work_t *)l_work;

	while(1)
	{
		
		
		gpio_set_value( LED, 1 );		
		msleep(blk_period/2);			//duty cycle 50%
		gpio_set_value( LED, 0 );
		msleep(blk_period/2);	
	}	


} 

/*
 * Timer variables
*/
static ktime_t start, end;
static u64 actual_time;
static unsigned int val;
static int ready=0;



/*
 * Device operations
 */
static struct file_operations sample_fops = {
	.read = sample_read,
	.write = sample_write,
	.open = sample_open,
	.release = sample_release,
	.ioctl = sample_ioctl
};

/*
 * ECHO interrupt handler
*/

static irq_handler_t echo_handler( unsigned int irq, struct pt_regs *regs )
{
	val = gpio_get_value(ECHO);
	if(val){
		start = ktime_get();
		ready=0;
		}

	else if(!val){
		end = ktime_get();
		ready=1;						//when time value is ready
		}

	return (irq_handler_t)IRQ_HANDLED;
}


/* 
 * Device read
 */
static ssize_t sample_read(struct file *filp, char *buffer,
			 size_t length, loff_t * offset)
{
	int ret = 1;
	int dummy=0;
	
	
	// Echo time

	if(ready){
	actual_time = ktime_to_ns(ktime_sub(end,start));
	memcpy(buffer, &actual_time, sizeof(actual_time));
	ready=0;
	}
	else memcpy(buffer, &dummy, sizeof(dummy));		//passing a dummy (zero value) to be sure of a correct reading (in app.c)


	
	#ifdef SAMPLE_DEBUG
		printk( KERN_INFO "%s: %s\n", module_name, __func__ );
	#endif
	
 
	return(ret);
}


/* 
 * Device write
 */
static ssize_t sample_write(struct file *filp, const char *buffer,
			  size_t length, loff_t * offset)
{
	int ret = 0;
	
	int trigger_period=0;
	
	switch(mode)
	{
		case TRIGGER_GO:
			
			memcpy( &trigger_period, buffer, sizeof(trigger_period) );
			work.delay = trigger_period;
			queue_work( my_wq, (struct work_struct *)&work );
			ret = 1;
			break;

		case LED_GO:
			
			memcpy(&blk_period, buffer, sizeof( blk_period) );
			l_work.blk_p = blk_period;
			queue_work( led_wq, (struct work_struct *)&l_work );
			ret = 1;
			break;

		case WRITE_PERIOD:
			
			memcpy( &blk_period, buffer, sizeof( blk_period) );
			l_work.blk_p = blk_period;
			ret = 1;
			break;
		
		case INVALID_CMD:
			ret = 0;
			break;
	}

	#ifdef SAMPLE_DEBUG
		printk( KERN_INFO "%s: %s\n", module_name, __func__ );
	#endif

	return(ret);
}


static ssize_t sample_ioctl(struct inode *inode, struct file *filep, 
			    const unsigned int cmd, const unsigned long arg)
{
	int ret = SUCCESS;

	switch(cmd)
	{
		case TRIGGER_GO:
			mode = TRIGGER_GO ;	
			break;

		case LED_GO:
			mode = LED_GO;	
			break;

		case WRITE_PERIOD:
			mode = WRITE_PERIOD;
			break;
		
		default:
			printk("INVALID MODE !!! \n");
			mode = INVALID_CMD;
			break;
	}

	#ifdef SAMPLE_DEBUG
		printk( KERN_INFO "%s: %s\n", module_name, __func__ );
	#endif

	return(ret);
}


/*
 * Device open
 */
static int sample_open(struct inode *inode, struct file *file)
{
	int ret = SUCCESS;

	/*
	 * One process at a time
	 */
	if (sample_lock) 
	{
		ret = -EBUSY;
	}
	else
	{
		sample_lock++;

		/*
 	 	* Increment the module use counter
 	 	*/
		try_module_get(THIS_MODULE);

		#ifdef SAMPLE_DEBUG 
			printk( KERN_INFO "%s: %s\n", module_name, __func__ ); 
		#endif
	}

	return(ret);
}

/*
 * Device close
 */
static int sample_release(struct inode *inode, struct file *file)
{
	int ret = SUCCESS;

	/*
 	 * Release device,ready for our next caller
 	 */
	sample_lock = 0;

	/*
 	 * Decrement module use counter
 	 */
	module_put(THIS_MODULE);

	#ifdef SAMPLE_DEBUG
		printk( KERN_INFO "%s: %s\n", module_name, __func__ );
	#endif

	return(ret);
}


/*
 * This functions are called to mount and unmount the module
 */

/*
 * This function is called when the module is loaded
 */
static int __init sample_init_module(void)
{
	/*
 	 * Register device
 	 */
	int	ret;



	

	ret = register_chrdev(module_major, module_name, &sample_fops);
	if (ret < 0) {
		printk(KERN_INFO "%s: registering device %s with major %d failed with %d\n",
		       __func__, module_name, module_major, module_major );
		return(ret);
	}
	else
	{
		printk(KERN_INFO "%s: registering device %s with major %d\n",
		       __func__, module_name, module_major );
		/*
 		 * Reserve gpios
		*/
		if( gpio_request( TRIG, module_name ) )	// Check if TRIG is available
		{
			printk( KERN_INFO "%s: %s unable to get TRIG gpio\n", module_name, __func__ );
			ret = -EBUSY;
			return(ret);
		}
	
		

		if( gpio_request( ECHO, module_name ) )	// Check if ECHO is available
		{
			printk( KERN_INFO "%s: %s unable to get ECHO gpio\n", module_name, __func__ );
			ret = -EBUSY;
			return(ret);
		}

		

		if( gpio_request( LED, module_name ) )	// Check if LED is available
		{
			printk( KERN_INFO "%s: %s unable to get LED gpio\n", module_name, __func__ );
			ret = -EBUSY;
			return(ret);
		}
		
			

		/*
 		 * Set gpios direction (as output, with default output value set to 0)
		*/	
		if( gpio_direction_output( TRIG, 0 ) < 0 )	// Set gpio D7 (TRIGGER) as output with default value 0
		{
			printk( KERN_INFO "%s: %s unable to set TRIG gpio as output\n", module_name, __func__ );
			ret = -EBUSY;
			return(ret);
		}
		
	

		if( gpio_direction_input( ECHO ) < 0 )	// Set gpio D8 as input (ECHO)
		{
			printk( KERN_INFO "%s: %s unable to set ECHO gpio as input\n", module_name, __func__ );
			ret = -EBUSY;
			return(ret);
		}


		if( gpio_direction_output( LED, 0 ) < 0 )	// Set LED gpio as output with default value 0
		{
			printk( KERN_INFO "%s: %s unable to set LED gpio as output\n", module_name, __func__ );
			ret = -EBUSY;
			return(ret);
		}

		
		//ECHO interrupts
		if( request_irq( gpio_to_irq( ECHO ), 
                                 (irq_handler_t) echo_handler,
				  IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING ,
				  module_name,
				  NULL ) < 0 )
		{
			printk( KERN_INFO "%s: %s unable to register gpio irq\n", module_name, __func__ );
			ret = -EBUSY;
			return( ret );
		}

		
		
		
		
		
		
		my_wq = create_workqueue( "My queque" );
		if( my_wq )
		{
			INIT_WORK( (struct work_struct *)&work, my_wq_function );
		}
	
		led_wq = create_workqueue( "Led queque" );
		if( led_wq )
		{
			INIT_WORK( (struct work_struct *)&l_work, led_wq_function );
		}

		
		
	}
	
	return(ret);
}

/*
 * This function is called when the module is unloaded
 */
static void __exit sample_cleanup_module(void)
{
	/*
	 * Free irq
	 */
	free_irq( gpio_to_irq( ECHO ), NULL );

	/*
	 * Release the gpios
	 */
	gpio_free(TRIG);
	gpio_free(ECHO);
	gpio_free(LED);

	/*
	 * Unregister device
	 */
	unregister_chrdev(module_major, module_name);

	printk(KERN_INFO "%s: unregistering %s done\n", __func__, module_name );
}

module_init(sample_init_module);
module_exit(sample_cleanup_module);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Luca Mocerino, luca.mocerino@polito.it");
MODULE_DESCRIPTION("Assigment #2");


