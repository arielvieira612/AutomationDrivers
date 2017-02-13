/**
 * @file    dth11.c
 * @author  Ariel Vieira
 * @date    4 April 2016
 * @version 0.1
 * @brief  Driver responsável por ler sensor DHT11.
*/
#include <linux/delay.h>
#include <linux/sched.h>          // Required for jiffies functions (timers)
#include <linux/gpio.h>           // Required for the GPIO functions
#include <linux/init.h>           // Macros used to mark up functions e.g. __init __exit
#include <linux/module.h>         // Core header for loading LKMs into the kernel
#include <linux/device.h>         // Header to support the kernel Driver Model
#include <linux/kernel.h>         // Contains types, macros, functions for the kernel
#include <linux/fs.h>             // Header for the Linux file system support
#include <asm/uaccess.h>          // Required for the copy to user function
#define  DEVICE_NAME "DHT11char"  ///< The device will appear at /dev/ebbchar using this value
#define  CLASS_NAME  "dht11"      ///< The device class -- this is a character device driver
 
MODULE_LICENSE("GPL");              ///< The license type -- this affects runtime behavior
MODULE_AUTHOR("Ariel Vieira");      ///< The author -- visible when you use modinfo
MODULE_DESCRIPTION("A Linux driver to read the sensor DHT11.");  ///< The description -- see modinfo
MODULE_VERSION("0.1");              ///< The version of the module
 
static char *name = "Dht11_Char";        ///< An example LKM argument
module_param(name, charp, S_IRUGO);     ///< Param desc. charp = char ptr, S_IRUGO can be read/not changed
MODULE_PARM_DESC(name, "The name to display in /var/log/kern.log");  ///< parameter description
 
static int    majorNumber;                     ///< Stores the device number -- determined automatically
static char   BUFFER_DHT11[256] = {0};         ///< Memory for the string that is passed from userspace
static struct class*  dht11_charClass  = NULL; ///< The device-driver class struct pointer
static struct device* dht11_charDevice = NULL; ///< The device-driver device struct pointer

// The prototype functions for the character driver -- must come before the struct definition
static int     dev_open(struct inode *, struct file *);
static int     dev_release(struct inode *, struct file *);
static ssize_t dev_read(struct file *, char *, size_t, loff_t *);
//static ssize_t dev_write(struct file *, const char *, size_t, loff_t *);

/** @brief Devices are represented as file structure in the kernel. The file_operations structure from
 *  /linux/fs.h lists the callback functions that you wish to associated with your file operations
 *  using a C99 syntax structure. char devices usually implement open, read, write and release calls
 */

 // The custom error codes
#define STB_NO_LOW_DETTECTED            -1  
#define STB_NO_SEC_LOW_DETTECTED        -2
#define STB_NO_HIGH_DETTECTED           -3
#define BIT_NO_START_OF_BIT_DETTECTED   -4
#define BIT_NO_END_OF_BIT_DETTECTED     -5
#define BIT_UNKNOW_BIT_VALUE            -6
#define CHK_INVALID_CHECK_SUM           -7

static struct file_operations fops =
{
   .open = dev_open,
   .read = dev_read,
 //  .write = dev_write,                  //Write not allowed
   .release = dev_release,
};

//Gpio
static unsigned int gpioDHT11 = 4;       ///< hard coding the LED gpio for this example to P9_23 (GPIO49)

//Wait Queue
static DECLARE_WAIT_QUEUE_HEAD(wq);

//Mutex
static DEFINE_MUTEX(dht11_device_mutex);

/** @brief The LKM initialization function
 *  The static keyword restricts the visibility of the function to within this C file. The __init
 *  macro means that for a built-in driver (not a LKM) the function is only used at initialization
 *  time and that it can be discarded and its memory freed up after that point.
 *  @return returns 0 if successful
    The list of all avaiable errors is in: /usr/include/asm-generic/errno-base.h
*/
 
static int __init DHT11_Char_init(void){
   int retvalue =0;
   printk(KERN_INFO "DHT11_Char: Initializing %s LKM!\n", name);
   
   // Try to dynamically allocate a major number for the device -- more difficult but worth it
   majorNumber = register_chrdev(0, DEVICE_NAME, &fops);
   
   if (majorNumber<0){
      printk(KERN_ALERT "DHT11_Char: failed to register a major number\n");
      return majorNumber;
   }
   printk(KERN_INFO "DHT11_Char: registered correctly with major number %d\n", majorNumber);
 
   // Register the device class
   dht11_charClass = class_create(THIS_MODULE, CLASS_NAME);
   if (IS_ERR(dht11_charClass)){                    // Check for error and clean up if there is
      printk(KERN_ALERT "Failed to register device class\n");
      retvalue= PTR_ERR(dht11_charClass);          // Correct way to return an error on a pointer
      goto class_error;
   }
   printk(KERN_INFO "DHT11_Char: device class registered correctly\n");
 
   // Register the device driver
   dht11_charDevice = device_create(dht11_charClass, NULL, MKDEV(majorNumber, 0), NULL, DEVICE_NAME);
   if (IS_ERR(dht11_charDevice)){                   // Clean up if there is an error
      printk(KERN_ALERT "Failed to create the device\n");
      retvalue= PTR_ERR(dht11_charDevice);
      goto device_error;
   }

   // Verify if gpioNumber is valid
   if (!gpio_is_valid(gpioDHT11)){
      printk(KERN_INFO "DHT11_Char: invalid GPIO %d\n", gpioDHT11);
      retvalue =-ENODEV;
      goto device_error;
   }

   // gpioLED is hardcoded to 49, request it
   // Return zero if ok
   // Label is showed in /sys/kernel/debug/gpio
   if(gpio_request(gpioDHT11, "DHT11") != 0){
       printk(KERN_INFO "DHT11_Char: request fail GPIO %d\n",gpioDHT11);
       retvalue= -ENODEV;
       goto device_error;
   }

   gpio_direction_input(gpioDHT11);            // Set the button GPIO to be an input

   gpio_export(gpioDHT11, true);               // Causes gpio115 to appear in /sys/class/gpio          

   printk(KERN_INFO "DHT11_Char: device class created correctly\n"); // Made it! device was initialized
   return retvalue;  

device_error:
    class_destroy(dht11_charClass);               // Repeated code but the alternative is goto statements
class_error:
    unregister_chrdev(majorNumber, DEVICE_NAME);
    return retvalue;
}


/** @brief The Wait Start Bit Function
 *  Start Bit is sended for DHT11 before every bit value. This function wait e dettect
 *  the end of start bit. If start bit was detected correctly 0 (zero) is returned
 */
typedef enum {StartMsg,StartBit} en_Mode;

static int read_start_bit(en_Mode Mode){
    bool end = false;
    int state=0,retvalue=0;
    long int resul_timeout;
    long int time_lowedge=0,time_upedge=0,totaltimeLow_usec=0,totaltimeHigh_usec=0;
    do{

        switch(state){
            
            case 0:

            if(gpio_get_value(gpioDHT11)==0){
                 state +=2;
            }  
            else
                state =1;
            break;
            case 1:
                resul_timeout = wait_event_timeout(wq, gpio_get_value(gpioDHT11)==0, HZ);
                if(resul_timeout >0){
                    state++;
                }else{
                    retvalue =STB_NO_LOW_DETTECTED;
                }
            break;
            case 2:
                time_lowedge = jiffies;
                resul_timeout = wait_event_timeout(wq, gpio_get_value(gpioDHT11)==1, HZ);
                if(resul_timeout >0){
                    state++;
                }else{
                    retvalue =STB_NO_HIGH_DETTECTED;
                }
            break;
            case 3:
                time_upedge = jiffies;
                totaltimeLow_usec = time_upedge - time_lowedge;
                totaltimeLow_usec = (totaltimeLow_usec / (HZ * 1000000));
                if(Mode == StartMsg){
                    state++;
                }else{
                    printk(KERN_INFO "DHT11_Char: Received StartBit Total Time in Low Signal: %li!\n", totaltimeLow_usec);
                    end=true;
                }
            break;
            case 4:
                resul_timeout = wait_event_timeout(wq, gpio_get_value(gpioDHT11)==0, HZ);
                if(resul_timeout >0){
                    state++;
                }else{
                    retvalue =STB_NO_SEC_LOW_DETTECTED;
                }
            break;
            case 5:
                time_lowedge = jiffies;
                totaltimeHigh_usec = time_lowedge - time_upedge;
                totaltimeHigh_usec = (totaltimeHigh_usec / (HZ * 1000000));
                if(Mode == StartMsg){
                    state++;
                }else{
                    printk(KERN_INFO "DHT11_Char: Received StarMsg Total Time in Low: %li usec and High: %li\n", totaltimeLow_usec,totaltimeHigh_usec);
                    end=true;
                }
            break;
        }
    }while((!end)&&(retvalue==0));

    
    if(retvalue!=0){
        printk(KERN_INFO "DHT11_Char: Fail in read_start_bit ErrorCode %d\n", retvalue);
    }

    return retvalue;
}

/** @brief The Red Bit Function
 *  Read the entire bit received form gpio line. Every bit has 80us of period.
 *  Bit =1 if gpio is high in about 70us of all time
 *  Bit =0 if gpio is high in abou 26u-28u and remaing time in low.
 */
static int read_bit(void){
    bool end = false;
    int state=0,retvalue=0;
    long int timestart=0,timeend=0,totaltime=0,resul_timeout=0;
    do{
        switch(state){
            case 0:
                retvalue = read_start_bit(StartBit);
                state++;
            break;
            case 1:
                //After startbit Shoud be High
                if(gpio_get_value(gpioDHT11)==1){
                    //Save time of start monitoring
                    timestart = jiffies;
                    state ++;
                }else{
                    //Lost High condition after start bit (ERROR)
                    retvalue = BIT_NO_START_OF_BIT_DETTECTED;
                }
            break;
            case 2:
                //Wait Low Edge to 100usec
                resul_timeout = wait_event_timeout(wq, gpio_get_value(gpioDHT11)==0, (HZ/10000));
                if(resul_timeout >0){
                    state++;
                }else{
                    retvalue =BIT_NO_END_OF_BIT_DETTECTED;
                }
            break;
            case 3:
                timeend = jiffies;
                totaltime = timeend - timestart;                //Calc total time in High
                totaltime = totaltime / HZ * 1000000;           //Transform to uSecs

                if(totaltime > 60){                             //Upper than 60usec
                    retvalue = 1;                               //Readed Bit 1
                }else if((totaltime < 35)&&(totaltime > 15)){   //Betwen 15usec and 35usec (datasheet says 26usc)
                    retvalue = 0;                               //Readed bit 0
                }else{
                    retvalue = BIT_UNKNOW_BIT_VALUE;            //The value is unknow
                }
                end=true;                                       //Force to exit
            break;
        }
    }while((!end)&&(retvalue==0));

    if(retvalue < 0){
         printk(KERN_INFO "DHT11_Char: Fail in read_bit ErrorCode %d\n", retvalue);
    }
    return retvalue;
}

/** @brief The Start Signal Function
 *  Send start signal to DHT e change GPIO to input.
 *  Start signal start with a low during 18ms and high during 20~40useg 
 */
 static int send_start_signal(void){
    int resultvalue=0,state=0;
    bool end=false;
    do{
        switch(state){
            case 0:
                //Change gpio to output model with high value
                if(gpio_direction_output(gpioDHT11,1)!=0){
                    resultvalue = -EBUSY;
                }
                state++;
            break;
            case 1:
                //Change gpio value to Low value e wait 18ms (Start Signal from MCU)
                gpio_set_value(gpioDHT11, 0);
                set_current_state(TASK_INTERRUPTIBLE);
                schedule_timeout((HZ*18/1000));
                state++;
            break;
            case 2:
                //Change gpio to high e wait 30usec
                gpio_set_value(gpioDHT11, 1);
                udelay(30);
                state++;
            break;
            case 3:
                //Change direction of pin to input and prepare to start reading
                if(gpio_direction_input(gpioDHT11)!=0){
                    resultvalue = -EBUSY;
                }
                end=true;
            break;
        }
    }while((!end)&& (resultvalue==0));
    return resultvalue;
}

//Checksum is the sum of 4 bytes received
static int checksum_valid(void){
    int i =0;
    char checkvalue=0;
    //Sum all bytes
    for(i=0;i<4;i++){
        checkvalue+= BUFFER_DHT11[i];
    }

    //Verify calc check with received check
    if(checkvalue == BUFFER_DHT11[4])
        return 0;
    else{
        printk(KERN_INFO "DHT11_Char: Fail in checksum_valid Check Calulated [%02X] CheckReceived [%02X] \n", checkvalue , BUFFER_DHT11[4]);
    }
        return CHK_INVALID_CHECK_SUM;
}

/** @brief Read Sensor
 *  The complet process of reading a sensor DHT11. The entire comunication receive 5 bytes
 *  [integral RH][decimal RH][integral T][decimal T][checksum]
 */
static int read_sensor(void){
    unsigned char byteReceived=0;
    int resultvalue=0,resultbit=0,state=0,totalbits=0,totalbytes=0;
    bool end=false;

    do{
        switch(state){
            case 0:
                //Send Start Signal to DHT11
                resultvalue = send_start_signal();
                state++;
            break;
            case 1:
                //Wait Message Signal sended by DHT11
                resultvalue = read_start_bit(StartMsg);
                state++;
            break;
            case 2:
                //read bit
                resultbit = read_bit();
                if(resultbit > 0) 
                    state++;
                else
                    resultvalue = resultbit;
            break;
            case 3:
                //Save received bit
                byteReceived = ((byteReceived << 1) | resultbit);
                totalbits++;
                //Received one byte?
                if(totalbits >=8){
                    //save byte
                    BUFFER_DHT11[totalbytes] = byteReceived;
                    byteReceived = 0;
                    totalbits = 0; 
                    //Received 5 bytes?
                    if(totalbytes >= 5){
                        //Verify checksum
                        resultvalue = checksum_valid();
                        end =true;
                    }
                }
                state = 2;
            break;
        }
    }while((!end)&&(resultvalue ==0));

    if(resultvalue != 0){
        printk(KERN_INFO "DHT11_Char: Fail in read_sensor ErrorCode %d\n", resultvalue);
    }
    return resultvalue;
}
 
/** @brief The LKM cleanup function
 *  Similar to the initialization function, it is static. The __exit macro notifies that if this
 *  code is used for a built-in driver (not a LKM) that this function is not required.
 */
static void __exit DHT11_Char_exit(void){
   gpio_unexport(gpioDHT11);                                    // remove from sysfs
   gpio_free(gpioDHT11);                                       // deallocate the GPIO line
   device_destroy(dht11_charClass, MKDEV(majorNumber, 0));      // remove the device
   class_unregister(dht11_charClass);                           // unregister the device class
   class_destroy(dht11_charClass);                              // remove the device class
   unregister_chrdev(majorNumber, DEVICE_NAME);                 // unregister the major number
   printk(KERN_INFO "DHT11_Char: Finished %s from the BBB LKM!\n", name);
}

/** @brief The device open function that is called each time the device is opened
 *  This will only increment the numberOpens counter in this case.
 *  @param inodep A pointer to an inode object (defined in linux/fs.h)
 *  @param filep A pointer to a file object (defined in linux/fs.h)
 */
static int dev_open(struct inode *inodep, struct file *filep){
   printk(KERN_INFO "DHT11_Char: Device has been opened time(s)\n");

    /* This Device does not allow write access */
    if ( ((filep->f_flags & O_ACCMODE) == O_WRONLY)
    || ((filep->f_flags & O_ACCMODE) == O_RDWR) ) {
         printk(KERN_INFO "DHT11_Char: Write access is prohibited\n");
         return -EACCES;
    }

    if (!mutex_trylock(&dht11_device_mutex)) {
        printk(KERN_INFO "DHT11_Char: another process is accessing the device\n");
        return -EBUSY;
    }

    mutex_unlock(&dht11_device_mutex);
    return 0;
}
 
/** @brief This function is called whenever device is being read from user space i.e. data is
 *  being sent from the device to the user. In this case is uses the copy_to_user() function to
 *  send the buffer string to the user and captures any errors.
 *  @param filep A pointer to a file object (defined in linux/fs.h)
 *  @param buffer The pointer to the buffer to which this function writes the data
 *  @param len The length of the b
 *  @param offset The offset if required
 */
static ssize_t dev_read(struct file *filep, char *buffer, size_t len, loff_t *offset){
   int error_count = 0;
   error_count = read_sensor();
   if(error_count == 0){
       error_count = copy_to_user(buffer, BUFFER_DHT11, 5);
   }
   return error_count;
   
}
 
/** @brief This function is called whenever the device is being written to from user space i.e.
 *  data is sent to the device from the user. The data is copied to the message[] array in this
 *  LKM using the sprintf() function along with the length of the string.
 *  @param filep A pointer to a file object
 *  @param buffer The buffer to that contains the string to write to the device
 *  @param len The length of the array of data that is being passed in the const char buffer
 *  @param offset The offset if required
 */
/*static ssize_t dev_write(struct file *filep, const char *buffer, size_t len, loff_t *offset){
   printk(KERN_INFO "DHT11_Char: Write Function is no allowed\n");
   return len;
}*/
 
/** @brief The device release function that is called whenever the device is closed/released by
 *  the userspace program
 *  @param inodep A pointer to an inode object (defined in linux/fs.h)
 *  @param filep A pointer to a file object (defined in linux/fs.h)
 */
static int dev_release(struct inode *inodep, struct file *filep){
    mutex_unlock(&dht11_device_mutex);
    printk(KERN_INFO "DHT11_Char: Device successfully closed\n");
    return 0;
}


 
/** @brief A module must use the module_init() module_exit() macros from linux/init.h, which
 *  identify the initialization function at insertion time and the cleanup function (as
 *  listed above)
 */
module_init(DHT11_Char_init);
module_exit(DHT11_Char_exit);