#include <linux/module.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/sched.h>
#include <linux/sched/signal.h>
#include <linux/poll.h>



#define GLOBALMEM_SIZE 0x1000  //4k内存
#define MEM_CLEAR 0x1 //清零内存命令
#define GLOBALMEM_MAJOR 350 //预设的主设备号


static int globalmem_major = GLOBALMEM_MAJOR; //define global var.
//设备结构体    
struct globalmem_dev{
    struct cdev cdev;// include struct cdev; 
    unsigned char mem[GLOBALMEM_SIZE]; //define the size of memory, using char[]
    struct mutex mutex; // add mutex-lock
    unsigned int current_len; //the length of valid data;
    wait_queue_head_t r_wait; //read wait queue
    wait_queue_head_t w_wait; //write wait queue
};

struct globalmem_dev *globalmem_devp; // init a pointer of devices struct

//here comes some functions, refer to file_operations(the args are the same in file_operations)
// open devices
int globalmem_open(struct inode *inode,struct file *filp){
    filp->private_data= globalmem_devp; 
    //assign the devices pointer to file->private. no given reason. 
    // It says, functinos of file_operations can using private_data to get device pointer.
    // "Many engineers do this"
    return 0;
}
// release devices
int globalmem_release(struct inode *inode,struct file *filp){
    return 0;
}
// function of devices control
	// long (*unlocked_ioctl) (struct file *, unsigned int, unsigned long);
static long globalmem_ioctl(struct file *filp, unsigned int cmd, unsigned long arg){
    struct globalmem_dev *dev = filp->private_data; 
    switch (cmd){
        case MEM_CLEAR:
            mutex_lock(&dev->mutex);

            memset(dev->mem,0,GLOBALMEM_SIZE); //clean the whole memory

            mutex_unlock(&dev->mutex);

            printk(KERN_INFO "globalmem is set to zero\n");
            break;
        default:
            return -EINVAL;
    }
    return 0;
}
// function of devices read
static ssize_t globalmem_read(struct file *filp, char __user *buf ,size_t count ,loff_t *ppos){

    int ret;
    struct globalmem_dev *dev = filp->private_data; 
    DECLARE_WAITQUEUE(wait,current); // declare a wait queue, named wait. and it contain current process info.
    //lock the area.
    mutex_lock(&dev->mutex);
    add_wait_queue(&dev->r_wait,&wait);
    while(dev->current_len ==0){
       if(filp->f_flags & O_NONBLOCK){
           ret = -EAGAIN;
           goto out; //jump to out-line, and execute the code in order;
       }
       __set_current_state(TASK_INTERRUPTIBLE);
       mutex_unlock(&dev->mutex);
       schedule();
       if(signal_pending(current)){
           ret = -ERESTARTSYS;
           goto out2;
       }
       mutex_lock(&dev->mutex);
   } 
if(count>dev->current_len){
    count = dev->current_len;
    }
if(copy_to_user(buf,dev->mem,count)){
    ret = -EFAULT;
    goto out;
}else{
    memcpy(dev->mem,dev->mem+count,dev->current_len-count); // this function can copy from  dev->mem+count to dev->mem (length = current-len)  
    dev->current_len-=count;
    printk(KERN_INFO "read %d bytes(s),current_len:%d\n",count,dev->current_len);
    wake_up_interruptible(&dev->w_wait); //wake up write progress.
    ret = count;
    }
// the code will excute the follows. In order!
// because it didn't encounter any return function.
out: 
mutex_unlock(&dev->mutex);
out2:
remove_wait_queue(&dev->r_wait,&wait);
set_current_state(TASK_RUNNING);
return ret;
}
// functions of device write
// imitate functions : read

static ssize_t globalmem_write(struct file *filp,const char __user *buf, size_t count,loff_t *ppos){
    int ret ;
    struct globalmem_dev *dev = filp->private_data;
    DECLARE_WAITQUEUE(wait,current);
    mutex_lock(&dev->mutex);
    add_wait_queue(&dev->w_wait,&wait);
    while(dev->current_len ==GLOBALMEM_SIZE){
        if(filp->f_flags & O_NONBLOCK){
            ret = -EAGAIN;
            goto out;
        }
        __set_current_state(TASK_INTERRUPTIBLE);
        mutex_unlock(&dev->mutex);
        schedule();
        if(signal_pending(current)){
            ret= -ERESTARTSYS;
            goto out2;
        }
        mutex_lock(&dev->mutex);
    } 

    if(count>GLOBALMEM_SIZE-dev->current_len){
        count = GLOBALMEM_SIZE-dev->current_len;
    }

    if(copy_from_user(dev->mem+dev->current_len,buf,count)){
        ret = -EFAULT;
        goto out;
    }else{
        dev->current_len += count;
        printk(KERN_INFO "written %d bytes(s) from %d\n",count,dev->current_len);
        wake_up_interruptible(&dev->r_wait);
        ret =count;
    }
out:
mutex_unlock(&dev->mutex);
out2:
remove_wait_queue(&dev->w_wait,&wait);
set_current_state(TASK_RUNNING);
return ret;

}
// functionds of devices seek
// int orig is the original position of file. it's a macro.
static loff_t globalmem_llseek(struct file *filp, loff_t offset, int orig){
    loff_t ret;
    switch (orig){
        case 0: //from the beginning
            if(offset<0)
            {
                ret = -EINVAL;
                break;
            }
            if((unsigned int)offset >GLOBALMEM_SIZE)
            {
                ret= -EINVAL;
            }
            filp->f_pos = (unsigned int) offset;
            ret = filp->f_pos;
            break;
        case 1: //current position
            if((filp->f_pos+offset)<0){
                ret =-EINVAL;
                break;
            }
            if((filp->f_pos+offset)>GLOBALMEM_SIZE){
                ret = -EINVAL;
                break;
            }
            filp->f_pos+=offset;
            ret = filp->f_pos;
            break;
        case 2: //end positon
            if(offset>0)
            {
                ret =-EINVAL;
                break;

            }
            if((unsigned int )offset >GLOBALMEM_SIZE)
            {
                ret = -EINVAL;
            }
            filp->f_pos = (unsigned int) GLOBALMEM_SIZE+offset;
            ret = filp->f_pos;
            break;
        default:
            ret = -EINVAL;
    }
    return ret;
}

//add poll function 
static unsigned int globalmem_poll(struct file *filp,poll_table *wait){
    unsigned int mask = 0;
    struct globalmem_dev *dev = filp->private_data;
    mutex_lock(&dev->mutex); // determine the state of write and read, it needs mutex.
    poll_wait(filp,&dev->r_wait,wait);
    poll_wait(filp,&dev->w_wait,wait);
    if(dev->current_len!=0){
        mask |=POLLIN |POLLRDNORM;
    }
    if(dev->current_len!=GLOBALMEM_SIZE){
        mask |= POLLOUT | POLLWRNORM;
    }
    mutex_unlock(&dev->mutex);
    return mask;
}
// define the file operations, and assign them to a struct file_operations.
static const struct file_operations globalmem_fops={
    .owner = THIS_MODULE,
    .llseek = globalmem_llseek,
    .read = globalmem_read,
    .write = globalmem_write,
    .unlocked_ioctl = globalmem_ioctl,
    .open = globalmem_open,
    .release = globalmem_release,
    .poll = globalmem_poll,
};

static void globalmem_setup_cdev(struct globalmem_dev *dev, int index){
    int err,devno = MKDEV(globalmem_major,index); //get devno
    cdev_init(&dev->cdev,&globalmem_fops);
    dev->cdev.owner =THIS_MODULE;
    err = cdev_add(&dev->cdev,devno,1);
    if(err){
        printk(KERN_INFO "Error %d adding globalmem %d",err,index);
    }
}

static int globalmem_init(void){
    int res;
    dev_t devno= MKDEV(globalmem_major,0);

    if(globalmem_major){
        res = register_chrdev_region(devno,1,"globalmem");  //refer to the api
    }else{
        res = alloc_chrdev_region(&devno,0,1,"globalmem"); //refer to the api
        globalmem_major = MAJOR(devno); //by usring MAJOR to get the allocated major number
    } 
    if(res<0){
        return res; // fail to register
    }
    // get mem for struct globalmem_devp
    globalmem_devp = kzalloc(sizeof(struct globalmem_dev),GFP_KERNEL); //second arg is flags, refer to kmalloc
    if(!globalmem_devp){
        res = -ENOMEM;
        goto fail_malloc; //fail to get mem.
    }


    globalmem_setup_cdev(globalmem_devp,0);

    mutex_init(&globalmem_devp->mutex); //init mutex
    init_waitqueue_head(&globalmem_devp->r_wait); //init r_wait queue
    init_waitqueue_head(&globalmem_devp->w_wait);// init w_wait queue
    
    return 0;

fail_malloc:
    unregister_chrdev_region(devno,1);
    return res;
}
void globalmem_exit(void){
    cdev_del(&globalmem_devp->cdev);
    kfree(globalmem_devp);
    unregister_chrdev_region(MKDEV(globalmem_major,0),1);

}


MODULE_AUTHOR("leo");
MODULE_LICENSE("Dual BSD/GPL");

module_init(globalmem_init);
module_exit(globalmem_exit);