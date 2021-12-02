#include <linux/module.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#define GLOBALMEM_SIZE 0x1000  //4k内存
#define MEM_CLEAR 0x1 //清零内存命令
#define GLOBALMEM_MAJOR 350 //预设的主设备号


static int globalmem_major = GLOBALMEM_MAJOR; //define global var.
//设备结构体    
struct globalmem_dev{
    struct cdev cdev;// include struct cdev; 
    unsigned char mem[GLOBALMEM_SIZE]; //define the size of memory, using char[]
    struct mutex mutex; // add mutex-lock
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
static ssize_t globalmem_read(struct file *filp, char __user *buf ,size_t size ,loff_t *ppos){

    unsigned long p = *ppos; // its definition is long.
    int ret = 0 ;
    unsigned int count = size;
    struct globalmem_dev *dev = filp->private_data; 
    if(p>=GLOBALMEM_SIZE){
        return 0; // p is invalid, crossing the boundary.
    
    }
    if(count>GLOBALMEM_SIZE-p)
    {
        count = GLOBALMEM_SIZE - p; // maximum of count, it can't cross the boundary
    }
    //lock the area.
    mutex_lock(&dev->mutex);
    if(copy_to_user(buf,(void *)(dev->mem+p),count)) //refer to the function: copy_to_user
    {
        ret = -EFAULT ;
    }else{
        *ppos+= count; //suppose it means that , after reading the current *ppos;
        ret = count;
        printk(KERN_INFO"read %d bytes(s) from %d \n",count,p);
    }
    mutex_unlock(&dev->mutex); // unlock the area
    return ret;
}
// functions of device write
// imitate functions : read

static ssize_t globalmem_write(struct file *filp,const char __user *buf, size_t size,loff_t *ppos){
    unsigned long p= *ppos;
    unsigned int count =size;
    int ret =0;
    struct globalmem_dev *dev = filp->private_data; 
    if(p>=GLOBALMEM_SIZE)
    {
        return 0;
    }
    if(count>GLOBALMEM_SIZE-p)
    {
        count=GLOBALMEM_SIZE-p; 
    }
    mutex_lock(&dev->mutex);

    if(copy_from_user(dev->mem+p,buf,count))
    {
        ret= -EFAULT;
    }else {
        *ppos += count;
        ret = count;
        printk(KERN_INFO "written %d bytes(s) from %d\n",count,p);
    }
    mutex_unlock(&dev->mutex);
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


// define the file operations, and assign them to a struct file_operations.
static const struct file_operations globalmem_fops={
    .owner = THIS_MODULE,
    .llseek = globalmem_llseek,
    .read = globalmem_read,
    .write = globalmem_write,
    .unlocked_ioctl = globalmem_ioctl,
    .open = globalmem_open,
    .release = globalmem_release,
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

int globalmem_init(void){
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
    mutex_init(&globalmem_devp->mutex);
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