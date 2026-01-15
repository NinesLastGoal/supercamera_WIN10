/*
 * SuperCamera USB Driver
 * 
 * Linux kernel driver for UseePlus/SuperCamera USB borescope cameras.
 * VID: 0x2ce3, PID: 0x3828
 * 
 * Protocol: Proprietary USB bulk transfer with JPEG frames
 * - USB header: 5 bytes (magic 0xBBAA, camera ID, payload length)
 * - Camera header: 7 bytes (frame ID, camera num, flags, padding)
 * - JPEG payload data
 * 
 * License: GPL v2
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/usb.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/mutex.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/vmalloc.h>

#define DRIVER_NAME "supercamera"
#define DRIVER_VERSION "1.0.0"

#define VENDOR_ID  0x2ce3
#define PRODUCT_ID 0x3828

#define INTERFACE_NUM 1
#define EP_IN  0x81
#define EP_OUT 0x01

#define URB_BUFFER_SIZE 4096
#define NUM_URBS 8
#define RING_BUFFER_SIZE (16*1024*1024)

struct supercamera_dev {
    struct usb_device *udev;
    struct usb_interface *interface;
    
    /* URBs with DMA-coherent buffers */
    struct urb *urbs[NUM_URBS];
    unsigned char *urb_buffers[NUM_URBS];
    dma_addr_t urb_dma_handles[NUM_URBS];
    
    /* Ring buffer */
    unsigned char *ring_buffer;
    size_t ring_write_pos;
    size_t ring_read_pos;
    size_t ring_available;
    spinlock_t ring_lock;
    
    /* Character device */
    struct cdev cdev;
    dev_t dev_num;
    struct class *class;
    struct device *device;
    struct mutex io_mutex;
    bool device_open;
    wait_queue_head_t read_wait;
    
    /* State */
    bool streaming;
    bool disconnecting;
    
    /* Stats */
    unsigned long urbs_completed;
    unsigned long bytes_received;
};

static int major_num;

/* Ring buffer write */
static void ring_write(struct supercamera_dev *dev, unsigned char *data, size_t len)
{
    unsigned long flags;
    size_t space, chunk1, chunk2;
    
    if (len == 0 || dev->disconnecting)
        return;
    
    spin_lock_irqsave(&dev->ring_lock, flags);
    
    space = RING_BUFFER_SIZE - dev->ring_available;
    if (len > space) {
        /* Overflow - drop oldest data */
        len = space;
    }
    
    if (len == 0) {
        spin_unlock_irqrestore(&dev->ring_lock, flags);
        return;
    }
    
    chunk1 = min(len, RING_BUFFER_SIZE - dev->ring_write_pos);
    memcpy(dev->ring_buffer + dev->ring_write_pos, data, chunk1);
    
    if (chunk1 < len) {
        chunk2 = len - chunk1;
        memcpy(dev->ring_buffer, data + chunk1, chunk2);
        dev->ring_write_pos = chunk2;
    } else {
        dev->ring_write_pos = (dev->ring_write_pos + chunk1) % RING_BUFFER_SIZE;
    }
    
    dev->ring_available += len;
    spin_unlock_irqrestore(&dev->ring_lock, flags);
    
    wake_up_interruptible(&dev->read_wait);
}

/* Ring buffer read */
static size_t ring_read(struct supercamera_dev *dev, unsigned char *data, size_t len)
{
    unsigned long flags;
    size_t chunk1, chunk2, to_read;
    
    spin_lock_irqsave(&dev->ring_lock, flags);
    
    to_read = min(len, dev->ring_available);
    if (to_read == 0) {
        spin_unlock_irqrestore(&dev->ring_lock, flags);
        return 0;
    }
    
    chunk1 = min(to_read, RING_BUFFER_SIZE - dev->ring_read_pos);
    memcpy(data, dev->ring_buffer + dev->ring_read_pos, chunk1);
    
    if (chunk1 < to_read) {
        chunk2 = to_read - chunk1;
        memcpy(data + chunk1, dev->ring_buffer, chunk2);
        dev->ring_read_pos = chunk2;
    } else {
        dev->ring_read_pos = (dev->ring_read_pos + chunk1) % RING_BUFFER_SIZE;
    }
    
    dev->ring_available -= to_read;
    spin_unlock_irqrestore(&dev->ring_lock, flags);
    
    return to_read;
}

/* URB completion callback */
static void supercamera_urb_complete(struct urb *urb)
{
    struct supercamera_dev *dev = urb->context;
    int status = urb->status;
    
    if (dev->disconnecting)
        return;
    
    if (status) {
        if (status != -ENOENT && status != -ECONNRESET && status != -ESHUTDOWN) {
            if (dev->streaming)
                usb_submit_urb(urb, GFP_ATOMIC);
        }
        return;
    }
    
    /* Copy received data to ring buffer */
    if (urb->actual_length > 0) {
        ring_write(dev, urb->transfer_buffer, urb->actual_length);
        dev->urbs_completed++;
        dev->bytes_received += urb->actual_length;
    }
    
    /* Resubmit URB */
    if (dev->streaming) {
        usb_submit_urb(urb, GFP_ATOMIC);
    }
}

/* Start streaming */
static int start_streaming(struct supercamera_dev *dev)
{
    int result, i;
    unsigned char cmd[5] = { 0xBB, 0xAA, 0x05, 0x00, 0x00 };
    int actual_length;
    
    if (dev->streaming)
        return 0;
    
    /* Set alternate interface 1 */
    result = usb_set_interface(dev->udev, INTERFACE_NUM, 1);
    if (result) {
        printk(KERN_ERR "supercamera: Failed to set interface: %d\n", result);
        return result;
    }
    
    /* Send start command */
    result = usb_bulk_msg(dev->udev, usb_sndbulkpipe(dev->udev, EP_OUT),
                         cmd, sizeof(cmd), &actual_length, 5000);
    if (result) {
        printk(KERN_ERR "supercamera: Failed to send start command: %d\n", result);
        usb_set_interface(dev->udev, INTERFACE_NUM, 0);
        return result;
    }
    
    /* Allocate and submit URBs with DMA-coherent memory */
    for (i = 0; i < NUM_URBS; i++) {
        dev->urbs[i] = usb_alloc_urb(0, GFP_KERNEL);
        if (!dev->urbs[i]) {
            result = -ENOMEM;
            goto error;
        }
        
        dev->urb_buffers[i] = usb_alloc_coherent(dev->udev, URB_BUFFER_SIZE,
                                                  GFP_KERNEL, &dev->urb_dma_handles[i]);
        if (!dev->urb_buffers[i]) {
            usb_free_urb(dev->urbs[i]);
            dev->urbs[i] = NULL;
            result = -ENOMEM;
            goto error;
        }
        
        usb_fill_bulk_urb(dev->urbs[i], dev->udev,
                         usb_rcvbulkpipe(dev->udev, EP_IN),
                         dev->urb_buffers[i], URB_BUFFER_SIZE,
                         supercamera_urb_complete, dev);
        
        dev->urbs[i]->transfer_dma = dev->urb_dma_handles[i];
        dev->urbs[i]->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
        
        result = usb_submit_urb(dev->urbs[i], GFP_KERNEL);
        if (result) {
            usb_free_coherent(dev->udev, URB_BUFFER_SIZE,
                             dev->urb_buffers[i], dev->urb_dma_handles[i]);
            usb_free_urb(dev->urbs[i]);
            dev->urbs[i] = NULL;
            dev->urb_buffers[i] = NULL;
            goto error;
        }
    }
    
    dev->streaming = true;
    printk(KERN_INFO "supercamera: Streaming started\n");
    return 0;
    
error:
    for (i = 0; i < NUM_URBS; i++) {
        if (dev->urbs[i]) {
            usb_kill_urb(dev->urbs[i]);
            if (dev->urb_buffers[i]) {
                usb_free_coherent(dev->udev, URB_BUFFER_SIZE,
                                 dev->urb_buffers[i], dev->urb_dma_handles[i]);
            }
            usb_free_urb(dev->urbs[i]);
            dev->urbs[i] = NULL;
            dev->urb_buffers[i] = NULL;
        }
    }
    usb_set_interface(dev->udev, INTERFACE_NUM, 0);
    return result;
}

/* Stop streaming */
static void stop_streaming(struct supercamera_dev *dev)
{
    int i;
    
    if (!dev->streaming)
        return;
    
    dev->streaming = false;
    
    for (i = 0; i < NUM_URBS; i++) {
        if (dev->urbs[i]) {
            usb_kill_urb(dev->urbs[i]);
            if (dev->urb_buffers[i]) {
                usb_free_coherent(dev->udev, URB_BUFFER_SIZE,
                                 dev->urb_buffers[i], dev->urb_dma_handles[i]);
            }
            usb_free_urb(dev->urbs[i]);
            dev->urbs[i] = NULL;
            dev->urb_buffers[i] = NULL;
        }
    }
    
    usb_set_interface(dev->udev, INTERFACE_NUM, 0);
    printk(KERN_INFO "supercamera: Streaming stopped\n");
}

/* Character device operations */
static int supercamera_open(struct inode *inode, struct file *file)
{
    struct supercamera_dev *dev = container_of(inode->i_cdev, struct supercamera_dev, cdev);
    unsigned long flags;
    
    if (mutex_lock_interruptible(&dev->io_mutex))
        return -ERESTARTSYS;
    
    if (dev->device_open) {
        mutex_unlock(&dev->io_mutex);
        return -EBUSY;
    }
    
    file->private_data = dev;
    dev->device_open = true;
    
    /* Reset read position to get fresh data */
    spin_lock_irqsave(&dev->ring_lock, flags);
    dev->ring_read_pos = dev->ring_write_pos;
    dev->ring_available = 0;
    spin_unlock_irqrestore(&dev->ring_lock, flags);
    
    mutex_unlock(&dev->io_mutex);
    return 0;
}

static int supercamera_release(struct inode *inode, struct file *file)
{
    struct supercamera_dev *dev = file->private_data;
    
    mutex_lock(&dev->io_mutex);
    dev->device_open = false;
    mutex_unlock(&dev->io_mutex);
    
    return 0;
}

static ssize_t supercamera_read(struct file *file, char __user *buf,
                               size_t count, loff_t *pos)
{
    struct supercamera_dev *dev = file->private_data;
    unsigned char *temp_buf;
    ssize_t bytes_read;
    int ret;
    
    /* Wait for data */
    while (dev->ring_available == 0) {
        if (dev->disconnecting)
            return -ENODEV;
        
        if (file->f_flags & O_NONBLOCK)
            return -EAGAIN;
        
        ret = wait_event_interruptible_timeout(dev->read_wait,
                                               dev->ring_available > 0 ||
                                               dev->disconnecting,
                                               HZ);
        if (ret == 0)
            continue;
        if (ret < 0)
            return -ERESTARTSYS;
    }
    
    temp_buf = kmalloc(count, GFP_KERNEL);
    if (!temp_buf)
        return -ENOMEM;
    
    bytes_read = ring_read(dev, temp_buf, count);
    
    if (bytes_read > 0) {
        if (copy_to_user(buf, temp_buf, bytes_read)) {
            kfree(temp_buf);
            return -EFAULT;
        }
    }
    
    kfree(temp_buf);
    return bytes_read;
}

static struct file_operations supercamera_fops = {
    .owner = THIS_MODULE,
    .open = supercamera_open,
    .release = supercamera_release,
    .read = supercamera_read,
};

/* USB probe */
static int supercamera_probe(struct usb_interface *interface,
                            const struct usb_device_id *id)
{
    struct usb_device *udev = interface_to_usbdev(interface);
    struct supercamera_dev *dev;
    int result, i;
    
    if (interface->cur_altsetting->desc.bInterfaceNumber != INTERFACE_NUM)
        return -ENODEV;
    
    printk(KERN_INFO "supercamera: Device connected\n");
    
    dev = kzalloc(sizeof(*dev), GFP_KERNEL);
    if (!dev)
        return -ENOMEM;
    
    dev->ring_buffer = vmalloc(RING_BUFFER_SIZE);
    if (!dev->ring_buffer) {
        kfree(dev);
        return -ENOMEM;
    }
    
    dev->udev = usb_get_dev(udev);
    dev->interface = interface;
    usb_disable_autosuspend(dev->udev);
    
    dev->streaming = false;
    dev->device_open = false;
    dev->disconnecting = false;
    dev->urbs_completed = 0;
    dev->bytes_received = 0;
    
    mutex_init(&dev->io_mutex);
    spin_lock_init(&dev->ring_lock);
    init_waitqueue_head(&dev->read_wait);
    
    for (i = 0; i < NUM_URBS; i++) {
        dev->urbs[i] = NULL;
        dev->urb_buffers[i] = NULL;
        dev->urb_dma_handles[i] = 0;
    }
    
    /* Create character device */
    result = alloc_chrdev_region(&dev->dev_num, 0, 1, DRIVER_NAME);
    if (result < 0) {
        goto error_chrdev;
    }
    
    major_num = MAJOR(dev->dev_num);
    
    cdev_init(&dev->cdev, &supercamera_fops);
    dev->cdev.owner = THIS_MODULE;
    
    result = cdev_add(&dev->cdev, dev->dev_num, 1);
    if (result < 0) {
        goto error_cdev;
    }
    
    dev->class = class_create(DRIVER_NAME);
    if (IS_ERR(dev->class)) {
        result = PTR_ERR(dev->class);
        goto error_class;
    }
    
    dev->device = device_create(dev->class, NULL, dev->dev_num, NULL, DRIVER_NAME);
    if (IS_ERR(dev->device)) {
        result = PTR_ERR(dev->device);
        goto error_device;
    }
    
    usb_set_intfdata(interface, dev);
    
    /* Start streaming immediately */
    result = start_streaming(dev);
    if (result) {
        goto error_streaming;
    }
    
    printk(KERN_INFO "supercamera: Device ready at /dev/%s\n", DRIVER_NAME);
    return 0;
    
error_streaming:
    device_destroy(dev->class, dev->dev_num);
error_device:
    class_destroy(dev->class);
error_class:
    cdev_del(&dev->cdev);
error_cdev:
    unregister_chrdev_region(dev->dev_num, 1);
error_chrdev:
    vfree(dev->ring_buffer);
    usb_put_dev(dev->udev);
    kfree(dev);
    return result;
}

/* USB disconnect */
static void supercamera_disconnect(struct usb_interface *interface)
{
    struct supercamera_dev *dev = usb_get_intfdata(interface);
    
    if (!dev)
        return;
    
    printk(KERN_INFO "supercamera: Device disconnected\n");
    
    dev->disconnecting = true;
    wake_up_interruptible(&dev->read_wait);
    
    stop_streaming(dev);
    
    device_destroy(dev->class, dev->dev_num);
    class_destroy(dev->class);
    cdev_del(&dev->cdev);
    unregister_chrdev_region(dev->dev_num, 1);
    
    vfree(dev->ring_buffer);
    usb_put_dev(dev->udev);
    kfree(dev);
}

static const struct usb_device_id supercamera_id_table[] = {
    { USB_DEVICE(VENDOR_ID, PRODUCT_ID) },
    {}
};
MODULE_DEVICE_TABLE(usb, supercamera_id_table);

static struct usb_driver supercamera_driver = {
    .name = DRIVER_NAME,
    .probe = supercamera_probe,
    .disconnect = supercamera_disconnect,
    .id_table = supercamera_id_table,
};

module_usb_driver(supercamera_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("SuperCamera Contributors");
MODULE_DESCRIPTION("USB driver for UseePlus/SuperCamera borescope cameras");
MODULE_VERSION(DRIVER_VERSION);
