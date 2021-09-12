/*!
 * Include files
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/console.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <asm/mach-types.h>
#include <linux/kthread.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
//#include <stdint.h>
#include <linux/broadcom/bcm2835_smi.h>
//#include <math.h>
#include <linux/workqueue.h>


#define DEBUG
#define SMI_FB_NAME      "smi_fb"
#define DRIVER_NAME      "smi-fb-bcm2835"
#define LINES            512
#define LEDS             64
#define BPP              4
#define WIDTH            (LEDS * 2)
#define LINE_LEN 	 (LEDS * BPP)
#define OFFSET_FROM_LINE(line) (line * LINE_LEN)
#define FIFO_LINES	 16

#define FRAME_GPIO       20
#define WAIT_GPIO        16
#define RST_GPIO	 5

//#define USE_TASK

static struct task_struct *task = NULL;
static struct fb_info * g_fb;
static struct bcm2835_smi_instance *smi_inst;
static struct workqueue_struct *frame_wq;



static int smifb_map_video_memory(struct fb_info *fbi);
static int smifb_unmap_video_memory(struct fb_info *fbi);

static int dump(void *arg);
static void frame_work_func(struct work_struct *work);

struct frame_work
{
	//struct fb_info * fb;
	struct work_struct work;
	u8 * back;
	struct bcm2835_smi_instance *inst;
	u8 * output;
	u8 * o_lut;
	u8 * p_lut;
	int (*f)(void*);
};

static struct frame_work ftd;


DECLARE_WORK(fwork, frame_work_func);
//DECLARE_TASKLET(frame_tasklet, frame_tasklet_func, (unsigned long)&ftd);

#include "polar_lut.h"
#include "offset_lut.h"
static u8 polar[LINES][LEDS][BPP];
//static u8 back_buffer[128*128*4];
static u8 flag = 0;
static u16 current_line = 0;

static void m_rgba(u8 i[4], u8 a, u8 o[4])
{
        o[0] = ((u32)i[0] * (u32)a) >> 8;
        o[1] = ((u32)i[1] * (u32)a) >> 8;
        o[2] = ((u32)i[2] * (u32)a) >> 8;
        o[3] = ((u32)i[3] * (u32)a) >> 8;
}

//void mf_rgba(u8 i[4], u8 a, u8 o[4])
//{
//        o[0] = (i[0] * a) >> 8;
//        o[1] = (i[1] * a) >> 8;
//        o[2] = (i[2] * a) >> 8;
//        o[3] = (i[3] * a) >> 8;
//}

static void af_rgba(u8 i[4], u8 i2[4], u8 o[4])
{
        o[0] = i[0] + i2[0];
        o[1] = i[1] + i2[1];
        o[2] = i[2] + i2[2];
        o[3] = i[3] + i2[3];
}

static void make_polar(u8 *in, u8 *out, u8 *o_lut, u8 *p_lut)
{
        u32 line = 0;
        u32 px = 0;

	//printk("polar\n");

	//kernel_fpu_begin();
        for (line = 0; line < LINES; line++)
        {
                for (px = 0; px < LEDS; px++)
                {
                        u8 ox, oy;
                        u8 tlcx, tlcy;
			u8 *tl, *tr, *bl, *br;
                        u8 pl[4], pr[4], p[4], t1[4], t2[4];

                        ox = o_lut[line*LEDS*2 + px*2 + 0];
                        oy = o_lut[line*LEDS*2 + px*2 + 1];

                        tlcx = p_lut[line*LEDS*2 + px*2 + 0];
                        tlcy = p_lut[line*LEDS*2 + px*2 + 1];

                        tl = &in[tlcy*LEDS*4 + tlcx*4];
                        tr = &in[tlcy*LEDS*4 + (min(LEDS-1, tlcx + 1)*4)];
                        bl = &in[min(LEDS-1, tlcy + 1)*LEDS*4 + tlcx*4];
                        br = &in[min(LEDS-1, tlcy + 1)*LEDS*4 + (min(LEDS-1, tlcx + 1)*4)];


                        m_rgba(tl, (255 - oy), t1);
                        m_rgba(bl, oy, t2);
                        af_rgba(t1, t2, pl);

                        m_rgba(tr, (255 - oy), t1);
                        m_rgba(br, oy, t2);
                        af_rgba(t1, t2, pr);

                        m_rgba(pl, (255 - ox), t1);
                        m_rgba(pr, ox, t2);
                        af_rgba(pl, pr, p);

//                      float pl = tl * (1 - offset.y) + bl * offset.y;
//                      float pr = tr * (1 - offset.y) + br * offset.y;
//                      float p = pl * (1 - offset.x) + pr * offset.x;

                        out[line*LEDS*BPP + px*BPP + 0] = p[0];
                        out[line*LEDS*BPP + px*BPP + 1] = p[1];
                        out[line*LEDS*BPP + px*BPP + 2] = p[2];
                }
        }
	//kernel_fpu_end();
}


static ssize_t dma_bounce_user(
	enum dma_transfer_direction dma_dir,
	char __user *user_ptr,
	size_t count,
	struct bcm2835_smi_bounce_info *bounce)
{
	int chunk_size;
	int chunk_no = 0;
	int count_left = count;

	while (count_left) {
		int rv;
		void *buf;

		/* Wait for current chunk to complete: */
		if (down_timeout(&bounce->callback_sem,
			msecs_to_jiffies(1000))) {
			printk("DMA bounce timed out");
			count -= (count_left);
			break;
		}

		if (bounce->callback_sem.count >= DMA_BOUNCE_BUFFER_COUNT - 1)
			printk("WARNING: Ring buffer overflow");
		chunk_size = count_left > DMA_BOUNCE_BUFFER_SIZE ?
			DMA_BOUNCE_BUFFER_SIZE : count_left;
		buf = bounce->buffer[chunk_no % DMA_BOUNCE_BUFFER_COUNT];
		if (dma_dir == DMA_DEV_TO_MEM)
			rv = copy_to_user(user_ptr, buf, chunk_size);
		else
			rv = copy_from_user(buf, user_ptr, chunk_size);
		if (rv)
			printk("copy_*_user() failed!: %d", rv);
		user_ptr += chunk_size;
		count_left -= chunk_size;
		chunk_no++;
	}
	return count;
}

static void frame_work_func(struct work_struct *work)
{
	//printk("i %lu\n", (unsigned long) work);
	(void)dump((void*)work);
	//kfree((void*)work);
}

static irq_handler_t frame_irq_handler(unsigned int irq, void *dev_id, struct pt_regs *regs)
{
	//printk("h\n");
	//flag = 1;
	//schedule_work(&(ftd.work));
	queue_work(frame_wq, &fwork);
	//tasklet_schedule(&frame_tasklet);
	return (irq_handler_t) IRQ_HANDLED;
}

static int dump(void *arg)
{
	struct bcm2835_smi_bounce_info *bounce;
	char *data;
	size_t size;
	size_t count;

//	struct frame_work * stuff = (struct frame_work *)arg;
//	printk("fb %08X %u %lu %lu %lu %lu\n", (unsigned int) stuff->back, stuff->back[0], (unsigned long) stuff->inst, (unsigned long) stuff->output, (unsigned long) stuff->o_lut, (unsigned long) stuff->p_lut);


#ifdef USE_TASK
	set_current_state(TASK_INTERRUPTIBLE);


	while (!kthread_should_stop())
	{
		//printk("here\n");
		set_current_state(TASK_RUNNING);


		//make_polar(stuff->back, stuff->output, stuff->o_lut, stuff->p_lut);

		//for (line = 0; line < LINES; line += fifo_lines)
		//{
	//		while (gpio_get_value(WAIT_GPIO))
	//		{
	//			//sleep
	//			usleep(1)
	//		}
		if (!gpio_get_value(WAIT_GPIO))
#else
		while (!gpio_get_value(WAIT_GPIO))
#endif
		{
			//printk("go\n");
			if (gpio_get_value(FRAME_GPIO))
			{
				//printk("fsync\n");
				current_line = 0;
			}
			if (current_line <= (LINES - FIFO_LINES))
			{
				//printk("%u %lu\n", current_line, (unsigned long)g_fb->screen_base);
				data = g_fb->screen_base + OFFSET_FROM_LINE(current_line);///line);//stuff->output;
				size = FIFO_LINES * LINE_LEN;// * (g_fb->screen_size / LINES);//LEDS*BPP;//128*128*4;

				count = bcm2835_smi_user_dma(smi_inst,
					DMA_MEM_TO_DEV, data, size,
					&bounce);
		

				if (count)
				{
					count = dma_bounce_user(DMA_MEM_TO_DEV,
						data,
						count, bounce);

					current_line += FIFO_LINES;

					if (current_line > (LINES - FIFO_LINES))
					{
						current_line = 0;
					}
				}
			}
		}
#ifdef USE_TASK		

		//set_current_state(TASK_INTERRUPTIBLE);
		//msleep(10);
		//usleep(1);
	//	schedule();
	}
#endif
	return 0;
}

/*
 * Set fixed framebuffer parameters based on variable settings.
 *
 * @param       info     framebuffer information pointer
 */
static int smifb_set_fix(struct fb_info *info)
{
	struct fb_fix_screeninfo *fix = &info->fix;
	struct fb_var_screeninfo *var = &info->var;
	printk("smifb_set_fix\n");
	fix->line_length = var->xres_virtual * var->bits_per_pixel / 8;

	fix->type = FB_TYPE_PACKED_PIXELS;
	fix->accel = FB_ACCEL_NONE;
	fix->visual = FB_VISUAL_TRUECOLOR;
	fix->xpanstep = 0;
	fix->ywrapstep = 0;
	fix->ypanstep = 0;

	return 0;
}


/*
 * Set framebuffer parameters and change the operating mode.
 *
 * @param       info     framebuffer information pointer
 */
static int smifb_set_par(struct fb_info *fbi)
{
	int retval = 0;
	u32 mem_len;
	printk("smifb_set_par\n");
	printk("Reconfiguring framebuffer\n");

	smifb_set_fix(fbi);

	mem_len = fbi->var.yres_virtual * fbi->fix.line_length;
	if (!fbi->fix.smem_start || (mem_len > fbi->fix.smem_len)) {
		if (fbi->fix.smem_start)
			smifb_unmap_video_memory(fbi);

		if (smifb_map_video_memory(fbi) < 0)
			return -ENOMEM;
	}

	//gpio_set_value(RST_GPIO, 1);
	//msleep(1000);
	//gpio_set_value(RST_GPIO, 0);
	return retval;
}


/*
 * Check framebuffer variable parameters and adjust to valid values.
 *
 * @param       var      framebuffer variable parameters
 *
 * @param       info     framebuffer information pointer
 */
static int smifb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	/* fg should not bigger than bg */
	printk("smifb_check_var\n");

	if (var->xres_virtual < var->xres)
		var->xres_virtual = var->xres;

	if (var->yres_virtual < var->yres)
		var->yres_virtual = var->yres;

	var->bits_per_pixel = 32;

	var->red.length = 8;
	var->red.offset = 0;
	var->red.msb_right = 0;

	var->green.length = 8;
	var->green.offset = 8;
	var->green.msb_right = 0;

	var->blue.length = 8;
	var->blue.offset = 16;
	var->blue.msb_right = 0;

	var->transp.length = 8;
	var->transp.offset = 24;
	var->transp.msb_right = 0;

	var->height = -1;
	var->width = -1;
	var->grayscale = 0;
//	printk("%lubpp %luxv %luyv %lux %luy\n", var->bits_per_pixel, var->xres_virtual, var->yres_virtual, var->xres, var->yres);

	return 0;
}


/*
 * Function to handle custom mmap for ual framebuffer.
 *
 * @param       fbi     framebuffer information pointer
 *
 * @param       vma     Pointer to vm_area_struct
 */
static int smifb_mmap(struct fb_info *fbi, struct vm_area_struct *vma)
{
	u32 len;
	unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;
	printk("smifb_mmap\n");

	if (offset < fbi->fix.smem_len) {
		/* mapping framebuffer memory */
		len = fbi->fix.smem_len - offset;
		vma->vm_pgoff = (fbi->fix.smem_start + offset) >> PAGE_SHIFT;
	} else {
		return -EINVAL;
	}

	len = PAGE_ALIGN(len);
	if (vma->vm_end - vma->vm_start > len)
		return -EINVAL;

	/* make buffers bufferable */
	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);

	vma->vm_flags |= VM_IO;// | VM_RESERVED;

	if (remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
			    vma->vm_end - vma->vm_start, vma->vm_page_prot)) {
		printk("mmap remap_pfn_range failed\n");
		return -ENOBUFS;
	}

	return 0;
}

/*!
 * This structure contains the pointers to the control functions that are
 * invoked by the core framebuffer driver to perform operations like
 * blitting, rectangle filling, copy regions and cursor definition.
 */
static struct fb_ops smifb_ops = {
	.owner = THIS_MODULE,
	.fb_set_par = smifb_set_par,
	.fb_check_var = smifb_check_var,
//	.fb_pan_display = smifb_pan_display,
	.fb_mmap = smifb_mmap,
};


/*
 * Main framebuffer functions
 */

/*!
 * Allocates the DRAM memory for the frame buffer.      This buffer is remapped
 * into a non-cached, non-buffered, memory region to allow palette and pixel
 * writes to occur without flushing the cache.  Once this area is remapped,
 * all ual memory access to the video memory should occur at the new region.
 *
 * @param       fbi     framebuffer information pointer
 *
 * @return      Error code indicating success or failure
 */
static int smifb_map_video_memory(struct fb_info *fbi)
{
	printk("smifb_map_video_memory\n");
	if (fbi->fix.smem_len < fbi->var.yres_virtual * fbi->fix.line_length)
		fbi->fix.smem_len = fbi->var.yres_virtual *
				    fbi->fix.line_length;

	fbi->screen_base = dma_alloc_writecombine(fbi->device,
				fbi->fix.smem_len,
				(dma_addr_t *)&fbi->fix.smem_start,
				GFP_DMA);
	if (fbi->screen_base == 0) {
		printk("Unable to allocate framebuffer memory\n");
		fbi->fix.smem_len = 0;
		fbi->fix.smem_start = 0;
		return -EBUSY;
	}

	dev_dbg(fbi->device, "allocated fb @ paddr=0x%08X, size=%d.\n",
		(uint32_t) fbi->fix.smem_start, fbi->fix.smem_len);

	fbi->screen_size = fbi->fix.smem_len;

	/* Clear the screen */
	memset((char *)fbi->screen_base, 0, fbi->fix.smem_len);

	return 0;
}

/*!
 * De-allocates the DRAM memory for the frame buffer.
 *
 * @param       fbi     framebuffer information pointer
 *
 * @return      Error code indicating success or failure
 */
static int smifb_unmap_video_memory(struct fb_info *fbi)
{
	printk("smifb_unmap_video_memory\n");
	dma_free_writecombine(fbi->device, fbi->fix.smem_len,
			      fbi->screen_base, fbi->fix.smem_start);
	fbi->screen_base = 0;
	fbi->fix.smem_start = 0;
	fbi->fix.smem_len = 0;
	return 0;
}

/*!
 * Initializes the framebuffer information pointer. After allocating
 * sufficient memory for the framebuffer structure, the fields are
 * filled with custom information passed in from the configurable
 * structures.  This includes information such as bits per pixel,
 * color maps, screen width/height and RGBA offsets.
 *
 * @return      Framebuffer structure initialized with our information
 */
static struct fb_info *smifb_init_fbinfo(struct fb_ops *ops)
{
	struct fb_info *fbi;
	printk("smifb_init_fbinfo\n");

	/*
	 * Allocate sufficient memory for the fb structure
	 */
	fbi = framebuffer_alloc(sizeof(unsigned int), NULL);
	if (!fbi)
		return NULL;


	fbi->var.activate = FB_ACTIVATE_NOW;

	fbi->fbops = ops;
	fbi->flags = FBINFO_FLAG_DEFAULT;


	return fbi;
}


static int smifb_register(struct fb_info *fbi, unsigned int id)
{
	struct fb_videomode m;
	int ret = 0;
	u32 line = 0;
	u32 pxl = 0;

	printk("smifb_register\n");

	//TODO: Set framebuffer ID
	sprintf(fbi->fix.id, "smi_fb%d", id);

	//Setup small default resolution
	fbi->var.xres_virtual = fbi->var.xres = LEDS;
	fbi->var.yres_virtual = fbi->var.yres  = LINES;
	fbi->var.bits_per_pixel = BPP*8;

	smifb_check_var(&fbi->var, fbi);

	smifb_set_par(fbi);
	//smifb_set_fix(fbi);

	/*added first mode to fbi modelist*/
	if (!fbi->modelist.next || !fbi->modelist.prev)
		INIT_LIST_HEAD(&fbi->modelist);
	fb_var_to_videomode(&m, &fbi->var);
	fb_add_videomode(&m, &fbi->modelist);

	console_lock();
	fbi->flags |= FBINFO_MISC_USEREVENT;
	ret = fb_set_var(fbi, &fbi->var);
	fbi->flags &= ~FBINFO_MISC_USEREVENT;
	console_unlock();


	ret = register_framebuffer(fbi);
	if (ret < 0)
		goto err0;

	//kernel_fpu_begin();
        for (line = 0; line < LINES; line++)
        {
                //float phi = 2 * 3.141592654 * line / 500;
                for (pxl = 0; pxl < LEDS; pxl++)
                {
                        u8 px, py;
                        u8 fx, fy;

                        px = ((pxl * 128) >> 8) + LEDS;//cos(phi) + 32;
                        py = ((pxl * 128) >> 8) + LEDS;//sin(phi) + 32;
                        fx = px;
                        fy = py;

                        polar_lut[line][pxl][0] = fx;
                        polar_lut[line][pxl][1] = fy;

                        offset_lut[line][pxl][0] = px - fx;
                        offset_lut[line][pxl][1] = py - fy;
                }
        }
	//kernel_fpu_end();

#ifdef USE_TASK
	task = kthread_run(dump, NULL, "dump");
#endif
	//ftd.fb = g_fb;
	ftd.back = NULL;//(u8*)back_buffer;
	ftd.inst = smi_inst;
	ftd.output = (u8*)polar;
	ftd.o_lut = (u8*)offset_lut;
	ftd.p_lut = (u8*)polar_lut;
	ftd.f = &dump;

	frame_wq = create_singlethread_workqueue("frame_queue");
	//INIT_WORK(&(ftd.work), frame_work_func);

	gpio_request(RST_GPIO, "sys_fs");
	gpio_direction_output(RST_GPIO, 1);
	msleep(100);
	gpio_set_value(RST_GPIO, 0);
	msleep(100);
	gpio_set_value(RST_GPIO, 1);
	msleep(100);
	gpio_set_value(RST_GPIO, 0);
	
	gpio_request(FRAME_GPIO, "sys_fs");
	gpio_direction_input(FRAME_GPIO);
	
	gpio_request(WAIT_GPIO, "sys_fs");
	gpio_direction_input(WAIT_GPIO);

	ret = request_irq(gpio_to_irq(WAIT_GPIO), (irq_handler_t)frame_irq_handler, IRQF_TRIGGER_FALLING, "smifb_frame_irq", NULL);
	printk("smifb frame irq result: %u\n", ret);

	return ret;
err0:
	return ret;
}

static void smifb_unregister(struct fb_info *fbi)
{
	printk("smifb_unregister\n");
	unregister_framebuffer(fbi);
}

/*!
 * Main entry function for the framebuffer. The function registers the power
 * management callback functions with the kernel and also registers the MXCFB
 * callback functions with the core Linux framebuffer driver \b fbmem.c
 *
 * @return      Error code indicating success or failure
 */
//int __init smifb_init(void)
static int smifb_probe(struct platform_device *pdev)
{
	//int err;
	//void *ptr_err;
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node, *smi_node;
	int ret = 0;

	printk("smifb_probe\n");

	if (!node) {
		dev_err(dev, "No device tree node supplied!");
		return -EINVAL;
	}

	smi_node = of_parse_phandle(node, "smi_handle", 0);

	if (!smi_node) {
		dev_err(dev, "No such property: smi_handle");
		return -ENXIO;
	}

	smi_inst = bcm2835_smi_get(smi_node);

	if (!smi_inst)
		return -EPROBE_DEFER;

	//TODO dirty hack
	struct smi_settings * settings = (struct smi_settings*)((u32)smi_inst + (u32)sizeof(struct device *));

	settings->data_width = SMI_WIDTH_8BIT;
	settings->pack_data = 1;
	settings->read_setup_time = 4;
	settings->read_hold_time = 4;
	settings->read_pace_time = 1;
	settings->read_strobe_time = 4;
	settings->write_setup_time = 16;
	settings->write_hold_time = 16;
	settings->write_pace_time = 1;
	settings->write_strobe_time = 32;
	settings->dma_enable = 1;
	settings->dma_passthrough_enable = 0;
	settings->dma_read_thresh = 1;
	settings->dma_write_thresh = 63;
	settings->dma_panic_read_thresh = 32;
	settings->dma_panic_write_thresh = 32;

	bcm2835_smi_set_regs_from_settings(smi_inst);

        /*
         * Initialize FB structures
         */

	//g_fb = kzalloc(sizeof(struct fb_info*), GFP_KERNEL);
        g_fb = smifb_init_fbinfo(&smifb_ops);
        if (!g_fb) {
                ret = -ENOMEM;
                goto init_fbinfo_failed;
        }

        ret = smifb_register(g_fb, 0);
        if (ret < 0)
                goto smifb_register_failed;


        return 0;
smifb_register_failed:
init_fbinfo_failed:
	if(g_fb)
	{
		smifb_unregister(g_fb);
       		framebuffer_release(g_fb);
	}
        return ret;

}

//void smifb_exit(void)
static int smifb_remove(struct platform_device *pdev)
{
	printk("smifb_exit\n");

	free_irq(gpio_to_irq(WAIT_GPIO), NULL);

	gpio_free(WAIT_GPIO);
	gpio_free(FRAME_GPIO);
	gpio_free(RST_GPIO);
	flush_workqueue(frame_wq);
	destroy_workqueue(frame_wq);

	//tasklet_kill(&frame_tasklet);

	if (task)
		kthread_stop(task);
	if(g_fb)
	{
		smifb_unregister(g_fb);
		smifb_unmap_video_memory(g_fb);

		framebuffer_release(g_fb);
	}
	return 0;
}

//module_init(smifb_init);
//module_exit(smifb_exit);

static const struct of_device_id bcm2835_smi_fb_of_match[] = {
	{.compatible = "brcm,bcm2835-smi-fb",},
	{ /* sentinel */ },
};

MODULE_DEVICE_TABLE(of, bcm2835_smi_fb_of_match);

static struct platform_driver bcm2835_smi_fb_driver = {
	.probe = smifb_probe,
	.remove = smifb_remove,
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = bcm2835_smi_fb_of_match,
	},
};

module_platform_driver(bcm2835_smi_fb_driver);
MODULE_ALIAS("platform:smi-fb-bcm2835");

MODULE_AUTHOR("Ivan Chichkine");
MODULE_DESCRIPTION("SMI framebuffer driver");
MODULE_LICENSE("GPL");
